#include "innobot_control_hardware/innobot_system.hpp"

#include <bits/stdc++.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace innobot_control_hardware
{

return_type INNOBOTSystemMultiInterfaceHardware::configure(
  const hardware_interface::HardwareInfo & info)
{

  if (configure_default(info) != return_type::OK)
  {
    return return_type::ERROR;
  }

  robot_serial_port_.SetDevice("/dev/ttyACM0");
  robot_serial_port_.SetBaudRate(115200);
  robot_serial_port_.SetTimeout(0);

  hw_joints_.resize(info_.joints.size(), Joint());

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (uint i = 0; i < info_.joints.size(); i++) {
    hw_joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    hw_joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    hw_joints_[i].state.acceleration = std::numeric_limits<double>::quiet_NaN();
    hw_joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    hw_joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    hw_joints_[i].command.acceleration = std::numeric_limits<double>::quiet_NaN();
  }
  control_level_.resize(info_.joints.size(), integration_level_t::POSITION);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // INNOBOTSystemMultiInterface has exactly 3 state interfaces
    // and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"),
        "Joint '%s' has %d command interfaces. 3 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"),
        "Joint '%s'has %d state interfaces. 3 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"),
        "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
INNOBOTSystemMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joints_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_joints_[i].state.acceleration));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
INNOBOTSystemMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joints_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joints_[i].command.velocity));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION,
      &hw_joints_[i].command.acceleration));
  }

  return command_interfaces;
}

return_type INNOBOTSystemMultiInterfaceHardware::start()
{
  robot_serial_port_.Open();
  
  RCLCPP_INFO(
    rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"), "Starting... please wait...");

  for (int i = 0; i <= hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }

  // Set some default values
  for (std::size_t i = 0; i < hw_joints_.size(); i++)
  {
    if (std::isnan(hw_joints_[i].state.position))
    {
      hw_joints_[i].state.position = 0;
    }
    if (std::isnan(hw_joints_[i].state.velocity))
    {
      hw_joints_[i].state.velocity = 0;
    }
    if (std::isnan(hw_joints_[i].state.acceleration))
    {
      hw_joints_[i].state.acceleration = 0;
    }
    if (std::isnan(hw_joints_[i].command.position))
    {
      hw_joints_[i].command.position = 0;
    }
    if (std::isnan(hw_joints_[i].command.velocity))
    {
      hw_joints_[i].command.velocity = 0;
    }
    if (std::isnan(hw_states_positions_[i]))
    {
      hw_states_positions_[i] = 0;
    }
    if (std::isnan(hw_joints_[i].command.acceleration))
    {
      hw_joints_[i].command.acceleration = 0;
    }
    control_level_[i] = integration_level_t::POSITION;
  }
  status_ = hardware_interface::status::STARTED;

  
  RCLCPP_INFO(
    rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"), "System successfully started! %u",
    control_level_[0]);
  return return_type::OK;
}

return_type INNOBOTSystemMultiInterfaceHardware::stop()
{

    RCLCPP_INFO(
    rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"), "Stopping... please wait...");

  for (int i = 0; i <= hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"), "System successfully stopped!");

  robot_serial_port_.Close();
  

  return return_type::OK;
}

return_type INNOBOTSystemMultiInterfaceHardware::read()
{
   
  robot_serial_port_.Write(arduino_cmd_msg_.c_str());
  
  robot_serial_port_.Read(joint_states_);

  std::stringstream check1(joint_states_);

  std::string intermediate; 

  unsigned int i = 0; 

  while(std::getline(check1,intermediate,','))
  {
    if(!std::isnan(std::stod(intermediate))){
      hw_states_positions_[i] = std::stod(intermediate);
      i++;
    }
    
  }

  // RCLCPP_INFO(
  //   rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"),
  //   "From Arduino received: %.5f,%.5f,%.5f,%.5f,%.5f,%.5f!",hw_states_positions_[0],hw_states_positions_[1],
  //   hw_states_positions_[2],hw_states_positions_[3],hw_states_positions_[4],hw_states_positions_[5]);


  for (std::size_t i = 0; i < hw_joints_.size(); i++)
  {
    switch (control_level_[i])
    {
      case integration_level_t::UNDEFINED:
        RCLCPP_INFO(
          rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"),
          "Nothing is using the hardware interface!");
        return return_type::OK;
        break;
      case integration_level_t::POSITION:
        hw_joints_[i].state.position = hw_states_positions_[i]; // read position values
        hw_joints_[i].state.velocity = 0;
        hw_joints_[i].state.acceleration = 0;
        break;
      case integration_level_t::VELOCITY:
        hw_joints_[i].state.position = 0;
        hw_joints_[i].state.velocity = 0;   // read velocity values
        hw_joints_[i].state.acceleration = 0;
        break;
      case integration_level_t::ACCELERATION:
        hw_joints_[i].state.velocity = 0;
        hw_joints_[i].state.velocity = 0;
        hw_joints_[i].state.acceleration = 0;  // read acceleration values
        break;
    }
  
  }
  return return_type::OK;
}

return_type INNOBOTSystemMultiInterfaceHardware::write()
{

  hw_joint_positions_ = "[" 
      + std::to_string( (int) round( ( 180 * hw_joints_[0].command.position / M_PI )) )
      + "," 
      + std::to_string( (int) round( ( 180 * hw_joints_[1].command.position / M_PI )) ) 
      + "," 
      + std::to_string( (int) round( ( 180 * hw_joints_[2].command.position / M_PI )) )
      + "," 
      + std::to_string( (int) round( ( 180 * hw_joints_[3].command.position / M_PI )) )
      + "," 
      + std::to_string( (int) round( ( 180 * hw_joints_[4].command.position / M_PI )) )
      + "," 
      + std::to_string( (int) round( ( 180 * hw_joints_[5].command.position / M_PI )) )
      + "]";
  
  robot_serial_port_.Write(hw_joint_positions_.c_str());

  // RCLCPP_INFO(
  //   rclcpp::get_logger("INNOBOTSystemMultiInterfaceHardware"),
  //   "Sent the : %s to Arduino",
  //   hw_joint_positions_.c_str());
  
  hw_joint_positions_.clear();

  return return_type::OK;
}

}  // namespace innobot_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  innobot_control_hardware::INNOBOTSystemMultiInterfaceHardware,
  hardware_interface::SystemInterface)
