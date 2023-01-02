#include "innobot_control_hardware/innobot_gripper.hpp"
#include "innobot_control_hardware/serial_port.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


std::string gripper_position; 

namespace innobot_control_hardware
{

// SerialPort serialPort("/dev/ttyACM0",115200);

return_type INNOBOTGripper::configure(
  const hardware_interface::HardwareInfo & info)
{

  if (configure_default(info) != return_type::OK)
  {
    return return_type::ERROR;
  }

  gripper_serial_port_.SetDevice("/dev/ttyACM1");
  gripper_serial_port_.SetBaudRate(115200);
  gripper_serial_port_.SetTimeout(0);

  gripper_joint_.resize(info_.joints.size(), Joint());

  gripper_start_sec_ = stod(info_.hardware_parameters["example_param_gripper_start_duration_sec"]);
  gripper_stop_sec_ = stod(info_.hardware_parameters["example_param_gripper_stop_duration_sec"]);
  gripper_slowdown_ = stod(info_.hardware_parameters["example_param_gripper_slowdown"]);

  gripper_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  gripper_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  gripper_states_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  gripper_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  gripper_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  gripper_commands_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (uint i = 0; i < info_.joints.size(); i++) {
    gripper_joint_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    gripper_joint_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    gripper_joint_[i].state.acceleration = std::numeric_limits<double>::quiet_NaN();
    gripper_joint_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    gripper_joint_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    gripper_joint_[i].command.acceleration = std::numeric_limits<double>::quiet_NaN();
  }


  gripper_control_level_.resize(info_.joints.size(), integration_level_t::POSITION);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // INNOBOTSystemMultiInterface has exactly 3 state interfaces
    // and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("INNOBOTGripper"),
        "Joint '%s' has %d command interfaces. 3 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("INNOBOTGripper"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("INNOBOTGripper"),
        "Joint '%s'has %d state interfaces. 3 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("INNOBOTGripper"),
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
INNOBOTGripper::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_joint_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_joint_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &gripper_joint_[i].state.acceleration));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
INNOBOTGripper::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_joint_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_joint_[i].command.velocity));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION,
      &gripper_joint_[i].command.acceleration));
  }

  return command_interfaces;
}

return_type INNOBOTGripper::start()
{
  gripper_serial_port_.Open();

  RCLCPP_INFO(
    rclcpp::get_logger("INNOBOTGripper"), "Starting... please wait...");

  for (int i = 0; i <= gripper_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("INNOBOTGripper"), "%.1f seconds left...",
      gripper_start_sec_ - i);
  }

  // Set some default values
  for (std::size_t i = 0; i < gripper_joint_.size(); i++)
  {
    if (std::isnan(gripper_joint_[i].state.position))
    {
      gripper_joint_[i].state.position = 0;
    }
    if (std::isnan(gripper_joint_[i].state.velocity))
    {
      gripper_joint_[i].state.velocity = 0;
    }
    if (std::isnan(gripper_joint_[i].state.acceleration))
    {
      gripper_joint_[i].state.acceleration = 0;
    }
    if (std::isnan(gripper_joint_[i].command.position))
    {
      gripper_joint_[i].command.position = 0;
    }
    if (std::isnan(gripper_joint_[i].command.velocity))
    {
      gripper_joint_[i].command.velocity = 0;
    }
    if (std::isnan(gripper_joint_[i].command.acceleration))
    {
      gripper_joint_[i].command.acceleration = 0;
    }
    gripper_control_level_[i] = integration_level_t::POSITION;
  }
  status_ = hardware_interface::status::STARTED;

  
  RCLCPP_INFO(
    rclcpp::get_logger("INNOBOTGripper"), "System successfully started! %u",
    gripper_control_level_[0]);
  return return_type::OK;
}

return_type INNOBOTGripper::stop()
{

    RCLCPP_INFO(
    rclcpp::get_logger("INNOBOTGripper"), "Stopping... please wait...");

  for (int i = 0; i <= gripper_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("INNOBOTGripper"), "%.1f seconds left...",
      gripper_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  gripper_serial_port_.Close();

  RCLCPP_INFO(
    rclcpp::get_logger("INNOBOTGripper"), "System successfully stopped!");

  return return_type::OK;

}

return_type INNOBOTGripper::read()
{
  
  // TODO: read values from gripper


  for (std::size_t i = 0; i < gripper_joint_.size(); i++)
  {
    switch (gripper_control_level_[i])
    {
      case integration_level_t::UNDEFINED:
        RCLCPP_INFO(
          rclcpp::get_logger("INNOBOTGripper"),
          "Nothing is using the hardware interface!");
        return return_type::OK;
        break;
      case integration_level_t::POSITION:
        gripper_joint_[i].state.position = gripper_joint_[i].command.position;
        gripper_joint_[i].state.velocity = 0;
        gripper_joint_[i].state.acceleration = 0;
        break;
      case integration_level_t::VELOCITY:
        gripper_joint_[i].state.position = 0;
        gripper_joint_[i].state.velocity = 0;
        gripper_joint_[i].state.acceleration = 0;
        break;
      case integration_level_t::ACCELERATION:
        gripper_joint_[i].state.position = 0;
        gripper_joint_[i].state.velocity = 0;
        gripper_joint_[i].state.acceleration = 0;
        break;
    }

    
  }
  return return_type::OK;
}

return_type INNOBOTGripper::write()
{

  gripper_joints_positions_ = "[" + std::to_string( gripper_joint_[0].command.position ) + "]";

  // RCLCPP_INFO(rclcpp::get_logger("INNOBOTGripper"), "Sending %.2f", gripper_joint_[0].command.position);

  gripper_serial_port_.Write(gripper_joints_positions_.c_str());

  gripper_joints_positions_.clear();
  
  return return_type::OK;
}

}  // namespace innobot_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  innobot_control_hardware::INNOBOTGripper,
  hardware_interface::SystemInterface)
