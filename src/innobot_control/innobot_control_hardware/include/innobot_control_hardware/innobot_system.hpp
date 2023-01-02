#ifndef INNOBOT_CONTROL_HARDWARE__SYSTEM_MULTI_INTERFACE_HPP_
#define INNOBOT_CONTROL_HARDWARE__SYSTEM_MULTI_INTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "innobot_control_hardware/visibility_control_system.h"

#include <rclcpp_lifecycle/state.hpp> 
#include "rclcpp/macros.hpp"

#include "innobot_control_hardware/serial_port.hpp"

using hardware_interface::return_type;

struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
};

namespace innobot_control_hardware
{
class INNOBOTSystemMultiInterfaceHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
// : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(INNOBOTSystemMultiInterfaceHardware);

  INNOBOT_CONTROL_HARDWARE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  INNOBOT_CONTROL_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  INNOBOT_CONTROL_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  INNOBOT_CONTROL_HARDWARE_PUBLIC
  return_type start() override;

  INNOBOT_CONTROL_HARDWARE_PUBLIC
  return_type stop() override;

  INNOBOT_CONTROL_HARDWARE_PUBLIC
  return_type read() override;

  INNOBOT_CONTROL_HARDWARE_PUBLIC
  return_type write() override;

  

private:
  // Parameters for the INNOBOT simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  std::vector<Joint> hw_joints_;

  char cstr_[32];
  std::string joint_states_;
  std::string arduino_cmd_msg_ = "{0}";

  std::vector<double> tokens_; 

  // Store the commands for the simulated robot
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_accelerations_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_accelerations_;
  SerialPort robot_serial_port_;
  std::string hw_joint_positions_;
  std::vector<double> gear_ratios_step_ = {10.32 * 16, -5.48 * 16, -4.3 * 5 * 2, -1 * 16, -4.5 * 16, 1 * 16};
  std::vector<double> gear_ratios_angle_ = {10.32, -5.48, -4.3 * 5, -1 , -4.5, 1};




  // Enum defining at which control level we are
  // Dumb way of maintaining the command_interface type per joint.
  enum class integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
    ACCELERATION = 3
  };

  // Active control mode for each actuator
  std::vector<integration_level_t> control_level_;
};

}  // namespace innobot_control_hardware
#endif  // INNOBOT_CONTROL_HARDWARE__SYSTEM_MULTI_INTERFACE_HPP_