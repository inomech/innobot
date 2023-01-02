#ifndef INNOBOT_CONTROL_HARDWARE__SYSTEM_GRIPPER_INTERFACE_HPP_
#define INNOBOT_CONTROL_HARDWARE__SYSTEM_GRIPPER_INTERFACE_HPP_

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
class INNOBOTGripper
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(INNOBOTGripper);

  return_type configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type start() override;

  return_type stop() override;

  return_type read() override;

  return_type write() override;

private:
  // Parameters for the INNOBOT simulation
  double gripper_start_sec_;
  double gripper_stop_sec_;
  double gripper_slowdown_;

  std::vector<Joint> gripper_joint_;

  // Store the commands for the simulated robot
  std::vector<double> gripper_commands_positions_;
  std::vector<double> gripper_commands_velocities_;
  std::vector<double> gripper_commands_accelerations_;
  std::vector<double> gripper_states_positions_;
  std::vector<double> gripper_states_velocities_;
  std::vector<double> gripper_states_accelerations_;

  SerialPort gripper_serial_port_; 
  std::string gripper_joints_positions_; 

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
  std::vector<integration_level_t> gripper_control_level_;
};

}  // namespace innobot_control_hardware
#endif  // INNOBOT_CONTROL_HARDWARE__SYSTEM_GRIPPER_INTERFACE_HPP_