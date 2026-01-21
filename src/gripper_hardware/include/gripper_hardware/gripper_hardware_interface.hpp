#ifndef GRIPPER_HARDWARE__GRIPPER_HARDWARE_INTERFACE_HPP_
#define GRIPPER_HARDWARE__GRIPPER_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

// Changed namespace to reflect the new package name
namespace gripper_hardware
{
class GripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GripperHardwareInterface)

  // Initialization: Reads URDF and prepares memory
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // Feedback: Maps internal hw_states_ to ROS 2 controllers
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Goal Commands: Maps internal hw_commands_ to ROS 2 controllers
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Real-time Loop: Reads actual data from physical hardware (Sensors)
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Real-time Loop: Sends goal commands to physical hardware (Actuators)
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Data vectors for Finger_1 and Finger_2
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> hw_velocities_;
};

}  // namespace gripper_hardware

#endif  // GRIPPER_HARDWARE__GRIPPER_HARDWARE_INTERFACE_HPP_