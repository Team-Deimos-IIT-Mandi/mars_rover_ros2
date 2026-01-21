#include "gripper_hardware/gripper_hardware_interface.hpp" // Changed to new package path
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"

namespace gripper_hardware // Changed namespace to match the new package
{

hardware_interface::CallbackReturn GripperHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Allocate memory for the joints (Finger_1, Finger_2)
  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  
  // NOTE: Ensure hw_velocities_ is declared in your .hpp private members
  // If not using velocity, you can remove this and its export below.
  hw_velocities_.resize(info_.joints.size(), 0.0); 

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GripperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    // Position Feedback
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    
    // Velocity Feedback
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GripperHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    // Goal Position Command
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("GripperHardwareInterface"), "Gripper Hardware Activating...");
  // TODO: Add code to open your Serial/USB port here
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("GripperHardwareInterface"), "Gripper Hardware Deactivating...");
  // TODO: Add code to safely close communication here
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GripperHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO: Read current finger positions from physical sensors and update hw_states_
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GripperHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO: Send target positions from hw_commands_ to your physical motors
  return hardware_interface::return_type::OK;
}

} // namespace gripper_hardware

#include "pluginlib/class_list_macros.hpp"
// Critical: Updated to the new namespace
PLUGINLIB_EXPORT_CLASS(gripper_hardware::GripperHardwareInterface, hardware_interface::SystemInterface)