#ifndef ROVER_HARDWARE__ROVER_CAN_INTERFACE_HPP_
#define ROVER_HARDWARE__ROVER_CAN_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rover_hardware
{

// ===== BINARY PROTOCOL STRUCTURES =====
#pragma pack(push, 1)
struct CommandPacket {
    uint8_t header = 0xA5;
    float fl_vel;  // rad/s
    float rl_vel;  // rad/s
    float fr_vel;  // rad/s
    float rr_vel;  // rad/s
    uint8_t terminator = 0x5A;
};

struct FeedbackPacket {
    uint8_t header = 0xA5;
    float fl_pos; float fl_vel;
    float rl_pos; float rl_vel;
    float fr_pos; float fr_vel;
    float rr_pos; float rr_vel;
    uint8_t terminator = 0x5A;
};
#pragma pack(pop)

class RoverCANInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoverCANInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // UART communication
  int uart_fd_;
  std::string uart_port_;
  int uart_baudrate_;
  
  // Wheel state storage
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  
  // UART receive buffer
  std::vector<uint8_t> rx_buffer_;
  
  // Communication stats
  unsigned long packets_sent_;
  unsigned long packets_received_;
  unsigned long packet_errors_;
  
  // Helper functions
  bool openUART();
  bool configureUART();
  void closeUART();
  bool readFeedbackPacket(FeedbackPacket& packet);
  bool sendCommandPacket(const CommandPacket& packet);
};

}  // namespace rover_hardware

#endif  // ROVER_HARDWARE__ROVER_CAN_INTERFACE_HPP_
