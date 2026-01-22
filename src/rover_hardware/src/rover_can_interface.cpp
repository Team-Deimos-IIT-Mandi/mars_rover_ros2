#include "rover_hardware/rover_can_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <algorithm>

namespace rover_hardware
{

hardware_interface::CallbackReturn RoverCANInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize vectors for 4 wheels (FL, RL, FR, RR)
  hw_positions_.resize(4, 0.0);
  hw_velocities_.resize(4, 0.0);
  hw_commands_.resize(4, 0.0);

  // Get UART port from URDF/YAML
  uart_port_ = info_.hardware_parameters["uart_port"];
  
  // Get baudrate (default 115200)
  try {
    uart_baudrate_ = std::stoi(info_.hardware_parameters["uart_baudrate"]);
  } catch (...) {
    uart_baudrate_ = 115200;
  }
  
  uart_fd_ = -1;
  packets_sent_ = 0;
  packets_received_ = 0;
  packet_errors_ = 0;
  
  RCLCPP_INFO(
    rclcpp::get_logger("RoverCANInterface"), 
    "Initialized with UART port: %s @ %d baud", 
    uart_port_.c_str(), 
    uart_baudrate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverCANInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoverCANInterface"), "Configuring UART...");
  
  if (!openUART()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RoverCANInterface"), 
      "Failed to open UART port: %s - %s", 
      uart_port_.c_str(), 
      strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!configureUART()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RoverCANInterface"), 
      "Failed to configure UART port");
    closeUART();
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RoverCANInterface"), "UART configured successfully");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
RoverCANInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  std::vector<std::string> wheel_names = {
    "front_left_wheel_joint",
    "rear_left_wheel_joint", 
    "front_right_wheel_joint",
    "rear_right_wheel_joint"
  };
  
  for (size_t i = 0; i < wheel_names.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        wheel_names[i], 
        hardware_interface::HW_IF_POSITION, 
        &hw_positions_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        wheel_names[i], 
        hardware_interface::HW_IF_VELOCITY, 
        &hw_velocities_[i]));
  }
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
RoverCANInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  std::vector<std::string> wheel_names = {
    "front_left_wheel_joint",
    "rear_left_wheel_joint",
    "front_right_wheel_joint", 
    "rear_right_wheel_joint"
  };
  
  for (size_t i = 0; i < wheel_names.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        wheel_names[i], 
        hardware_interface::HW_IF_VELOCITY, 
        &hw_commands_[i]));
  }
  
  return command_interfaces;
}

hardware_interface::CallbackReturn RoverCANInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoverCANInterface"), "Activating...");
  
  // Send initial zero command
  CommandPacket cmd;
  cmd.fl_vel = 0.0f;
  cmd.rl_vel = 0.0f;
  cmd.fr_vel = 0.0f;
  cmd.rr_vel = 0.0f;
  
  sendCommandPacket(cmd);
  
  // Clear command buffer
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = 0.0;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RoverCANInterface"), "Activated - ready to receive commands");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverCANInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoverCANInterface"), "Deactivating...");
  
  // Send stop command
  CommandPacket cmd;
  cmd.fl_vel = 0.0f;
  cmd.rl_vel = 0.0f;
  cmd.fr_vel = 0.0f;
  cmd.rr_vel = 0.0f;
  
  sendCommandPacket(cmd);
  
  RCLCPP_INFO(
    rclcpp::get_logger("RoverCANInterface"), 
    "Deactivated - Stats: Sent=%lu, Received=%lu, Errors=%lu",
    packets_sent_, packets_received_, packet_errors_);
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverCANInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoverCANInterface"), "Cleaning up...");
  
  closeUART();
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverCANInterface::read(
    const rclcpp::Time & /*time*/, 
    const rclcpp::Duration & /*period*/)
{
  FeedbackPacket feedback;
  
  if (readFeedbackPacket(feedback)) {
    // Update state interfaces
    hw_positions_[0] = feedback.fl_pos;
    hw_velocities_[0] = feedback.fl_vel;
    
    hw_positions_[1] = feedback.rl_pos;
    hw_velocities_[1] = feedback.rl_vel;
    
    hw_positions_[2] = feedback.fr_pos;
    hw_velocities_[2] = feedback.fr_vel;
    
    hw_positions_[3] = feedback.rr_pos;
    hw_velocities_[3] = feedback.rr_vel;
    
    packets_received_++;
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverCANInterface::write(
    const rclcpp::Time & /*time*/, 
    const rclcpp::Duration & /*period*/)
{
  // Prepare command packet
  CommandPacket cmd;
  cmd.fl_vel = static_cast<float>(hw_commands_[0]);
  cmd.rl_vel = static_cast<float>(hw_commands_[1]);
  cmd.fr_vel = static_cast<float>(hw_commands_[2]);
  cmd.rr_vel = static_cast<float>(hw_commands_[3]);
  
  if (sendCommandPacket(cmd)) {
    packets_sent_++;
    return hardware_interface::return_type::OK;
  } else {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("RoverCANInterface"),
      *rclcpp::Clock::make_shared(),
      1000,
      "Failed to send command packet");
    return hardware_interface::return_type::ERROR;
  }
}

// ===== UART HELPER FUNCTIONS =====

bool RoverCANInterface::openUART()
{
  uart_fd_ = open(uart_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  
  if (uart_fd_ < 0) {
    return false;
  }
  
  // Set non-blocking mode
  int flags = fcntl(uart_fd_, F_GETFL, 0);
  fcntl(uart_fd_, F_SETFL, flags | O_NONBLOCK);
  
  return true;
}

bool RoverCANInterface::configureUART()
{
  if (uart_fd_ < 0) return false;
  
  struct termios options;
  
  if (tcgetattr(uart_fd_, &options) != 0) {
    return false;
  }
  
  // Set baud rate
  speed_t baud;
  switch (uart_baudrate_) {
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    case 921600: baud = B921600; break;
    default:     baud = B115200; break;
  }
  
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);
  
  // 8N1 mode
  options.c_cflag &= ~PARENB;  // No parity
  options.c_cflag &= ~CSTOPB;  // 1 stop bit
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;      // 8 data bits
  
  // Enable receiver, ignore modem control lines
  options.c_cflag |= (CLOCAL | CREAD);
  
  // Raw mode
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_oflag &= ~OPOST;
  
  // Non-blocking reads with small timeout
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 1;  // 0.1 second timeout
  
  if (tcsetattr(uart_fd_, TCSANOW, &options) != 0) {
    return false;
  }
  
  // Flush any existing data
  tcflush(uart_fd_, TCIOFLUSH);
  
  return true;
}

void RoverCANInterface::closeUART()
{
  if (uart_fd_ >= 0) {
    close(uart_fd_);
    uart_fd_ = -1;
  }
}

bool RoverCANInterface::readFeedbackPacket(FeedbackPacket& packet)
{
  if (uart_fd_ < 0) return false;
  
  // Read available data
  uint8_t temp_buf[256];
  ssize_t bytes_read = ::read(uart_fd_, temp_buf, sizeof(temp_buf));
  
  if (bytes_read > 0) {
    rx_buffer_.insert(rx_buffer_.end(), temp_buf, temp_buf + bytes_read);
  }
  
  // Look for valid packet
  while (rx_buffer_.size() >= sizeof(FeedbackPacket)) {
    // Find header
    auto it = std::find(rx_buffer_.begin(), rx_buffer_.end(), 0xA5);
    
    if (it == rx_buffer_.end()) {
      // No header found, clear buffer
      rx_buffer_.clear();
      break;
    }
    
    // Remove bytes before header
    rx_buffer_.erase(rx_buffer_.begin(), it);
    
    // Check if we have a complete packet
    if (rx_buffer_.size() >= sizeof(FeedbackPacket)) {
      // Check terminator
      if (rx_buffer_[sizeof(FeedbackPacket) - 1] == 0x5A) {
        // Valid packet found
        std::memcpy(&packet, rx_buffer_.data(), sizeof(FeedbackPacket));
        
        // Remove processed packet
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sizeof(FeedbackPacket));
        
        return true;
      } else {
        // Invalid terminator, remove header and continue searching
        rx_buffer_.erase(rx_buffer_.begin());
        packet_errors_++;
      }
    } else {
      // Not enough data yet
      break;
    }
  }
  
  // Prevent buffer from growing too large
  if (rx_buffer_.size() > 1024) {
    rx_buffer_.erase(
      rx_buffer_.begin(), 
      rx_buffer_.end() - sizeof(FeedbackPacket));
    packet_errors_++;
  }
  
  return false;
}

bool RoverCANInterface::sendCommandPacket(const CommandPacket& packet)
{
  if (uart_fd_ < 0) return false;
  
  ssize_t bytes_written = ::write(uart_fd_, &packet, sizeof(CommandPacket));
  
  if (bytes_written != sizeof(CommandPacket)) {
    packet_errors_++;
    return false;
  }
  
  return true;
}

}  // namespace rover_hardware

PLUGINLIB_EXPORT_CLASS(
  rover_hardware::RoverCANInterface,
  hardware_interface::SystemInterface)
