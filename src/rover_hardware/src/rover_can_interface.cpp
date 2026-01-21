#include "rover_hardware/rover_can_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <vector>
#include <cmath>

namespace rover_hardware
{

// --- 4-WHEEL BINARY PROTOCOL ---

// 18 Bytes - Command from ROS2 to ESP32
struct CommandPacket {
    uint8_t header = 0xA5;
    float fl_vel;  // rad/s
    float rl_vel;  // rad/s
    float fr_vel;  // rad/s
    float rr_vel;  // rad/s
    uint8_t terminator = 0x5A;
} __attribute__((packed));

// 34 Bytes - Feedback from ESP32 to ROS2
struct FeedbackPacket {
    uint8_t header = 0xA5;
    float fl_pos;  // radians
    float fl_vel;  // rad/s
    float rl_pos;  // radians
    float rl_vel;  // rad/s
    float fr_pos;  // radians
    float fr_vel;  // rad/s
    float rr_pos;  // radians
    float rr_vel;  // rad/s
    uint8_t terminator = 0x5A;
} __attribute__((packed));

class RoverSerialInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        // Reserve memory for 4 joints: FL, RL, FR, RR
        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);

        for (const auto & hardware_param : info_.hardware_parameters) {
            if (hardware_param.first == "serial_port") {
                serial_port_name_ = hardware_param.second;
            }
        }
        
        RCLCPP_INFO(rclcpp::get_logger("RoverSerial"), 
                    "Initialized for %zu joints", info_.joints.size());
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        serial_fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(rclcpp::get_logger("RoverSerial"), 
                        "Failed to open %s: %s", 
                        serial_port_name_.c_str(), strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        }

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("RoverSerial"), "tcgetattr failed");
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; 
        tty.c_iflag &= ~IGNBRK;                     
        tty.c_lflag = 0;                            
        tty.c_oflag = 0;                            
        tty.c_cc[VMIN]  = 0;                        
        tty.c_cc[VTIME] = 1;                        

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);     
        tty.c_cflag |= (CLOCAL | CREAD);            
        tty.c_cflag &= ~(PARENB | PARODD);          
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("RoverSerial"), "tcsetattr failed");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("RoverSerial"), 
                    "Serial interface configured on %s @ 115200 baud", 
                    serial_port_name_.c_str());
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
        RCLCPP_INFO(rclcpp::get_logger("RoverSerial"), "Hardware interface activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("RoverSerial"), "Hardware interface deactivated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.joints[i].name, 
                    hardware_interface::HW_IF_POSITION, 
                    &hw_positions_[i]));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.joints[i].name, 
                    hardware_interface::HW_IF_VELOCITY, 
                    &hw_velocities_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, 
                    hardware_interface::HW_IF_VELOCITY, 
                    &hw_commands_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        uint8_t buffer[512]; // Larger buffer to handle multiple packets
        int n = ::read(serial_fd_, buffer, sizeof(buffer));
        
        if (n >= (int)sizeof(FeedbackPacket)) {
            // Find the MOST RECENT valid packet (scan from end)
            for (int i = n - sizeof(FeedbackPacket); i >= 0; i--) {
                if (buffer[i] == 0xA5 && buffer[i + sizeof(FeedbackPacket) - 1] == 0x5A) {
                    FeedbackPacket fb;
                    std::memcpy(&fb, &buffer[i], sizeof(fb));

                    // Map to ROS2 joint order: FL, RL, FR, RR
                    hw_positions_[0] = (double)fb.fl_pos;
                    hw_velocities_[0] = (double)fb.fl_vel;
                    
                    hw_positions_[1] = (double)fb.rl_pos;
                    hw_velocities_[1] = (double)fb.rl_vel;

                    hw_positions_[2] = (double)fb.fr_pos;
                    hw_velocities_[2] = (double)fb.fr_vel;

                    hw_positions_[3] = (double)fb.rr_pos;
                    hw_velocities_[3] = (double)fb.rr_vel;
                    
                    break; // Found most recent packet
                }
            }
        }
        
        // Drain any remaining data to prevent buffer buildup
        tcflush(serial_fd_, TCIFLUSH);
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        CommandPacket cmd;
        cmd.header = 0xA5;
        
        // ROS2 sends rad/s commands, ESP32 expects rad/s (±26.18 max)
        // Direct pass-through (no scaling needed)
        cmd.fl_vel = (float)hw_commands_[0]; 
        cmd.rl_vel = (float)hw_commands_[1];
        cmd.fr_vel = (float)hw_commands_[2];
        cmd.rr_vel = (float)hw_commands_[3];
        
        cmd.terminator = 0x5A;

        ssize_t bytes_written = ::write(serial_fd_, &cmd, sizeof(cmd));
        
        if (bytes_written != sizeof(cmd)) {
            RCLCPP_WARN_THROTTLE(
                rclcpp::get_logger("RoverSerial"),
                *rclcpp::Clock::make_shared(),
                1000, // Log once per second
                "Write incomplete: %ld/%zu bytes", bytes_written, sizeof(cmd));
        }
        
        return hardware_interface::return_type::OK;
    }

private:
    std::string serial_port_name_ = "/dev/ttyACM0";
    int serial_fd_ = -1;
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
};

} // namespace rover_hardware

PLUGINLIB_EXPORT_CLASS(
  rover_hardware::RoverSerialInterface,
  hardware_interface::SystemInterface
)
