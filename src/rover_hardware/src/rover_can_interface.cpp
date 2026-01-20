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
#include <chrono>

namespace rover_hardware
{

// --- 4-WHEEL BINARY PROTOCOL ---

// 18 Bytes
struct CommandPacket {
    uint8_t header = 0xA5;
    float fl_vel;
    float rl_vel;
    float fr_vel;
    float rr_vel;
    uint8_t terminator = 0x5A;
} __attribute__((packed));

// 34 Bytes
struct FeedbackPacket {
    uint8_t header = 0xA5;
    float fl_pos;
    float fl_vel;
    float rl_pos;
    float rl_vel;
    float fr_pos;
    float fr_vel;
    float rr_pos;
    float rr_vel;
    uint8_t terminator = 0x5A;
} __attribute__((packed));

class RoverSerialInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        // Reserve memory for 4 joints
        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);

        for (const auto & hardware_param : info_.hardware_parameters) {
            if (hardware_param.first == "serial_port") {
                serial_port_name_ = hardware_param.second;
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        serial_fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(rclcpp::get_logger("RoverSerial"), "Failed to open %s: %s", serial_port_name_.c_str(), strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        }

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return hardware_interface::CallbackReturn::ERROR;

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

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return hardware_interface::CallbackReturn::ERROR;

        RCLCPP_INFO(rclcpp::get_logger("RoverSerial"), "4-Wheel Serial Interface Configured on %s", serial_port_name_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        if (serial_fd_ >= 0) close(serial_fd_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        static uint32_t packets_received = 0;
        static uint32_t read_errors = 0;
        static auto last_debug_time = std::chrono::steady_clock::now();
        
        uint8_t buffer[128];
        int n = ::read(serial_fd_, buffer, sizeof(buffer));
        
        if (n < 0) {
            read_errors++;
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("RoverSerial"), *rclcpp::Clock::make_shared(), 1000,
                                  "Serial read error: %s (total errors: %u)", strerror(errno), read_errors);
            return hardware_interface::return_type::OK;
        }
        
        if (n >= (int)sizeof(FeedbackPacket)) {
            // Scan buffer for valid packet
            for (int i = 0; i <= n - (int)sizeof(FeedbackPacket); i++) {
                if (buffer[i] == 0xA5 && buffer[i + sizeof(FeedbackPacket) - 1] == 0x5A) {
                    FeedbackPacket fb;
                    std::memcpy(&fb, &buffer[i], sizeof(fb));
                    
                    packets_received++;

                    // --- DIRECT MAPPING (No Mirroring) ---
                    
                    // Front Left (Index 0)
                    hw_positions_[0] = (double)fb.fl_pos;
                    hw_velocities_[0] = (double)fb.fl_vel;
                    
                    // Rear Left (Index 1)
                    hw_positions_[1] = (double)fb.rl_pos;
                    hw_velocities_[1] = (double)fb.rl_vel;

                    // Front Right (Index 2)
                    hw_positions_[2] = (double)fb.fr_pos;
                    hw_velocities_[2] = (double)fb.fr_vel;

                    // Rear Right (Index 3)
                    hw_positions_[3] = (double)fb.rr_pos;
                    hw_velocities_[3] = (double)fb.rr_vel;
                    
                    // Debug output (throttled to every 2 seconds)
                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_debug_time).count() >= 2) {
                        RCLCPP_INFO(rclcpp::get_logger("RoverSerial"),
                                    "📥 RX Feedback #%u | FL: %.2f rad/s | RL: %.2f rad/s | FR: %.2f rad/s | RR: %.2f rad/s",
                                    packets_received, fb.fl_vel, fb.rl_vel, fb.fr_vel, fb.rr_vel);
                        last_debug_time = now;
                    }
                }
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        static uint32_t packets_sent = 0;
        static uint32_t write_errors = 0;
        static auto last_cmd_debug_time = std::chrono::steady_clock::now();
        
        CommandPacket cmd;
        cmd.header = 0xA5;
        
        // Map ROS commands directly to packet
        cmd.fl_vel = (float)hw_commands_[0]; 
        cmd.rl_vel = (float)hw_commands_[1];
        cmd.fr_vel = (float)hw_commands_[2];
        cmd.rr_vel = (float)hw_commands_[3];
        
        cmd.terminator = 0x5A;

        ssize_t bytes_written = ::write(serial_fd_, &cmd, sizeof(cmd));
        
        if (bytes_written < 0) {
            write_errors++;
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("RoverSerial"), *rclcpp::Clock::make_shared(), 1000,
                                  "Serial write error: %s (total errors: %u)", strerror(errno), write_errors);
            return hardware_interface::return_type::ERROR;
        }
        
        if (bytes_written != sizeof(cmd)) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RoverSerial"), *rclcpp::Clock::make_shared(), 1000,
                                 "Incomplete write: %ld/%lu bytes", bytes_written, sizeof(cmd));
        } else {
            packets_sent++;
        }
        
        // Debug output (throttled to every 500ms to see commands more frequently)
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cmd_debug_time).count() >= 500) {
            RCLCPP_INFO(rclcpp::get_logger("RoverSerial"),
                        "📤 TX Command #%u → FL: %.3f | RL: %.3f | FR: %.3f | RR: %.3f rad/s",
                        packets_sent, cmd.fl_vel, cmd.rl_vel, cmd.fr_vel, cmd.rr_vel);
            last_cmd_debug_time = now;
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
