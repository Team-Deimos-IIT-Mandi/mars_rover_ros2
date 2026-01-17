# Hardware Setup Guide for Mars Rover

This guide will help you set up the rover hardware on Jetson Nano with all sensors and controllers.

## Hardware Requirements

### Motor Controllers
- ESP32-based CAN motor controllers
- CAN interface (MCP2515 or similar)
- CAN bus connection to Jetson Nano

### Sensors
- **3x USB Cameras** (including 1 depth camera)
  - Front depth camera (RGBD)
  - Left side camera
  - Right side camera
- **IMU** (MPU6050, BNO055, or similar)
- **GPS Module** (NMEA-compatible)
- **LiDAR** (RPLidar, YDLidar, or similar)

## Installation Steps

### 1. Install Required ROS2 Packages

```bash
# Camera drivers
sudo apt install ros-humble-usb-cam

# For Intel RealSense depth camera (if using D435i/D455):
# sudo apt install ros-humble-realsense2-camera

# GPS driver
sudo apt install ros-humble-nmea-navsat-driver

# LiDAR driver (choose based on your hardware)
# For RPLidar:
sudo apt install ros-humble-rplidar-ros

# For YDLidar (build from source):
# cd ~/ros2_ws/src
# git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
# cd ~/ros2_ws && colcon build

# IMU driver (install based on your IMU model)
# Example for MPU6050:
# sudo apt install ros-humble-mpu6050-driver
```

### 2. Setup CAN Interface

```bash
# Install can-utils
sudo apt install can-utils

# Bring up CAN interface (add to /etc/network/interfaces for persistence)
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Test CAN interface
candump can0
```

### 3. Configure USB Camera Devices

Check which video devices your cameras are assigned to:

```bash
# List all video devices
ls -l /dev/video*

# Check camera info
v4l2-ctl --list-devices
```

Update `/src/rover_description/config/sensors.yaml` with the correct video device paths.

### 4. Configure Serial Devices

For GPS and LiDAR, identify the correct serial ports:

```bash
# List USB serial devices
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*

# Check device info
udevadm info --name=/dev/ttyUSB0 --attribute-walk
```

Update sensor configuration with correct ports.

### 5. Set Permissions

```bash
# Add user to dialout group (for serial devices)
sudo usermod -a -G dialout $USER

# Add user to video group (for cameras)
sudo usermod -a -G video $USER

# Log out and log back in for changes to take effect
```

## Running the Hardware System

### Basic Launch

```bash
# Source the workspace
cd ~/mars_rover_ros2
source install/setup.bash

# Launch hardware system
ros2 launch rover_description hardware.launch.py
```

### Testing Individual Components

```bash
# Test CAN interface
candump can0

# Test cameras
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0

# Test GPS
ros2 topic echo /gps/fix

# Test LiDAR
ros2 topic echo /scan

# Test IMU
ros2 topic echo /imu/data

# Test motor control
ros2 topic pub /diff_drive_controller/cmd_vel_stamped geometry_msgs/msg/TwistStamped "{header: {frame_id: ''}, twist: {linear: {x: 0.1}}}"
```

### Monitor Robot State

```bash
# View joint states
ros2 topic echo /joint_states

# View odometry
ros2 topic echo /diff_drive_controller/odom

# View TF tree
ros2 run tf2_tools view_frames
```

## Troubleshooting

### CAN Interface Issues
- Check if can0 interface is up: `ip link show can0`
- Verify bitrate matches ESP32 controllers
- Check CAN bus wiring and termination resistors

### Camera Not Detected
- Run `v4l2-ctl --list-devices` to find correct device
- Try different USB ports
- Check USB bandwidth limitations on Jetson Nano

### Permission Denied Errors
-Ensure user is in correct groups: `groups $USER`
- Check device permissions: `ls -l /dev/video* /dev/ttyUSB*`

### Controller Manager Fails to Load
- Check URDF syntax: `check_urdf src/rover_description/urdf/rover.urdf`
- Verify rover_hardware plugin is built: `ros2 pkg list | grep rover_hardware`
- Check hardware interface loads: `ros2 control list_hardware_interfaces`

## Configuration Files

Key configuration files to adjust:

- **URDF**: `src/rover_description/urdf/rover.urdf`
- **Controllers**: `src/rover_description/config/controllers.yaml`  
- **Sensors**: `src/rover_description/config/sensors.yaml`
- **Hardware Launch**: `src/rover_description/launch/hardware.launch.py`

## Performance Optimization on Jetson Nano

```bash
# Set Jetson to maximum performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Monitor resource usage
sudo tegrastats
```

## Next Steps

1. Calibrate cameras using `camera_calibration` package
2. Configure navigation parameters
3. Test autonomous navigation with GPS waypoints
4. Tune PID parameters for motor controllers
