# Mars Rover Hardware Configuration - Summary of Changes

## Overview
This document summarizes all the changes made to configure the Mars Rover for 4-wheel skid-steer operation with Foxglove web visualization.

## 1. Hardware Interface Fixes

### Fixed Plugin Export (`rover_hardware.xml`)
- **Changed**: Plugin class from `RoverCANInterface` to `RoverSerialInterface`
- **Reason**: The actual implementation uses serial communication, not CAN bus
- **Impact**: Hardware plugin now correctly matches the implementation

### Updated URDF ros2_control Section
- **Changed**: Plugin reference from `rover_hardware/RoverCANInterface` to `rover_hardware/RoverSerialInterface`
- **Changed**: Parameter from `can_interface` to `serial_port` with value `/dev/ttyUSB0`
- **Changed**: Joint names from `front_left_wheel_joint`, etc. to `wheel_1`, `wheel_2`, `wheel_3`, `wheel_4`
- **Reason**: Match actual URDF joint definitions
- **Impact**: ros2_control can now properly interface with the hardware

## 2. Controller Configuration

### Updated to 4-Wheel Skid-Steer Configuration (`controllers.yaml`)
- **Controller**: Using `diff_drive_controller/DiffDriveController` with 4 wheels
- **Left wheels**: `wheel_1` (front-left), `wheel_2` (rear-left)
- **Right wheels**: `wheel_4` (front-right), `wheel_3` (rear-right)
- **Physical parameters**:
  - Wheel separation: 0.59 m (measured from URDF)
  - Wheel radius: 0.11 m (from URDF collision geometry)
- **Added features**:
  - Velocity and acceleration limits
  - Odometry covariance matrices
  - Position feedback enabled

## 3. Serial Port Configuration

### Fixed Port Conflicts (`sensors.yaml`)
| Device | Old Port | New Port | Baud Rate |
|--------|----------|----------|-----------|
| Motor Controller | `/dev/ttyUSB0` | `/dev/ttyUSB0` | 115200 |
| GPS Module | `/dev/ttyUSB0` | `/dev/ttyUSB1` | 9600 |
| LiDAR Sensor | `/dev/ttyUSB1` | `/dev/ttyUSB2` | 115200 |

**Created**: `HARDWARE_PORTS.md` documentation for port assignments

## 4. Foxglove Bridge Integration

### Added to Launch Files
- **Files modified**: 
  - `rover_hardware/launch/bringup.launch.py`
  - `rover_description/launch/hardware.launch.py`

### Foxglove Configuration
- **Port**: 8765
- **Address**: 0.0.0.0 (accessible from any network interface)
- **Features**: All topics published, parameters, services, connection graph
- **Access**: Open Foxglove Studio and connect to `ws://<robot-ip>:8765`

## 5. Files Modified

### rover_hardware package
1. `rover_hardware.xml` - Plugin export fix
2. `config/controllers.yaml` - Skid-steer configuration
3. `launch/bringup.launch.py` - Added Foxglove bridge
4. `HARDWARE_PORTS.md` - New documentation file

### rover_description package
1. `urdf/rover.urdf` - ros2_control section updates
2. `config/sensors.yaml` - Serial port conflict fixes
3. `launch/hardware.launch.py` - Added Foxglove bridge

## 6. How to Use

### Launch Hardware System
```bash
# Option 1: Basic hardware bringup (motors + controllers + Foxglove)
ros2 launch rover_hardware bringup.launch.py

# Option 2: Complete hardware system (all sensors + Foxglove)
ros2 launch rover_description hardware.launch.py use_sim_time:=false
```

### Connect to Foxglove
1. Open Foxglove Studio (web or desktop)
2. Connect to: `ws://<robot-ip>:8765`
3. All topics, camera feeds, and sensor data will be available

### Control the Robot
```bash
# Send velocity commands
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

## 7. Next Steps

- [ ] Test hardware connections on actual robot
- [ ] Verify Foxglove camera feed visualization
- [ ] Calibrate wheel separation and radius if needed
- [ ] Configure IMU driver when hardware is available
- [ ] Set up udev rules for persistent device names
- [ ] Test skid-steer driving behavior

## 8. Important Notes

### Wheel Mapping
The wheel joints in URDF are named `wheel_1` through `wheel_4`:
- `wheel_1`: Front-left
- `wheel_2`: Rear-left  
- `wheel_3`: Rear-right
- `wheel_4`: Front-right

### Skid-Steer Operation
The differential drive controller treats the 4 wheels as two virtual wheels:
- Left virtual wheel = average of wheel_1 and wheel_2
- Right virtual wheel = average of wheel_4 and wheel_3

This provides proper skid-steer behavior for turning.
