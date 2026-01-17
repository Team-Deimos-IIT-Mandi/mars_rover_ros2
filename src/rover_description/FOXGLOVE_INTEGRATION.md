# Foxglove Studio Integration Guide

This guide explains how to use Foxglove Studio for real-time visualization and monitoring of the Mars Rover ROS2 system.

## Overview

Foxglove Studio is a powerful visualization tool for robotics that provides:
- Real-time 3D visualization
- Camera feed viewing
- Map visualization (occupancy grids, GPS paths)
- Diagnostic monitoring
- Parameter configuration
- Topic inspection
- Custom dashboards

## Installation

### 1. Install Foxglove Bridge (ROS2 package)
```bash
sudo apt install ros-humble-foxglove-bridge
```

### 2. Install Foxglove Studio (Desktop App)
Download from: https://foxglove.dev/download

Or use the web version at: https://app.foxglove.dev

## Quick Start

### Step 1: Launch Your Mission

Choose one of the following missions:

#### Option A: Basic Gazebo Simulation
```bash
cd ~/test/mars_rover_ros2
source install/setup.bash
ros2 launch rover_description gazebo_sim.launch.py enable_foxglove:=true
```

#### Option B: GPS Navigation Mission
```bash
cd ~/test/mars_rover_ros2
source install/setup.bash
ros2 launch rover_description gps_navigation.launch.py enable_foxglove:=true
```

#### Option C: Complete System (Mapping + Navigation)
```bash
cd ~/test/mars_rover_ros2
source install/setup.bash
ros2 launch rover_description complete_system.launch.py \
    enable_foxglove:=true \
    enable_mapping:=true \
    enable_navigation:=false
```

### Step 2: Connect Foxglove Studio

1. Open Foxglove Studio
2. Click **"Open Connection"**
3. Select **"Rosbridge (ROS 1 & ROS 2)"**
4. Enter WebSocket URL: `ws://localhost:8765`
5. Click **"Open"**

### Step 3: Load the Appropriate Layout

After connecting, import the pre-configured layout for your mission:

1. Click the **Layout** dropdown (top-right)
2. Select **"Import from file..."**
3. Navigate to: `~/test/mars_rover_ros2/src/rover_description/config/foxglove_layouts/`
4. Choose the appropriate layout:
   - `gazebo_sim_layout.json` - For basic Gazebo simulation
   - `gps_navigation_layout.json` - For GPS/GNSS navigation
   - `complete_system_layout.json` - For full system with mapping

## Available Layouts

### 1. Gazebo Simulation Layout (`gazebo_sim_layout.json`)

**Features:**
- **3D View**: Robot model and sensor visualization
- **Camera Views**: RGBD camera, Camera 1, Camera 2
- **LiDAR View**: 2D laser scan visualization
- **Diagnostics**: System health monitoring
- **TF Tree**: Transform tree visualization
- **Topic Monitor**: Real-time topic inspection

**Best for:** Testing basic robot functionality, sensor testing, teleoperation

---

### 2. GPS Navigation Layout (`gps_navigation_layout.json`)

**Features:**
- **Map Panel**: GPS path visualization with waypoints
  - Displays `/gps/fix` topic on a map
  - Shows planned path and current position
  - Latitude/Longitude overlay
- **Camera Views**: All camera feeds
- **GPS Data Viz**: 
  - Current GPS coordinates
  - Heading/orientation
  - Fix quality and satellite count
- **Nav2 Monitoring**:
  - Goal status
  - Path visualization
  - Velocity commands
- **Diagnostics**: Navigation stack health
- **Plot Panel**: GPS accuracy over time

**Best for:** Waypoint navigation, GPS-based missions, outdoor navigation testing

**Key Topics Visualized:**
- `/gps/fix` - GPS position (sensor_msgs/NavSatFix)
- `/plan` - Planned path
- `/local_costmap/costmap` - Local navigation costmap
- `/cmd_vel` - Velocity commands

---

### 3. Complete System Layout (`complete_system_layout.json`)

**Features:**
- **3D Scene**: Full robot state with TF frames
- **Camera Grid**: All 3 cameras + depth view
- **Mapping Panel**: 
  - Cartographer occupancy grid
  - Real-time SLAM visualization
  - Map updates
- **GPS Overlay**: GPS position on map
- **Navigation Panel**:
  - Path planning visualization
  - Goal markers
  - Costmap layers
- **Multi-sensor Fusion**:
  - LiDAR + Camera + GPS combined view
- **State Inspector**: 
  - Joint states
  - Controller status
  - Transform tree
- **Diagnostics Dashboard**:
  - CPU/Memory usage
  - Topic frequencies
  - Node graph
- **Parameter Panel**: Dynamic reconfiguration

**Best for:** Complex missions, research, development, multi-sensor integration

**Key Topics Visualized:**
- `/map` - Cartographer SLAM map
- `/gps/fix` - GPS position
- `/scan` - LiDAR data
- `/rgbd_camera/image`, `/camera_1/image_raw`, `/camera_2/image_raw` - Camera feeds
- `/tf` - All transforms
- `/diagnostics` - System diagnostics

---

## Custom Configuration

### Creating Your Own Layout

1. Open Foxglove and connect to the robot
2. Add panels from the **Panel** menu (top-left)
3. Configure each panel:
   - Click the settings icon on the panel
   - Select topics to visualize
   - Adjust display settings
4. Save your layout:
   - Click **Layout** → **Export to file...**
   - Save to `config/foxglove_layouts/`

### Useful Panels for Robotics

| Panel Type | Use Case | Topics |
|------------|----------|--------|
| **3D** | Robot visualization, sensor data | `/tf`, `/robot_description`, `/scan`, `/camera/points` |
| **Image** | Camera feeds | `/camera/image_raw`, `/rgbd_camera/image` |
| **Map** | GPS navigation | `/gps/fix`, `/navsat/fix` |
| **Plot** | Time-series data | `/odom`, `/imu`, `/battery_state` |
| **Diagnostics** | System health | `/diagnostics` |
| **Topic Graph** | Data flow | All topics |
| **Parameters** | Configuration | All parameters |
| **Raw Messages** | Debugging | Any topic |

---

## Advanced Features

### 1. Recording and Playback

Foxglove can record data directly:
```bash
# Start recording
ros2 bag record -a
```

Then play back in Foxglove:
1. **File** → **Open local file...**
2. Select `.mcap` or `.bag` file
3. Use timeline controls to play/pause/seek

### 2. GPS Map Configuration

To use GPS visualization with real maps:

1. Add **Map** panel
2. Configure settings:
   - **Topic**: `/gps/fix`
   - **Map Type**: OpenStreetMap
   - **Follow Mode**: Follow (tracks robot)
   - **Layer**: GPS Path + Markers

3. For waypoint visualization:
   - Add **Markers** topic: `/waypoint_markers`
   - Enable path visualization

### 3. Multi-Robot Monitoring

For multiple rovers:
```bash
# Launch with namespace
ros2 launch rover_description complete_system.launch.py \
    namespace:=rover1 \
    enable_foxglove:=true \
    port:=8765

# Second rover
ros2 launch rover_description complete_system.launch.py \
    namespace:=rover2 \
    enable_foxglove:=true \
    port:=8766
```

Connect to both in Foxglove by opening multiple connections.

---

## Troubleshooting

### Cannot Connect to Foxglove Bridge

**Check if bridge is running:**
```bash
ros2 node list | grep foxglove
```

**Expected output:** `/foxglove_bridge`

**If not running, check launch arguments:**
```bash
ros2 launch rover_description gazebo_sim.launch.py enable_foxglove:=true
```

**Check port:**
```bash
netstat -tulpn | grep 8765
```

### No Data in Panels

1. **Verify topics are publishing:**
```bash
ros2 topic list
ros2 topic echo /scan --once
```

2. **Check panel configuration:**
   - Click panel settings (gear icon)
   - Verify correct topic selected
   - Check topic message type matches

3. **Verify time synchronization:**
   - Ensure `use_sim_time` is set correctly
   - In Foxglove, check timestamp display

### Camera Feeds Not Showing

**Check camera topics:**
```bash
ros2 topic hz /rgbd_camera/image
ros2 topic hz /camera_1/image_raw
```

**Increase buffer limit** (if images are large):
Edit `foxglove.launch.py`:
```python
'send_buffer_limit': 20000000,  # 20MB
```

### GPS Map Not Loading

1. **Check GPS data:**
```bash
ros2 topic echo /gps/fix
```

2. **Verify coordinates are valid:**
   - Latitude: -90 to 90
   - Longitude: -180 to 180
   - Status: GPS fix (not 0)

3. **Internet connection required** for map tiles (OpenStreetMap)

---

## Performance Tips

### For Slow Connections / Low Bandwidth:

1. **Reduce image quality:**
   - Panel settings → Compression: JPEG
   - Reduce image size/framerate

2. **Disable unused topics:**
   - Unsubscribe from topics not in use
   - Use topic filters

3. **Use local Foxglove installation** instead of web version

### For High-Performance Visualization:

1. **Enable hardware acceleration** in Foxglove settings
2. **Use dedicated GPU** for 3D rendering
3. **Increase buffer sizes** in foxglove_bridge

---

## Topic Reference

### Key Topics by Mission

| Mission | Critical Topics |
|---------|----------------|
| **Gazebo Sim** | `/scan`, `/rgbd_camera/image`, `/tf`, `/joint_states` |
| **GPS Navigation** | `/gps/fix`, `/plan`, `/cmd_vel`, `/goal_pose`, `/map` |
| **Complete System** | All above + `/diagnostics`, `/cartographer/map`, `/odom` |

### Sensor Topics

```
/scan                           # LiDAR (sensor_msgs/LaserScan)
/rgbd_camera/image              # RGB Camera (sensor_msgs/Image)
/rgbd_camera/depth_image        # Depth (sensor_msgs/Image)
/rgbd_camera/points             # Point Cloud (sensor_msgs/PointCloud2)
/camera_1/image_raw             # Camera 1 (sensor_msgs/Image)
/camera_2/image_raw             # Camera 2 (sensor_msgs/Image)
/imu                            # IMU (sensor_msgs/Imu)
/gps/fix                        # GPS (sensor_msgs/NavSatFix)
```

### Control Topics

```
/cmd_vel                        # Velocity commands (geometry_msgs/Twist)
/diff_drive_controller/cmd_vel  # Controller input (geometry_msgs/TwistStamped)
/joint_states                   # Joint positions (sensor_msgs/JointState)
```

### Navigation Topics

```
/map                            # SLAM map (nav_msgs/OccupancyGrid)
/plan                           # Global path (nav_msgs/Path)
/local_plan                     # Local path (nav_msgs/Path)
/goal_pose                      # Navigation goal (geometry_msgs/PoseStamped)
/amcl_pose                      # Localization (geometry_msgs/PoseWithCovarianceStamped)
```

---

## Additional Resources

- **Foxglove Documentation**: https://docs.foxglove.dev
- **ROS2 Foxglove Bridge**: https://github.com/foxglove/ros-foxglove-bridge
- **Layout Examples**: https://github.com/foxglove/studio/tree/main/layouts

---

## Support

For issues or questions:
1. Check ROS2 node status: `ros2 node list`
2. Check Foxglove bridge logs: `ros2 node info /foxglove_bridge`
3. Verify network connectivity
4. Review this guide's troubleshooting section

---

**Happy Visualizing! 🦊**
