# Foxglove Integration Summary

## Completed Integration

✅ Foxglove Studio has been successfully integrated into the Mars Rover ROS2 project!

## What Was Implemented

### 1. **Foxglove Bridge Integration**
- Added `foxglove_bridge` dependency to [package.xml](src/rover_description/package.xml)
- Created [foxglove.launch.py](src/rover_description/launch/foxglove.launch.py) with layout support
- Configured WebSocket server on port 8765
- Set high buffer limit (10MB) for camera streams

### 2. **Pre-configured Layouts**
Created three mission-specific layouts in `config/foxglove_layouts/`:

#### 📊 **gazebo_sim_layout.json**
- 3D robot visualization
- All camera feeds (RGBD, Camera 1, Camera 2)
- LiDAR scan visualization
- Diagnostics monitoring
- TF tree display

#### 🗺️ **gps_navigation_layout.json**
- GPS map visualization with waypoints
- Navigation path planning
- GPS coordinates display
- Camera feeds
- Nav2 monitoring panels

#### 🚀 **complete_system_layout.json**
- Full SLAM map (Cartographer)
- Multi-sensor fusion (LiDAR + Camera + GPS)
- Joint states monitor
- Parameter configuration panel
- Topic graph
- Diagnostics dashboard
- Velocity gauge

### 3. **Launch File Integration**
All launch files now support Foxglove with appropriate layouts:

```bash
# Basic Gazebo Simulation
ros2 launch rover_description gazebo_sim.launch.py enable_foxglove:=true

# GPS Navigation Mission
ros2 launch rover_description gps_navigation.launch.py enable_foxglove:=true

# Complete System (SLAM + Navigation)
ros2 launch rover_description complete_system.launch.py \
    enable_foxglove:=true \
    enable_mapping:=true
```

### 4. **Mesh Path Resolution**
- Fixed Gazebo mesh loading issues
- Set `GZ_SIM_RESOURCE_PATH` environment variable
- Updated URDF path resolution for model:// URIs

### 5. **Documentation**
Created comprehensive guides:
- **[FOXGLOVE_INTEGRATION.md](src/rover_description/FOXGLOVE_INTEGRATION.md)** - Full integration guide
- **[FOXGLOVE_QUICKSTART.md](src/rover_description/FOXGLOVE_QUICKSTART.md)** - Quick reference

## How to Use

### Quick Start
1. **Launch a mission:**
   ```bash
   source install/setup.bash
   ros2 launch rover_description gazebo_sim.launch.py
   ```

2. **Connect Foxglove Studio:**
   - Open Foxglove Studio
   - Connect to `ws://localhost:8765`

3. **Load the layout:**
   - Layout → Import from file
   - Select layout from: `install/rover_description/share/rover_description/config/foxglove_layouts/`

## Available Features

### Visualization
- ✅ Real-time 3D robot visualization
- ✅ Multiple camera feeds
- ✅ LiDAR point cloud
- ✅ GPS map with waypoints
- ✅ SLAM map visualization
- ✅ Navigation path planning

### Monitoring
- ✅ Topic monitoring
- ✅ Diagnostics dashboard
- ✅ TF tree visualization
- ✅ Joint states
- ✅ Parameter inspection
- ✅ Node graph

### Control
- ✅ Goal pose publishing
- ✅ Parameter configuration
- ✅ Service calls
- ✅ Dynamic reconfiguration

## Key Plugins Enabled

All Foxglove layouts include:
1. **3D Panel** - Robot and sensor visualization
2. **Image Panel** - Camera feeds
3. **Map Panel** - GPS navigation (GPS layouts)
4. **Plot Panel** - Time-series data
5. **Raw Messages** - Topic inspection
6. **Diagnostics** - System health
7. **State Transitions** - TF tree
8. **Topic Graph** - Data flow
9. **Parameters** - Configuration
10. **Gauge** - Velocity/metrics

## Topics Visualized

### Sensors
- `/scan` - LiDAR
- `/rgbd_camera/image` - RGB camera
- `/rgbd_camera/depth_image` - Depth
- `/rgbd_camera/points` - Point cloud
- `/camera_1/image_raw`, `/camera_2/image_raw` - Additional cameras
- `/imu` - IMU data
- `/gps/fix` - GPS position

### Navigation
- `/map` - SLAM map
- `/plan` - Global path
- `/local_plan` - Local path
- `/goal_pose` - Navigation goal
- `/cmd_vel` - Velocity commands

### State
- `/tf`, `/tf_static` - Transforms
- `/joint_states` - Joint positions
- `/diagnostics` - System diagnostics
- `/odom` - Odometry

## Next Steps

1. **Test the integration:**
   ```bash
   source install/setup.bash
   ros2 launch rover_description gazebo_sim.launch.py enable_foxglove:=true
   ```

2. **Open Foxglove Studio** and connect to `ws://localhost:8765`

3. **Import a layout** from `config/foxglove_layouts/`

4. **Customize layouts** as needed for your specific missions

5. **For GPS missions:**
   ```bash
   ros2 launch rover_description gps_navigation.launch.py enable_foxglove:=true
   ```

6. **For full system testing:**
   ```bash
   ros2 launch rover_description complete_system.launch.py \
       enable_foxglove:=true \
       enable_mapping:=true \
       enable_navigation:=false
   ```

## Troubleshooting

If you encounter mesh loading errors, ensure:
- Package is built: `colcon build --packages-select rover_description`
- Paths are sourced: `source install/setup.bash`
- Gazebo resource path is set (handled automatically by launch file)

For detailed troubleshooting, see [FOXGLOVE_INTEGRATION.md](src/rover_description/FOXGLOVE_INTEGRATION.md#troubleshooting)

---

**Integration Complete! 🎉**

All missions now have full Foxglove Studio visualization support with pre-configured layouts optimized for each task.
