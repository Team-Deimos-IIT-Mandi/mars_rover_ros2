# Foxglove Quick Start Guide

## 1. Install Foxglove Bridge
```bash
sudo apt install ros-humble-foxglove-bridge
```

## 2. Install Foxglove Studio
Download from https://foxglove.dev/download

## 3. Launch a Mission

### Option A: Basic Simulation
```bash
source install/setup.bash
ros2 launch rover_description gazebo_sim.launch.py
```

### Option B: GPS Navigation
```bash
source install/setup.bash
ros2 launch rover_description gps_navigation.launch.py
```

### Option C: Complete System (SLAM + Navigation)
```bash
source install/setup.bash
ros2 launch rover_description complete_system.launch.py \
    enable_mapping:=true \
    enable_navigation:=false
```

## 4. Connect Foxglove

1. Open Foxglove Studio
2. Click "Open Connection"
3. Select "Rosbridge (ROS 1 & ROS 2)"
4. URL: `ws://localhost:8765`
5. Click "Open"

## 5. Load Layout

1. Click **Layout** → **Import from file...**
2. Navigate to: `install/rover_description/share/rover_description/config/foxglove_layouts/`
3. Select:
   - `gazebo_sim_layout.json` - For basic simulation
   - `gps_navigation_layout.json` - For GPS/waypoint navigation
   - `complete_system_layout.json` - For full SLAM + mapping

## Key Features by Layout

### Gazebo Sim Layout
- 3D robot visualization
- Camera feeds (RGBD, Camera 1, Camera 2)
- LiDAR scan visualization
- Diagnostics monitoring
- TF tree

### GPS Navigation Layout
- Map panel with GPS path visualization
- Waypoint markers
- GPS coordinates display
- Nav2 goal tracking
- Path planning visualization

### Complete System Layout
- Everything from above plus:
- Cartographer SLAM map
- Multi-sensor fusion view
- Joint states monitor
- Parameter configuration
- Topic graph
- Velocity gauge

## Troubleshooting

**Can't connect?**
```bash
# Check if bridge is running
ros2 node list | grep foxglove

# Check port
netstat -tulpn | grep 8765
```

**No camera image?**
```bash
# Verify camera topics are publishing
ros2 topic hz /rgbd_camera/image
```

**GPS map not loading?**
- Ensure internet connection (for map tiles)
- Verify GPS coordinates: `ros2 topic echo /gps/fix`

## Full Documentation
See [FOXGLOVE_INTEGRATION.md](FOXGLOVE_INTEGRATION.md) for detailed guide.
