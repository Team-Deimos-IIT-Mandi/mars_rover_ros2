# Foxglove Quick Start Guide

## 🚀 Launch Commands

### 1. Basic Gazebo Simulation
```bash
cd ~/test/mars_rover_ros2
source install/setup.bash
ros2 launch rover_description gazebo_sim.launch.py enable_foxglove:=true
```

### 2. GPS Navigation Mission
```bash
cd ~/test/mars_rover_ros2
source install/setup.bash  
ros2 launch rover_description gps_navigation.launch.py enable_foxglove:=true
```

### 3. Complete System (SLAM + All Sensors)
```bash
cd ~/test/mars_rover_ros2
source install/setup.bash
ros2 launch rover_description complete_system.launch.py \
    enable_foxglove:=true \
    enable_mapping:=true \
    enable_navigation:=false
```

## 📡 Connect to Foxglove Studio

1. **Open Foxglove Studio** (desktop app or https://app.foxglove.dev)
2. Click **"Open Connection"**
3. Select **"Rosbridge (ROS 1 & ROS 2)"**
4. Enter: `ws://localhost:8765`
5. Click **"Open"**

## 🎨 Load Pre-configured Layout

After connecting:
1. **Layout** menu → **"Import from file..."**
2. Navigate to: `install/rover_description/share/rover_description/config/foxglove_layouts/`
3. Choose layout:
   - `gazebo_sim_layout.json` - Basic simulation
   - `gps_navigation_layout.json` - GPS missions
   - `complete_system_layout.json` - Full system

## 📊 What You'll See

### Gazebo Sim Layout
- 3D robot view
- 3 camera feeds
- LiDAR visualization
- Diagnostics

### GPS Navigation Layout
- Map with GPS path
- Waypoint markers
- Navigation goals
- Camera feeds
- GPS coordinates

### Complete System Layout
- SLAM map
- All sensors combined
- Joint states
- Topic graph
- Parameters panel
- Diagnostics

## ✅ Verify Connection

```bash
# Check if Foxglove bridge is running
ros2 node list | grep foxglove

# Expected output: /foxglove_bridge

# Check topics
ros2 topic list

# Test a topic
ros2 topic echo /scan --once
```

## 🛠️ Troubleshooting

**Can't connect?**
- Verify bridge is running: `ros2 node list | grep foxglove`
- Check port: `netstat -tulpn | grep 8765`

**No mesh in Gazebo?**
- This is now fixed! Meshes use absolute file:// paths

**No camera feed?**
- Wait a few seconds for cameras to initialize
- Check: `ros2 topic hz /rgbd_camera/image`

## 📖 Full Documentation

See [FOXGLOVE_INTEGRATION.md](src/rover_description/FOXGLOVE_INTEGRATION.md) for detailed guide.

---

**Ready to visualize! 🦊**
