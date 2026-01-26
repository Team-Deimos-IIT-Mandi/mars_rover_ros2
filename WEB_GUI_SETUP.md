# Mars Rover Web GUI - Complete Setup Guide

This guide covers everything needed to set up the Mars Rover Web GUI on a fresh Ubuntu 22.04 machine.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble installed
- Git installed

## 1. System Dependencies

### Install ROS 2 Packages
```bash
sudo apt update
sudo apt install -y \
  ros-humble-rosbridge-server \
  ros-humble-web-video-server \
  ros-humble-image-transport-plugins \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-robot-localization \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2
```

### Install VNC and noVNC for RViz Web Streaming
```bash
sudo apt install -y \
  xvfb \
  x11vnc \
  novnc \
  websockify \
  python3-websockify
```

### Install Node.js and npm
```bash
# Install Node.js 22.x
curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash -
sudo apt install -y nodejs

# Verify installation
node --version  # Should be v22.x
npm --version   # Should be 10.x
```

## 2. Clone and Build ROS Workspace

```bash
# Clone the repository
cd ~
git clone https://github.com/Team-Deimos-IIT-Mandi/mars_rover_ros2.git
cd mars_rover_ros2

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## 3. Set Up Web GUI

### Install Web GUI Dependencies
```bash
cd ~/mars_rover_ros2/my-app

# Install npm packages
npm install

# Build the web application (optional, for production)
# npm run build
```

## 4. Make Scripts Executable

```bash
cd ~/mars_rover_ros2
chmod +x src/start_ros_bridge.sh
chmod +x src/start_rviz_vnc.sh
```

## 5. Running the System

You need **3 terminals** to run the complete system:

### Terminal 1: ROS Bridge (Required)
```bash
cd ~/mars_rover_ros2
./src/start_ros_bridge.sh
```
This starts:
- rosbridge_websocket (port 9090)
- web_video_server (port 8080)
- cmd_vel relay

### Terminal 2: Web GUI (Required)
```bash
cd ~/mars_rover_ros2/my-app
npm run dev
```
Access at: `http://localhost:3000`

### Terminal 3: RViz VNC Server (Optional - for Navigation View)
```bash
cd ~/mars_rover_ros2
./src/start_rviz_vnc.sh
```
This enables RViz visualization in the browser's Navigation View tab.

## 6. Launching Missions

Once the web GUI is running:

1. Open browser to `http://localhost:3000`
2. Go to **Launch Files** tab
3. Click on a mission (e.g., "ArUco Mission - Gazebo")
4. Fill in parameters if needed
5. Click **Launch**
6. Switch to **Control Center** tab to see cameras and controls
7. Switch to **Navigation View** tab to see RViz (if VNC server is running)

## 7. Features

### Control Center Tab
- **Camera Streams**: 3 live camera feeds from Gazebo
- **GPS Map**: Real-time rover position on OpenStreetMap
- **Teleop Control**: Joystick for manual rover control
- **System Status**: ROS connection and process monitoring

### Navigation View Tab
- **Full RViz Interface**: Streamed via VNC
- Shows: paths, costmaps, laser scans, robot model, waypoints
- Fully interactive (zoom, pan, rotate)

### Launch Files Tab
- **One-click launch** for all missions
- **Process monitoring**: See running processes and their status
- **Log viewing**: Real-time logs for each process

### Mission Tab
- **GPS waypoint setting**: Click map to set navigation goals
- **Mission control**: Start/stop autonomous navigation

## 8. Troubleshooting

### Cameras showing black screens
1. Make sure `web_video_server` is running (check Terminal 1)
2. Verify Gazebo simulation is running and publishing images:
   ```bash
   ros2 topic hz /rgbd_camera/image
   ```
3. Restart the ROS bridge: Ctrl+C in Terminal 1, then `./src/start_ros_bridge.sh`

### RViz not showing in Navigation View
1. Make sure Terminal 3 (RViz VNC) is running
2. Check that port 6080 is accessible:
   ```bash
   curl http://localhost:6080/vnc.html
   ```
3. Restart the VNC server: Ctrl+C in Terminal 3, then `./src/start_rviz_vnc.sh`

### Web GUI not connecting to ROS
1. Check rosbridge is running on port 9090:
   ```bash
   netstat -tuln | grep 9090
   ```
2. Look for "Connected to websocket server" in browser console (F12)
3. Restart the ROS bridge

### Process launch fails
1. Check logs in the **Logs** tab
2. Verify all ROS dependencies are installed
3. Make sure workspace is built: `colcon build`
4. Source the workspace: `source install/setup.bash`

## 9. Port Reference

| Service | Port | Purpose |
|---------|------|---------|
| Web GUI | 3000 | Next.js development server |
| rosbridge | 9090 | WebSocket connection to ROS |
| web_video_server | 8080 | Camera stream HTTP server |
| noVNC | 6080 | RViz VNC web interface |
| VNC | 5901 | VNC server for RViz |

## 10. Optional: Auto-start on Boot

To automatically start services on boot, create systemd services:

### Create rosbridge service
```bash
sudo nano /etc/systemd/system/rover-bridge.service
```

```ini
[Unit]
Description=Mars Rover ROS Bridge
After=network.target

[Service]
Type=simple
User=YOUR_USERNAME
WorkingDirectory=/home/YOUR_USERNAME/mars_rover_ros2
ExecStart=/home/YOUR_USERNAME/mars_rover_ros2/src/start_ros_bridge.sh
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Replace `YOUR_USERNAME` with your actual username, then:
```bash
sudo systemctl daemon-reload
sudo systemctl enable rover-bridge.service
sudo systemctl start rover-bridge.service
```

## 11. Development

### Rebuilding ROS packages
```bash
cd ~/mars_rover_ros2
colcon build --packages-select PACKAGE_NAME
source install/setup.bash
```

### Updating Web GUI
```bash
cd ~/mars_rover_ros2/my-app
# Edit files, then restart dev server (Ctrl+C and npm run dev)
```

### Adding new launch files
1. Add `.launch.py` file to appropriate package
2. Update `ROVER_LAUNCH_FILES` in `my-app/app/page.tsx`
3. Rebuild if needed

## 12. Additional Resources

- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **Nav2 Documentation**: https://navigation.ros.org/
- **Next.js Documentation**: https://nextjs.org/docs
- **rosbridge Protocol**: https://github.com/RobotWebTools/rosbridge_suite

## Quick Start Summary

```bash
# Terminal 1: ROS Bridge
cd ~/mars_rover_ros2 && ./src/start_ros_bridge.sh

# Terminal 2: Web GUI  
cd ~/mars_rover_ros2/my-app && npm run dev

# Terminal 3: RViz VNC (optional)
cd ~/mars_rover_ros2 && ./src/start_rviz_vnc.sh

# Open browser
firefox http://localhost:3000
```

That's it! Your Mars Rover Web GUI is ready to use! 🚀
