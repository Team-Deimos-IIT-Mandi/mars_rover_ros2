# RViz Web Viewer - Quick Start Guide

## Overview
RViz is now streamable to your web browser! The Navigation View tab shows a live RViz window with all visualization panels (paths, costmaps, laser scans, robot model, etc.).

## How to Use

### 1. Start the RViz VNC Server
Open a new terminal and run:
```bash
cd ~/test/mars_rover_ros2
./src/start_rviz_vnc.sh
```

This will:
- Start a VNC server with RViz
- Launch noVNC web interface on port 6080
- Open RViz with your rover navigation configuration

### 2. Access RViz in Web GUI
1. Open your web browser to `http://localhost:3000`
2. Launch any process (e.g., ArUco Mission)
3. Click the **"Navigation View"** tab
4. You'll see RViz streaming live!

### 3. Interact with RViz
- **Zoom**: Mouse wheel
- **Pan**: Middle mouse button drag
- **Rotate**: Left mouse button drag
- All RViz panels and tools work normally

## What You'll See

The RViz window shows:
- **Robot Model**: 3D model of your rover
- **Laser Scan**: Red point cloud from lidar
- **Global Path**: Green line (planned path)
- **Local Path**: Yellow line (local planner)
- **Costmaps**: Local and global costmaps
- **TF Frames**: Coordinate frames
- **GPS Position**: Robot location on map

## Troubleshooting

### RViz not showing in browser?
1. Make sure `./src/start_rviz_vnc.sh` is running
2. Check that port 6080 is not blocked
3. Try accessing directly: `http://localhost:6080/vnc.html?autoconnect=true`

### VNC password prompt?
The script creates a VNC server without password for localhost. If prompted, press "Send Credentials" without entering anything.

### Performance issues?
- Reduce RViz window size in the script (change `-geometry 1920x1080` to `-geometry 1280x720`)
- Close unused RViz panels
- Reduce point cloud density

## Stopping the Server

Press `Ctrl+C` in the terminal where `start_rviz_vnc.sh` is running. This will cleanly shut down VNC and RViz.

## Advanced: Auto-start with Bridge

To start RViz VNC automatically with the ROS bridge, add this to your launch routine:
```bash
# Terminal 1: ROS Bridge
./src/start_ros_bridge.sh

# Terminal 2: RViz VNC
./src/start_rviz_vnc.sh
```

Or create a combined launcher script if needed.
