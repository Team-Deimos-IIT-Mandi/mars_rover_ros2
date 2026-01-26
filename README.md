# Mars Rover ROS 2 Web Interface

This project provides a premium web-based control center for the Mars Rover simulation, featuring:
- **3D Visualization** (RViz-like experience in the browser)
- **Multi-Camera Feeds** (Front/Depth, Left, Right)
- **GPS Map** (Interactive mission planning)
- **Teleoperation** (Keyboard/Touch control)
- **System Monitoring** (Process management)

## 📋 Prerequisites

Ensure you have **ROS 2 Humble** installed on Ubuntu 22.04 (Linux).

### 1. Install Required ROS 2 Packages
The web interface relies on several standard ROS 2 packages for bridging communication and streaming video/TF data.

```bash
sudo apt update
sudo apt install -y \
    ros-humble-rosbridge-server \
    ros-humble-web-video-server \
    ros-humble-tf2-web-republisher \
    ros-humble-robot-localization \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-clam-toolbox
```

> **Note**: `rosbridge_server` provides the WebSocket connection. `web_video_server` streams camera feeds via HTTP. `tf2_web_republisher` allows efficient TF updates for the 3D view.

### 2. Install Frontend Dependencies
The web dashboard is built with Next.js.

```bash
cd my-app
npm install
```
*If you encounter conflicts (especially with `react-leaflet`), use:*
```bash
npm install leaflet react-leaflet@4.2.1 ros3d three roslib --legacy-peer-deps
```

---

## 🚀 How to Run

### 1. Build the Workspace
Ensure the custom packages (`rover_description`, etc.) are built.

```bash
cd ~/test/mars_rover_ros2
colcon build --packages-select rover_description
source install/setup.bash
```

### 2. Start the ROS Bridge & Backend
We have provided a helper script to start the necessary bridges (`rosbridge`, `web_video_server`, `cmd_vel_relay`).

```bash
./src/start_ros_bridge.sh
```

### 3. Start the Web Dashboard
In a new terminal:

```bash
cd ~/test/mars_rover_ros2/my-app
npm run dev
```
Open [http://localhost:3000](http://localhost:3000) in your browser.

### 4. Launch Simulation
From the Web Dashboard:
1.  Go to the **Launch Files** tab.
2.  Click **Launch** on **"Gazebo Simulation"** (or "Complete System").
3.  Go to **Mission Control** or **Control Center** to operate the rover.

---

## 🛠️ Troubleshooting

- **Cameras are black/loading**:
  - Ensure `web_video_server` is installed: `sudo apt install ros-humble-web-video-server`
  - Ensure the simulation is running and publishing to `/rgbd_camera/image` etc.
- **3D View is empty**:
  - Ensure `tf2_web_republisher` is installed.
  - Refresh the page after the simulation has fully loaded.
- **Map Error**:
  - If you see `map.setView` errors, ensure you are using the latest code (fixed in `gps-map.tsx`).