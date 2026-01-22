# Mars Rover Web GUI - Quick Start

## Step 1: Install rosbridge_server
```bash
sudo apt install ros-humble-rosbridge-server
```

## Step 2: Install Web Dependencies
```bash
cd my-app
npm install
```

---

## Running the System

### Terminal 1: Start ROS Bridge
```bash
cd src
./start-ros-bridge.sh
```
Keep this running!

### Terminal 2: Start Web GUI
```bash
cd my-app
npm run dev
```

### Browser
1. Open http://localhost:3000
2. Click **"Connect to ROS Bridge"**
3. Launch files now work with Stop button!

---

## Quick Tips

**Launch Order:**
1. Start gazebo_sim or hardware first
2. Then start controllers

**Troubleshooting:**
- ROS Bridge not connecting? Check Terminal 1 is running
- Stop button disabled? Click "Connect to ROS Bridge"
