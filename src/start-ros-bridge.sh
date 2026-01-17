#!/bin/bash
# Start ROS Bridge for Web GUI
# Run this from anywhere - it will auto-detect the workspace

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Workspace is the root directory (mars_rover_ros2)
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "============================================"
echo "Starting ROS Bridge for Mars Rover Web GUI"
echo "============================================"
echo "Workspace: $WS_DIR"
echo ""

# Check if install directory exists
if [ ! -d "$WS_DIR/install" ]; then
    echo "ERROR: install directory not found at $WS_DIR/install"
    echo "Please build the workspace first with: colcon build"
    exit 1
fi

# Source ROS2 and workspace
echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# Start rosbridge
echo ""
echo "Launching rosbridge_server on ws://localhost:9090"
echo "Keep this terminal open - Web GUI needs this running!"
echo "Press Ctrl+C to stop"
echo ""

ros2 launch rover_description web_bridge.launch.py
