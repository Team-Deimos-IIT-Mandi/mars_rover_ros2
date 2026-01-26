#!/bin/bash
# Mars Rover Web GUI - Jetson Nano Installation Script
# Run this inside your Docker container on Jetson Nano
# This installs ONLY the web GUI (no VNC, no desktop tools)

set -e  # Exit on error

echo "============================================"
echo "Mars Rover Web GUI - Jetson Nano Setup"
echo "============================================"
echo ""

# Check if running as root (needed for apt)
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

# Get the actual user (not root)
ACTUAL_USER="${SUDO_USER:-$USER}"
ACTUAL_HOME=$(eval echo ~$ACTUAL_USER)

echo "Installing for user: $ACTUAL_USER"
echo "Home directory: $ACTUAL_HOME"
echo ""

# 1. Install Node.js 22.x (LTS)
echo "============================================"
echo "Step 1: Installing Node.js 22.x"
echo "============================================"

# Remove old Node.js if exists
apt-get remove -y nodejs npm 2>/dev/null || true

# Install Node.js from NodeSource
curl -fsSL https://deb.nodesource.com/setup_22.x | bash -
apt-get install -y nodejs

# Verify installation
echo ""
echo "Node.js version: $(node --version)"
echo "npm version: $(npm --version)"
echo ""

# 2. Install ROS 2 packages (if not already installed)
echo "============================================"
echo "Step 2: Installing ROS 2 Dependencies"
echo "============================================"

apt-get update
apt-get install -y \
    ros-humble-rosbridge-server \
    ros-humble-web-video-server \
    ros-humble-image-transport-plugins \
    python3-pip

echo "ROS packages installed"
echo ""

# 3. Install Web GUI npm dependencies
echo "============================================"
echo "Step 3: Installing Web GUI Dependencies"
echo "============================================"

# Navigate to web GUI directory
WEB_GUI_DIR="$ACTUAL_HOME/mars_rover_ros2/my-app"

if [ ! -d "$WEB_GUI_DIR" ]; then
    echo "ERROR: Web GUI directory not found at $WEB_GUI_DIR"
    echo "Make sure you've cloned the repository first"
    exit 1
fi

cd "$WEB_GUI_DIR"

# Install npm packages as the actual user (not root)
echo "Installing npm packages..."
su - $ACTUAL_USER -c "cd $WEB_GUI_DIR && npm install"

echo ""
echo "Web GUI dependencies installed"
echo ""

# 4. Configure firewall (if ufw is installed)
echo "============================================"
echo "Step 4: Configuring Firewall"
echo "============================================"

if command -v ufw &> /dev/null; then
    echo "Opening required ports..."
    ufw allow 3000/tcp  # Web GUI
    ufw allow 9090/tcp  # ROS Bridge
    ufw allow 8080/tcp  # Camera streams
    echo "Firewall configured"
else
    echo "ufw not installed, skipping firewall configuration"
fi

echo ""

# 5. Get Jetson IP address
echo "============================================"
echo "Installation Complete!"
echo "============================================"
echo ""
echo "Your Jetson Nano IP address:"
JETSON_IP=$(hostname -I | awk '{print $1}')
echo "  $JETSON_IP"
echo ""
echo "To start the web GUI:"
echo ""
echo "  Terminal 1 (ROS Bridge):"
echo "    cd $ACTUAL_HOME/mars_rover_ros2"
echo "    ./src/start_ros_bridge.sh"
echo ""
echo "  Terminal 2 (Web GUI):"
echo "    cd $ACTUAL_HOME/mars_rover_ros2/my-app"
echo "    npm run dev"
echo ""
echo "Access from any device on your network:"
echo "  http://$JETSON_IP:3000"
echo ""
echo "============================================"
echo "Network Access Notes:"
echo "============================================"
echo ""
echo "✓ Web GUI accessible from PC/mobile on same network"
echo "✓ ROS topics shared via ROS_DOMAIN_ID (if configured)"
echo "✓ RViz runs on your PC, connects to Jetson's ROS topics"
echo "✓ No VNC needed - everything works over network"
echo ""
echo "If Jetson is on Ubiquiti (no internet):"
echo "  - Pre-download Node.js installer on PC"
echo "  - Copy to Jetson via USB/network share"
echo "  - Or use a temporary internet connection for setup"
echo ""
echo "============================================"
