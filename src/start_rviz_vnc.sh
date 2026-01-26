#!/bin/bash
# Start RViz with VNC for web viewing
# This allows RViz to be viewed in the web browser via noVNC

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "============================================"
echo "Starting RViz with VNC for Web Viewing"
echo "============================================"

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# VNC display number and ports
DISPLAY_NUM=99
VNC_PORT=5901
NOVNC_PORT=6080

# Kill any existing processes
echo "Cleaning up any existing servers..."
pkill -f "Xvfb.*:$DISPLAY_NUM" || true
pkill -f "x11vnc.*$VNC_PORT" || true
pkill -f "websockify.*$NOVNC_PORT" || true
pkill -f "rviz2" || true
sleep 2

# Start virtual display with Xvfb
echo "Starting virtual display :$DISPLAY_NUM..."
Xvfb :$DISPLAY_NUM -screen 0 1920x1080x24 &
XVFB_PID=$!
sleep 2

# Start x11vnc server
echo "Starting x11vnc on port $VNC_PORT..."
x11vnc -display :$DISPLAY_NUM -rfbport $VNC_PORT -forever -shared -nopw &
X11VNC_PID=$!
sleep 2

# Start noVNC websockify
echo "Starting noVNC websockify on port $NOVNC_PORT..."
websockify --web=/usr/share/novnc/ $NOVNC_PORT localhost:$VNC_PORT &
WEBSOCKIFY_PID=$!
sleep 2

# Launch RViz on the virtual display
echo "Launching RViz on virtual display..."
RVIZ_CONFIG="$WS_DIR/src/rover_description/config/aruco_mission.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "Warning: RViz config not found at $RVIZ_CONFIG, using default"
    DISPLAY=:$DISPLAY_NUM rviz2 &
else
    DISPLAY=:$DISPLAY_NUM rviz2 -d "$RVIZ_CONFIG" &
fi
RVIZ_PID=$!

echo ""
echo "============================================"
echo "RViz VNC Server Started Successfully!"
echo "============================================"
echo "Virtual Display: :$DISPLAY_NUM"
echo "VNC Port: $VNC_PORT"
echo "noVNC Web URL: http://localhost:$NOVNC_PORT/vnc.html"
echo ""
echo "Access RViz in your web browser at:"
echo "  http://localhost:$NOVNC_PORT/vnc.html?autoconnect=true"
echo ""
echo "Or use the Navigation View tab in the dashboard"
echo ""
echo "Press Ctrl+C to stop"
echo "============================================"

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down RViz VNC server..."
    kill $RVIZ_PID 2>/dev/null || true
    kill $WEBSOCKIFY_PID 2>/dev/null || true
    kill $X11VNC_PID 2>/dev/null || true
    kill $XVFB_PID 2>/dev/null || true
    echo "Cleanup complete"
    exit 0
}

# Trap Ctrl+C
trap cleanup INT TERM

# Wait for processes
wait
