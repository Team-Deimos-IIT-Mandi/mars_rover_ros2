#!/bin/bash
# Setup script for Mars Rover Ros2 Workspace
# This script configures the environment for the robotic arm simulation

# ROS 2 Humble
source /opt/ros/humble/setup.bash
echo "✓ Sourced ROS 2 Humble"

# This workspace (auto-detect path)
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MARS_ROVER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../mars_rover_ros2" && pwd)"

# Source mars_rover_ros2 workspace first (for gz_ros2_control)
if [ -d "${MARS_ROVER_DIR}/install" ]; then
    source "${MARS_ROVER_DIR}/install/setup.bash"
    echo "✓ Sourced mars_rover_ros2 workspace (gz_ros2_control)"
else
    echo "⚠ mars_rover_ros2 workspace not built. Building it now..."
    cd "${MARS_ROVER_DIR}" && colcon build --packages-select gz_ros2_control
    source "${MARS_ROVER_DIR}/install/setup.bash"
    cd "${WORKSPACE_DIR}"
fi

# Check if workspace is built
if [ ! -d "${WORKSPACE_DIR}/install" ]; then
    echo "⚠ Workspace not built yet. Run: colcon build"
    return 1
fi

# Source this workspace
source "${WORKSPACE_DIR}/install/setup.bash"

# Gazebo Harmonic configuration
export GZ_VERSION=harmonic

# Gazebo resource path (for finding meshes and models)
export GZ_SIM_RESOURCE_PATH=${WORKSPACE_DIR}/install/my_robotic_arm/share:${WORKSPACE_DIR}/install/rover_description/share:${MARS_ROVER_DIR}/install/rover_description/share:${GZ_SIM_RESOURCE_PATH}

# Helpful aliases
alias build='colcon build && source install/setup.bash'
alias build_arm='colcon build --packages-select my_robotic_arm && source install/setup.bash'
alias launch_arm='ros2 launch my_robotic_arm display.launch.py'

echo "═══════════════════════════════════════════════════════"
echo "✓ Robotic Arm ROS2 Environment Configured"
echo "═══════════════════════════════════════════════════════"
echo "  Workspace:      ${WORKSPACE_DIR}"
echo "  ROS_DISTRO:     humble"
echo "  GZ_VERSION:     harmonic"
echo "  gz_ros2_ctrl:   ${MARS_ROVER_DIR}"
echo ""
echo "Quick Commands:"
echo "  build         - Build all packages"
echo "  build_arm     - Build only my_robotic_arm"
echo "  launch_arm    - Launch the complete system"
echo "═══════════════════════════════════════════════════════"