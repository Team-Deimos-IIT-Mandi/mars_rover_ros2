#!/bin/bash
# Mars Rover ROS2 Workspace Setup Script
# Source this script to configure the environment

# ROS 2 Humble
source /opt/ros/humble/setup.bash
echo "✓ Sourced ROS 2 Humble"

# Auto-detect workspace directory
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if workspace is built
if [ ! -d "${WORKSPACE_DIR}/install" ]; then
    echo "⚠ Workspace not built yet. Run: colcon build"
    return 1
fi

# Source this workspace
source "${WORKSPACE_DIR}/install/setup.bash"
echo "✓ Sourced Mars Rover workspace"

# Gazebo Harmonic configuration
export GZ_VERSION=harmonic

# Helpful aliases for Mars Rover
alias build='colcon build --symlink-install && source install/setup.bash'
alias build_rover='colcon build --packages-select rover_description --symlink-install && source install/setup.bash'
alias clean_build='rm -rf build install log && colcon build --symlink-install'

# Launch aliases
alias launch_gazebo='ros2 launch rover_description gazebo_sim.launch.py'
alias launch_gps='ros2 launch rover_description gps_navigation.launch.py'
alias launch_complete='ros2 launch rover_description complete_system.launch.py'
alias launch_foxglove='ros2 launch rover_description foxglove.launch.py'

# With Foxglove
alias launch_gazebo_fox='ros2 launch rover_description gazebo_sim.launch.py enable_foxglove:=true'
alias launch_gps_fox='ros2 launch rover_description gps_navigation.launch.py enable_foxglove:=true'
alias launch_complete_fox='ros2 launch rover_description complete_system.launch.py enable_foxglove:=true enable_mapping:=true'

echo "═══════════════════════════════════════════════════════"
echo "✓ Mars Rover ROS2 Environment Configured"
echo "═══════════════════════════════════════════════════════"
echo "  Workspace:      ${WORKSPACE_DIR}"
echo "  ROS_DISTRO:     ${ROS_DISTRO}"
echo "  GZ_VERSION:     ${GZ_VERSION}"
echo ""
echo "Build Commands:"
echo "  build              - Build all packages"
echo "  build_rover        - Build only rover_description"
echo "  clean_build        - Clean and rebuild everything"
echo ""
echo "Launch Commands (without Foxglove):"
echo "  launch_gazebo      - Launch Gazebo simulation"
echo "  launch_gps         - Launch GPS navigation"
echo "  launch_complete    - Launch complete system"
echo ""
echo "Launch Commands (with Foxglove):"
echo "  launch_gazebo_fox  - Gazebo + Foxglove"
echo "  launch_gps_fox     - GPS + Foxglove"
echo "  launch_complete_fox - Complete + Foxglove + SLAM"
echo ""
echo "Foxglove Connection: ws://localhost:8765"
echo "═══════════════════════════════════════════════════════"
