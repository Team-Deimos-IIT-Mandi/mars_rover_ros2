# Mars Rover ROS2 Navigation

This project implements a complete simulation and navigation stack for a Mars Rover using ROS 2 Humble, Gazebo Harmonic, and the Nav2 stack. It features GPS-based localization, sensor fusion (Dual EKF), and autonomous waypoint following.

## Video Demo

[Insert Video Link Here]

## Project Structure

Here is an overview of the key files and directories in the project:

```text
src/rover_description/
├── config/                 # Configuration files for ROS nodes
│   ├── ekf.yaml            # Dual EKF (Local & Global) parameters
│   ├── nav2_gps_params.yaml # Navigation2 stack parameters
│   ├── controllers.yaml    # ros2_control controller configurations
│   └── ...
├── launch/                 # Launch files
│   ├── complete_system.launch.py # Main entry point: launches EVERYTHING
│   ├── gps_navigation.launch.py  # Launches Nav2 with GPS support
│   └── ...
├── scripts/                # Python scripts and nodes
│   ├── gps_waypoint_follower.py  # Autonomous GPS waypoint follower node
│   └── ...
├── urdf/                   # Robot description (URDF/Xacro)
├── worlds/                 # Gazebo simulation worlds
└── package.xml
src/setup.sh                # Environment setup helper script
```

## Setup and Installation

### 1. Prerequisites
Ensure you have the following installed:
*   Ubuntu 22.04 (Jammy Jellyfish)
*   ROS 2 Humble
*   Gazebo Harmonic (gz-sim)

### 2. Build the Project
Clone the repository and build the workspace using `colcon`:

```bash
cd ~/mars_rover_ros2
colcon build --symlink-install
```

### 3. Configure Environment
We provide a setup script to configure your shell environment variables, source the workspace, and set up Gazebo resource paths.

**You must run this in every new terminal:**

```bash
source src/setup.sh
```

## Running the Simulation

The project uses a single master launch file to start the entire system sequentially (Gazebo, Controllers, Localization, Navigation, RViz).

### Launch the Complete System

To start the simulation with GPS navigation enabled:

```bash
ros2 launch rover_description complete_system.launch.py enable_navigation:=true
```

**Launch Arguments:**
*   `enable_navigation` (default: `false`): Starts the Nav2 stack (required for waypoint following).
*   `enable_mapping` (default: `true`): Starts Cartographer for SLAM.
*   `use_sim_time` (default: `true`): Uses simulation time.

## Autonomous GPS Navigation

Once the system is running (with `enable_navigation:=true`), you can instruct the rover to follow a set of GPS coordinates.

### Run the Waypoint Follower
Open a **new terminal**, source the setup script, and run the waypoint follower node:

```bash
# 1. Source the environment
source src/setup.sh

# 2. Run the GPS waypoint follower
ros2 run rover_description gps_waypoint_follower.py
```

This node will:
1.  Wait for a GPS fix to establish a "Datum" (reference point).
2.  Convert the hardcoded list of GPS waypoints (Lat/Lon) into local Cartesian coordinates (X/Y) relative to the map.
3.  Send these goals to the Nav2 stack using the `FollowWaypoints` action.
4.  The rover will autonomously navigate to each point in sequence.

## Architecture

The navigation system relies on a "Dual EKF" setup provided by the `robot_localization` package to fuse data from:
*   **GPS**: Absolute global position (noisy, low frequency).
*   **IMU**: Orientation and angular velocity.
*   **Wheel Encoders**: Continuous odometry (smooth, drifts over time).

### System Data Flow

```mermaid
graph TD
    subgraph Sensors
        GPS[GPS Receiver<br/>/gps/fix]
        IMU[IMU<br/>/imu]
        Encoders[Wheel Encoders<br/>/diff_drive_controller/odom]
    end

    subgraph "Robot Localization (Sensor Fusion)"
        NavSat[navsat_transform_node]
        EKF_Local[ekf_filter_node_odom<br/>(Local EKF)]
        EKF_Global[ekf_filter_node_map<br/>(Global EKF)]
    end

    subgraph "Navigation Stack"
        Nav2[Nav2 Stack]
        WaypointNode[gps_waypoint_follower.py]
    end

    %% Connections
    GPS --> NavSat
    IMU --> NavSat
    IMU --> EKF_Local
    IMU --> EKF_Global
    
    Encoders --> EKF_Local
    
    NavSat -- /odometry/gps --> EKF_Global
    
    EKF_Local -- /odometry/local --> Nav2
    EKF_Local -- tf: odom->base_footprint --> TF_Tree[TF Tree]
    
    EKF_Global -- /odometry/global --> Nav2
    EKF_Global -- tf: map->odom --> TF_Tree
    
    WaypointNode -- Action: FollowWaypoints --> Nav2
```

### GPS Navigation Explained
1.  **Local EKF (`ekf_filter_node_odom`)**: Fuses **Wheel Odometry** and **IMU** to provide a smooth, continuous `odom` frame. This is used by the local planner for obstacle avoidance and velocity control.
2.  **NavSat Transform**: Converts raw GPS (Lat/Lon) into Cartesian coordinates (`/odometry/gps`).
3.  **Global EKF (`ekf_filter_node_map`)**: Fuses **GPS Odometry** and **IMU** to publish the `map` -> `odom` transform. This corrects the drift of the local odometry, keeping the robot localized in the global frame.