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

The navigation system relies on a "Dual EKF" (Extended Kalman Filter) setup provided by the `robot_localization` package to fuse data from multiple sensors:
*   **GPS**: Absolute global position (noisy, low frequency).
*   **IMU**: Orientation and angular velocity.
*   **Wheel Encoders**: Continuous odometry (smooth, drifts over time).

### TF Tree Structure

The system uses a hierarchical coordinate frame tree for localization and navigation:

```
utm -> map -> odom -> base_footprint -> base_link -> chassis -> [sensors/wheels]
```

**Frame Hierarchy Explanation:**

1. **utm**: Universal Transverse Mercator coordinate frame (static, published by navsat_transform_node)
2. **map**: Global fixed frame aligned with the initial GPS position (published by ekf_filter_node_map via map->odom transform)
3. **odom**: Local odometry frame that drifts over time (published by ekf_filter_node_odom via odom->base_footprint transform)
4. **base_footprint**: Projection of robot base on the ground plane (published by ekf_filter_node_odom)
5. **base_link**: Robot's base center (published by robot_state_publisher)
6. **chassis**: Robot's main body frame
7. **Sensor frames**: imu_link, gps_link, lidar_link, camera_link, camera_link_1, camera_link_2
8. **Wheel frames**: steering1-4, wheel1-4 (published by joint_state_broadcaster)

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

### Sensor Fusion and Localization

The system uses three key nodes from the `robot_localization` package:

#### 1. Local EKF (`ekf_filter_node_odom`)
**Purpose**: Provides smooth, continuous local odometry for real-time navigation.

**Inputs:**
- `/diff_drive_controller/odom` (wheel encoders): Linear velocities (vx, vy) and angular velocity (wz)
- `/imu`: Orientation (yaw) and angular velocity (wz)

**Outputs:**
- `/odometry/local`: Fused local odometry estimate
- **TF Transform**: Publishes `odom -> base_footprint` transform

**Configuration**: Operates in `world_frame: odom`, fuses wheel odometry for position and IMU for orientation.

#### 2. NavSat Transform Node (`navsat_transform`)
**Purpose**: Converts GPS coordinates (latitude/longitude) to Cartesian coordinates in the map frame.

**Inputs:**
- `/gps/fix`: Raw GPS coordinates (NavSatFix messages)
- `/imu`: IMU data for orientation
- `/odometry/global`: Global odometry from the map EKF

**Outputs:**
- `/odometry/gps`: GPS position converted to Cartesian coordinates (Odometry message)
- **TF Transform**: Publishes `utm -> map` transform (static offset between UTM and local map frame)

**Configuration**: Uses datum (reference point) from first GPS fix, publishes filtered GPS in Cartesian coordinates.

#### 3. Global EKF (`ekf_filter_node_map`)
**Purpose**: Fuses GPS with IMU to provide drift-corrected global localization.

**Inputs:**
- `/odometry/gps` (from navsat_transform): Global position from GPS in Cartesian coordinates
- `/imu`: Orientation (roll, pitch, yaw) for attitude correction

**Outputs:**
- `/odometry/global`: Globally-corrected odometry estimate
- **TF Transform**: Publishes `map -> odom` transform

**Configuration**: Operates in `world_frame: map`, fuses GPS position (x, y, z) and IMU orientation to correct local odometry drift.

### How It Works Together

1.  **Local EKF** fuses wheel odometry and IMU to provide smooth `odom -> base_footprint` transform for local navigation
2.  **NavSat Transform** converts GPS lat/lon to Cartesian `/odometry/gps` and establishes the `utm -> map` relationship
3.  **Global EKF** fuses GPS odometry and IMU to publish `map -> odom` transform, correcting drift in the odometry frame
4.  **Complete chain**: `utm -> map -> odom -> base_footprint -> base_link`, enabling both precise local control and accurate global positioning

## Launch Files

The project includes several launch files, each with a specific purpose. Here's a detailed description:

### 1. `complete_system.launch.py`
**Description**: Master launch file that sequentially starts the entire Mars Rover system.

**What it launches:**
1. Gazebo simulation with sensor bridges
2. Robot controllers and state publishers
3. Dual EKF + NavSat localization system
4. Cartographer SLAM (optional, with delay for controller initialization)
5. Nav2 navigation stack (optional)
6. RViz2 visualization

**Launch arguments:**
- `use_sim_time` (default: `true`): Use simulation time
- `enable_mapping` (default: `true`): Enable/disable Cartographer SLAM
- `enable_navigation` (default: `false`): Enable/disable Nav2 autonomous navigation

**Usage:**
```bash
ros2 launch rover_description complete_system.launch.py enable_navigation:=true enable_mapping:=true
```

### 2. `gazebo_sim.launch.py`
**Description**: Launches Gazebo Harmonic simulator with the Mars Rover and all required sensor bridges.

**What it does:**
- Starts Gazebo with the configured world file (`empty.world`)
- Spawns the rover robot from URDF at position (0, 0, 0.2)
- Creates ROS-Gazebo bridges for clock, cameras, LiDAR, IMU, and GPS sensors
- Starts robot_state_publisher to broadcast robot's TF tree from URDF

**Topics bridged:**
- `/clock`: Simulation time
- `/camera/image_raw`, `/camera_1/image_raw`, `/camera_2/image_raw`: Camera feeds
- `/scan`: LiDAR data
- `/imu`: IMU sensor data (orientation, angular velocity)
- `/gps/fix`: GPS coordinates (NavSatFix)

### 3. `controllers.launch.py`
**Description**: Launches ros2_control nodes for robot motor control.

**What it launches:**
- `ros2_control_node`: Main controller manager
- `joint_state_broadcaster`: Publishes joint states to `/joint_states`
- `diff_drive_controller`: Differential drive controller for wheel control

**Configuration**: Loads parameters from `config/controllers.yaml`

**Published topics:**
- `/joint_states`: Current state of all robot joints
- `/diff_drive_controller/odom`: Wheel odometry
- `/diff_drive_controller/cmd_vel`: Command velocity input (TwistStamped)

### 4. `dual_ekf_navsat.launch.py`
**Description**: Launches the dual EKF localization system with GPS integration.

**What it launches:**
- `ekf_filter_node_odom`: Local EKF (fuses wheel odometry + IMU)
- `ekf_filter_node_map`: Global EKF (fuses GPS odometry + IMU)
- `navsat_transform`: Converts GPS lat/lon to Cartesian coordinates

**Configuration**: Loads parameters from `config/ekf.yaml`

**Published topics:**
- `/odometry/local`: Local odometry estimate
- `/odometry/global`: Global odometry estimate (GPS-corrected)
- `/odometry/gps`: GPS position in Cartesian coordinates

**Published transforms:**
- `odom -> base_footprint` (from ekf_filter_node_odom)
- `map -> odom` (from ekf_filter_node_map)
- `utm -> map` (from navsat_transform)

### 5. `gps_navigation.launch.py`
**Description**: Launches Nav2 navigation stack configured for GPS-based navigation (without AMCL).

**What it launches:**
- Nav2 stack from `nav2_bringup/navigation_launch.py`
- `cmd_vel_relay.py`: Converts Twist to TwistStamped messages

**Configuration**: Loads parameters from `config/nav2_gps_params.yaml`

**Launch arguments:**
- `use_sim_time` (default: `true`): Use simulation time
- `autostart` (default: `true`): Auto-start Nav2 lifecycle nodes

**Topic remappings:**
- Nav2's `/cmd_vel` → `/diff_drive_controller/cmd_vel` (via relay node)

### 6. `mapping.launch.py`
**Description**: Launches Cartographer SLAM for 2D occupancy grid mapping.

**What it launches:**
- `cartographer_node`: SLAM node for map building
- `cartographer_occupancy_grid_node`: Converts SLAM data to occupancy grid

**Configuration**: Loads Lua configuration from `config/rover.lua`

**Topic remappings:**
- `/imu` → `/imu`
- `/scan` → `/scan` (LiDAR)
- `/odom` → `/odometry/global`
- `/fix` → `/gps/fix`

**Published topics:**
- `/map`: Occupancy grid map

### 7. `navigation.launch.py`
**Description**: Alternative navigation launch file using pre-built map with AMCL localization (deprecated in favor of GPS navigation).

**What it launches:**
- Gazebo simulation
- Controllers
- Local EKF only
- NavSat transform with custom remappings
- Nav2 with AMCL localization
- RViz2

**Note**: This file is kept for reference but is superseded by `gps_navigation.launch.py` for GPS-based navigation.

### 8. `gz_plus_control.launch.py`
**Description**: Minimal launch file for testing Gazebo simulation with controllers and localization.

**What it launches:**
- Gazebo simulation (`gazebo_sim.launch.py`)
- Controllers (`controllers.launch.py`)
- Dual EKF + NavSat (`dual_ekf_navsat.launch.py`)

**Purpose**: Quick launch for testing basic robot movement and localization without navigation or mapping.

### 9. `display.launch.py`
**Description**: Launches robot model visualization in RViz without Gazebo simulation.

**What it launches:**
- `robot_state_publisher`: Publishes robot TF tree from URDF
- `joint_state_publisher_gui`: GUI for manually controlling joint positions
- `rviz2`: 3D visualization

**Purpose**: For visualizing and debugging the robot URDF model without running simulation.

## Nodes and Scripts

The project includes several Python nodes/scripts for specific functionalities:

### 1. `gps_waypoint_follower.py`
**Description**: Autonomous GPS waypoint navigation node that converts GPS coordinates to map coordinates and sends them to Nav2.

**Functionality:**
- Subscribes to `/gps/fix` to establish a datum (reference point)
- Converts GPS waypoints (lat/lon) to Cartesian coordinates using the datum
- Sends waypoints to Nav2 using the `FollowWaypoints` action server
- Provides feedback on navigation progress

**Usage:**
```bash
ros2 run rover_description gps_waypoint_follower.py [optional_waypoints.yaml]
```

**Default waypoints**: Hardcoded demo waypoints (can be replaced via YAML file)

**Topics subscribed:**
- `/gps/fix`: GPS position for datum establishment

**Action clients:**
- `/follow_waypoints`: Sends waypoint goals to Nav2

### 2. `cmd_vel_relay.py`
**Description**: Message type converter that relays velocity commands from Nav2 to the differential drive controller.

**Functionality:**
- Subscribes to Nav2's `/cmd_vel` (geometry_msgs/Twist)
- Converts to TwistStamped and publishes to `/diff_drive_controller/cmd_vel`
- Adds timestamp and frame_id for proper message handling

**Topics subscribed:**
- `/cmd_vel`: Velocity commands from Nav2 (Twist)

**Topics published:**
- `/diff_drive_controller/cmd_vel`: Stamped velocity commands (TwistStamped)

**Why needed**: The differential drive controller requires TwistStamped messages, while Nav2 publishes Twist messages.

### 3. `goal_to_pose.py`
**Description**: Converts GPS odometry to Nav2 goal poses (alternative to waypoint follower).

**Functionality:**
- Subscribes to GPS-converted odometry on `/gps_point`
- Converts odometry messages to PoseStamped goal poses
- Publishes to `/goal_pose` for Nav2

**Topics subscribed:**
- `/gps_point`: GPS position in Cartesian coordinates

**Topics published:**
- `/goal_pose`: Navigation goal for Nav2

**Note**: This is an alternative approach to `gps_waypoint_follower.py`, converting each GPS point to individual goals.

### 4. `teleop.py`
**Description**: Keyboard teleoperation node for manual rover control.

**Functionality:**
- Reads keyboard input in real-time
- Publishes TwistStamped messages to control the rover
- Supports forward/backward (w/s), rotation (a/d), and emergency stop (space/k)

**Controls:**
- `w`: Move forward
- `s`: Move backward
- `a`: Rotate left
- `d`: Rotate right
- `space` or `k`: Emergency stop
- `Ctrl+C`: Quit

**Topics published:**
- `/diff_drive_controller/cmd_vel`: Direct control commands to robot (TwistStamped)

**Usage:**
```bash
ros2 run rover_description teleop.py
```

**Parameters:**
- Linear velocity: 1.5 m/s
- Angular velocity: 0.5 rad/s