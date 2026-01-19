#!/usr/bin/env python3
"""
Complete Mars Rover Launch: Gazebo Simulation + Foxglove Studio Visualization
This launches:
- Gazebo simulation with the rover
- All sensor bridges (cameras, lidar, IMU, GPS)
- Robot state publisher
- Foxglove Bridge for visualization
"""

import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def resolve_package_path(urdf_content, pkg_path):
    """Replace package:// URIs and hardcoded paths with absolute paths"""
    mesh_path = os.path.join(pkg_path, 'meshes')
    config_path = os.path.join(pkg_path, 'config', 'controllers.yaml')
    
    # Replace package://rover_description/meshes
    urdf_content = urdf_content.replace(
        'package://rover_description/meshes',
        f'file://{mesh_path}'
    )
    
    # Regex to replace the hardcoded controllers.yaml path
    pattern = r'<parameters>.*controllers\.yaml</parameters>'
    replacement = f'<parameters>{config_path}</parameters>'
    urdf_content = re.sub(pattern, replacement, urdf_content)
    
    return urdf_content

def generate_launch_description():
    pkg_path = get_package_share_directory('rover_description')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    urdf_path = os.path.join(pkg_path, 'urdf', 'rover.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    
    # Process the URDF to fix paths
    robot_description = resolve_package_path(robot_description, pkg_path)

    # Launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='Port for Foxglove Bridge'
    )

    # Gazebo Simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description, 
            'use_sim_time': True,
            'publish_frequency': 30.0
        }]
    )
    
    # Clock Bridge - synchronizes simulation time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Camera and Lidar Bridges
    cameras_lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cameras_lidar_bridge',
        arguments=[
            '/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera_1/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_2/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # IMU and GPS Bridges
    imu_gps_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_gps_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mars_rover',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # Foxglove Bridge - for visualization
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': ['.*'],
            'service_whitelist': ['.*'],
            'param_whitelist': ['.*'],
            'send_buffer_limit': 10000000,
            'use_compression': True,
            'use_sim_time': True,
            'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe', 
                           'services', 'connectionGraph', 'assets'],
            'asset_uri_allowlist': ['^package://.*'],
        }]
    )

    # Info message
    info_msg = LogInfo(
        msg=[
            '\n',
            '='*70, '\n',
            '  🚀 Mars Rover Simulation + Foxglove Visualization LAUNCHED! 🚀\n',
            '='*70, '\n',
            '\n',
            '  🌐 Foxglove Studio Connection:\n',
            '     URL: ws://localhost:', LaunchConfiguration('port'), '\n',
            '     Protocol: Foxglove WebSocket (NOT Rosbridge)\n',
            '\n',
            '  📡 Available Topics:\n',
            '     /rgbd_camera/image - RGB camera feed\n',
            '     /rgbd_camera/depth_image - Depth camera\n',
            '     /rgbd_camera/points - Point cloud\n',
            '     /camera_1/image_raw - Camera 1\n',
            '     /camera_2/image_raw - Camera 2\n',
            '     /scan - Lidar data\n',
            '     /imu - IMU sensor\n',
            '     /gps/fix - GPS position\n',
            '     /joint_states - Joint positions\n',
            '     /tf, /tf_static - Transforms\n',
            '\n',
            '  🎮 Control the Rover:\n',
            '     Open new terminal and run:\n',
            '     ros2 run teleop_twist_keyboard teleop_twist_keyboard\n',
            '\n',
            '  🦊 Foxglove Panels to Add:\n',
            '     1. 3D - View robot model, TF tree, point clouds\n',
            '     2. Image - View camera feeds\n',
            '     3. Raw Messages - Monitor /joint_states, /imu\n',
            '     4. Plot - Graph sensor data over time\n',
            '\n',
            '='*70, '\n'
        ]
    )

    return LaunchDescription([
        # Arguments
        port_arg,
        
        # Gazebo Simulation
        gz_sim,
        clock_bridge,
        
        # Robot
        robot_state_publisher,
        spawn_robot,
        
        # Sensor Bridges
        cameras_lidar_bridge,
        imu_gps_bridge,
        
        # Visualization
        foxglove_bridge,
        
        # Info
        info_msg
    ])
