#!/usr/bin/env python3
"""
Launch file for Mars Rover visualization with Foxglove Studio
This launches:
- Robot State Publisher (publishes URDF and TF)
- Joint State Publisher GUI (for manual joint control)
- Foxglove Bridge (WebSocket server for Foxglove Studio)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the URDF file path
    urdf_file_name = 'rover.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('rover_description'),
        'urdf',
        urdf_file_name
    )

    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='Port for Foxglove Bridge WebSocket server'
    )

    address_arg = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Address for Foxglove Bridge to bind to'
    )

    # Robot State Publisher - publishes robot_description and transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 30.0
        }]
    )

    # Joint State Publisher GUI - for manual control of joints
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Foxglove Bridge - WebSocket server for Foxglove Studio
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'tls': False,
            'certfile': '',
            'keyfile': '',
            'topic_whitelist': ['.*'],  # Allow all topics
            'service_whitelist': ['.*'],  # Allow all services
            'param_whitelist': ['.*'],  # Allow all parameters
            'client_topic_whitelist': ['.*'],
            'send_buffer_limit': 10000000,
            'use_compression': True,
            'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe', 
                           'services', 'connectionGraph', 'assets'],
            'asset_uri_allowlist': ['^package://.*'],
        }]
    )

    # Info message
    info_msg = LogInfo(
        msg=[
            '\n',
            '='*60, '\n',
            '  Mars Rover Foxglove Visualization Launched!\n',
            '='*60, '\n',
            '  Foxglove Bridge running on: ws://localhost:', 
            LaunchConfiguration('port'), '\n',
            '='*60, '\n',
            '\n',
            '  Next Steps:\n',
            '  1. Open Foxglove Studio (desktop app or https://app.foxglove.dev)\n',
            '  2. Click "Open connection"\n',
            '  3. Select "Rosbridge (ROS 1 & 2)"\n',
            '  4. Enter WebSocket URL: ws://localhost:', LaunchConfiguration('port'), '\n',
            '  5. Click "Open"\n',
            '\n',
            '  Recommended Panels:\n',
            '  - 3D: Visualize robot model and TF tree\n',
            '  - Raw Messages: /joint_states, /tf\n',
            '  - Plot: Joint positions and velocities\n',
            '  - Image: Add camera topics if available\n',
            '\n',
            '='*60, '\n'
        ]
    )

    return LaunchDescription([
        # Launch arguments
        port_arg,
        address_arg,
        
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        foxglove_bridge_node,
        
        # Info message
        info_msg
    ])
