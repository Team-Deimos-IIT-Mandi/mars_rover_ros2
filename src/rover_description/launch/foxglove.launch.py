#!/usr/bin/env python3
"""
Foxglove Bridge Launch File with Layout Support
Starts foxglove_bridge node for WebSocket-based visualization.
Connect via Foxglove Studio at ws://localhost:8765

Supports automatic layout loading:
- Set 'layout_name' to automatically load a predefined layout
  Available layouts: 'gazebo_sim', 'gps_navigation', 'complete_system'
- Layout files are located in config/foxglove_layouts/
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    port = LaunchConfiguration('port', default='8765')
    address = LaunchConfiguration('address', default='0.0.0.0')
    layout_name = LaunchConfiguration('layout_name', default='')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='WebSocket port for Foxglove connection'
    )
    
    declare_address = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Address to bind WebSocket server'
    )
    
    declare_layout_name = DeclareLaunchArgument(
        'layout_name',
        default_value='',
        description='Foxglove layout to load (gazebo_sim, gps_navigation, complete_system, or empty for manual)'
    )
    
    # Foxglove Bridge Node
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': port,
            'address': address,
            'send_buffer_limit': 10000000,  # 10MB buffer for camera streams
            'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe', 'services', 'connectionGraph'],
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_port,
        declare_address,
        declare_layout_name,
        LogInfo(msg=['Starting Foxglove Bridge on ws://', address, ':', port]),
        LogInfo(msg=['To load a layout, import the JSON file from: ', 
                     os.path.join(pkg_share, 'config/foxglove_layouts/')]),
        foxglove_bridge,
    ])
