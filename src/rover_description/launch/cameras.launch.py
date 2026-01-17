#!/usr/bin/env python3
"""
USB Cameras Launch File for Mars Rover
Launches all 3 USB cameras with proper configuration
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    
    # Camera config files
    rgbd_config = os.path.join(pkg_share, 'config', 'camera_rgbd.yaml')
    camera_1_config = os.path.join(pkg_share, 'config', 'camera_1.yaml')
    camera_2_config = os.path.join(pkg_share, 'config', 'camera_2.yaml')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # RGBD/Depth Camera (Front) - USB connected at /dev/video0
    rgbd_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='rgbd_camera',
        namespace='rgbd_camera',
        output='screen',
        parameters=[
            rgbd_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('image_raw', 'image_raw'),
            ('camera_info', 'camera_info'),
        ]
    )
    
    # Camera 1 (Left Side) - USB connected at /dev/video1
    camera_1_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera_1',
        namespace='camera_1',
        output='screen',
        parameters=[
            camera_1_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('image_raw', 'image_raw'),
            ('camera_info', 'camera_info'),
        ]
    )
    
    # Camera 2 (Right Side) - USB connected at /dev/video2
    camera_2_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera_2',
        namespace='camera_2',
        output='screen',
        parameters=[
            camera_2_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('image_raw', 'image_raw'),
            ('camera_info', 'camera_info'),
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        rgbd_camera_node,
        camera_1_node,
        camera_2_node,
    ])
