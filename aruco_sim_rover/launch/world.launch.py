#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package paths
    pkg_share = get_package_share_directory('aruco_sim_rover')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    # World file
    default_world = os.path.join(pkg_share, 'worlds', 'empty.sdf')

    # Launch argument
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Full path to the world file'
    )

    # Gazebo Harmonic launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                '-r ',                 # run immediately
                LaunchConfiguration('world')
            ]
        }.items()
    )

    return LaunchDescription([
        world_arg,
        gazebo,
    ])
