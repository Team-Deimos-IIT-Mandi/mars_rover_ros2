import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('rover_description')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Declare launch argument
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set to false for hardware)'
    )

    return LaunchDescription([
        declare_use_sim_time,
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/odometry/filtered', '/odometry/local')
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/odometry/filtered', '/odometry/global')
            ]
        ),

        # NavSat Transform Node: Integrates GPS and IMU
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/gps/fix', '/gps/fix'),
                ('/imu', '/imu'),
                ('/odometry/filtered', '/odometry/global')
            ]
        ),

    ])