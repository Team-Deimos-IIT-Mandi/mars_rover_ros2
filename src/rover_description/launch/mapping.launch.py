import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    config_dir = os.path.join(pkg_share, 'config')

    # Declare launch argument
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set to false for hardware)'
    )

    return LaunchDescription([
    declare_use_sim_time,

    Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='mapping_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[ '-configuration_directory', config_dir,
                    '-configuration_basename','rover.lua'],
        remappings=[
                ('/imu', '/imu'), 
                ('/scan', '/scan'),
                ('/odom', '/odometry/global'),
                ('/fix', '/gps/fix')
            ]
        ),
    Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])