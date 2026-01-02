import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    pkg = 'rover_description'
    
    # Path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory(pkg),
        'urdf',
        'rover.urdf.xacro'
    )

    # Path to the saved RViz configuration
    rviz_config_file = os.path.join(
        get_package_share_directory(pkg),
        'rviz',
        'rover.rviz'
    )

    # 2. Process Xacro to URDF 
    robot_description = Command(['xacro ', xacro_file])

    # 3. Create Launch Argument for Sim Time
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        declare_use_sim_time,

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }],
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])