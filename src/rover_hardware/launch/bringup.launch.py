import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Adjusted Paths based on your tree
    description_pkg = get_package_share_directory('rover_description')
    hardware_pkg = get_package_share_directory('rover_hardware')
    
    # 1. Get URDF from rover_description
    urdf_file = os.path.join(description_pkg, 'urdf', 'rover.urdf')
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': robot_description_content}

    # 2. Get Controllers from rover_hardware
    controller_config = os.path.join(hardware_pkg, 'config', 'controllers.yaml')

    # Main ros2_control_node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Controller Spawners
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        diff_drive_spawner,
        joint_broadcaster_spawner
    ])