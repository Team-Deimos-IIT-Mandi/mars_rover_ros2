import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'rover_description'
    pkg_share = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')

    robot_description_config = Command(['xacro ',xacro_file])
    params = {'robot_description': robot_description_config, 'use_sim_time': True}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params],
        output='screen'
    )

    # Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mars_rover'],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Diff Drive Controller 
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )
    
    # Bridge (clock + TF)
    bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    ],
    output='screen'
    )
    camera_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        # Front depth camera
        '/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/front_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

        # Right RGB camera
        '/right_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/right_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

        # Left RGB camera
        '/left_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/left_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    ],
    output='screen'
    )
    imu_gps_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        # IMU
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

        # GPS
        '/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
    ],
    output='screen'
    )
    

    return LaunchDescription([
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        camera_bridge,   
        imu_gps_bridge,
        
    ])  