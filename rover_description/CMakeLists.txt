cmake_minimum_required(VERSION 3.8)
project(rover_description)

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(robot_state_publisher REQUIRED)
# find_package(xacro REQUIRED)
# find_package(tf2_ros REQUIRED)
find_package(gz_ros2_control REQUIRED)

# Install URDF, meshes, config, launch
install(
  DIRECTORY urdf meshes launch config rviz 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
