cmake_minimum_required(VERSION 3.8)
project(gripper_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 2. Build the Shared Library
# The name 'gripper_hardware_lib' must match the <library path="..."> in your gripper_hardware.xml
add_library(gripper_hardware_lib SHARED
  src/gripper_hardware_interface.cpp
)

# 3. Include Directories
target_include_directories(gripper_hardware_lib PRIVATE
  include
)

# 4. Link Dependencies
target_link_libraries(gripper_hardware_lib
  PUBLIC
    hardware_interface::hardware_interface
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
)

# 5. Export the Plugin (CRITICAL STEP)
# This macro tells ROS 2 that this package contains a hardware interface plugin
pluginlib_export_plugin_description_file(hardware_interface gripper_hardware.xml)

# 6. Install Logic
install(TARGETS gripper_hardware_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# This is necessary for the build system to correctly bundle everything
ament_package()