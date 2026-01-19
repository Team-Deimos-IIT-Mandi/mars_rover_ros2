"""
ROS Environment Setup Script
Initializes the ROS environment and starts necessary services for the Kinova robot control system.
"""

import subprocess
import sys
import time
import os
from pathlib import Path

def run_command(command, description=""):
    """Execute a shell command and handle errors."""
    print(f"Running: {description or command}")
    try:
        result = subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
        print(f"✓ Success: {description or command}")
        return result.stdout
    except subprocess.CalledProcessError as e:
        print(f"✗ Error: {description or command}")
        print(f"  {e.stderr}")
        return None

def check_ros_installation():
    """Check if ROS is properly installed."""
    print("Checking ROS installation...")
    
    # Check if ROS is sourced
    ros_distro = os.environ.get('ROS_DISTRO')
    if not ros_distro:
        print("✗ ROS not sourced. Please run: source /opt/ros/melodic/setup.bash")
        return False
    
    print(f"✓ ROS {ros_distro} detected")
    
    # Check roscore
    result = run_command("pgrep roscore", "Checking for running roscore")
    if not result:
        print("Starting roscore...")
        subprocess.Popen(["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(3)
    else:
        print("✓ roscore already running")
    
    return True

def setup_kinova_driver():
    """Setup Kinova robot driver."""
    print("\nSetting up Kinova driver...")
    
    # Check if kinova_driver package exists
    result = run_command("rospack find kinova_driver", "Finding kinova_driver package")
    if not result:
        print("✗ kinova_driver package not found")
        print("  Please install: sudo apt-get install ros-$ROS_DISTRO-kinova-ros")
        return False
    
    print("✓ kinova_driver package found")
    return True

def setup_rosbridge():
    """Setup ROSBridge server for web interface."""
    print("\nSetting up ROSBridge server...")
    
    # Check if rosbridge_server is installed
    result = run_command("rospack find rosbridge_server", "Finding rosbridge_server package")
    if not result:
        print("✗ rosbridge_server package not found")
        print("  Please install: sudo apt-get install ros-$ROS_DISTRO-rosbridge-suite")
        return False
    
    # Start rosbridge server
    print("Starting ROSBridge server on port 9090...")
    subprocess.Popen([
        "roslaunch", "rosbridge_server", "rosbridge_websocket.launch", 
        "port:=9090"
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    time.sleep(2)
    print("✓ ROSBridge server started")
    return True

def setup_vision_system():
    """Setup camera and vision processing."""
    print("\nSetting up vision system...")
    
    # Check for USB camera
    result = run_command("ls /dev/video*", "Checking for cameras")
    if result:
        print("✓ Camera devices found")
        
        # Start USB camera node
        subprocess.Popen([
            "rosrun", "usb_cam", "usb_cam_node", 
            "_video_device:=/dev/video0",
            "_image_width:=640",
            "_image_height:=480",
            "_pixel_format:=yuyv"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        print("✓ Camera node started")
    else:
        print("⚠ No camera devices found")
    
    return True

def create_launch_files():
    """Create custom launch files for the system."""
    print("\nCreating launch files...")
    
    # Create launch directory
    launch_dir = Path("launch")
    launch_dir.mkdir(exist_ok=True)
    
    # Main system launch file
    system_launch = """<?xml version="1.0"?>
<launch>
  <!-- Kinova Robot Driver -->
  <include file="$(find kinova_driver)/launch/kinova_robot.launch">
    <arg name="kinova_robotType" value="j2n6s300"/>
    <arg name="use_urdf" value="true"/>
  </include>
  
  <!-- ROSBridge Server -->
  <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch">
    <arg name="port" value="9090"/>
  </include>
  
  <!-- Vision System -->
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
    <param name="video_device" value="/dev/video0"/>
    <param name="image_width" value="640"/>
    <param name="image_height" value="480"/>
    <param name="pixel_format" value="yuyv"/>
    <param name="camera_frame_id" value="usb_cam"/>
    <param name="io_method" value="mmap"/>
  </node>
  
  <!-- Robot State Publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
  
  <!-- Joint State Publisher -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"/>
  
</launch>
"""
    
    with open(launch_dir / "system.launch", "w") as f:
        f.write(system_launch)
    
    print("✓ Launch files created")
    return True

def main():
    """Main setup function."""
    print("=== ROS Kinova Robot Setup ===\n")
    
    # Check ROS installation
    if not check_ros_installation():
        sys.exit(1)
    
    # Setup components
    setup_kinova_driver()
    setup_rosbridge()
    setup_vision_system()
    create_launch_files()
    
    print("\n=== Setup Complete ===")
    print("To start the full system, run:")
    print("  roslaunch launch/system.launch")
    print("\nTo access the web interface:")
    print("  Open http://localhost:3000 in your browser")
    print("\nROSBridge WebSocket available at:")
    print("  ws://localhost:9090")

if __name__ == "__main__":
    main()
