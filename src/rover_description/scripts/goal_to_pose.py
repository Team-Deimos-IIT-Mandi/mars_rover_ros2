#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class GpsToNavGoal(Node):
    def __init__(self):
        super().__init__('gps_waypoint_listener')
        
        # ROS 2 uses /goal_pose instead of /move_base_simple/goal
        self.pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscriber to your GPS-converted odometry
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/gps_point', 
            self.odom_callback, 
            10)
            
        self.get_logger().info("GPS Waypoint Listener Node Started")

    def odom_callback(self, msg):
        # Create a PoseStamped message for Nav2
        goal_msg = PoseStamped()
        
        # Use the header from the incoming message (ensure frame is 'map')
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map' 
        
        # Set position and orientation
        goal_msg.pose.position = msg.pose.pose.position
        goal_msg.pose.position.z = 0.0 # Keep it on the ground
        goal_msg.pose.orientation = msg.pose.pose.orientation

        # Publish the goal
        self.get_logger().info(f"Publishing Nav2 Goal from GPS: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}")
        self.pose_pub.publish(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsToNavGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()