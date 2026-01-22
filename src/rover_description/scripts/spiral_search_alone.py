#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math
import numpy as np

class SpiralWaypointGenerator(Node):
    def __init__(self):
        super().__init__('spiral_waypoint_generator')

        # Parameters
        self.threshold_distance = 1.5  # Distance (meters) to consider a waypoint reached
        self.initial_pose = None       # Set automatically on first odom
        self.spiral_radius = 30.0      # Total distance from center
        self.step_increment = 0.2      # Density of points (0.2 is smooth for Nav2)
        self.waypoints = []
        
        self.current_position = None

        # Subscribers
        # Listens to odometry to determine start location and track progress
        self.odom_subscriber = self.create_subscription(
            Odometry, 
            '/odometry/local', 
            self.odom_callback, 
            10)
        
        # Publishers
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.marker_publisher = self.create_publisher(Marker, '/waypoints_marker', 10)

        # Timer: Checks distance and manages the mission every 500ms
        self.timer = self.create_timer(0.5, self.control_loop)

        self.get_logger().info("Spiral Search Node Initialized. Awaiting Odometry to begin...")

    def generate_spiral_waypoints(self):
        """Generates an Archimedean spiral starting from the current initial_pose."""
        waypoints = []
        t = 0.0
        while t <= self.spiral_radius:
            # Formula: r = a * theta
            # x = center_x + (a * theta * cos(theta))
            x = self.initial_pose.x + (math.cos(t) * t / 3.0)
            y = self.initial_pose.y + (math.sin(t) * t / 3.0)
            
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'odom'
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.pose.position = Point(x=x, y=y, z=0.0)
            
            # Orientation: Pointing toward the next curve in the spiral
            tangent_angle = math.atan2(y - self.initial_pose.y, x - self.initial_pose.x)
            q = self.euler_to_quaternion(0, 0, tangent_angle)
            
            waypoint.pose.orientation.x = q[0]
            waypoint.pose.orientation.y = q[1]
            waypoint.pose.orientation.z = q[2]
            waypoint.pose.orientation.w = q[3]

            waypoints.append(waypoint)
            t += self.step_increment 
        
        return waypoints

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Helper to convert euler angles to geometry_msgs Quaternion."""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def publish_markers(self):
        """Visualizes the remaining path in RViz."""
        if not self.waypoints: return
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'spiral_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width
        marker.color.a = 1.0
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0 # Cyan line

        for wp in self.waypoints:
            marker.points.append(wp.pose.position)

        self.marker_publisher.publish(marker)

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

    def control_loop(self):
        # Wait for the first odom message
        if self.current_position is None:
            return

        # Phase 1: Initialize Mission
        if self.initial_pose is None:
            self.initial_pose = self.current_position
            self.waypoints = self.generate_spiral_waypoints()
            self.get_logger().info(f"Center fixed at: {self.initial_pose.x:.2f}, {self.initial_pose.y:.2f}")
            self.get_logger().info(f"Spiral generated with {len(self.waypoints)} points. Starting now.")
            
            # Send the first goal immediately
            if self.waypoints:
                first_goal = self.waypoints.pop(0)
                first_goal.header.stamp = self.get_clock().now().to_msg()
                self.goal_publisher.publish(first_goal)

        self.publish_markers()
        
        # Phase 2: Mission Tracking
        if self.waypoints:
            next_wp = self.waypoints[0].pose.position
            dx = self.current_position.x - next_wp.x
            dy = self.current_position.y - next_wp.y
            distance = math.sqrt(dx**2 + dy**2)
            
            # If robot is within threshold, pop the next waypoint and send to Nav2
            if distance < self.threshold_distance:
                goal_msg = self.waypoints.pop(0)
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                self.goal_publisher.publish(goal_msg)
                self.get_logger().info(f"Target reached. {len(self.waypoints)} waypoints remaining.")
        else:
            self.get_logger().info("Spiral Search Complete. All waypoints processed.", once=True)

def main(args=None):
    rclpy.init(args=args)
    node = SpiralWaypointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()