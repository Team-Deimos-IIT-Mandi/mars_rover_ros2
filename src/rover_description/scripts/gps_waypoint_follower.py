#!/usr/bin/env python3
"""
GPS Waypoint Follower for Mars Rover (Nav2 Humble Compatible)
Converts GPS coordinates to cartesian and sends to Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import yaml
import sys
import math


class GPSWaypointFollower(Node):
    def __init__(self, waypoints_file=None):
        super().__init__('gps_waypoint_follower')
        
        # Action client for following waypoints
        self.action_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )
        
        # Subscribe to GPS to get datum and convert coordinates
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        # Subscribe to converted GPS odometry to get cartesian coordinates
        self.gps_odom_poses = []
        self.datum_set = False
        self.current_gps = None
        
        self.get_logger().info('Waiting for GPS fix to establish datum...')
        
        # Load waypoints
        if waypoints_file:
            self.waypoints = self.load_waypoints(waypoints_file)
        else:
            # Default demo waypoints (modify for your location)
            self.waypoints = [
                {'latitude': 49.9001, 'longitude': 8.9001, 'yaw': 0.0},
                {'latitude': 49.9002, 'longitude': 8.9002, 'yaw': 1.57},
                {'latitude': 49.9003, 'longitude': 8.9001, 'yaw': 3.14},
            ]
        
        self.get_logger().info(f'Loaded {len(self.waypoints)} GPS waypoints')
        
        # Store datum (first GPS reading)
        self.datum_lat = None
        self.datum_lon = None
        
    def gps_callback(self, msg):
        """Store current GPS fix"""
        self.current_gps = msg
        if not self.datum_set and msg.status.status >= 0:
            self.datum_lat = msg.latitude
            self.datum_lon = msg.longitude
            self.datum_set = True
            self.get_logger().info(f'Datum set: lat={self.datum_lat:.6f}, lon={self.datum_lon:.6f}')
            
    def load_waypoints(self, filepath):
        """Load waypoints from YAML file"""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []
    
    def gps_to_cartesian(self, lat, lon):
        """
        Convert GPS coordinates to local cartesian coordinates using simple approximation.
        For small distances, this is accurate enough.
        """
        if self.datum_lat is None or self.datum_lon is None:
            return None, None
            
        # Earth radius in meters
        R = 6378137.0
        
        # Convert to radians
        lat1 = math.radians(self.datum_lat)
        lon1 = math.radians(self.datum_lon)
        lat2 = math.radians(lat)
        lon2 = math.radians(lon)
        
        # Calculate x, y in meters (local tangent plane approximation)
        # x points East, y points North (ENU convention)
        x = R * (lon2 - lon1) * math.cos((lat1 + lat2) / 2.0)
        y = R * (lat2 - lat1)
        
        return x, y
    
    def create_cartesian_waypoints(self):
        """Convert GPS waypoints to cartesian PoseStamped messages"""
        poses = []
        
        for i, wp in enumerate(self.waypoints):
            x, y = self.gps_to_cartesian(wp['latitude'], wp['longitude'])
            
            if x is None or y is None:
                self.get_logger().error(f'Failed to convert waypoint {i+1} to cartesian')
                continue
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion (2D rotation around z-axis)
            yaw = wp.get('yaw', 0.0)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            poses.append(pose)
            self.get_logger().info(
                f'Waypoint {i+1}: GPS({wp["latitude"]:.6f}, {wp["longitude"]:.6f}) '
                f'-> Cartesian({x:.2f}, {y:.2f}) heading={math.degrees(yaw):.1f}°'
            )
        
        return poses
    
    def send_waypoints(self):
        """Send waypoints to Nav2"""
        # Wait for datum to be set
        timeout = 10.0
        start_time = self.get_clock().now()
        while not self.datum_set:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().error('Timeout waiting for GPS datum!')
                return
        
        # Convert GPS waypoints to cartesian
        cartesian_poses = self.create_cartesian_waypoints()
        
        if len(cartesian_poses) == 0:
            self.get_logger().error('No valid waypoints to send!')
            return
        
        # Wait for action server
        self.get_logger().info('Waiting for FollowWaypoints action server...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('FollowWaypoints action server not available!')
            return
        
        # Create goal message
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = cartesian_poses
        
        self.get_logger().info(f'Sending {len(cartesian_poses)} waypoints to Nav2...')
        
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint goal was rejected!')
            return
        
        self.get_logger().info('Waypoint goal accepted! Robot navigating...')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        
        if len(result.missed_waypoints) == 0:
            self.get_logger().info('✓ Successfully reached all waypoints!')
        else:
            self.get_logger().warn(
                f'Completed with {len(result.missed_waypoints)} missed waypoints'
            )
        
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Progress: waypoint {feedback.current_waypoint + 1}/{len(self.waypoints)}'
        )


def main(args=None):
    rclpy.init(args=args)
    
    waypoints_file = None
    if len(sys.argv) > 1:
        waypoints_file = sys.argv[1]
    
    node = GPSWaypointFollower(waypoints_file)
    
    # Send waypoints
    node.send_waypoints()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
