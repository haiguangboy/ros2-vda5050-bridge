#!/usr/bin/env python3
"""
ROS2 Path Publisher - Simulates Nav2 path planning
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import time


class PathPublisher(Node):
    """Publishes planned paths to simulate Nav2 behavior"""
    
    def __init__(self):
        super().__init__('path_publisher')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.nav_result_pub = self.create_publisher(String, '/navigation_result', 10)
        
        # Timer for publishing test paths
        self.timer = self.create_timer(10.0, self.publish_test_path)
        
        self.path_counter = 0
        
        self.get_logger().info("Path Publisher initialized")
    
    def publish_test_path(self):
        """Publish a test path"""
        self.path_counter += 1
        
        # Create different path patterns
        if self.path_counter % 3 == 1:
            path = self.create_straight_path()
            self.get_logger().info("Publishing straight line path")
        elif self.path_counter % 3 == 2:
            path = self.create_curved_path()
            self.get_logger().info("Publishing curved path")
        else:
            path = self.create_square_path()
            self.get_logger().info("Publishing square path")
        
        # Publish path
        self.path_pub.publish(path)
        
        # Simulate navigation completion after 8 seconds
        self.create_timer(8.0, self.publish_navigation_success)
    
    def create_straight_path(self) -> Path:
        """Create a straight line path"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Create waypoints from (0,0) to (5,0)
        num_points = 20
        for i in range(num_points):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path.header.stamp
            
            # Linear interpolation
            t = i / (num_points - 1)
            pose.pose.position.x = t * 5.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            
            # Orientation pointing forward
            pose.pose.orientation.w = 1.0
            
            path.poses.append(pose)
        
        return path
    
    def create_curved_path(self) -> Path:
        """Create a curved path (quarter circle)"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Create quarter circle from (0,0) to (3,3)
        num_points = 25
        radius = 3.0
        
        for i in range(num_points):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path.header.stamp
            
            # Parametric circle
            t = i / (num_points - 1)
            angle = t * math.pi / 2  # Quarter circle
            
            pose.pose.position.x = radius * (1 - math.cos(angle))
            pose.pose.position.y = radius * math.sin(angle)
            pose.pose.position.z = 0.0
            
            # Orientation tangent to curve
            yaw = angle + math.pi / 2
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            path.poses.append(pose)
        
        return path
    
    def create_square_path(self) -> Path:
        """Create a square path"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Square corners
        corners = [
            (0.0, 0.0, 0.0),      # Start
            (2.0, 0.0, 0.0),      # Right
            (2.0, 2.0, math.pi/2), # Up
            (0.0, 2.0, math.pi),   # Left
            (0.0, 0.0, -math.pi/2) # Down (back to start)
        ]
        
        points_per_side = 8
        
        for i in range(len(corners) - 1):
            start = corners[i]
            end = corners[i + 1]
            
            for j in range(points_per_side):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = path.header.stamp
                
                # Linear interpolation between corners
                t = j / (points_per_side - 1) if j < points_per_side - 1 else 1.0
                
                pose.pose.position.x = start[0] + t * (end[0] - start[0])
                pose.pose.position.y = start[1] + t * (end[1] - start[1])
                pose.pose.position.z = 0.0
                
                # Interpolate orientation
                yaw = start[2] + t * (end[2] - start[2])
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                
                path.poses.append(pose)
        
        return path
    
    def publish_navigation_success(self):
        """Publish navigation success result"""
        result_msg = String()
        result_msg.data = "SUCCEEDED"
        self.nav_result_pub.publish(result_msg)
        self.get_logger().info("Published navigation SUCCESS result")
    
    def publish_navigation_failure(self):
        """Publish navigation failure result"""
        result_msg = String()
        result_msg.data = "FAILED"
        self.nav_result_pub.publish(result_msg)
        self.get_logger().info("Published navigation FAILURE result")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        publisher = PathPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()