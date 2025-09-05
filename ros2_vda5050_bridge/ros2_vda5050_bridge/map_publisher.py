#!/usr/bin/env python3
"""
Map Publisher - Simulates shared map between ROS2 and VDA5050
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import numpy as np


class MapPublisher(Node):
    """Publishes a shared map for ROS2 and VDA5050"""
    
    def __init__(self):
        super().__init__('map_publisher')
        
        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Timer for publishing map
        self.timer = self.create_timer(5.0, self.publish_map)
        
        self.get_logger().info("Map Publisher initialized")
    
    def publish_map(self):
        """Publish a test map"""
        map_msg = OccupancyGrid()
        
        # Map metadata
        map_msg.header.frame_id = "map"
        map_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Map info
        map_msg.info.resolution = 0.1  # 10cm per pixel
        map_msg.info.width = 100       # 10m x 10m map
        map_msg.info.height = 100
        
        # Map origin (bottom-left corner)
        map_msg.info.origin.position.x = -5.0
        map_msg.info.origin.position.y = -5.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Create a simple test map
        # 0 = free space, 100 = occupied, -1 = unknown
        map_data = np.zeros((map_msg.info.height, map_msg.info.width), dtype=np.int8)
        
        # Add some obstacles (walls)
        # Outer walls
        map_data[0, :] = 100    # Bottom wall
        map_data[-1, :] = 100   # Top wall
        map_data[:, 0] = 100    # Left wall
        map_data[:, -1] = 100   # Right wall
        
        # Add some internal obstacles
        # Vertical wall in the middle
        map_data[20:80, 50] = 100
        
        # Horizontal wall
        map_data[60, 10:40] = 100
        
        # Small room
        map_data[30:50, 70:90] = 100
        map_data[35:45, 75:85] = 0  # Door opening
        
        # Convert to 1D array (row-major order)
        map_msg.data = map_data.flatten().tolist()
        
        # Publish map
        self.map_pub.publish(map_msg)
        self.get_logger().info(f"Published map: {map_msg.info.width}x{map_msg.info.height}, resolution: {map_msg.info.resolution}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        publisher = MapPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()