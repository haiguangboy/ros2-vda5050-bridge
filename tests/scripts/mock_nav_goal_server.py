#!/usr/bin/env python3
"""
Mock GoToPose service server + /Odom publisher for testing.

- Provides `/nav/go_to_pose` (forklift_interfaces/srv/GoToPose)
- Publishes `/Odom` at a fixed rate with configurable pose
- Prints incoming GoToPose requests and returns a configurable response
"""

import math
import argparse
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# Try to import the generated GoToPose type. If unavailable, run in Odom-only mode.
SERVICE_AVAILABLE = True
try:
    from forklift_interfaces.srv import GoToPose  # type: ignore
except Exception:
    GoToPose = None  # type: ignore
    SERVICE_AVAILABLE = False


def euler_to_q(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class MockNavServer(Node):
    def __init__(self, odom_x: float, odom_y: float, odom_yaw: float, odom_rate: float,
                 target_x: float, target_y: float, target_yaw: float,
                 arrived: bool, message: str):
        super().__init__('mock_nav_server')
        self.odom_x = odom_x
        self.odom_y = odom_y
        self.odom_yaw = odom_yaw
        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw = target_yaw
        self.arrived = arrived
        self.message = message

        # Publisher: /Odom (publish once)
        self.pub_odom = self.create_publisher(Odometry, '/Odom', 10)
        self.publish_odom()

        # Service: /nav/go_to_pose (only if type is available)
        if SERVICE_AVAILABLE and GoToPose is not None:
            self.srv_nav = self.create_service(GoToPose, '/nav/go_to_pose', self.on_nav_request)
            self.get_logger().info('MockNavServer ready: /Odom + /nav/go_to_pose')
        else:
            self.srv_nav = None
            self.get_logger().warn('forklift_interfaces.srv.GoToPose not found. Running in Odom-only mode.\n'
                                   'Please build and source the workspace (colcon build; source install/setup.bash)\n'
                                   'to enable the /nav/go_to_pose service.')

        # Print target info (CLI fallback if service type missing)
        deg = math.degrees(self.target_yaw)
        self.get_logger().info(f"目标点(模拟): x={self.target_x:.3f}, y={self.target_y:.3f}, yaw={deg:.1f}°")

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        # msg.header.child.frame_id = 'base_link'
        msg.pose.pose.position.x = self.odom_x
        msg.pose.pose.position.y = self.odom_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = euler_to_q(0.0, 0.0, self.odom_yaw)
        self.pub_odom.publish(msg)
        # Print published /Odom values
        self.get_logger().info(
            f"发布 /Odom: x={self.odom_x:.3f}, y={self.odom_y:.3f}, yaw={math.degrees(self.odom_yaw):.1f}°")

    def on_nav_request(self, req: GoToPose.Request, resp: GoToPose.Response):
        yaw = self.quaternion_to_yaw(req.target.pose.orientation)
        self.get_logger().info(
            f"GoToPose request: mode={req.mode}, target=({req.target.pose.position.x:.3f}, "
            f"{req.target.pose.position.y:.3f}), yaw={math.degrees(yaw):.1f}°, timeout={req.timeout_sec:.1f}s")
        # Also print target point summary
        self.get_logger().info(
            f"目标点(服务): x={req.target.pose.position.x:.3f}, y={req.target.pose.position.y:.3f}, yaw={math.degrees(yaw):.1f}°")
        resp.arrived = self.arrived
        resp.message = self.message or 'mock: accepted'
        return resp

    @staticmethod
    def quaternion_to_yaw(q: Quaternion) -> float:
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main():
    parser = argparse.ArgumentParser(description='Mock GoToPose server + /Odom publisher')
    parser.add_argument('--odom-x', type=float, default=1.0)
    parser.add_argument('--odom-y', type=float, default=0.0)
    parser.add_argument('--odom-yaw', type=float, default=0.3, help='Yaw in radians')
    parser.add_argument('--odom-rate', type=float, default=10.0, help='Publish rate (Hz)')
    parser.add_argument('--target-x', type=float, default=3.0)
    parser.add_argument('--target-y', type=float, default=0.0)
    parser.add_argument('--target-yaw', type=float, default=0.3, help='Yaw in radians')
    parser.add_argument('--arrived', action='store_true', help='Respond arrived=true')
    parser.add_argument('--message', type=str, default='')
    args = parser.parse_args()

    rclpy.init()
    node = MockNavServer(
        args.odom_x, args.odom_y, args.odom_yaw, args.odom_rate,
        args.target_x, args.target_y, args.target_yaw,
        args.arrived, args.message)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
