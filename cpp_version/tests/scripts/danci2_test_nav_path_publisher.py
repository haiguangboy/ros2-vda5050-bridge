# test_nav_path_publisher.py
#
# ⚠️  仅用于测试目的！
# 在生产环境中，真实的Nav2会自动发布/plan话题，无需此模拟器
# This is ONLY for testing purposes!
# In production, real Nav2 will publish /plan topic automatically

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import math # 仅用于可能需要计算角度的场景，如果你直接用四元数则无需

# --- Configuration ---
PUBLISH_TOPIC = '/plans' # Nav2 通常发布到 /plan, 但也可以是其他自定义话题
ODOM_TOPIC = '/Odom'   # 里程计话题，获取当前位置
PATH_FRAME_ID = 'map' # 路径的参考系，通常是 'map'
NUM_PATH_POINTS = 2   # 生成的路径点数量
FORWARD_DISTANCE = 2.0  # 沿着朝向前进的距离（米）

# --- End Configuration ---

class SimpleNavPathPublisher(Node):

    def __init__(self):
        super().__init__('simple_nav_path_publisher_node')

        # 创建发布器和订阅器
        self.publisher_ = self.create_publisher(Path, PUBLISH_TOPIC, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            10
        )

        # 标志变量，确保只发布一次
        self.path_published = False
        self.current_pose = None

        self.get_logger().info(f'Path Publisher started. Waiting for odom data from "{ODOM_TOPIC}"...')

    def odom_callback(self, msg):
        # 获取当前位置，如果还没有发布过路径，则发布一次
        if not self.path_published:
            self.current_pose = msg.pose.pose
            self.generate_and_publish_path()
            self.path_published = True
            self.get_logger().info('Path published once. Node will continue running for visualization.')

    def quaternion_to_yaw(self, q):
        """将四元数转换为yaw角度（弧度）"""
        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def generate_and_publish_path(self):
        if self.current_pose is None:
            self.get_logger().warn('No odom data received yet.')
            return

        # 创建路径消息
        path_msg = Path()
        path_msg.header.frame_id = PATH_FRAME_ID
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # 获取当前位置作为基准
        base_x = self.current_pose.position.x
        base_y = self.current_pose.position.y
        base_z = self.current_pose.position.z
        base_orientation = self.current_pose.orientation

        # 计算基准yaw角
        base_yaw = self.quaternion_to_yaw(base_orientation)

        # 存储路径点信息用于打印
        path_points = []

        # 生成路径点
        for i in range(NUM_PATH_POINTS):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = PATH_FRAME_ID
            pose_stamped.header.stamp = path_msg.header.stamp

            # 计算沿着当前朝向前进的距离
            forward_dist = i * FORWARD_DISTANCE

            # 根据yaw角计算新的位置
            # x方向：cos(yaw) * distance
            # y方向：sin(yaw) * distance
            current_x = base_x + forward_dist * math.cos(base_yaw)
            current_y = base_y + forward_dist * math.sin(base_yaw)

            pose_stamped.pose.position = Point(
                x=current_x,
                y=current_y,
                z=base_z
            )

            # 姿态：与第一个点保持相同（保持同一朝向）
            pose_stamped.pose.orientation = Quaternion(
                x=base_orientation.x,
                y=base_orientation.y,
                z=base_orientation.z,
                w=base_orientation.w
            )

            path_msg.poses.append(pose_stamped)

            # 存储点信息
            path_points.append({
                'index': i,
                'x': current_x,
                'y': current_y,
                'z': base_z,
                'yaw': base_yaw,
                'forward_distance': forward_dist,
                'quaternion': {
                    'x': base_orientation.x,
                    'y': base_orientation.y,
                    'z': base_orientation.z,
                    'w': base_orientation.w
                }
            })

        # 发布路径
        self.publisher_.publish(path_msg)

        # 打印详细的路径信息
        print("=" * 60)
        print("PATH INFORMATION")
        print("=" * 60)

        # 打印起点信息
        start_point = path_points[0]
        print(f"起点 (Start Point):")
        print(f"  Position: x={start_point['x']:.3f}, y={start_point['y']:.3f}")
        print(f"  Yaw: {start_point['yaw']:.3f} rad ({math.degrees(start_point['yaw']):.1f}°)")
        print(f"  Forward Distance: {start_point['forward_distance']:.1f} m")
        print(f"  Quaternion: x={start_point['quaternion']['x']:.3f}, y={start_point['quaternion']['y']:.3f}, "
              f"z={start_point['quaternion']['z']:.3f}, w={start_point['quaternion']['w']:.3f}")

        # 打印终点信息
        end_point = path_points[-1]
        print(f"\n终点 (End Point):")
        print(f"  Position: x={end_point['x']:.3f}, y={end_point['y']:.3f}")
        print(f"  Yaw: {end_point['yaw']:.3f} rad ({math.degrees(end_point['yaw']):.1f}°)")
        print(f"  Forward Distance: {end_point['forward_distance']:.1f} m")
        print(f"  Quaternion: x={end_point['quaternion']['x']:.3f}, y={end_point['quaternion']['y']:.3f}, "
              f"z={end_point['quaternion']['z']:.3f}, w={end_point['quaternion']['w']:.3f}")

        # 计算实际移动距离
        actual_distance = math.sqrt((end_point['x'] - start_point['x'])**2 +
                                   (end_point['y'] - start_point['y'])**2)

        print(f"\n路径统计 (Path Statistics):")
        print(f"  设定前进距离: {FORWARD_DISTANCE:.1f} m")
        print(f"  实际移动距离: {actual_distance:.3f} m")
        print(f"  朝向角度: {math.degrees(base_yaw):.1f}°")

        # 打印所有路径点
        print(f"\n所有路径点 (All Path Points): {len(path_points)} points")
        print("-" * 60)
        for point in path_points:
            print(f"Point {point['index']}: (x={point['x']:.3f}, y={point['y']:.3f}, "
                  f"forward={point['forward_distance']:.1f}m, yaw={point['yaw']:.3f} rad)")

        print("=" * 60)

        self.get_logger().info(f'Published path with {len(path_msg.poses)} poses. '
                              f'Forward distance: {FORWARD_DISTANCE}m, '
                              f'Direction: {math.degrees(base_yaw):.1f}°')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # 允许Ctrl+C退出
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
