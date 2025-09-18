# test_nav_path_publisher.py
#
# ⚠️  仅用于测试目的！
# 在生产环境中，真实的Nav2会自动发布/plan话题，无需此模拟器
# This is ONLY for testing purposes!
# In production, real Nav2 will publish /plan topic automatically

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import math # 仅用于可能需要计算角度的场景，如果你直接用四元数则无需

# --- Configuration ---
PUBLISH_TOPIC = '/plan' # Nav2 通常发布到 /plan, 但也可以是其他自定义话题
PUBLISH_RATE_HZ = 10.0 # Publish 0.5 times per second (每2秒发布一次)
PATH_FRAME_ID = 'map' # 路径的参考系，通常是 'map'

# --- Fixed Path Data ---
# 定义一系列路径点 (PoseStamped)。每个点包含位置和姿态。
# 姿态需要以四元数形式给出。你可以使用在线工具或之前提到的Python片段计算。

# 示例路径点：直线 + 一个转弯
# 点1: (0,0) 位置，Yaw 0度 (q: 0,0,0,1)
# 点2: (1,0) 位置，Yaw 0度 (q: 0,0,0,1)
# 点3: (2,0) 位置，Yaw 0度 (q: 0,0,0,1)
# 点4: (3,1) 位置，Yaw 45度 (q: 0,0,0.3826834,0.9238795)
# 点5: (3,2) 位置，Yaw 90度 (q: 0,0,0.7071068,0.7071068)


FIXED_PATH_POSES = [
    # Point 1: (x,y,z), (qx,qy,qz,qw)
    {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
    # Point 2
    {'position': {'x': 1.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
    # Point 3
    {'position': {'x': 2.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
    # Point 4 (转向)
    {'position': {'x': 3.0, 'y': 1.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.3826834, 'w': 0.9238795}}, # Yaw 45 deg
    # Point 5 (继续转向)
    {'position': {'x': 3.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.7071068, 'w': 0.7071068}}, # Yaw 90 deg
    {'position': {'x': 4.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.7071068, 'w': 0.7071068}}, # Yaw 90 deg
    # 你可以继续添加更多点
]
# --- End Configuration ---

class SimpleNavPathPublisher(Node):

    def __init__(self):
        super().__init__('simple_nav_path_publisher_node')
        self.publisher_ = self.create_publisher(Path, PUBLISH_TOPIC, 10)
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_path)
        self.get_logger().info(f'"{PUBLISH_TOPIC}" Path Publisher started, publishing at {PUBLISH_RATE_HZ} Hz.')

        self.path_msg = self._create_fixed_path_message()

    def _create_fixed_path_message(self):
        path_msg = Path()
        path_msg.header.frame_id = PATH_FRAME_ID

        for p_data in FIXED_PATH_POSES:
            pose_stamped = PoseStamped()
            # PoseStamped 自身的 header，也可以设置为与 Path 消息的 header 相同
            pose_stamped.header.frame_id = PATH_FRAME_ID

            # Position
            pose_stamped.pose.position = Point(
                x=p_data['position']['x'],
                y=p_data['position']['y'],
                z=p_data['position']['z']
            )
            # Orientation (Quaternion)
            pose_stamped.pose.orientation = Quaternion(
                x=p_data['orientation']['x'],
                y=p_data['orientation']['y'],
                z=p_data['orientation']['z'],
                w=p_data['orientation']['w']
            )
            path_msg.poses.append(pose_stamped)
        return path_msg

    def publish_path(self):
        # 每次发布时更新时间戳
        current_time = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = current_time
        # 更新路径中每个 PoseStamped 的时间戳
        for pose_stamped in self.path_msg.poses:
            pose_stamped.header.stamp = current_time

        self.publisher_.publish(self.path_msg)
        self.get_logger().debug(f'Published fixed Path message with {len(self.path_msg.poses)} poses.')


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
