import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy
import math

TOLERANCE = 1e-6

def are_paths_geometrically_close(poses1, poses2, node_logger):
    """
    【诊断版本】
    当路径被判断为不同时，会打印出具体的差异点和数值。
    """
    if poses1 is None or poses2 is None:
        return False

    if len(poses1) != len(poses2):
        node_logger.warn(f"路径点数量不同: {len(poses1)} vs {len(poses2)}")
        return False

    for i, (pose1, pose2) in enumerate(zip(poses1, poses2)):
        pos1 = pose1.position
        pos2 = pose2.position
        if not math.isclose(pos1.x, pos2.x, rel_tol=TOLERANCE, abs_tol=TOLERANCE) or \
           not math.isclose(pos1.y, pos2.y, rel_tol=TOLERANCE, abs_tol=TOLERANCE):
            node_logger.warn(f"--- 差异点索引: {i} ---")
            node_logger.warn(f"  旧位置(x, y): ({pos1.x}, {pos1.y})")
            node_logger.warn(f"  新位置(x, y): ({pos2.x}, {pos2.y})")
            return False

        ori1 = pose1.orientation
        ori2 = pose2.orientation
        if not math.isclose(ori1.z, ori2.z, rel_tol=TOLERANCE, abs_tol=TOLERANCE) or \
           not math.isclose(ori1.w, ori2.w, rel_tol=TOLERANCE, abs_tol=TOLERANCE):
            node_logger.warn(f"--- 差异点索引: {i} ---")
            node_logger.warn(f"  旧姿态(z, w): ({ori1.z}, {ori1.w})")
            node_logger.warn(f"  新姿态(z, w): ({ori2.z}, {ori2.w})")
            return False
            
    return True


class PlanFilterNode(Node):
    def __init__(self):
        super().__init__('plan_filter_node')
        self.last_geometric_poses = None
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(Path, '/plans', qos_profile)
        self.subscription = self.create_subscription(
            Path, '/plan', self.listener_callback, 10)
        self.get_logger().info('路径过滤器节点已启动（诊断版本），正在监听 /plan ...')

    def listener_callback(self, new_plan_msg):
        new_geometric_poses = [ps.pose for ps in new_plan_msg.poses]
        
        if not are_paths_geometrically_close(self.last_geometric_poses, new_geometric_poses, self.get_logger()):
            self.get_logger().info(f'检测到新的几何路径({len(new_plan_msg.poses)}个点)，正在发布到 /plans ...')
            self.publisher_.publish(new_plan_msg)
            self.last_geometric_poses = new_geometric_poses

# ... (main function is the same) ...
def main(args=None):
    rclpy.init(args=args)
    plan_filter_node = PlanFilterNode()
    try:
        rclpy.spin(plan_filter_node)
    except KeyboardInterrupt:
        pass
    finally:
        plan_filter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
