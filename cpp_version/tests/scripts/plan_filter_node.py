import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy

class GoalBasedPlanCollector(Node):
    """
    【最终解决方案】
    本节点监听/goal_pose。当新目标发布时，它会进入“等待”状态，
    捕获接下来收到的第一条/plan，将其发布到/plans，然后停止等待，
    完全忽略之后由于定位波动而产生的所有路径更新。
    """
    def __init__(self):
        super().__init__('goal_based_plan_collector')
        
        # 一个标志位，用于判断我们是否在等待一个新的计划
        self.waiting_for_new_plan = False
        
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.plan_publisher = self.create_publisher(Path, '/plans', qos_profile)
        
        # 订阅/plan，用于捕获路径
        self.plan_subscriber = self.create_subscription(
            Path, '/plan', self.plan_callback, 10)
            
        # 订阅/goal_pose，作为触发器
        self.goal_subscriber = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
            
        self.get_logger().info('目标驱动的路径收集器已启动。')
        self.get_logger().info('请在RViz中发布一个新的目标点...')

    def goal_callback(self, msg):
        """当一个新目标被发布时，此函数被调用"""
        self.get_logger().info('接收到新的目标点！现在开始等待下一条路径规划...')
        # 设置标志位，表示我们期待一条新的路径
        self.waiting_for_new_plan = True

    def plan_callback(self, msg):
        """当/plan发布路径时，此函数被调用"""
        # 只有当我们正在等待一条新路径时，才处理它
        if self.waiting_for_new_plan:
            self.get_logger().info('已成功捕获到与新目标对应的路径！正在发布到/plans...')
            self.plan_publisher.publish(msg)
            
            # 任务完成，重置标志位，忽略后续的路径微小更新
            self.waiting_for_new_plan = False

def main(args=None):
    rclpy.init(args=args)
    node = GoalBasedPlanCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
