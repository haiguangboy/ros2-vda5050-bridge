#!/usr/bin/env python3
"""
ComplexTrajectoryPlanner 完整工作流程测试

测试流程：
1. 订阅 /Odom 获取当前位置（起点）
2. 订阅 /nav_goal 获取目标点
3. 根据起点和目标点自动计算转弯角度和前进距离
4. 使用 ComplexTrajectoryPlanner 规划前向轨迹（Traj1 + Traj2）
5. 发布轨迹到 /plans 话题
6. 等待MQTT完成信号
7. 规划并发布后向轨迹（Traj3）
8. 提供 /trajectory_status service 供状态机查询
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import time
import signal
import sys
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from example_interfaces.srv import Trigger
from trajectory_planner import ComplexTrajectoryPlanner


# ==================== 配置参数 ====================

# MQTT配置
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
ROBOT_ID = "robot-001"

# ROS2配置
ODOM_TOPIC = "/Odom"
GOAL_TOPIC = "/nav_goal"
PATH_TOPIC = "/plans"
ODOM_TIMEOUT = 10.0  # 等待Odom超时时间（秒）

# 默认位置（Odom超时时使用）
DEFAULT_X = 0.0
DEFAULT_Y = 0.0
DEFAULT_YAW = 0.0

# 倒车距离（固定参数）
BACKWARD_DISTANCE = 1.0  # 倒车1米

# 容器配置（用于后向轨迹）
CONTAINER_TYPE = "AGV-T300"
CONTAINER_OFFSET_X = 1.0
CONTAINER_OFFSET_Y = 1.0
CONTAINER_Z = 0.1
CONTAINER_THETA = 0.0
CONTAINER_WIDTH = 1.2


# ==================== 测试节点 ====================

class ComplexPlannerTester(Node):
    def __init__(self):
        super().__init__('complex_planner_tester')

        # 创建规划器
        self.planner = ComplexTrajectoryPlanner(
            forward_step=0.15,
            backward_step=0.15
        )

        # MQTT客户端配置
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # ROS2发布器和订阅器
        self.path_publisher = self.create_publisher(Path, PATH_TOPIC, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 10)
        self.goal_subscriber = self.create_subscription(
            PoseStamped, GOAL_TOPIC, self.goal_callback, 10)

        # 状态查询service
        self.status_service = self.create_service(
            Trigger, '/trajectory_status', self.handle_status_query)

        # 状态管理
        self.current_pose = None
        self.goal_pose = None
        self.odom_received = False
        self.goal_received = False
        self.mqtt_complete_received = False
        self.last_published_trajectory_id = None

        # 轨迹状态存储
        self.trajectory_status_dict = {}
        self.latest_status = None

        print("✅ ComplexPlannerTester 节点已启动")
        print(f"   规划器: ComplexTrajectoryPlanner")
        print(f"   Service: /trajectory_status\n")

    def odom_callback(self, msg):
        """里程计回调，持续更新当前位置"""
        self.current_pose = msg.pose.pose

        if not self.odom_received:
            self.odom_received = True
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            print(f"✅ 已接收到 /Odom 话题数据")
            print(f"   当前位置: ({x:.3f}, {y:.3f}), 朝向: {yaw:.3f} ({math.degrees(yaw):.1f}°)\n")

    def goal_callback(self, msg):
        """目标点回调"""
        self.goal_pose = msg.pose

        if not self.goal_received:
            self.goal_received = True
            x = msg.pose.position.x
            y = msg.pose.position.y
            yaw = self.quaternion_to_yaw(msg.pose.orientation)
            print(f"✅ 已接收到目标点 /nav_goal")
            print(f"   目标位置: ({x:.3f}, {y:.3f}), 朝向: {yaw:.3f} ({math.degrees(yaw):.1f}°)\n")

    def wait_for_odom_or_timeout(self, timeout_seconds=ODOM_TIMEOUT):
        """等待Odom话题或超时使用默认数据"""
        print(f"⏳ 等待 {ODOM_TOPIC} 话题数据（最多等待 {timeout_seconds:.0f} 秒）...")

        start_time = time.time()
        while not self.odom_received and (time.time() - start_time) < timeout_seconds:
            rclpy.spin_once(self, timeout_sec=0.5)

        if not self.odom_received:
            print(f"\n⚠️  {timeout_seconds:.0f}秒内未收到 {ODOM_TOPIC} 话题，使用默认位置数据")
            print(f"   默认位置: ({DEFAULT_X:.3f}, {DEFAULT_Y:.3f}), 朝向: {DEFAULT_YAW:.3f} ({math.degrees(DEFAULT_YAW):.1f}°)\n")
            # 创建默认pose
            self.current_pose = Pose()
            self.current_pose.position.x = DEFAULT_X
            self.current_pose.position.y = DEFAULT_Y
            self.current_pose.position.z = 0.0
            self.current_pose.orientation = self.euler_to_quaternion(0.0, 0.0, DEFAULT_YAW)
        else:
            print(f"✅ 成功获取 {ODOM_TOPIC} 话题数据\n")

    def wait_for_goal(self):
        """等待目标点数据"""
        print(f"⏳ 等待目标点 {GOAL_TOPIC} 话题数据...")
        print(f"   无超时限制（按Ctrl+C可中断）\n")

        while not self.goal_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)

        if self.goal_received:
            print(f"✅ 成功获取目标点数据\n")

    def publish_forward_trajectory(self):
        """
        发布前向轨迹（Traj1 + Traj2组合）
        根据起点和目标点自动计算：第一次转弯 → 前进 → 第二次转弯
        Beta-3参数：orientation=0.0, flag=0
        """
        print("\n" + "="*80)
        print("📤 规划并发布前向轨迹（Traj1 + Traj2 组合）")
        print("="*80)

        # 主动获取最新Odom数据
        print(f"📡 获取最新Odom数据...")
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        # 提取起点和终点信息
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        start_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        goal_yaw = self.quaternion_to_yaw(self.goal_pose.orientation)

        print(f"\n📍 起点: ({start_x:.3f}, {start_y:.3f}), yaw={start_yaw:.3f} ({math.degrees(start_yaw):.1f}°)")
        print(f"📍 终点: ({goal_x:.3f}, {goal_y:.3f}), yaw={goal_yaw:.3f} ({math.degrees(goal_yaw):.1f}°)\n")

        # 计算需要的转弯角度和前进距离
        # 策略：根据起点和终点自动计算两次转弯和前进距离
        # 模式：转弯1 → 前进 → 转弯2
        # 目标：第1段轨迹结束后，位置在(goal_x, goal_y - backward_distance * sin(goal_yaw))
        #      朝向等于goal_yaw，这样倒车后能到达终点

        # 由于第2段要倒车，所以第1段的终点应该在倒车前的位置
        # 倒车方向是车尾方向（goal_yaw + π）
        backward_yaw = goal_yaw + math.pi
        intermediate_x = goal_x - BACKWARD_DISTANCE * math.cos(backward_yaw)
        intermediate_y = goal_y - BACKWARD_DISTANCE * math.sin(backward_yaw)

        print(f"📍 中间点（第1段终点，第2段起点）:")
        print(f"   位置: ({intermediate_x:.3f}, {intermediate_y:.3f})")
        print(f"   朝向: {goal_yaw:.3f} ({math.degrees(goal_yaw):.1f}°)\n")

        # 第一次转弯：从起点朝向转到能前进到中间点X坐标的朝向
        # 假设第一次转弯后沿着某个方向前进，能让X坐标对齐
        # 由于题目要求的模式，我们假设：
        # 第一次转弯到0°方向，前进改变X坐标，第二次转弯到goal_yaw

        # 根据你的示例：从90°转到0°，前进1米，再转到-90°
        # 第一次转弯角度：0 - start_yaw
        first_turn_angle = 0 - start_yaw

        yaw_after_first_turn = 0.0  # 转到0°

        # 前进距离：X方向的差值
        forward_distance = intermediate_x - start_x

        # 第二次转弯角度：从0°转到goal_yaw
        second_turn_angle = goal_yaw - yaw_after_first_turn

        print(f"📐 自动计算参数:")
        print(f"   第一次转弯: {first_turn_angle:.3f} rad ({math.degrees(first_turn_angle):.1f}°)")
        print(f"   前进距离: {forward_distance:.3f} m")
        print(f"   第二次转弯: {second_turn_angle:.3f} rad ({math.degrees(second_turn_angle):.1f}°)\n")

        # 验证：计算规划后的终点位置和朝向
        planned_x = start_x + forward_distance * math.cos(yaw_after_first_turn)
        planned_y = start_y + forward_distance * math.sin(yaw_after_first_turn)
        planned_yaw = yaw_after_first_turn + second_turn_angle

        print(f"📍 第1段轨迹终点验证:")
        print(f"   规划终点: ({planned_x:.3f}, {planned_y:.3f}), yaw={planned_yaw:.3f} ({math.degrees(planned_yaw):.1f}°)")
        print(f"   中间点: ({intermediate_x:.3f}, {intermediate_y:.3f}), yaw={goal_yaw:.3f} ({math.degrees(goal_yaw):.1f}°)")

        # 计算误差
        pos_error = math.sqrt((planned_x - intermediate_x)**2 + (planned_y - intermediate_y)**2)
        yaw_error = abs(self._normalize_angle(planned_yaw - goal_yaw))
        print(f"   位置误差: {pos_error:.3f} m")
        print(f"   朝向误差: {yaw_error:.3f} rad ({math.degrees(yaw_error):.1f}°)\n")

        # 使用ComplexTrajectoryPlanner规划
        waypoints = self.planner.plan_forward_with_turns(
            start_pose=self.current_pose,
            first_turn_angle=first_turn_angle,
            forward_distance=forward_distance,
            second_turn_angle=second_turn_angle
        )

        # 生成轨迹ID
        trajectory_id = f"complex_forward_{int(time.time() * 1000)}"
        self.last_published_trajectory_id = trajectory_id

        # 创建Path消息
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        # 前向轨迹：orientation=0.0, flag=0
        path.header.frame_id = f"map|none|none|0.0|0|0|0|0|0|0|{trajectory_id}"

        # 转换为PoseStamped
        poses = []
        for x, y, yaw in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation = self.euler_to_quaternion(0.0, 0.0, yaw)
            poses.append(pose_stamped)

        path.poses = poses

        # 打印详细信息
        print(f"\n✅ 前向轨迹生成完成，共 {len(waypoints)} 个路径点")
        print(f"\n所有路径点:")
        for i, (x, y, yaw) in enumerate(waypoints):
            print(f"  点{i+1}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} ({math.degrees(yaw):.1f}°)")

        print(f"\nBeta-3参数: orientation=0.0, flag=0 (前向运动)")
        print("="*80)

        # 发布轨迹
        self.path_publisher.publish(path)
        print(f"📤 轨迹已发布到 /plans")
        print(f"📋 轨迹ID: {trajectory_id}\n")

    def publish_backward_trajectory(self):
        """
        发布后向轨迹（Traj3）
        流程：倒车0.3m
        Beta-3参数：orientation=3.14, flag=1
        """
        print("\n" + "="*80)
        print("📤 规划并发布后向轨迹（Traj3 倒车）")
        print("="*80)

        # 主动获取最新Odom数据
        print(f"📡 获取最新Odom数据...")
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        # 使用ComplexTrajectoryPlanner规划
        waypoints = self.planner.plan_backward(
            start_pose=self.current_pose,
            backward_distance=BACKWARD_DISTANCE
        )

        # 生成轨迹ID
        trajectory_id = f"complex_backward_{int(time.time() * 1000)}"
        self.last_published_trajectory_id = trajectory_id

        # 容器位姿（相对于当前位置）
        container_x = self.current_pose.position.x + CONTAINER_OFFSET_X
        container_y = self.current_pose.position.y + CONTAINER_OFFSET_Y

        # 创建Path消息
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        # 后向轨迹：orientation=3.14, flag=1，带容器信息
        path.header.frame_id = f"map|pub_unload_params|{CONTAINER_TYPE}|3.14|1|{container_x}|{container_y}|{CONTAINER_Z}|{CONTAINER_THETA}|{CONTAINER_WIDTH}|{trajectory_id}"

        # 转换为PoseStamped
        poses = []
        for x, y, yaw in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation = self.euler_to_quaternion(0.0, 0.0, yaw)
            poses.append(pose_stamped)

        path.poses = poses

        # 打印详细信息
        print(f"\n✅ 后向轨迹生成完成，共 {len(waypoints)} 个路径点")
        print(f"\n所有路径点:")
        for i, (x, y, yaw) in enumerate(waypoints):
            print(f"  点{i+1}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} ({math.degrees(yaw):.1f}°)")

        print(f"\nBeta-3参数: orientation=3.14, flag=1 (倒车运动)")
        print(f"容器信息: type={CONTAINER_TYPE}, pos=({container_x:.3f}, {container_y:.3f})")
        print("="*80)

        # 发布轨迹
        self.path_publisher.publish(path)
        print(f"📤 轨迹已发布到 /plans")
        print(f"📋 轨迹ID: {trajectory_id}\n")

    def handle_status_query(self, request, response):
        """处理轨迹状态查询service"""
        if self.latest_status:
            response.success = True
            response.message = json.dumps({
                'trajectory_id': self.latest_status.get('trajectoryId', 'N/A'),
                'status': self.latest_status.get('status', 'unknown'),
                'timestamp': self.latest_status.get('timestamp', 0),
                'message': self.latest_status.get('message', '')
            })
        else:
            response.success = False
            response.message = json.dumps({
                'trajectory_id': '',
                'status': 'no_data',
                'timestamp': 0,
                'message': '暂无轨迹状态数据'
            })
        return response

    # MQTT回调函数
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT连接成功")
            # 订阅轨迹状态主题
            status_topic = f"EP/{ROBOT_ID}/cerebellum/embrain/trajectory_status"
            client.subscribe(status_topic)
            print(f"📡 订阅轨迹状态主题: {status_topic}\n")
        else:
            print(f"❌ MQTT连接失败，返回代码: {rc}")

    def on_message(self, client, userdata, msg):
        try:
            trajectory_data = json.loads(msg.payload.decode())

            # 检查是否是轨迹状态消息
            if 'trajectory_status' in msg.topic:
                status = trajectory_data.get('status', '')
                trajectory_id = trajectory_data.get('trajectoryId', 'N/A')

                # 存储状态
                self.trajectory_status_dict[trajectory_id] = {
                    'status': status,
                    'timestamp': trajectory_data.get('timestamp'),
                    'message': trajectory_data.get('message', '')
                }
                self.latest_status = trajectory_data

                print("\n" + "="*80)
                print(f"📊 收到MQTT轨迹状态消息")
                print("="*80)
                print(f"📋 轨迹ID: {trajectory_id}")
                print(f"📍 状态: {status}")
                print(f"⏰ 时间戳: {trajectory_data.get('timestamp', 'N/A')}")

                if status == 'completed':
                    print(f"\n✅ 轨迹已完成！")
                    self.mqtt_complete_received = True
                elif status == 'running':
                    print(f"\n🏃 轨迹执行中...")
                elif status == 'failed':
                    print(f"\n❌ 轨迹执行失败！")

                print("="*80 + "\n")

        except json.JSONDecodeError:
            print(f"❌ JSON解析失败: {msg.payload.decode()}")
        except Exception as e:
            print(f"❌ 消息处理错误: {e}")

    def start_mqtt_listener(self):
        """启动MQTT监听器"""
        try:
            print(f"🚀 连接到MQTT代理: {MQTT_BROKER}:{MQTT_PORT}")
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            time.sleep(1)  # 等待连接完成
            return True
        except Exception as e:
            print(f"❌ MQTT连接失败: {e}")
            return False

    def quaternion_to_yaw(self, q):
        """将四元数转换为yaw角度（弧度）"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        from geometry_msgs.msg import Quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q

    def stop(self):
        """停止测试"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        print("\n🛑 测试已停止")


def signal_handler(sig, frame):
    """处理中断信号"""
    print("\n⏹️ 收到中断信号，正在停止测试...")
    global tester
    if tester:
        tester.stop()
    rclpy.shutdown()
    sys.exit(0)


def main():
    global tester

    print("🧪 ComplexTrajectoryPlanner 完整工作流程测试")
    print("=" * 80)
    print("测试说明：")
    print("  1. 等待 /Odom 获取当前位置（起点）")
    print("  2. 等待 /nav_goal 获取目标点")
    print("  3. 自动计算转弯角度和前进距离")
    print("  4. 规划并发布前向轨迹（两次转弯 + 前进）")
    print("  5. 等待完成后规划并发布后向轨迹（倒车1米）")
    print("=" * 80)
    print(f"倒车距离: {BACKWARD_DISTANCE}m")
    print("=" * 80 + "\n")

    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)

    # 初始化ROS2
    rclpy.init()

    tester = ComplexPlannerTester()

    # 启动MQTT监听器
    if not tester.start_mqtt_listener():
        return

    # 等待Odom话题或超时
    tester.wait_for_odom_or_timeout()

    # 等待目标点
    tester.wait_for_goal()

    try:
        # 阶段1：发布前向轨迹
        print("\n⏱️  准备发布前向轨迹...")
        time.sleep(1)
        tester.publish_forward_trajectory()

        # 等待MQTT完成信号
        print(f"⏳ 等待MQTT轨迹完成信号（按Ctrl+C可中断）...\n")
        tester.mqtt_complete_received = False
        while not tester.mqtt_complete_received and rclpy.ok():
            rclpy.spin_once(tester, timeout_sec=0.5)

        if tester.mqtt_complete_received:
            print(f"✅ 前向轨迹已完成，等待3秒后发布后向轨迹...\n")
            time.sleep(3)
            # 额外spin几次确保Odom更新
            for _ in range(5):
                rclpy.spin_once(tester, timeout_sec=0.1)

            # 阶段2：发布后向轨迹
            tester.publish_backward_trajectory()

            # 等待后向轨迹完成
            print(f"⏳ 等待MQTT轨迹完成信号（按Ctrl+C可中断）...\n")
            tester.mqtt_complete_received = False
            while not tester.mqtt_complete_received and rclpy.ok():
                rclpy.spin_once(tester, timeout_sec=0.5)

        print("\n✅ 所有轨迹发布完成")
        print("💡 保持运行以监听MQTT轨迹消息...")
        print("   按 Ctrl+C 停止测试\n")

        # 保持ROS2节点运行，监听MQTT消息
        while rclpy.ok():
            rclpy.spin_once(tester, timeout_sec=1.0)

    except KeyboardInterrupt:
        signal_handler(None, None)


if __name__ == '__main__':
    main()
