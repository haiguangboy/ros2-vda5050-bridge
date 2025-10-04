#!/usr/bin/env python3
"""
基于目标点的轨迹规划工作流程测试

功能：
1. 订阅 /Odom 获取当前位置
2. 订阅 /nav_goal 获取目标点（从调度器）
3. 使用 SimpleTrajectoryPlanner 规划路径
4. 发布轨迹到 /plans 话题
5. 监听MQTT轨迹完成状态
6. 提供ROS2 service查询轨迹状态
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import time
import signal
import sys
import math
from collections import defaultdict
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from std_msgs.msg import Header
from example_interfaces.srv import Trigger  # 使用标准srv类型

# 导入轨迹规划器
from trajectory_planner import SimpleTrajectoryPlanner


# ==================== 配置参数 ====================

# MQTT配置
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
ROBOT_ID = "robot-001"

# ROS2配置
ODOM_TOPIC = "/Odom"
PATH_TOPIC = "/plans"
GOAL_TOPIC = "/nav_goal"         # 目标点话题（从调度器订阅）
ODOM_TIMEOUT = 10.0              # 等待Odom超时时间（秒）
GOAL_TIMEOUT = 30.0              # 等待目标点超时时间（秒）

# 默认位置（Odom超时时使用）
DEFAULT_X = 0.0
DEFAULT_Y = 0.0
DEFAULT_YAW = 0.0

# 路径规划配置
PLANNER_STEP_SIZE = 0.15         # 路径点间距（米）

# Beta-3协议配置
ORIENTATION_FORWARD = 0.0        # 前向运动
FLAG_NORMAL = 0                  # 非分支状态


# ==================== 测试节点 ====================

class GoalBasedTrajectoryTester(Node):
    def __init__(self):
        super().__init__('goal_based_trajectory_tester')

        # 初始化轨迹规划器
        self.planner = SimpleTrajectoryPlanner(step_size=PLANNER_STEP_SIZE)

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

        # ROS2 Service服务器（用于状态机查询轨迹状态）
        self.status_service = self.create_service(
            Trigger, '/trajectory_status', self.handle_status_query)

        # 状态管理
        self.running = True
        self.current_pose = None
        self.goal_pose = None
        self.odom_received = False
        self.goal_received = False
        self.mqtt_complete_received = False
        self.last_published_trajectory_id = None

        # 轨迹状态存储（存储从MQTT接收的状态）
        self.trajectory_status_dict = {}  # {trajectory_id: {status, timestamp, message}}
        self.latest_status = None  # 最新接收到的状态

    def odom_callback(self, msg):
        """里程计回调，持续更新当前位置"""
        self.current_pose = msg.pose.pose

        if not self.odom_received:
            self.odom_received = True
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            print(f"✅ 已接收到 /Odom 话题数据")
            print(f"   当前位置: ({x:.3f}, {y:.3f}), 朝向: {math.degrees(yaw):.1f}°")

    def goal_callback(self, msg):
        """目标点回调，接收调度器发布的导航目标"""
        self.goal_pose = msg.pose

        if not self.goal_received:
            self.goal_received = True
            x = msg.pose.position.x
            y = msg.pose.position.y
            yaw = self.quaternion_to_yaw(msg.pose.orientation)
            print(f"✅ 已接收到目标点")
            print(f"   目标位置: ({x:.3f}, {y:.3f}), 朝向: {math.degrees(yaw):.1f}°")

    def handle_status_query(self, request, response):
        """
        处理轨迹状态查询service请求

        Args:
            request: Trigger.Request (空请求)
            response: Trigger.Response

        Returns:
            response.success: bool - 是否有状态信息
            response.message: str - JSON格式的状态信息
        """
        if self.latest_status:
            response.success = True
            # 将状态信息转为JSON字符串
            status_json = json.dumps({
                'trajectory_id': self.latest_status.get('trajectoryId', ''),
                'status': self.latest_status.get('status', ''),
                'timestamp': self.latest_status.get('timestamp', 0),
                'message': self.latest_status.get('message', '')
            })
            response.message = status_json
            self.get_logger().info(f"📞 Service查询: 返回最新状态 - {self.latest_status.get('status', 'unknown')}")
        else:
            response.success = False
            response.message = json.dumps({
                'trajectory_id': '',
                'status': 'no_data',
                'timestamp': 0,
                'message': '暂无轨迹状态数据'
            })
            self.get_logger().info("📞 Service查询: 暂无状态数据")

        return response

    def wait_for_odom_or_timeout(self, timeout_seconds=ODOM_TIMEOUT):
        """等待Odom话题或超时使用默认数据"""
        print(f"\n⏳ 等待 {ODOM_TOPIC} 话题数据（最多等待 {timeout_seconds:.0f} 秒）...")

        start_time = time.time()
        while not self.odom_received and (time.time() - start_time) < timeout_seconds:
            rclpy.spin_once(self, timeout_sec=0.5)

        if not self.odom_received:
            print(f"\n⚠️  {timeout_seconds:.0f}秒内未收到 {ODOM_TOPIC} 话题，使用默认位置数据")
            print(f"   默认位置: ({DEFAULT_X:.3f}, {DEFAULT_Y:.3f}), 朝向: {math.degrees(DEFAULT_YAW):.1f}°")
            # 创建默认pose
            self.current_pose = Pose()
            self.current_pose.position.x = DEFAULT_X
            self.current_pose.position.y = DEFAULT_Y
            self.current_pose.position.z = 0.0
            self.current_pose.orientation = self.euler_to_quaternion(0.0, 0.0, DEFAULT_YAW)
        else:
            print(f"✅ 成功获取 {ODOM_TOPIC} 话题数据")

    def wait_for_goal(self, timeout_seconds=GOAL_TIMEOUT):
        """
        等待目标点话题

        Args:
            timeout_seconds: 超时时间（秒），None表示无限等待

        Returns:
            bool: 是否成功接收到目标点
        """
        print(f"\n⏳ 等待目标点 {GOAL_TOPIC} 话题数据...")
        if timeout_seconds:
            print(f"   超时设置: {timeout_seconds:.0f} 秒")
        else:
            print(f"   无超时限制（按Ctrl+C可中断）")

        start_time = time.time()
        while not self.goal_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            if timeout_seconds and (time.time() - start_time) >= timeout_seconds:
                print(f"\n⚠️  {timeout_seconds:.0f}秒内未收到目标点")
                return False

        if self.goal_received:
            print(f"✅ 成功接收到目标点")
            return True
        return False

    def refresh_odom(self):
        """主动刷新Odom数据"""
        print(f"📡 获取最新Odom数据...")
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

    def plan_and_publish_trajectory(self, goal_pose):
        """
        规划并发布轨迹

        Args:
            goal_pose (Pose): 目标位姿
        """
        # 刷新当前Odom
        self.refresh_odom()

        # 获取当前位置
        start_pose = self.current_pose
        start_x = start_pose.position.x
        start_y = start_pose.position.y
        start_yaw = self.quaternion_to_yaw(start_pose.orientation)

        # 获取目标位置
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        goal_yaw = self.quaternion_to_yaw(goal_pose.orientation)

        print("\n" + "="*80)
        print("📍 轨迹规划")
        print("="*80)
        print(f"起点: ({start_x:.3f}, {start_y:.3f}), yaw={math.degrees(start_yaw):.1f}°")
        print(f"终点: ({goal_x:.3f}, {goal_y:.3f}), yaw={math.degrees(goal_yaw):.1f}°")

        # 使用规划器规划路径
        waypoints = self.planner.plan_from_pose(start_pose, goal_pose)

        # 打印规划结果
        self.planner.print_waypoints(waypoints, max_points=10)

        # 生成轨迹ID
        trajectory_id = f"goal_traj_{int(time.time() * 1000)}"
        self.last_published_trajectory_id = trajectory_id

        # 创建ROS Path消息
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        # Beta-3协议：orientation=0.0, flag=0（前向运动，非分支）
        path.header.frame_id = f"map|none|none|{ORIENTATION_FORWARD}|{FLAG_NORMAL}|0|0|0|0|0|{trajectory_id}"

        # 转换waypoints为PoseStamped
        path.poses = []
        for x, y, yaw in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation = self.euler_to_quaternion(0.0, 0.0, yaw)
            path.poses.append(pose_stamped)

        # 发布路径
        self.path_publisher.publish(path)

        print(f"\n📤 轨迹已发布到 {PATH_TOPIC}")
        print(f"📋 轨迹ID: {trajectory_id}")
        print(f"📍 路径点数: {len(waypoints)}")
        print("="*80)

    def quaternion_to_yaw(self, q):
        """将四元数转换为yaw角度（弧度）"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
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

    # MQTT回调函数
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT连接成功")
            # 订阅轨迹状态主题
            status_topic = f"EP/{ROBOT_ID}/cerebellum/embrain/trajectory_status"
            client.subscribe(status_topic)
            print(f"📡 订阅轨迹状态主题: {status_topic}")
        else:
            print(f"❌ MQTT连接失败，返回代码: {rc}")

    def on_message(self, client, userdata, msg):
        try:
            trajectory_data = json.loads(msg.payload.decode())

            # 检查是否是轨迹状态消息
            if 'trajectory_status' in msg.topic:
                status = trajectory_data.get('status', '')
                trajectory_id = trajectory_data.get('trajectoryId', 'N/A')

                print("\n" + "="*80)
                print(f"📊 收到MQTT轨迹状态消息")
                print("="*80)
                print(f"📋 轨迹ID: {trajectory_id}")
                print(f"📍 状态: {status}")

                # 存储状态到字典
                self.trajectory_status_dict[trajectory_id] = {
                    'status': status,
                    'timestamp': trajectory_data.get('timestamp', int(time.time() * 1000)),
                    'message': trajectory_data.get('message', '')
                }

                # 更新最新状态（供service查询使用）
                self.latest_status = trajectory_data
                print(f"💾 已存储状态到内存，可通过 /trajectory_status service 查询")

                # 如果是完成状态，设置标志
                if status == 'completed':
                    print(f"\n✅ 轨迹已完成！")
                    print("="*80)
                    self.mqtt_complete_received = True
                elif status == 'running':
                    print(f"\n🏃 轨迹执行中...")
                    print("="*80)
                elif status == 'failed':
                    print(f"\n❌ 轨迹执行失败！")
                    print("="*80)

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
            return True
        except Exception as e:
            print(f"❌ MQTT连接失败: {e}")
            return False

    def stop(self):
        """停止测试"""
        self.running = False
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

    print("🧪 基于目标点的轨迹规划工作流程测试")
    print("=" * 80)

    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)

    # 初始化ROS2
    rclpy.init()

    tester = GoalBasedTrajectoryTester()

    # 启动MQTT监听器
    if not tester.start_mqtt_listener():
        return

    # 等待Odom话题
    tester.wait_for_odom_or_timeout()

    try:
        # 等待目标点（从调度器订阅）
        if not tester.wait_for_goal(timeout_seconds=None):  # 无限等待
            print("\n❌ 未接收到目标点，退出")
            return

        # 规划并发布轨迹
        tester.plan_and_publish_trajectory(tester.goal_pose)

        # 等待完成信号
        print(f"\n⏳ 等待MQTT轨迹完成信号（按Ctrl+C可中断）...")
        tester.mqtt_complete_received = False
        while not tester.mqtt_complete_received and rclpy.ok():
            rclpy.spin_once(tester, timeout_sec=0.5)

        if tester.mqtt_complete_received:
            print(f"\n✅ 轨迹执行完成！")

        print("\n💡 保持运行以监听新的目标点和MQTT消息...")
        print("   按 Ctrl+C 停止测试\n")

        # 保持ROS2节点运行
        while tester.running and rclpy.ok():
            rclpy.spin_once(tester, timeout_sec=1.0)

    except KeyboardInterrupt:
        signal_handler(None, None)


if __name__ == '__main__':
    main()
