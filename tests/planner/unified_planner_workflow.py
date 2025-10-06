#!/usr/bin/env python3
"""
统一轨迹规划器 - 自动选择规划策略

工作流程：
1. 第一个点（观察点）→ 使用 SimpleTrajectoryPlanner
2. 第二个点（取货点）→ 使用 ComplexTrajectoryPlanner

使用方法：
1. 启动本程序
2. 发布第一个目标点（观察点）：python3 publish_test_goal.py --x 3.0 --y 0.0 --yaw-deg 90
3. 等待第一段轨迹完成
4. 发布第二个目标点（取货点）：python3 publish_test_goal.py --x 4.0 --y 1.0 --yaw-deg 90
5. 等待第二段轨迹完成
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from example_interfaces.srv import Trigger
from forklift_interfaces.srv import GoToPose
import paho.mqtt.client as mqtt
import json
import time
import math
from trajectory_planner import SimpleTrajectoryPlanner, ComplexTrajectoryPlanner


# ==================== 配置参数 ====================

ODOM_TOPIC = "/Odom"
GOAL_TOPIC = "/nav_goal"
PATH_TOPIC = "/plans"
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
ROBOT_ID = "robot-001"

# 倒车距离（取货点专用）- 注意：实际倒车距离会根据目标点的y坐标动态计算
# BACKWARD_DISTANCE = 1.0  # 已废弃，改为动态计算

# 默认位置（Odom超时时使用）
DEFAULT_X = 0.0
DEFAULT_Y = 0.0
DEFAULT_YAW = 0.0


# ==================== 统一规划器节点 ====================

class UnifiedPlannerNode(Node):
    def __init__(self):
        super().__init__('unified_planner_node')

        # 创建两个规划器
        self.simple_planner = SimpleTrajectoryPlanner(step_size=0.15)
        self.complex_planner = ComplexTrajectoryPlanner(forward_step=0.15, backward_step=0.15)

        # ROS2订阅器
        self.odom_subscriber = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 10)
        self.goal_subscriber = self.create_subscription(
            PoseStamped, GOAL_TOPIC, self.goal_callback, 10)

        # ROS2发布器
        self.path_publisher = self.create_publisher(Path, PATH_TOPIC, 10)

        # MQTT客户端
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        # 状态变量
        self.current_odom = None
        self.odom_received = False
        self.goal_count = 0  # 收到的目标点计数
        self.current_trajectory_id = None
        self.waiting_for_completion = False
        self.trajectory_completed = False  # 轨迹是否完成的标志

        # ROS2发布器（用于更新Odom）
        self.odom_publisher = self.create_publisher(Odometry, ODOM_TOPIC, 10)

        # ROS2 Service服务器（提供轨迹状态查询）
        self.status_service = self.create_service(
            Trigger, '/trajectory_status', self.handle_status_query)

        # ROS2 Service服务器（接收调度器的GoToPose请求）
        self.go_to_pose_service = self.create_service(
            GoToPose, '/go_to_pose', self.handle_go_to_pose)

        # 轨迹状态记录
        self.last_trajectory_status = {
            'trajectory_id': '',
            'status': 'no_data',
            'timestamp': 0,
            'message': '暂无轨迹数据'
        }

        # GoToPose请求队列（用于异步处理）
        self.goto_request_queue = []
        self.goto_response_future = None

        print("✅ 统一轨迹规划器已启动")
        print("   规划器1: SimpleTrajectoryPlanner（观察点）")
        print("   规划器2: ComplexTrajectoryPlanner（取货点）")
        print("   Service: /trajectory_status（轨迹状态查询）")
        print("   Service: /go_to_pose（接收调度器目标点）\n")

    def odom_callback(self, msg):
        """接收Odom数据"""
        self.current_odom = msg
        if not self.odom_received:
            self.odom_received = True
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            print(f"✅ 已接收到 /Odom 话题数据")
            print(f"   当前位置: ({x:.3f}, {y:.3f}), 朝向: {yaw:.3f} ({math.degrees(yaw):.1f}°)\n")

    def create_default_odom(self):
        """创建默认Odom数据"""
        odom = Odometry()
        odom.pose.pose.position.x = DEFAULT_X
        odom.pose.pose.position.y = DEFAULT_Y
        odom.pose.pose.orientation = self.euler_to_quaternion(0, 0, DEFAULT_YAW)
        self.current_odom = odom
        self.odom_received = True
        print(f"⚠️  使用默认起点位置: ({DEFAULT_X:.3f}, {DEFAULT_Y:.3f}), 朝向: {DEFAULT_YAW:.3f} ({math.degrees(DEFAULT_YAW):.1f}°)\n")

    def goal_callback(self, msg):
        """接收目标点并自动选择规划器"""
        if not self.odom_received:
            print("⚠️  等待/Odom数据...")
            return

        if self.waiting_for_completion:
            print("⚠️  上一段轨迹还在执行中，请等待完成...")
            return

        self.goal_count += 1
        goal_pose = msg.pose

        x = goal_pose.position.x
        y = goal_pose.position.y
        yaw = self.quaternion_to_yaw(goal_pose.orientation)

        print("\n" + "="*80)
        if self.goal_count == 1:
            print("📍 第1个目标点（观察点）")
            print("="*80)
            print(f"目标位置: ({x:.3f}, {y:.3f}), 朝向: {yaw:.3f} ({math.degrees(yaw):.1f}°)")
            print(f"规划策略: SimpleTrajectoryPlanner（前进 + 转弯）\n")
            self.plan_and_publish_simple(goal_pose)
        elif self.goal_count == 2:
            print("📍 第2个目标点（取货点）")
            print("="*80)
            print(f"目标位置: ({x:.3f}, {y:.3f}), 朝向: {yaw:.3f} ({math.degrees(yaw):.1f}°)")
            print(f"规划策略: ComplexTrajectoryPlanner（转弯 + 前进 + 转弯 + 倒车）\n")
            self.plan_and_publish_complex(goal_pose)
        else:
            print(f"📍 收到第{self.goal_count}个目标点")
            print("="*80)
            print(f"⚠️  已完成观察点和取货点的轨迹规划（共2个目标点）")
            print(f"⚠️  忽略额外目标点: ({x:.3f}, {y:.3f})")
            print(f"💡 如需继续规划，请重启程序")
            print("="*80)

    def plan_and_publish_simple(self, goal_pose):
        """使用SimpleTrajectoryPlanner规划并发布"""
        print("🔧 使用 SimpleTrajectoryPlanner 规划轨迹...")
        print("-"*80)

        # 获取当前位置
        start_pose = self.current_odom.pose.pose
        start_x = start_pose.position.x
        start_y = start_pose.position.y
        start_yaw = self.quaternion_to_yaw(start_pose.orientation)

        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        goal_yaw = self.quaternion_to_yaw(goal_pose.orientation)

        print(f"📍 起点: ({start_x:.3f}, {start_y:.3f}), yaw={start_yaw:.3f} ({math.degrees(start_yaw):.1f}°)")
        print(f"📍 终点: ({goal_x:.3f}, {goal_y:.3f}), yaw={goal_yaw:.3f} ({math.degrees(goal_yaw):.1f}°)\n")

        # 计算前进距离和转弯角度（用于显示）
        forward_distance = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
        turn_angle = goal_yaw - start_yaw

        print(f"📐 自动计算参数:")
        print(f"   前进距离: {forward_distance:.3f} m")
        print(f"   转弯角度: {turn_angle:.3f} rad ({math.degrees(turn_angle):.1f}°)\n")

        # 规划轨迹（使用plan_from_pose方法）
        waypoints = self.simple_planner.plan_from_pose(start_pose, goal_pose)

        print(f"✅ 轨迹生成完成，共 {len(waypoints)} 个路径点\n")
        self.print_all_waypoints(waypoints)

        # 保存waypoints供MQTT完成后更新Odom使用
        self.first_trajectory_waypoints = waypoints

        # 发布轨迹
        trajectory_id = f"observation_{int(time.time() * 1000)}"
        self.publish_path(waypoints, trajectory_id, orientation=0.0, flag=0)
        self.current_trajectory_id = trajectory_id
        self.waiting_for_completion = True

        print("="*80)
        print(f"📤 第1段轨迹已发布（观察点）")
        print(f"📋 轨迹ID: {trajectory_id}")
        print("⏳ 等待MQTT完成信号...\n")

    def plan_and_publish_complex(self, goal_pose):
        """使用ComplexTrajectoryPlanner规划并发布（前向+后向）"""
        print("🔧 使用 ComplexTrajectoryPlanner 规划轨迹...")
        print("-"*80)

        # 获取当前位置（从/Odom读取，由第一段轨迹完成后更新）
        start_pose = self.current_odom.pose.pose
        start_x = start_pose.position.x
        start_y = start_pose.position.y
        start_yaw = self.quaternion_to_yaw(start_pose.orientation)

        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        goal_yaw = self.quaternion_to_yaw(goal_pose.orientation)

        print(f"📍 起点: ({start_x:.3f}, {start_y:.3f}), yaw={start_yaw:.3f} ({math.degrees(start_yaw):.1f}°)")
        print(f"📍 终点: ({goal_x:.3f}, {goal_y:.3f}), yaw={goal_yaw:.3f} ({math.degrees(goal_yaw):.1f}°)\n")

        # === 动态计算倒车距离 ===
        # 倒车距离 = 目标点y坐标 - 起点y坐标
        backward_distance = goal_y - start_y

        print(f"📐 自动计算轨迹分解:")
        print(f"   X方向距离: {goal_x - start_x:.3f} m")
        print(f"   Y方向距离（倒车）: {backward_distance:.3f} m")
        print(f"   总角度变化: {goal_yaw - start_yaw:.3f} rad ({math.degrees(goal_yaw - start_yaw):.1f}°)\n")

        # 第1段：前向轨迹（到达倒车起点）
        # 轨迹分解：
        # 1. 从起点yaw转到0（第一次转弯）
        # 2. 沿x轴前进到goal_x（前进）
        # 3. 从0转到goal_yaw（第二次转弯）

        first_turn_angle = 0 - start_yaw
        forward_distance = goal_x - start_x
        second_turn_angle = goal_yaw - 0

        print(f"📐 前向轨迹参数:")
        print(f"   第一次转弯: {first_turn_angle:.3f} rad ({math.degrees(first_turn_angle):.1f}°)")
        print(f"   前进距离: {forward_distance:.3f} m")
        print(f"   第二次转弯: {second_turn_angle:.3f} rad ({math.degrees(second_turn_angle):.1f}°)")
        print(f"   前向终点预期: ({goal_x:.3f}, {start_y:.3f}), yaw={goal_yaw:.3f}\n")

        # 规划前向轨迹
        forward_waypoints = self.complex_planner.plan_forward_with_turns(
            start_pose, first_turn_angle, forward_distance, second_turn_angle)

        print(f"✅ 前向轨迹生成完成，共 {len(forward_waypoints)} 个路径点\n")
        self.print_all_waypoints(forward_waypoints)

        # 保存waypoints供MQTT完成后更新Odom使用
        self.forward_trajectory_waypoints = forward_waypoints

        # 发布前向轨迹
        forward_trajectory_id = f"pickup_forward_{int(time.time() * 1000)}"
        self.publish_path(forward_waypoints, forward_trajectory_id, orientation=0.0, flag=0)
        self.current_trajectory_id = forward_trajectory_id
        self.waiting_for_completion = True

        print("="*80)
        print(f"📤 第2段轨迹（前向）已发布")
        print(f"📋 轨迹ID: {forward_trajectory_id}")
        print("⏳ 等待MQTT完成信号，然后发布倒车轨迹...\n")

        # 保存后向轨迹参数，等待前向完成后发布
        self.backward_params = {
            'backward_distance': backward_distance,  # 动态计算的倒车距离
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_yaw': goal_yaw
        }

    def publish_backward_trajectory(self):
        """发布后向轨迹（倒车）"""
        print("\n" + "="*80)
        print("📤 规划并发布后向轨迹（倒车）")
        print("="*80)

        # 从/Odom读取当前位置（已被前向轨迹完成后更新）
        intermediate_pose = self.current_odom.pose.pose

        # 获取动态计算的倒车距离
        backward_distance = self.backward_params['backward_distance']
        goal_x = self.backward_params['goal_x']
        goal_y = self.backward_params['goal_y']

        print(f"📐 倒车参数:")
        print(f"   倒车距离: {backward_distance:.3f} m")
        print(f"   倒车终点: ({goal_x:.3f}, {goal_y:.3f})\n")

        # 规划后向轨迹
        backward_waypoints = self.complex_planner.plan_backward(intermediate_pose, backward_distance)

        print(f"✅ 后向轨迹生成完成，共 {len(backward_waypoints)} 个路径点\n")
        self.print_all_waypoints(backward_waypoints)

        # 发布后向轨迹
        backward_trajectory_id = f"pickup_backward_{int(time.time() * 1000)}"
        self.publish_path(backward_waypoints, backward_trajectory_id, orientation=3.14, flag=1,
                         container_type="AGV-T300", container_x=goal_x, container_y=goal_y)
        self.current_trajectory_id = backward_trajectory_id
        self.waiting_for_completion = True

        print("="*80)
        print(f"📤 第2段轨迹（后向）已发布")
        print(f"📋 轨迹ID: {backward_trajectory_id}")
        print("⏳ 等待MQTT完成信号...\n")

    def publish_path(self, waypoints, trajectory_id, orientation=0.0, flag=0,
                    container_type="", container_x=0.0, container_y=0.0):
        """发布路径到/plans话题"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()

        # Beta-3协议：frame_id格式
        path.header.frame_id = f"map|0|0|{orientation}|{flag}|0|0|{container_type}|{container_x}|{container_y}|{trajectory_id}"

        # 添加路径点
        from geometry_msgs.msg import PoseStamped
        for x, y, yaw in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path.header.stamp
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation = self.euler_to_quaternion(0.0, 0.0, yaw)
            path.poses.append(pose_stamped)

        self.path_publisher.publish(path)

        print(f"Beta-3参数: orientation={orientation}, flag={flag}")
        if container_type:
            print(f"容器信息: type={container_type}, pos=({container_x}, {container_y})")

    def print_all_waypoints(self, waypoints):
        """打印所有路径点"""
        print(f"所有路径点:")
        for i, (x, y, yaw) in enumerate(waypoints, 1):
            print(f"  点{i}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} ({math.degrees(yaw):.1f}°)")
        print()

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            print("✅ MQTT连接成功")
            status_topic = f"EP/{ROBOT_ID}/cerebellum/embrain/trajectory_status"
            self.mqtt_client.subscribe(status_topic)
            print(f"📡 订阅轨迹状态主题: {status_topic}\n")
        else:
            print(f"❌ MQTT连接失败: {rc}")

    def update_odom_from_trajectory_end(self, waypoints):
        """
        将轨迹终点更新到/Odom话题
        用于测试环境下模拟位置更新（生产环境有真实Odom数据时可注释掉）
        """
        end_x, end_y, end_yaw = waypoints[-1]

        # 创建新的Odom消息
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = end_x
        odom.pose.pose.position.y = end_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.euler_to_quaternion(0, 0, end_yaw)

        # 发布到/Odom话题
        self.odom_publisher.publish(odom)

        # 同时更新内部状态
        self.current_odom = odom

        print(f"📡 更新/Odom: ({end_x:.3f}, {end_y:.3f}), yaw={end_yaw:.3f} ({math.degrees(end_yaw):.1f}°)")
        print(f"   (测试模式：轨迹终点 → /Odom)\n")

    def handle_status_query(self, request, response):
        """处理轨迹状态查询service请求"""
        response.success = True
        response.message = json.dumps(self.last_trajectory_status)
        return response

    def handle_go_to_pose(self, request, response):
        """
        处理GoToPose service请求（调度器格式）

        注意：此方法会阻塞直到轨迹执行完成或超时

        请求格式：
        - mode: 0=NORMAL, 1=FORK
        - target: PoseStamped (目标位置)
        - timeout_sec: 超时时间
        - pallet_pose: 托盘位置（mode=1时使用）
        - pallet_size: 托盘尺寸（mode=1时使用）
        """
        print("\n" + "="*80)
        print("📞 收到GoToPose请求（调度器）")
        print("="*80)

        mode = request.mode
        target = request.target.pose
        timeout = request.timeout_sec

        x = target.position.x
        y = target.position.y
        yaw = self.quaternion_to_yaw(target.orientation)

        mode_str = "NORMAL" if mode == GoToPose.Request.MODE_NORMAL else "FORK"
        print(f"模式: {mode_str}")
        print(f"目标: ({x:.3f}, {y:.3f}), yaw={yaw:.3f} ({math.degrees(yaw):.1f}°)")
        print(f"超时: {timeout:.1f}秒")

        if mode == GoToPose.Request.MODE_FORK:
            pallet_x = request.pallet_pose.position.x
            pallet_y = request.pallet_pose.position.y
            print(f"托盘位置: ({pallet_x:.3f}, {pallet_y:.3f})")

        # 检查是否可以接受新目标
        if self.waiting_for_completion:
            response.arrived = False
            response.message = "上一段轨迹还在执行中，请稍后"
            print("⚠️  拒绝请求：上一段轨迹未完成")
            print("="*80)
            return response

        if self.goal_count >= 2:
            response.arrived = False
            response.message = "已完成2个目标点，请重启规划器"
            print("⚠️  拒绝请求：已完成2个目标点")
            print("="*80)
            return response

        # 将PoseStamped格式转换为内部处理
        goal_pose = target

        # 根据目标点数量选择规划器
        self.goal_count += 1

        if self.goal_count == 1:
            print(f"✅ 接受为第1个目标点（观察点）")
            print(f"规划策略: SimpleTrajectoryPlanner\n")
            self.plan_and_publish_simple(goal_pose)
        elif self.goal_count == 2:
            print(f"✅ 接受为第2个目标点（取货点）")
            print(f"规划策略: ComplexTrajectoryPlanner\n")
            self.plan_and_publish_complex(goal_pose)

        # ===== 等待轨迹完成 =====
        print("="*80)
        print("⏳ 等待轨迹执行完成...")
        print(f"   超时时间: {timeout:.1f}秒")
        print("="*80 + "\n")

        # 重置完成标志
        self.trajectory_completed = False

        # 等待轨迹完成，带超时
        start_time = time.time()
        check_interval = 0.1  # 100ms检查一次

        while (time.time() - start_time) < timeout:
            # 处理ROS2回调以接收MQTT消息
            rclpy.spin_once(self, timeout_sec=check_interval)

            # 检查是否完成
            if self.trajectory_completed:
                elapsed = time.time() - start_time
                print("\n" + "="*80)
                print(f"✅ 轨迹执行完成！耗时: {elapsed:.1f}秒")
                print("="*80 + "\n")

                response.arrived = True
                response.message = f"目标点{self.goal_count}已到达"
                print(f"📤 返回响应: arrived=True, message={response.message}")
                print("="*80 + "\n")
                return response

        # 超时处理
        print("\n" + "="*80)
        print(f"⏱️  超时：轨迹执行超过 {timeout:.1f} 秒")
        print("="*80 + "\n")

        response.arrived = False
        response.message = f"目标点{self.goal_count}执行超时"
        print(f"📤 返回响应: arrived=False, message={response.message}")
        print("="*80 + "\n")

        return response

    def on_mqtt_message(self, client, userdata, msg):
        """MQTT消息回调"""
        try:
            payload = json.loads(msg.payload.decode())
            trajectory_id = payload.get("trajectoryId")
            status = payload.get("status")
            timestamp = payload.get("timestamp", int(time.time() * 1000))
            message = payload.get("message", "")

            # 更新轨迹状态记录
            self.last_trajectory_status = {
                'trajectory_id': trajectory_id,
                'status': status,
                'timestamp': timestamp,
                'message': message
            }

            if trajectory_id == self.current_trajectory_id and status == "completed":
                print("\n" + "="*80)
                print("📊 收到MQTT轨迹完成信号")
                print("="*80)
                print(f"📋 轨迹ID: {trajectory_id}")
                print(f"📍 状态: {status}")
                print("✅ 轨迹已完成！")
                print("="*80 + "\n")

                self.waiting_for_completion = False

                # 如果是第1段轨迹（观察点）完成
                if "observation" in trajectory_id:
                    # TODO: 生产环境有真实Odom时，注释掉下面这行
                    # 测试环境：将第1段轨迹终点更新到/Odom，供第2段使用
                    if hasattr(self, 'first_trajectory_waypoints'):
                        self.update_odom_from_trajectory_end(self.first_trajectory_waypoints)

                    # 设置完成标志，通知GoToPose service
                    self.trajectory_completed = True

                # 如果是第2段的前向轨迹完成，发布后向轨迹
                elif "pickup_forward" in trajectory_id:
                    # TODO: 生产环境有真实Odom时，注释掉下面这行
                    # 测试环境：将前向轨迹终点更新到/Odom，供倒车使用
                    if hasattr(self, 'forward_trajectory_waypoints'):
                        self.update_odom_from_trajectory_end(self.forward_trajectory_waypoints)

                    print("⏳ 等待3秒后发布倒车轨迹...\n")
                    time.sleep(3)
                    self.publish_backward_trajectory()

                elif "pickup_backward" in trajectory_id:
                    print("🎉 所有轨迹已完成！")
                    print("✅ 观察点和取货点任务完成")
                    print("💡 程序将继续监听，按Ctrl+C退出\n")

                    # 设置完成标志，通知GoToPose service
                    self.trajectory_completed = True

        except Exception as e:
            print(f"❌ MQTT消息解析错误: {e}")

    @staticmethod
    def quaternion_to_yaw(q):
        """四元数转yaw角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
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

    def start_mqtt(self):
        """启动MQTT连接"""
        try:
            print(f"🚀 连接到MQTT代理: {MQTT_BROKER}:{MQTT_PORT}")
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            time.sleep(1)
            return True
        except Exception as e:
            print(f"❌ MQTT连接失败: {e}")
            return False

    def stop(self):
        """停止节点"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()


def main():
    print("🚀 统一轨迹规划器")
    print("="*80)
    print("📋 使用说明：")
    print("  1. 本程序启动后等待目标点")
    print("  2. 【第1个点 - 观察点】使用 SimpleTrajectoryPlanner（前进 + 转弯）")
    print("  3. 【第2个点 - 取货点】使用 ComplexTrajectoryPlanner（转弯 + 前进 + 转弯 + 倒车）")
    print("="*80)
    print()
    print("⏳ 等待 /Odom 话题数据...")
    print()

    rclpy.init()
    node = UnifiedPlannerNode()

    # 启动MQTT
    if not node.start_mqtt():
        return

    # 等待Odom数据
    timeout = 5.0
    start_time = time.time()
    while not node.odom_received and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.odom_received:
        print("⚠️  超时：未收到/Odom数据，使用默认起点位置\n")
        node.create_default_odom()

    print("="*80)
    print("📍 请按顺序发布目标点：")
    print()
    print("第1步 - 发布观察点：")
    print("  python3 publish_test_goal.py --x 3.0 --y 0.0 --yaw-deg 90")
    print()
    print("第2步 - 等待第1段轨迹完成后，发布取货点：")
    print("  python3 publish_test_goal.py --x 4.0 --y 1.0 --yaw-deg-90")
    print("="*80)
    print()

    try:
        print("💡 等待目标点，按 Ctrl+C 停止\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⏹️  收到停止信号")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
