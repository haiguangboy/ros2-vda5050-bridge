#!/usr/bin/env python3
"""
统一轨迹规划器 - 基于模式自动选择规划策略

工作流程：
1. 观察点（MODE_NORMAL）→ 使用 SimpleTrajectoryPlanner
2. 取货点（MODE_FORK）→ 使用 误差消除轨迹 + ComplexTrajectoryPlanner

使用方法：
1. 启动本程序: python3 unified_planner_workflow.py
2. 使用 GoToPose Service 发送目标点: python3 test_goto_service.py
   - 第1个目标点：MODE_NORMAL（观察点）
   - 第2个目标点：MODE_FORK（取货点，需提供托盘信息）
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


# ==================== 配置参数 ====================

ODOM_TOPIC = "/Odom"
PATH_TOPIC = "/plans"
MQTT_BROKER = "192.168.1.102" #localhost for local test  192.168.1.102
# MQTT_BROKER = "localhost"
MQTT_PORT = 1883
ROBOT_ID = "robot-001"

# ==================== 环境配置 ====================
# 测试环境 vs 生产环境
TEST_MODE = False  # True=测试环境（模拟Odom更新），False=生产环境（使用真实Odom）
# TEST_MODE = True
# ==================== 轨迹开关配置 ====================
# 观察点轨迹配置
ENABLE_OBSERVATION_TRAJECTORY = True  # 是否启用观察点轨迹（SimpleTrajectoryPlanner）

# 取货轨迹配置
ENABLE_PICKUP_TRAJECTORY = True  # 是否启用取货轨迹（ComplexTrajectoryPlanner）
ENABLE_CORRECTION_TRAJECTORY = False  # 是否启用误差消除轨迹（观察点完成后回正+倒车）
CORRECTION_BACKWARD_DISTANCE = 0.6   # 误差消除轨迹的倒车距离（米）

# 卸货轨迹配置
ENABLE_UNLOAD_TRAJECTORY = False  # 是否启用卸货轨迹（叉取完成后返回主干道并送到卸货点）
MAIN_ROAD_Y = 0.0  # 主干道的y坐标（米）

# 默认位置（Odom超时时使用）
DEFAULT_X = 0.0
DEFAULT_Y = 0.0
DEFAULT_YAW = 0.0

# 等待的时间
WAIT_TIME = 2.1
# 在前向段完成后等待/Odom刷新（生产环境）
ODOM_WAIT_TIMEOUT = 1.5  # 秒
ODOM_POS_TOL = 0.15      # 位置容差（米）
ODOM_YAW_TOL = 0.35      # 朝向容差（弧度）

# 曲线规划参数
CURVE_STEP_SIZE = 0.15               # 路径点间距（米）
CURVE_MAX_ANGLE_CHANGE = 0.105       # 相邻点最大角度差（6° = 0.105 rad）
CURVE_POSITION_TOLERANCE = 0.08      # 终点位置容差（米）
CURVE_ANGLE_TOLERANCE = 0.09         # 终点角度容差（约5°）
CURVE_MAX_ITERATIONS = 200           # 最大搜索迭代次数


# ==================== 统一规划器节点 ====================

class UnifiedPlannerNode(Node):
    def __init__(self):
        super().__init__('unified_planner_node')

        # 创建两个规划器
        self.simple_planner = SimpleTrajectoryPlanner(step_size=0.15)
        self.complex_planner = ComplexTrajectoryPlanner(forward_step=0.15, backward_step=0.15)

        # ROS2订阅器（显式QoS，确保与发布端兼容）
        odom_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, odom_qos)

        # ROS2发布器
        self.path_publisher = self.create_publisher(Path, PATH_TOPIC, 10)

        # MQTT客户端（使用唯一的client_id避免冲突）
        self.mqtt_client = mqtt.Client(client_id="unified_planner_python", clean_session=True)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_subscribe = self.on_mqtt_subscribe

        # 添加日志回调用于调试
        self.mqtt_client.on_log = self.on_mqtt_log

        # 状态变量
        self.current_odom = None
        self.odom_received = False
        self.current_trajectory_id = None
        self.waiting_for_completion = False
        self.trajectory_completed = False  # 轨迹是否完成的标志
        self.pending_pickup_goal = None  # 暂存第二个目标点（等待误差消除轨迹完成后使用）
        self.pending_unload_goal = None  # 暂存卸货目标点
        self.pallet_info = None  # 托盘信息（mode=FORK时使用）
        self.last_mode = None  # 记录上一次的模式，用于判断是否触发卸货轨迹
        self.odom_update_count = 0  # /Odom更新计数（用于验证订阅是否持续接收）
        self.forward_start_stamp = None  # 前向轨迹起点读取的/Odom时间戳（sec, nsec）
        # 倒车段触发（ROS线程驱动）
        self.backward_pending = False
        self.backward_prev_stamp = None
        self.backward_prev_pose = None
        self.backward_prev_count = 0
        self.backward_forward_end = None
        self.backward_deadline = 0.0
        # 定时器：在ROS线程检查是否收到新鲜/Odom后再发布倒车段
        self.backward_timer = self.create_timer(0.05, self._check_and_publish_backward)
        # 最近一次接收到的/Odom时间戳与姿态（用于判定是否收到“新鲜”数据）
        self.last_odom_stamp = None
        self.last_odom_tuple = None  # (x, y, yaw)

        # 卸货轨迹的waypoints（用于测试环境更新Odom）
        self.unload_stage1_waypoints = None  # 倒车回主干道
        self.unload_stage2_waypoints = None  # 右转90° + 沿主干道前进
        self.unload_stage3_waypoints = None  # 右转90° + 倒车到卸货点

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
        print("   Service: /go_to_pose（接收调度器目标点）")
        print(f"   误差消除轨迹: {'启用' if ENABLE_CORRECTION_TRAJECTORY else '禁用'}")
        if ENABLE_CORRECTION_TRAJECTORY:
            print(f"   - 倒车距离: {CORRECTION_BACKWARD_DISTANCE}米\n")
        else:
            print()

    def odom_callback(self, msg):
        """接收Odom数据"""
        self.current_odom = msg
        # 计数与轻量打印（每20次）
        self.odom_update_count += 1
        if self.odom_update_count % 20 == 0:
            x_dbg = msg.pose.pose.position.x
            y_dbg = msg.pose.pose.position.y
            yaw_dbg = self.quaternion_to_yaw(msg.pose.pose.orientation)
            print(f"📡 /Odom 更新计数: {self.odom_update_count} (x={x_dbg:.3f}, y={y_dbg:.3f}, yaw={yaw_dbg:.3f})")
        # 记录最近一次的/Odom时间戳与姿态
        try:
            self.last_odom_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        except Exception:
            self.last_odom_stamp = (0, 0)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.last_odom_tuple = (x, y, yaw)
        if not self.odom_received:
            self.odom_received = True
            print(f"✅ 已接收到 /Odom 话题数据")
            print(f"   当前位置: ({x:.3f}, {y:.3f}), 朝向: {yaw:.3f} ({math.degrees(yaw):.1f}°)\n")

    def create_default_odom(self):
        """创建默认Odom数据"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = DEFAULT_X
        odom.pose.pose.position.y = DEFAULT_Y
        odom.pose.pose.orientation = self.euler_to_quaternion(0, 0, DEFAULT_YAW)
        self.current_odom = odom
        self.odom_received = True
        # 记录默认的时间戳与姿态
        try:
            self.last_odom_stamp = (odom.header.stamp.sec, odom.header.stamp.nanosec)
        except Exception:
            self.last_odom_stamp = (0, 0)
        self.last_odom_tuple = (DEFAULT_X, DEFAULT_Y, DEFAULT_YAW)
        print(f"⚠️  使用默认起点位置: ({DEFAULT_X:.3f}, {DEFAULT_Y:.3f}), 朝向: {DEFAULT_YAW:.3f} ({math.degrees(DEFAULT_YAW):.1f}°)\n")

    def _wait_for_fresh_odom(self, prev_stamp, prev_pose):
        """等待/Odom刷新，直至时间戳或位姿发生变化，或超时。

        Args:
            prev_stamp: 上一次记录的时间戳 (sec, nsec) 元组
            prev_pose: 上一次的 (x, y, yaw) 元组
        Returns:
            True 如果收到了看起来更新的/Odom；False 超时未更新。
        """
        start = time.time()
        while time.time() - start < ODOM_WAIT_TIMEOUT:
            now_stamp = self.last_odom_stamp
            now_pose = self.last_odom_tuple
            if now_stamp is None or now_pose is None:
                time.sleep(0.02)
                continue
            # 时间戳变化或位姿变化超过极小阈值即认为更新
            if now_stamp != prev_stamp:
                return True
            px, py, pyaw = prev_pose
            nx, ny, nyaw = now_pose
            if math.hypot(nx - px, ny - py) > 1e-3 or abs(self._normalize_angle(nyaw - pyaw)) > 1e-3:
                return True
            time.sleep(0.02)
        return False

    def _check_and_publish_backward(self):
        """在ROS线程中检查是否准备好发布倒车段，并执行发布。

        条件：
        - 收到与触发快照不同的/Odom时间戳，或
        - 位姿有微小变化（>1e-3），或
        - 超过截止时间（ODOM_WAIT_TIMEOUT）。
        满足其一则发布倒车段，并清理挂起标志。
        """
        if not self.backward_pending:
            return
        now = time.time()
        # 当前/上次快照
        prev_stamp = self.backward_prev_stamp
        prev_pose = self.backward_prev_pose
        prev_count = self.backward_prev_count
        forward_end = self.backward_forward_end
        now_stamp = self.last_odom_stamp
        now_pose = self.last_odom_tuple
        now_count = self.odom_update_count

        ready = False
        # 条件1：计数增长（收到新样本）
        if now_count > prev_count:
            ready = True
        # 条件2：时间戳变化
        if not ready and now_stamp is not None and prev_stamp is not None and now_stamp != prev_stamp:
            ready = True
        # 条件3：位姿发生变化
        if not ready and now_pose is not None and prev_pose is not None:
            nx, ny, nyaw = now_pose
            px, py, pyaw = prev_pose
            if math.hypot(nx - px, ny - py) > 1e-3 or abs(self._normalize_angle(nyaw - pyaw)) > 1e-3:
                ready = True
        # 条件4：当前位置已接近前向终点（允许直接开始倒车）
        if not ready and now_pose is not None and forward_end is not None:
            fx, fy, fyaw = forward_end
            nx, ny, nyaw = now_pose
            if math.hypot(nx - fx, ny - fy) < 0.25 and abs(self._normalize_angle(nyaw - fyaw)) < 0.4:
                ready = True
        if not ready and now >= self.backward_deadline:
            ready = True

        if not ready:
            return

        # 清理状态并发布
        self.backward_pending = False
        try:
            self.publish_backward_trajectory()
        except Exception as e:
            print(f"❌ 发布倒车段失败: {e}")

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

        # 计算前进距离和策略选择所需的角度差
        dx = goal_x - start_x
        dy = goal_y - start_y
        forward_distance = math.sqrt(dx**2 + dy**2)

        # 计算指向目标位置的方向
        target_angle = math.atan2(dy, dx)

        # 角度差：起始yaw 和 指向目标位置的方向 之间的差
        angle_diff = abs(self._normalize_angle(target_angle - start_yaw))

        print(f"📐 自动计算参数:")
        print(f"   前进距离: {forward_distance:.3f} m")
        print(f"   指向目标的方向: {target_angle:.3f} rad ({math.degrees(target_angle):.1f}°)")
        print(f"   角度差（起始yaw→目标方向）: {angle_diff:.3f} rad ({math.degrees(angle_diff):.1f}°)\n")

        # 根据角度差选择规划策略
        ANGLE_THRESHOLD = math.radians(20)  # 45° = 0.785 rad

        if angle_diff <= ANGLE_THRESHOLD:
            # 小角度：使用曲线规划（边走边转）
            print(f"📋 策略选择: 曲线规划（角度差 ≤ 20°）\n")
            waypoints = self.simple_planner.plan_from_pose_curve(start_pose, goal_pose)
        else:
            # 大角度：使用传统规划（原地转+直行+原地转）
            print(f"📋 策略选择: 传统规划（角度差 > 20°）\n")
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

    def plan_and_publish_correction_trajectory(self):
        """
        规划并发布误差消除轨迹（观察点完成后）
        流程：回正（-90° → 0°）+ 倒车0.6米（沿-x方向）
        目的：消除旋转开环控制带来的x,y误差
        """
        print("🔧 规划误差消除轨迹（回正 + 倒车）...")
        print("-"*80)

        # 从/Odom获取当前实时位置（观察点终点）
        start_pose = self.current_odom.pose.pose
        start_x = start_pose.position.x
        start_y = start_pose.position.y
        start_yaw = self.quaternion_to_yaw(start_pose.orientation)

        print(f"📍 起点（观察点终点）: ({start_x:.3f}, {start_y:.3f}), yaw={start_yaw:.3f} ({math.degrees(start_yaw):.1f}°)")
        print(f"📐 倒车距离: {CORRECTION_BACKWARD_DISTANCE}米\n")

        # 计算回正后的yaw
        # 观察点结束后yaw约为-90°（-pi/2 rad），回正需要加上90°（+pi/2 rad）
        # 例如：start_yaw=-1.588 rad（-91°），回正后 target_yaw = -1.588 + 1.571 ≈ -0.017 rad（约-1°）
        target_yaw = start_yaw + math.pi / 2  # 加90度（弧度制）

        # 倒车后的位置：倒车方向与回正后的yaw相反
        # 使用三角函数计算: backward方向 = -cos(yaw)*distance, -sin(yaw)*distance
        # 注意：即使target_yaw=0，实际执行后可能有偏差（如0.12 rad），倒车时x,y都会变化
        target_x = start_x - CORRECTION_BACKWARD_DISTANCE * math.cos(target_yaw)
        target_y = start_y - CORRECTION_BACKWARD_DISTANCE * math.sin(target_yaw)

        print(f"📍 目标（回正+倒车后）: ({target_x:.3f}, {target_y:.3f}), yaw={target_yaw:.3f} ({math.degrees(target_yaw):.1f}°)")
        print(f"   说明: 从当前yaw回正到0°，倒车时按实际yaw计算（x,y都会变化）\n")

        # 使用SimpleTrajectoryPlanner规划这段轨迹
        # 策略：先旋转到0° → 倒车（沿yaw相反方向）
        waypoints = []

        # 阶段1: 原地旋转回正
        angle_diff = self._normalize_angle(target_yaw - start_yaw)
        if abs(angle_diff) > 0.01:
            print(f"   阶段1: 原地旋转 {math.degrees(angle_diff):.1f}° 回正到0°")
            waypoints.append((start_x, start_y, start_yaw))  # 起点
            waypoints.append((start_x, start_y, target_yaw))  # 回正后
        else:
            waypoints.append((start_x, start_y, start_yaw))

        # 阶段2: 倒车（沿yaw相反方向，x和y都会变化）
        backward_distance = abs(CORRECTION_BACKWARD_DISTANCE)
        step_size = 0.15
        num_steps = int(backward_distance / step_size)
        remaining_distance = backward_distance - num_steps * step_size

        print(f"   阶段2: 倒车 {CORRECTION_BACKWARD_DISTANCE}米（yaw保持{target_yaw:.3f}，点间距{step_size}m）")

        # 生成中间点（不包括目标点）
        for i in range(1, num_steps + 1):
            dist = i * step_size
            # 倒车方向与yaw相反：dx = -distance * cos(yaw), dy = -distance * sin(yaw)
            dx = -dist * math.cos(target_yaw)
            dy = -dist * math.sin(target_yaw)
            waypoints.append((start_x + dx, start_y + dy, target_yaw))

        # 计算精确的目标点
        target_final_x = start_x - CORRECTION_BACKWARD_DISTANCE * math.cos(target_yaw)
        target_final_y = start_y - CORRECTION_BACKWARD_DISTANCE * math.sin(target_yaw)

        # 处理剩余距离，确保最后一个点是精确目标点
        if remaining_distance > 0.001:  # 有剩余距离
            if remaining_distance < 0.05:  # 剩余距离太小，去掉上一个点
                if len(waypoints) > 1:  # 确保有上一个点可以去掉
                    waypoints.pop()
                    print(f"   剩余距离 {remaining_distance:.3f}m < 0.05m，去掉上一个点，合并到目标点")
            else:
                # 0.05 <= remaining_distance < 0.15，保留上一个点，再添加目标点
                print(f"   剩余距离 {remaining_distance:.3f}m，保留上一个点并添加目标点")
            # 添加精确目标点
            waypoints.append((target_final_x, target_final_y, target_yaw))
        else:
            # 没有剩余距离（distance刚好是0.15的整数倍），最后一个中间点就是目标点
            # 不需要再添加重复点
            print(f"   距离刚好是{step_size}m的整数倍，最后一个点已是目标点")

        print(f"   ✅ 误差消除轨迹规划完成: 共 {len(waypoints)} 个路径点\n")
        self.print_all_waypoints(waypoints)

        # 保存waypoints供MQTT完成后更新Odom使用
        self.correction_trajectory_waypoints = waypoints

        # 发布轨迹（使用倒车参数：orientation=3.14, flag=0）
        correction_trajectory_id = f"correction_{int(time.time() * 1000)}"
        self.publish_path(waypoints, correction_trajectory_id, orientation=3.14, flag=0)
        self.current_trajectory_id = correction_trajectory_id
        self.waiting_for_completion = True

        print("="*80)
        print(f"📤 误差消除轨迹已发布")
        print(f"📋 轨迹ID: {correction_trajectory_id}")
        print(f"📋 Beta-3参数: orientation=3.14, flag=0（倒车）")
        print("⏳ 等待MQTT完成信号，然后规划取货轨迹...\n")

    def plan_and_publish_complex(self, goal_pose):
        """使用ComplexTrajectoryPlanner规划并发布（前向+后向）"""
        print("🔧 使用 ComplexTrajectoryPlanner 规划轨迹...")
        print("-"*80)

        # 获取当前位置（从/Odom读取，由第一段轨迹完成后更新）
        start_pose = self.current_odom.pose.pose
        # 记录并打印当前/Odom时间戳（作为前向起点的定位时间戳）
        try:
            fs = self.current_odom.header.stamp
            fs_sec = getattr(fs, 'sec', 0)
            fs_nsec = getattr(fs, 'nanosec', getattr(fs, 'nsec', 0))
            self.forward_start_stamp = (fs_sec, fs_nsec)
            print(f"🕒 前向起点 /Odom 时间戳: {fs_sec}.{fs_nsec:09d}")
        except Exception:
            self.forward_start_stamp = None
            print("🕒 前向起点 /Odom 时间戳: 无（header.stamp不可用）")
        start_x = start_pose.position.x
        start_y = start_pose.position.y
        start_yaw = self.quaternion_to_yaw(start_pose.orientation)

        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        goal_yaw = self.quaternion_to_yaw(goal_pose.orientation)

        print(f"📍 起点: ({start_x:.3f}, {start_y:.3f}), yaw={start_yaw:.3f} ({math.degrees(start_yaw):.1f}°)")
        print(f"📍 终点: ({goal_x:.3f}, {goal_y:.3f}), yaw={goal_yaw:.3f} ({math.degrees(goal_yaw):.1f}°)\n")

        # === 动态计算倒车距离 ===
        # 倒车距离 = 目标y - 起点y（标准位移计算）
        backward_distance = goal_y - start_y

        print(f"📐 自动计算轨迹分解:")
        print(f"   X方向距离: {goal_x - start_x:.3f} m")
        print(f"   Y方向距离（目标 - 起点）: {backward_distance:.3f} m")
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

        # 如果有托盘信息（MODE_FORK），使用完整的托盘信息
        if self.pallet_info:
            container_type = "AGV-T300"  # 默认容器类型
            container_x = self.pallet_info['x']
            container_y = self.pallet_info['y']
            container_z = self.pallet_info['pose'].position.z  # 托盘z坐标
            container_theta = self.quaternion_to_yaw(self.pallet_info['pose'].orientation)  # 托盘朝向
            container_width = self.pallet_info['size'].y  # 使用托盘尺寸的y作为宽度

            print(f"📦 ContainerPose:")
            print(f"   x: {container_x:.3f}")
            print(f"   y: {container_y:.3f}")
            print(f"   z: {container_z:.3f}")
            print(f"   theta: {container_theta:.3f}")
            print(f"   width: {container_width:.2f}")
            print(f"   container_type: {container_type}\n")
            print(f"   action_type: pub_load_params\n")

            self.publish_path(
                forward_waypoints, forward_trajectory_id,
                orientation=3.14, flag=0,
                action_type="pub_load_params",  # 地面取货动作
                container_type=container_type,
                container_x=container_x,
                container_y=container_y,
                container_z=container_z,
                container_theta=container_theta,
                container_width=container_width
            )
        else:
            # 没有托盘信息（兼容旧方式）
            self.publish_path(forward_waypoints, forward_trajectory_id, orientation=0.0, flag=0)

        # self.publish_path(forward_waypoints, forward_trajectory_id, orientation=0.0, flag=0)
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

        # 从/Odom读取当前位置（前向轨迹完成后的实际位置），
        # 若/Odom未更新（生产环境下机器人未发布最新位置），则回退使用“前向轨迹终点”作为起点。
        odom_pose = self.current_odom.pose.pose
        # 记录并打印当前/Odom时间戳（作为倒车起点的定位时间戳）
        try:
            bs = self.current_odom.header.stamp
            bs_sec = getattr(bs, 'sec', 0)
            bs_nsec = getattr(bs, 'nanosec', getattr(bs, 'nsec', 0))
            print(f"🕒 倒车起点 /Odom 时间戳: {bs_sec}.{bs_nsec:09d}")
            if self.forward_start_stamp is not None:
                fs_sec, fs_nsec = self.forward_start_stamp
                same_stamp = (bs_sec == fs_sec and bs_nsec == fs_nsec)
                if same_stamp:
                    print("⚠️ 时间戳相同：定位信息可能未更新（倒车段将使用与前向起点相同的/Odom）")
                else:
                    print(f"✅ 时间戳不同：定位信息已更新（前向: {fs_sec}.{fs_nsec:09d} → 倒车: {bs_sec}.{bs_nsec:09d}）")
            else:
                print("ℹ️ 无前向起点时间戳记录，无法比较")
        except Exception:
            print("🕒 倒车起点 /Odom 时间戳: 无（header.stamp不可用）")
        odom_x = odom_pose.position.x
        odom_y = odom_pose.position.y
        odom_yaw = self.quaternion_to_yaw(odom_pose.orientation)

        use_forward_end = False
        forward_end_x = forward_end_y = forward_end_yaw = None
        if hasattr(self, 'forward_trajectory_waypoints') and self.forward_trajectory_waypoints:
            forward_end_x, forward_end_y, forward_end_yaw = self.forward_trajectory_waypoints[-1]
            # 判定/Odom是否明显滞后（与前向终点相差较大）
            pos_diff = math.hypot(odom_x - forward_end_x, odom_y - forward_end_y)
            yaw_diff = abs(self._normalize_angle(odom_yaw - forward_end_yaw))
            if pos_diff > 0.20 or yaw_diff > 0.40:  # 阈值：20cm或>~23°
                use_forward_end = True

        if use_forward_end:
            print(f"⚠️  检测到/Odom未更新至前向终点，使用前向终点作为倒车起点")
            current_x, current_y, current_yaw = forward_end_x, forward_end_y, forward_end_yaw
            intermediate_pose = Pose()
            intermediate_pose.position.x = current_x
            intermediate_pose.position.y = current_y
            intermediate_pose.position.z = 0.0
            intermediate_pose.orientation = self.euler_to_quaternion(0, 0, current_yaw)
        else:
            intermediate_pose = odom_pose
            current_x, current_y, current_yaw = odom_x, odom_y, odom_yaw

        # 获取目标位置
        goal_x = self.backward_params['goal_x']
        goal_y = self.backward_params['goal_y']

        # 🔧 关键修复：根据当前实际位置重新计算倒车距离
        # 不使用预先计算的距离，而是从当前真实位置到目标位置
        backward_distance = goal_y - current_y

        print(f"📍 当前位置（从/Odom读取）: ({current_x:.3f}, {current_y:.3f}), yaw={current_yaw:.3f} ({math.degrees(current_yaw):.1f}°)")
        print(f"📍 目标位置: ({goal_x:.3f}, {goal_y:.3f})")
        print(f"📐 倒车参数:")
        print(f"   倒车距离（重新计算）: {backward_distance:.3f} m")
        print(f"   计算方式: goal_y({goal_y:.3f}) - current_y({current_y:.3f})")
        print(f"   倒车终点: ({goal_x:.3f}, {goal_y:.3f})\n")

        # 规划后向轨迹
        backward_waypoints = self.complex_planner.plan_backward(intermediate_pose, backward_distance)

        # 保存waypoints供MQTT完成后更新Odom使用
        self.backward_trajectory_waypoints = backward_waypoints

        print(f"✅ 后向轨迹生成完成，共 {len(backward_waypoints)} 个路径点\n")
        self.print_all_waypoints(backward_waypoints)

        # 发布后向轨迹
        backward_trajectory_id = f"pickup_backward_{int(time.time() * 1000)}"

        # 如果有托盘信息（MODE_FORK），使用完整的托盘信息
        # if self.pallet_info:
        #     container_type = "AGV-T300"  # 默认容器类型
        #     container_x = self.pallet_info['x']
        #     container_y = self.pallet_info['y']
        #     container_z = self.pallet_info['pose'].position.z  # 托盘z坐标
        #     container_theta = self.quaternion_to_yaw(self.pallet_info['pose'].orientation)  # 托盘朝向
        #     container_width = self.pallet_info['size'].y  # 使用托盘尺寸的y作为宽度

        #     print(f"📦 ContainerPose:")
        #     print(f"   x: {container_x:.3f}")
        #     print(f"   y: {container_y:.3f}")
        #     print(f"   z: {container_z:.3f}")
        #     print(f"   theta: {container_theta:.3f}")
        #     print(f"   width: {container_width:.2f}")
        #     print(f"   container_type: {container_type}\n")
        #     print(f"   action_type: pub_load_params\n")

        #     self.publish_path(
        #         backward_waypoints, backward_trajectory_id,
        #         orientation=3.14, flag=1,
        #         action_type="pub_load_params",  # 地面取货动作
        #         container_type=container_type,
        #         container_x=container_x,
        #         container_y=container_y,
        #         container_z=container_z,
        #         container_theta=container_theta,
        #         container_width=container_width
        #     )
        # else:
        #     # 没有托盘信息（兼容旧方式）
        #     self.publish_path(backward_waypoints, backward_trajectory_id, orientation=3.14, flag=0)

        self.publish_path(backward_waypoints, backward_trajectory_id, orientation=3.14, flag=1)

        self.current_trajectory_id = backward_trajectory_id
        self.waiting_for_completion = True

        print("="*80)
        print(f"📤 第2段轨迹（后向）已发布")
        print(f"📋 轨迹ID: {backward_trajectory_id}")
        print("⏳ 等待MQTT完成信号...\n")

    def plan_and_publish_unload_trajectory(self):
        """
        规划并发布卸货轨迹 - 第1段：倒车回主干道

        从叉取点出发，倒车返回主干道（y=0）
        """
        print("\n" + "="*80)
        print("🚛 规划卸货轨迹 - 第1段：倒车回主干道")
        print("="*80)

        # 从/Odom读取当前位置（叉取点）
        start_pose = self.current_odom.pose.pose
        start_x = start_pose.position.x
        start_y = start_pose.position.y
        start_yaw = self.quaternion_to_yaw(start_pose.orientation)

        # 获取卸货目标点
        goal_pose = self.pending_unload_goal
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        goal_yaw = self.quaternion_to_yaw(goal_pose.orientation)

        print(f"📍 当前位置（叉取点）: ({start_x:.3f}, {start_y:.3f}), yaw={start_yaw:.3f} ({math.degrees(start_yaw):.1f}°)")
        print(f"📍 最终目标（卸货点）: ({goal_x:.3f}, {goal_y:.3f}), yaw={goal_yaw:.3f} ({math.degrees(goal_yaw):.1f}°)")
        print(f"📍 主干道y坐标: {MAIN_ROAD_Y:.3f}\n")

        # 计算前进距离
        forward_distance = abs(MAIN_ROAD_Y - start_y)
        print(f"📐 向前行驶距离: {forward_distance:.3f}米")
        print(f"   起点: ({start_x:.3f}, {start_y:.3f})")
        print(f"   终点: ({start_x:.3f}, {MAIN_ROAD_Y:.3f})")
        print(f"   车头朝向: yaw={start_yaw:.3f} ({math.degrees(start_yaw):.1f}°)\n")

        # 使用ComplexTrajectoryPlanner的plan_forward_with_turns方法生成前进轨迹
        # 参数：不转弯（0）、前进、不转弯（0）
        stage1_waypoints = self.complex_planner.plan_forward_with_turns(
            start_pose,
            first_turn_angle=0,  # 不转弯
            forward_distance=forward_distance,
            second_turn_angle=0  # 不转弯
        )
        self.unload_stage1_waypoints = stage1_waypoints

        print(f"✅ 第1段轨迹生成完成，共 {len(stage1_waypoints)} 个路径点")
        self.print_all_waypoints(stage1_waypoints)

        # 发布第1段轨迹（向前直行模式）
        stage1_id = f"unload_stage1_{int(time.time() * 1000)}"
        self.publish_path(stage1_waypoints, stage1_id, orientation=0.0, flag=0)
        self.current_trajectory_id = stage1_id
        self.waiting_for_completion = True

        print("="*80)
        print(f"📤 卸货第1段轨迹已发布（向前行驶回主干道）")
        print(f"📋 轨迹ID: {stage1_id}")
        print("⏳ 等待MQTT完成信号...\n")

    def publish_unload_stage2(self):
        """
        发布卸货轨迹第2段：右转90° + 沿主干道前进

        从主干道（段1终点）右转90°，然后沿-x方向（yaw=π）行驶到目标x坐标
        """
        print("\n" + "="*80)
        print("🚛 规划卸货轨迹 - 第2段：右转 + 沿主干道前进")
        print("="*80)

        # 从/Odom读取段1完成后的位置
        current_pose = self.current_odom.pose.pose
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        current_yaw = self.quaternion_to_yaw(current_pose.orientation)

        print(f"📍 当前位置（从/Odom读取）: ({current_x:.3f}, {current_y:.3f}), yaw={current_yaw:.3f} ({math.degrees(current_yaw):.1f}°)")

        # 获取目标x坐标（卸货点的x）
        goal_x = self.pending_unload_goal.position.x
        # 🔧 修复：根据实际位置重新计算前进距离
        forward_distance = abs(goal_x - current_x)

        print(f"📐 右转90° + 沿主干道前进（重新计算距离）")
        print(f"   前进距离: {forward_distance:.3f}米")
        print(f"   起点x: {current_x:.3f} → 终点x: {goal_x:.3f}")
        print(f"   计算方式: abs(goal_x({goal_x:.3f}) - current_x({current_x:.3f}))\n")

        # 使用ComplexTrajectoryPlanner的plan_forward_with_turns方法
        # 参数：右转90°（+π/2）、前进、不转弯（0）
        stage2_waypoints = self.complex_planner.plan_forward_with_turns(
            current_pose,
            first_turn_angle=math.pi / 2,  # 右转90°
            forward_distance=forward_distance,
            second_turn_angle=0  # 不需要第二次转弯
        )

        self.unload_stage2_waypoints = stage2_waypoints

        print(f"✅ 第2段轨迹生成完成，共 {len(stage2_waypoints)} 个路径点")
        self.print_all_waypoints(stage2_waypoints)

        # 发布第2段轨迹
        stage2_id = f"unload_stage2_{int(time.time() * 1000)}"
        self.publish_path(stage2_waypoints, stage2_id, orientation=0.0, flag=0)
        self.current_trajectory_id = stage2_id
        self.waiting_for_completion = True

        print("="*80)
        print(f"📤 卸货第2段轨迹已发布（右转 + 沿主干道前进）")
        print(f"📋 轨迹ID: {stage2_id}")
        print("⏳ 等待MQTT完成信号...\n")

    def publish_unload_stage3(self):
        """
        发布卸货轨迹第3段：右转90° + 倒车到卸货点

        从主干道右转90°，然后倒车到卸货点（沿+y方向倒车）
        """
        print("\n" + "="*80)
        print("🚛 规划卸货轨迹 - 第3段：左转 + 倒车到卸货点")
        print("="*80)

        # 从/Odom读取段2完成后的位置
        current_pose = self.current_odom.pose.pose
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        current_yaw = self.quaternion_to_yaw(current_pose.orientation)

        print(f"📍 当前位置: ({current_x:.3f}, {current_y:.3f}), yaw={current_yaw:.3f} ({math.degrees(current_yaw):.1f}°)")

        # 向右旋转90°：yaw + π/2，归一化到[-π, π]
        intermediate_yaw = self._normalize_angle(current_yaw + math.pi / 2)
        print(f"📐 左转90°后yaw: {intermediate_yaw:.3f} ({math.degrees(intermediate_yaw):.1f}°)")

        # 获取卸货点的y坐标
        goal_y = self.pending_unload_goal.position.y
        goal_yaw = self.quaternion_to_yaw(self.pending_unload_goal.orientation)
        # 🔧 修复：根据实际位置重新计算倒车距离（不使用abs，保持方向）
        backward_distance = goal_y - current_y

        print(f"📐 倒车距离（重新计算）: {backward_distance:.3f}米")
        print(f"   起点y: {current_y:.3f} → 终点y: {goal_y:.3f}")
        print(f"   计算方式: goal_y({goal_y:.3f}) - current_y({current_y:.3f})")
        print(f"   终点yaw: {goal_yaw:.3f} ({math.degrees(goal_yaw):.1f}°)\n")

        # 生成第3段轨迹：旋转 + 倒车
        waypoints = []

        # 1. 原地左转90°
        waypoints.append((current_x, current_y, current_yaw))
        waypoints.append((current_x, current_y, intermediate_yaw))

        # 2. 倒车到卸货点
        # 使用ComplexTrajectoryPlanner的plan_backward方法
        temp_pose = Pose()
        temp_pose.position.x = current_x
        temp_pose.position.y = current_y
        temp_pose.position.z = 0.0
        temp_pose.orientation = self.euler_to_quaternion(0, 0, intermediate_yaw)

        backward_waypoints = self.complex_planner.plan_backward(temp_pose, backward_distance)
        waypoints.extend(backward_waypoints)

        self.unload_stage3_waypoints = waypoints

        print(f"✅ 第3段轨迹生成完成，共 {len(waypoints)} 个路径点")
        print(f"   - 左转: 2个点")
        print(f"   - 倒车: {len(backward_waypoints)}个点")
        self.print_all_waypoints(waypoints)

        # 发布第3段轨迹（倒车模式 + 卸货动作）
        stage3_id = f"unload_stage3_{int(time.time() * 1000)}"
        self.publish_path(waypoints, stage3_id, orientation=3.14, flag=1,
                         action_type="pub_unload_params", container_type="AGV-T300")
        self.current_trajectory_id = stage3_id
        self.waiting_for_completion = True

        print("="*80)
        print(f"📤 卸货第3段轨迹已发布（右转 + 倒车到卸货点）")
        print(f"📋 轨迹ID: {stage3_id}")
        print("⏳ 等待MQTT完成信号...\n")

    def publish_path(self, waypoints, trajectory_id, orientation=0.0, flag=0,
                    action_type="", container_type="",
                    container_x=0.0, container_y=0.0, container_z=0.0,
                    container_theta=0.0, container_width=1.2):
        """
        发布路径到/plans话题

        Args:
            waypoints: 路径点列表 [(x, y, yaw), ...]
            trajectory_id: 轨迹ID
            orientation: 朝向 (0=前向, 3.14=倒车)
            flag: 标志 (0=正常, 1=分支)
            action_type: 动作类型 (ground_pick, load等)
            container_type: 容器类型 (如 "AGV-T300")
            container_x, container_y, container_z: 容器位置
            container_theta: 容器朝向 (弧度)
            container_width: 容器宽度 (米)
        """
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()

        # Beta-3协议：frame_id格式
        # "map|action_type|container_type|orientation|flag|container_x|container_y|container_z|container_theta|container_width|trajectory_id"
        path.header.frame_id = (
            f"map|{action_type}|{container_type}|{orientation}|{flag}|"
            f"{container_x}|{container_y}|{container_z}|{container_theta}|{container_width}|{trajectory_id}"
        )

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
            result, mid = self.mqtt_client.subscribe(status_topic)
            print(f"📡 订阅轨迹状态主题: {status_topic}")
            print(f"   订阅结果: result={result}, mid={mid}\n")
        else:
            print(f"❌ MQTT连接失败: {rc}")

    def on_mqtt_subscribe(self, client, userdata, mid, granted_qos):
        """MQTT订阅确认回调"""
        print(f"✅ MQTT订阅确认: mid={mid}, QoS={granted_qos}\n")

    def on_mqtt_log(self, client, userdata, level, buf):
        """MQTT日志回调（用于调试）"""
        print(f"🔍 MQTT日志: {buf}")

    def update_odom_from_trajectory_end(self, waypoints):
        """
        将轨迹终点更新到/Odom话题（仅测试模式）

        测试环境：用于模拟机器人位置更新，下一段轨迹会从这个位置开始规划
        生产环境：此方法不会被调用（TEST_MODE=False），使用真实/Odom话题数据
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
            pallet_size = request.pallet_size
            print(f"托盘位置: ({pallet_x:.3f}, {pallet_y:.3f})")
            print(f"托盘尺寸: ({pallet_size.x:.2f}, {pallet_size.y:.2f}, {pallet_size.z:.2f})")

            # 保存托盘信息供后续倒车轨迹使用
            self.pallet_info = {
                'pose': request.pallet_pose,
                'size': pallet_size,
                'x': pallet_x,
                'y': pallet_y
            }

        # 检查Odom是否就绪
        if not self.odom_received:
            print("⚠️  /Odom数据未就绪，使用默认起点位置")
            self.create_default_odom()

        # 检查是否可以接受新目标
        if self.waiting_for_completion:
            response.arrived = False
            response.message = "上一段轨迹还在执行中，请稍后"
            print("⚠️  拒绝请求：上一段轨迹未完成")
            print("="*80)
            return response

        # 将PoseStamped格式转换为内部处理
        goal_pose = target

        # 根据mode选择规划器
        # MODE_NORMAL (0) = 观察点 或 卸货点
        # MODE_FORK (1) = 叉取点

        # 添加调试日志
        print(f"🔍 调试信息:")
        print(f"   当前模式: {mode} ({'NORMAL' if mode == 0 else 'FORK'})")
        print(f"   上一次模式: {getattr(self, 'last_mode', None)}")
        print(f"   卸货轨迹启用: {ENABLE_UNLOAD_TRAJECTORY}")

        # 先检查是否是卸货场景（MODE_NORMAL + 上次是MODE_FORK）
        if mode == GoToPose.Request.MODE_NORMAL and hasattr(self, 'last_mode') and self.last_mode == GoToPose.Request.MODE_FORK:
            if ENABLE_UNLOAD_TRAJECTORY:
                print(f"✅ 检测到取货后的目标点，触发卸货轨迹（3段）")
                print(f"规划策略: 卸货轨迹\n")
                self.pending_unload_goal = goal_pose
                self.plan_and_publish_unload_trajectory()
            else:
                # 禁用卸货轨迹时，按普通点处理
                print(f"✅ 接受为目标点（MODE_NORMAL）")
                print(f"规划策略: SimpleTrajectoryPlanner\n")
                self.pending_goal = goal_pose
                self.plan_and_publish_simple(goal_pose)

        # 普通观察点
        elif mode == GoToPose.Request.MODE_NORMAL:
            if ENABLE_OBSERVATION_TRAJECTORY:
                print(f"✅ 接受为观察点（MODE_NORMAL）")
                print(f"规划策略: SimpleTrajectoryPlanner\n")
                # 先旋转+直行 轨迹
                self.plan_and_publish_simple(goal_pose)
                # 边走边转 轨迹
                # self.plan_and_publish_simple_curve(goal_pose)
            else:
                print(f"⚠️ 观察点轨迹未启用（ENABLE_OBSERVATION_TRAJECTORY=False）")
                response.arrived = False
                response.message = "观察点轨迹未启用"
                return response

        # 叉取点
        elif mode == GoToPose.Request.MODE_FORK:
            if ENABLE_PICKUP_TRAJECTORY:
                print(f"✅ 接受为叉取点（MODE_FORK）")
                if ENABLE_CORRECTION_TRAJECTORY:
                    print(f"规划策略: 误差消除轨迹 + ComplexTrajectoryPlanner\n")
                    self.pending_pickup_goal = goal_pose
                    self.plan_and_publish_correction_trajectory()
                else:
                    print(f"规划策略: ComplexTrajectoryPlanner\n")
                    self.plan_and_publish_complex(goal_pose)
            else:
                print(f"⚠️ 取货轨迹未启用（ENABLE_PICKUP_TRAJECTORY=False）")
                response.arrived = False
                response.message = "取货轨迹未启用"
                return response

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
                # 判断模式名称
                if mode == GoToPose.Request.MODE_NORMAL:
                    # 如果是取货后的目标点，则为卸货点
                    if hasattr(self, 'last_mode') and self.last_mode == GoToPose.Request.MODE_FORK and ENABLE_UNLOAD_TRAJECTORY:
                        mode_name = "卸货点"
                    else:
                        mode_name = "观察点"
                elif mode == GoToPose.Request.MODE_FORK:
                    mode_name = "取货点"
                else:
                    mode_name = "未知模式"

                response.message = f"{mode_name}已到达"
                print(f"📤 返回响应: arrived=True, message={response.message}")
                print("="*80 + "\n")

                # 保存当前模式供下次使用
                self.last_mode = mode
                return response

        # 超时处理
        print("\n" + "="*80)
        print(f"⏱️  超时：轨迹执行超过 {timeout:.1f} 秒")
        print("="*80 + "\n")

        response.arrived = False
        # 判断模式名称
        if mode == GoToPose.Request.MODE_NORMAL:
            # 如果是取货后的目标点，则为卸货点
            if hasattr(self, 'last_mode') and self.last_mode == GoToPose.Request.MODE_FORK and ENABLE_UNLOAD_TRAJECTORY:
                mode_name = "卸货点"
            else:
                mode_name = "观察点"
        elif mode == GoToPose.Request.MODE_FORK:
            mode_name = "取货点"
        else:
            mode_name = "未知模式"
        response.message = f"{mode_name}执行超时"
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

            # 打印所有MQTT消息（包括running状态）
            print(f"📨 MQTT消息: ID={trajectory_id}, status={status}")

            # 更新轨迹状态记录
            self.last_trajectory_status = {
                'trajectory_id': trajectory_id,
                'status': status,
                'timestamp': timestamp,
                'message': message
            }

            # 处理轨迹完成信号
            # 注意：由于MQTT Bridge可能生成不同的轨迹ID，我们放宽匹配条件
            # 只要status是completed且waiting_for_completion为True，就认为是当前轨迹完成
            if status == "completed" and self.waiting_for_completion:
                print("\n" + "="*80)
                print("📊 收到MQTT轨迹完成信号")
                print("="*80)
                print(f"📋 MQTT轨迹ID: {trajectory_id}")
                print(f"📋 本地轨迹ID: {self.current_trajectory_id}")
                print(f"📍 状态: {status}")
                print("✅ 轨迹已完成！")
                print("="*80 + "\n")

                self.waiting_for_completion = False

                # 根据本地轨迹ID判断是哪一段轨迹
                # 如果是第1段轨迹（观察点）完成
                if "observation" in self.current_trajectory_id:
                    # 测试环境：将第1段轨迹终点更新到/Odom，供第2段使用
                    if TEST_MODE and hasattr(self, 'first_trajectory_waypoints'):
                        self.update_odom_from_trajectory_end(self.first_trajectory_waypoints)

                    # 设置完成标志，通知GoToPose service
                    self.trajectory_completed = True

                # 如果是误差消除轨迹完成，发布取货轨迹
                elif "correction" in self.current_trajectory_id:
                    # 测试环境：将误差消除轨迹终点更新到/Odom，供取货轨迹使用
                    if TEST_MODE and hasattr(self, 'correction_trajectory_waypoints'):
                        self.update_odom_from_trajectory_end(self.correction_trajectory_waypoints)

                    print("⏳ 等待0.1秒后规划取货轨迹...\n")
                    time.sleep(WAIT_TIME)

                    # 使用之前保存的目标点规划取货轨迹
                    if self.pending_pickup_goal:
                        self.plan_and_publish_complex(self.pending_pickup_goal)
                        self.pending_pickup_goal = None  # 清除已使用的目标点

                # 如果是第2段的前向轨迹完成，发布后向轨迹
                elif "pickup_forward" in self.current_trajectory_id:
                    # 测试环境：将前向轨迹终点更新到/Odom，供倒车使用
                    if TEST_MODE and hasattr(self, 'forward_trajectory_waypoints'):
                        self.update_odom_from_trajectory_end(self.forward_trajectory_waypoints)

                    # 生产环境：只置位，由ROS线程（定时器）在检测到/Odom刷新或超时后发布倒车段
                    self.backward_prev_stamp = self.last_odom_stamp
                    self.backward_prev_pose = self.last_odom_tuple
                    self.backward_prev_count = self.odom_update_count
                    self.backward_forward_end = self.forward_trajectory_waypoints[-1] if hasattr(self, 'forward_trajectory_waypoints') and self.forward_trajectory_waypoints else None
                    self.backward_deadline = time.time() + ODOM_WAIT_TIMEOUT
                    self.backward_pending = True
                    print("⏳ 已置位倒车段触发，交由ROS线程等待/Odom刷新后发布\n")

                elif "pickup_backward" in self.current_trajectory_id:
                    print("🎉 取货轨迹已完成！")
                    print("✅ 观察点和取货点任务完成")

                    # 测试环境：将倒车轨迹终点更新到/Odom
                    if TEST_MODE and hasattr(self, 'backward_trajectory_waypoints'):
                        self.update_odom_from_trajectory_end(self.backward_trajectory_waypoints)

                    print("💡 程序将继续监听，按Ctrl+C退出\n")

                    # 设置完成标志，通知GoToPose service
                    self.trajectory_completed = True

                # ===== 卸货轨迹的3段处理 =====
                elif "unload_stage1" in self.current_trajectory_id:
                    print("✅ 卸货第1段（向前行驶回主干道）已完成")

                    # 测试环境：模拟Odom更新
                    if TEST_MODE and hasattr(self, 'unload_stage1_waypoints'):
                        self.update_odom_from_trajectory_end(self.unload_stage1_waypoints)

                    print("⏳ 等待0.1秒后发布第2段...\n")
                    time.sleep(WAIT_TIME)
                    self.publish_unload_stage2()

                elif "unload_stage2" in self.current_trajectory_id:
                    print("✅ 卸货第2段（右转 + 沿主干道前进）已完成")

                    # 测试环境：模拟Odom更新
                    if TEST_MODE and hasattr(self, 'unload_stage2_waypoints'):
                        self.update_odom_from_trajectory_end(self.unload_stage2_waypoints)

                    print("⏳ 等待0.1秒后发布第3段...\n")
                    time.sleep(WAIT_TIME)
                    self.publish_unload_stage3()

                elif "unload_stage3" in self.current_trajectory_id:
                    print("🎉 卸货轨迹全部完成！")
                    print("✅ 货物已送达卸货点")
                    print("💡 程序将继续监听，按Ctrl+C退出\n")

                    # 测试环境：模拟Odom更新
                    if TEST_MODE and hasattr(self, 'unload_stage3_waypoints'):
                        self.update_odom_from_trajectory_end(self.unload_stage3_waypoints)

                    # 设置完成标志，通知GoToPose service
                    self.trajectory_completed = True
                    self.pending_unload_goal = None  # 清除已使用的目标点

        except Exception as e:
            print(f"❌ MQTT消息解析错误: {e}")

    @staticmethod
    def quaternion_to_yaw(q):
        """四元数转yaw角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

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
    print("📋 环境配置：")
    if TEST_MODE:
        print("  运行模式: 🧪 测试模式（模拟Odom更新）")
        print("  说明: 轨迹完成后会自动更新/Odom到轨迹终点")
    else:
        print("  运行模式: 🏭 生产模式（使用真实Odom）")
        print("  说明: 从真实/Odom话题订阅机器人位置")
    print("="*80)
    print()
    print("📋 轨迹开关配置：")
    print(f"  观察点轨迹: {'✅ 启用' if ENABLE_OBSERVATION_TRAJECTORY else '❌ 禁用'}")
    print(f"  取货轨迹:   {'✅ 启用' if ENABLE_PICKUP_TRAJECTORY else '❌ 禁用'}")
    if ENABLE_PICKUP_TRAJECTORY:
        print(f"    ├─ 误差消除: {'✅ 启用' if ENABLE_CORRECTION_TRAJECTORY else '❌ 禁用'}")
        if ENABLE_CORRECTION_TRAJECTORY:
            print(f"    └─ 倒车距离: {CORRECTION_BACKWARD_DISTANCE}米")
    print(f"  卸货轨迹:   {'✅ 启用' if ENABLE_UNLOAD_TRAJECTORY else '❌ 禁用'}")
    if ENABLE_UNLOAD_TRAJECTORY:
        print(f"    └─ 主干道y: {MAIN_ROAD_Y}米")
    print("="*80)
    print()
    print("📋 使用说明：")
    print("  1. 本程序启动后等待目标点")
    print("  2. 【观察点】MODE_NORMAL - SimpleTrajectoryPlanner（前进 + 转弯）")
    print("  3. 【取货点】MODE_FORK - ComplexTrajectoryPlanner")
    if ENABLE_CORRECTION_TRAJECTORY:
        print(f"     - 步骤1: 回正 + 倒车{CORRECTION_BACKWARD_DISTANCE}米（消除旋转误差）")
        print("     - 步骤2: 转弯 + 前进 + 转弯 + 倒车（到达取货点）")
    else:
        print("     - 直接规划: 转弯 + 前进 + 转弯 + 倒车")
    if ENABLE_UNLOAD_TRAJECTORY:
        print("  4. 【卸货点】MODE_NORMAL（取货后）- 3段卸货轨迹")
        print("     - 第1段: 向前行驶回主干道")
        print("     - 第2段: 右转 + 沿主干道前进")
        print("     - 第3段: 左转 + 倒车到卸货点")
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
    timeout = 2.0
    start_time = time.time()
    while not node.odom_received and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.odom_received:
        print("⚠️  超时：未收到/Odom数据，使用默认起点位置\n")
        node.create_default_odom()

    print("="*80)
    print("📍 测试方法：")
    print()
    print("使用 GoToPose Service（推荐）：")
    print("  # 测试观察点")
    print("  python3 test_goto_service.py         # 观察点 + 取货点")
    print()
    print("  # 测试完整流程")
    print("  python3 test_unload.py               # 观察点 + 取货点 + 卸货点")
    print()
    print("💡 提示：")
    print("  - 可以通过修改配置文件顶部的开关来启用/禁用特定轨迹")
    print("  - ENABLE_OBSERVATION_TRAJECTORY: 观察点轨迹")
    print("  - ENABLE_PICKUP_TRAJECTORY: 取货轨迹")
    print("  - ENABLE_CORRECTION_TRAJECTORY: 误差消除轨迹（取货轨迹的子选项）")
    print("  - ENABLE_UNLOAD_TRAJECTORY: 卸货轨迹")
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
