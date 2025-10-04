#!/usr/bin/env python3
"""
Beta-3协议增强型轨迹工作流程测试

测试说明：
- 支持灵活配置的两条轨迹测试
- 10秒等待/Odom话题，超时使用默认位置
- 严格遵循Beta-3协议：flag只使用0或1，orientation使用0或±3.14
- 原地转弯只发布起点和终点（x,y不变，只有yaw变化）
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import time
import signal
import sys
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from std_msgs.msg import Header, String
import math
import uuid


# ==================== 配置参数 ====================

# 轨迹开关
ENABLE_TRAJECTORY0 = True     # 是否发布第零条轨迹
ENABLE_TRAJECTORY1 = True     # 是否发布第一条轨迹
ENABLE_TRAJECTORY2 = True     # 是否发布第二条轨迹
ENABLE_TRAJECTORY3 = True     # 是否发布第三条轨迹

# 第零条轨迹配置（orientation=0.0, flag=0）- 仅直行+右转
TRAJ0_FORWARD_DISTANCE = 14.0     # 直行距离（米）
TRAJ0_FORWARD_STEP = 0.15        # 直行路径点间距（米）
TRAJ0_RIGHT_TURN_ANGLE = -math.pi / 2  # 右转角度（弧度）
TRAJ0_RIGHT_TURN_STEPS = 2       # 右转分几步完成（含起点终点）

# 第一条轨迹配置（orientation=0.0, flag=0）- 原地左转+前进
TRAJ1_LEFT_TURN_ANGLE = math.pi / 2   # 左转角度（弧度）
TRAJ1_LEFT_TURN_STEPS = 2        # 左转分几步完成（含起点终点）
TRAJ1_FORWARD_DISTANCE = 0.5     # 前进距离（米）
TRAJ1_FORWARD_STEP = 0.15        # 前进路径点间距（米）

# 第二条轨迹配置（orientation=0.0, flag=0）- 仅原地左转
TRAJ2_LEFT_TURN_ANGLE = math.pi / 2    # 左转角度（弧度）
TRAJ2_LEFT_TURN_STEPS = 2        # 左转分几步完成（含起点终点）

# 第三条轨迹配置（orientation=3.14, flag=1）- 倒车
TRAJ3_BACKWARD_DISTANCE = 0.3    # 倒车距离（米）
TRAJ3_BACKWARD_STEP = 0.15       # 倒车路径点间距（米）

# 容器位姿配置（第二条轨迹）
CONTAINER_TYPE = "AGV-T300"
CONTAINER_OFFSET_X = 1.0         # 容器相对轨迹终点的X偏移
CONTAINER_OFFSET_Y = 1.0         # 容器相对轨迹终点的Y偏移
CONTAINER_Z = 0.1                # 容器高度
CONTAINER_THETA = 0.0            # 容器朝向
CONTAINER_WIDTH = 1.2            # 容器宽度

# MQTT配置
MQTT_BROKER = "192.168.1.102"
MQTT_PORT = 1883
ROBOT_ID = "robot-001"

# ROS2配置
ODOM_TOPIC = "/Odom"
PATH_TOPIC = "/plans"
ODOM_TIMEOUT = 10.0              # 等待Odom超时时间（秒）

# 默认位置（Odom超时时使用）
DEFAULT_X = 0.0
DEFAULT_Y = 0.0
DEFAULT_YAW = 0.0

# 轨迹间隔时间
TRAJECTORY_INTERVAL = 5.0        # 两条轨迹之间的间隔（秒）
WAIT_FOR_MQTT_COMPLETE = 3.0     # 等待MQTT完成信号后的延迟（秒）

# ==================== 测试节点 ====================

class EnhancedTrajectoryTester(Node):
    def __init__(self):
        super().__init__('enhanced_trajectory_tester')

        # MQTT客户端配置
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # ROS2发布器和订阅器
        self.path_publisher = self.create_publisher(Path, PATH_TOPIC, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 10)

        # 状态管理
        self.trajectory_count = 0
        self.running = True
        self.current_pose = None
        self.odom_received = False
        self.mqtt_complete_received = False  # MQTT完成信号标志
        self.last_published_trajectory_id = None  # 上一次发布的轨迹ID
        self.completed_trajectory_id = None  # 完成的轨迹ID

    def odom_callback(self, msg):
        """里程计回调，持续更新当前位置"""
        # 持续更新current_pose，而不是只在第一次更新
        self.current_pose = msg.pose.pose

        if not self.odom_received:
            self.odom_received = True
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            print(f"✅ 已接收到 /Odom 话题数据")
            print(f"   当前位置: ({x:.3f}, {y:.3f}), 朝向: {math.degrees(yaw):.1f}°")

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

    def publish_zero_trajectory(self):
        """
        发布第零条轨迹
        流程：直行 → 原地右转90度
        参数：orientation=0.0, flag=0 (前向运动，非分支)
        """
        # 计算路径点数量
        num_forward_points = int(TRAJ0_FORWARD_DISTANCE / TRAJ0_FORWARD_STEP) + 1

        print("\n" + "="*80)
        print("📤 发布第零条轨迹（Beta-3: orientation=0.0, flag=0）")
        print("="*80)
        print("流程：")
        print(f"  1. 直行 {TRAJ0_FORWARD_DISTANCE}米 (点间距{TRAJ0_FORWARD_STEP}米，共{num_forward_points}个点)")
        print(f"  2. 原地右转 {math.degrees(abs(TRAJ0_RIGHT_TURN_ANGLE)):.0f}度 ({TRAJ0_RIGHT_TURN_STEPS}个点)")
        print("="*80)

        # 主动获取最新的Odom数据
        print(f"📡 获取最新Odom数据...")
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        base_x = self.current_pose.position.x
        base_y = self.current_pose.position.y
        base_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        print(f"📍 起始位置: ({base_x:.3f}, {base_y:.3f})")
        print(f"📐 起始朝向: {base_yaw:.3f} rad ({math.degrees(base_yaw):.1f}°)\n")

        # 生成轨迹ID
        trajectory_id = f"traj0_{int(time.time() * 1000)}"
        self.last_published_trajectory_id = trajectory_id

        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        # 第零条轨迹：orientation=0.0, flag=0，添加轨迹ID
        path.header.frame_id = f"map|none|none|0.0|0|0|0|0|0|0|{trajectory_id}"

        poses = []

        # 1. 直行（按固定步长生成路径点）
        for i in range(num_forward_points):
            dist = i * TRAJ0_FORWARD_STEP
            if dist > TRAJ0_FORWARD_DISTANCE:
                dist = TRAJ0_FORWARD_DISTANCE
            x = base_x + dist * math.cos(base_yaw)
            y = base_y + dist * math.sin(base_yaw)
            poses.append(self.create_pose_stamped(x, y, base_yaw))

        # 直行终点
        forward_end_x = base_x + TRAJ0_FORWARD_DISTANCE * math.cos(base_yaw)
        forward_end_y = base_y + TRAJ0_FORWARD_DISTANCE * math.sin(base_yaw)

        # 2. 原地右转（原地转弯：x,y不变，只有yaw变化）
        for i in range(1, TRAJ0_RIGHT_TURN_STEPS):
            angle_offset = (i / (TRAJ0_RIGHT_TURN_STEPS - 1)) * TRAJ0_RIGHT_TURN_ANGLE
            current_yaw = base_yaw + angle_offset
            poses.append(self.create_pose_stamped(forward_end_x, forward_end_y, current_yaw))

        path.poses = poses

        # 打印详细信息
        self.print_trajectory_details(0, path, poses, "orientation=0.0, flag=0 (前向运动，非分支)")

        self.path_publisher.publish(path)
        print(f"📡 第零条轨迹已发布到 /plans 话题")
        print(f"📋 发布的轨迹ID: {trajectory_id}\n")

    def publish_first_trajectory(self):
        """
        发布第一条轨迹
        流程：原地左转90度 → 前进
        参数：orientation=0.0, flag=0 (前向运动，非分支)
        注意：使用当前Odom位置作为起点
        """
        # 计算路径点数量
        num_forward_points = int(TRAJ1_FORWARD_DISTANCE / TRAJ1_FORWARD_STEP) + 1

        print("\n" + "="*80)
        print("📤 发布第一条轨迹（Beta-3: orientation=0.0, flag=0）")
        print("="*80)
        print("流程：")
        print(f"  1. 原地左转 {math.degrees(TRAJ1_LEFT_TURN_ANGLE):.0f}度 ({TRAJ1_LEFT_TURN_STEPS}个点)")
        print(f"  2. 前进 {TRAJ1_FORWARD_DISTANCE}米 (点间距{TRAJ1_FORWARD_STEP}米，共{num_forward_points}个点)")
        print("="*80)

        # 主动获取最新的Odom数据
        print(f"📡 获取最新Odom数据...")
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        # 使用当前Odom位置作为起点
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        start_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        print(f"📍 起始位置: ({start_x:.3f}, {start_y:.3f})")
        print(f"📐 起始朝向: {start_yaw:.3f} rad ({math.degrees(start_yaw):.1f}°)\n")

        # 生成轨迹ID
        trajectory_id = f"traj1_{int(time.time() * 1000)}"
        self.last_published_trajectory_id = trajectory_id

        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        # 第一条轨迹：orientation=0.0, flag=0，添加轨迹ID
        path.header.frame_id = f"map|none|none|0.0|0|0|0|0|0|0|{trajectory_id}"

        poses = []

        # 1. 原地左转（原地转弯：x,y不变，只有yaw变化）
        for i in range(TRAJ1_LEFT_TURN_STEPS):
            angle_offset = (i / (TRAJ1_LEFT_TURN_STEPS - 1)) * TRAJ1_LEFT_TURN_ANGLE if TRAJ1_LEFT_TURN_STEPS > 1 else 0
            current_yaw = start_yaw + angle_offset
            poses.append(self.create_pose_stamped(start_x, start_y, current_yaw))

        yaw_after_left = start_yaw + TRAJ1_LEFT_TURN_ANGLE

        # 2. 前进（按固定步长生成路径点）
        for i in range(1, num_forward_points + 1):
            dist = i * TRAJ1_FORWARD_STEP
            if dist > TRAJ1_FORWARD_DISTANCE:
                dist = TRAJ1_FORWARD_DISTANCE
            x = start_x + dist * math.cos(yaw_after_left)
            y = start_y + dist * math.sin(yaw_after_left)
            poses.append(self.create_pose_stamped(x, y, yaw_after_left))

        path.poses = poses

        # 打印详细信息
        self.print_trajectory_details(1, path, poses, "orientation=0.0, flag=0 (前向运动，非分支)")

        self.path_publisher.publish(path)
        print(f"📡 第一条轨迹已发布到 /plans 话题")
        print(f"📋 发布的轨迹ID: {trajectory_id}\n")

    def publish_second_trajectory(self):
        """
        发布第二条轨迹
        流程：仅原地左转90度
        参数：orientation=0.0, flag=0 (正常运动，非分支)
        注意：使用当前Odom位置作为起点
        """
        print("\n" + "="*80)
        print("📤 发布第二条轨迹（Beta-3: orientation=0.0, flag=0）")
        print("="*80)
        print("流程：")
        print(f"  1. 原地左转 {math.degrees(TRAJ2_LEFT_TURN_ANGLE):.0f}度 ({TRAJ2_LEFT_TURN_STEPS}个点)")
        print("="*80)

        # 主动获取最新的Odom数据
        print(f"📡 获取最新Odom数据...")
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        # 使用当前Odom位置作为起点
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        start_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        print(f"📍 起始位置: ({start_x:.3f}, {start_y:.3f})")
        print(f"📐 起始朝向: {start_yaw:.3f} rad ({math.degrees(start_yaw):.1f}°)\n")

        # 生成轨迹ID
        trajectory_id = f"traj2_{int(time.time() * 1000)}"
        self.last_published_trajectory_id = trajectory_id

        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        # 第二条轨迹：orientation=0.0, flag=0（正常前向运动）
        path.header.frame_id = f"map|none|none|0.0|0|0|0|0|0|0|{trajectory_id}"

        poses = []

        # 原地左转（原地转弯：x,y不变，只有yaw变化）
        for i in range(TRAJ2_LEFT_TURN_STEPS):
            angle_offset = (i / (TRAJ2_LEFT_TURN_STEPS - 1)) * TRAJ2_LEFT_TURN_ANGLE if TRAJ2_LEFT_TURN_STEPS > 1 else 0
            current_yaw = start_yaw + angle_offset
            poses.append(self.create_pose_stamped(start_x, start_y, current_yaw))

        path.poses = poses

        # 打印详细信息
        self.print_trajectory_details(2, path, poses, "orientation=0.0, flag=0 (正常运动，非分支)")

        self.path_publisher.publish(path)
        print(f"📡 第二条轨迹已发布到 /plans 话题")
        print(f"📋 发布的轨迹ID: {trajectory_id}\n")

    def publish_third_trajectory(self):
        """
        发布第三条轨迹
        流程：倒车
        参数：orientation=3.14, flag=1 (倒车运动，进入分支)
        注意：使用当前Odom位置作为起点，根据当前朝向倒车
        """
        # 计算路径点数量
        num_backward_points = int(TRAJ3_BACKWARD_DISTANCE / TRAJ3_BACKWARD_STEP) + 1

        print("\n" + "="*80)
        print("📤 发布第三条轨迹（Beta-3: orientation=3.14, flag=1）")
        print("="*80)
        print("流程：")
        print(f"  1. 倒车 {TRAJ3_BACKWARD_DISTANCE}米 (点间距{TRAJ3_BACKWARD_STEP}米，共{num_backward_points}个点)")
        print("="*80)

        # 主动获取最新的Odom数据
        print(f"📡 获取最新Odom数据...")
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        # 使用当前Odom位置作为起点
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        start_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        print(f"📍 起始位置: ({start_x:.3f}, {start_y:.3f})")
        print(f"📐 起始朝向: {start_yaw:.3f} rad ({math.degrees(start_yaw):.1f}°)")
        print(f"🔙 倒车方向: 沿朝向反方向移动\n")

        # 容器位姿（相对于当前位置）
        container_x = start_x + CONTAINER_OFFSET_X
        container_y = start_y + CONTAINER_OFFSET_Y

        # 生成轨迹ID
        trajectory_id = f"traj3_{int(time.time() * 1000)}"
        self.last_published_trajectory_id = trajectory_id

        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        # 第三条轨迹：orientation=3.14, flag=1，添加轨迹ID和容器信息
        path.header.frame_id = f"map|pub_unload_params|{CONTAINER_TYPE}|3.14|1|{container_x}|{container_y}|{CONTAINER_Z}|{CONTAINER_THETA}|{CONTAINER_WIDTH}|{trajectory_id}"
        # path.header.frame_id = f"map|none|none|3.14|0|0|0|0|0|0|{trajectory_id}"
        poses = []

        # 倒车（按固定步长生成路径点，沿当前朝向的反方向移动）
        for i in range(num_backward_points):
            dist = i * TRAJ3_BACKWARD_STEP
            if dist > TRAJ3_BACKWARD_DISTANCE:
                dist = TRAJ3_BACKWARD_DISTANCE
            # 倒车：沿当前朝向的反方向移动
            x = start_x - dist * math.cos(start_yaw)
            y = start_y - dist * math.sin(start_yaw)
            poses.append(self.create_pose_stamped(x, y, start_yaw))

        path.poses = poses

        # 打印详细信息
        self.print_trajectory_details(3, path, poses,
                                     f"orientation=3.14, flag=1 (倒车运动，进入分支)\n"
                                     f"                        action=pub_unload_params, containerType={CONTAINER_TYPE}")

        self.path_publisher.publish(path)
        print(f"📡 第三条轨迹已发布到 /plans 话题")
        print(f"📋 发布的轨迹ID: {trajectory_id}\n")

    def print_trajectory_details(self, traj_num, path, poses, beta3_params):
        """打印轨迹详细信息"""
        print(f"\n✅ 第{traj_num}条轨迹生成完成，共 {len(poses)} 个路径点")

        print("\n" + "="*80)
        print(f"📋 第{traj_num}条轨迹发布详情（用于对比接收方）")
        print("="*80)
        print(f"Header:")
        print(f"  frame_id: {path.header.frame_id}")
        print(f"  timestamp: {path.header.stamp.sec}.{path.header.stamp.nanosec:09d}")
        print(f"\n路径点数量: {len(poses)}")

        # 打印所有路径点
        print(f"\n所有路径点:")
        for i in range(len(poses)):
            p = poses[i].pose
            yaw = self.quaternion_to_yaw(p.orientation)
            print(f"  点{i+1}: x={p.position.x:.3f}, y={p.position.y:.3f}, yaw={yaw:.3f} ({math.degrees(yaw):.1f}°)")

        print(f"\n解析后的Beta-3参数:")
        print(f"  {beta3_params}")
        print("="*80)

    def create_pose_stamped(self, x, y, theta):
        """创建姿态点"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, theta)
        return pose

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
            # 订阅轨迹回传主题（ROS2发送到MQTT的轨迹）
            trajectory_topic = f"EP/{ROBOT_ID}/embrain/cerebellum/trajectory"
            client.subscribe(trajectory_topic)
            print(f"📡 订阅轨迹主题: {trajectory_topic}")
            # 订阅轨迹状态主题（MQTT返回的轨迹执行状态）
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
                print(f"⏰ 时间戳: {trajectory_data.get('timestamp', 'N/A')}")

                # 显示状态消息的完整内容
                print(f"\n📦 状态消息完整内容:")
                for key, value in trajectory_data.items():
                    print(f"   {key}: {value}")

                # 如果是完成状态，设置标志
                if status == 'completed':
                    print(f"\n✅ 轨迹已完成！")
                    print(f"   (注意：MQTT broker会生成自己的轨迹ID，可能与发布时的ID不同)")
                    print("="*80)
                    self.mqtt_complete_received = True
                    self.completed_trajectory_id = trajectory_id
                elif status == 'running':
                    print(f"\n🏃 轨迹执行中...")
                    print("="*80)
                elif status == 'failed':
                    print(f"\n❌ 轨迹执行失败！")
                    print("="*80)
                else:
                    print("="*80)

                return

            print("\n" + "="*80)
            print("🚀 收到Beta-3轨迹消息！")
            print("="*80)
            print(f"📋 轨迹ID: {trajectory_data.get('trajectoryId', 'N/A')}")
            print(f"⏰ 时间戳: {trajectory_data.get('timestamp', 'N/A')}")
            print(f"🏃 最大速度: {trajectory_data.get('maxSpeed', 'N/A')} m/s")

            trajectory_points = trajectory_data.get('trajectoryPoints', [])
            print(f"📍 轨迹点数量: {len(trajectory_points)}")

            if trajectory_points:
                first_point = trajectory_points[0]
                last_point = trajectory_points[-1]

                orientation = first_point.get('orientation', 'missing')
                flag = first_point.get('flag', 'missing')
                action = first_point.get('action')

                print(f"\n🔍 Beta-3字段验证:")
                print(f"   🔄 运动方向 (orientation): {orientation}")
                if orientation == 0.0:
                    print("      ✅ 前向运动")
                elif abs(orientation - 3.14) < 0.01 or abs(orientation + 3.14) < 0.01:
                    print("      ✅ 倒车运动")
                else:
                    print(f"      ⚠️  异常角度 ({orientation})")

                print(f"   🌿 分支标志 (flag): {flag}")
                if flag == 0 or flag == 0.0:
                    print("      ✅ 非分支状态")
                elif flag == 1 or flag == 1.0:
                    print("      ✅ 进入分支状态")
                else:
                    print(f"      ❌ 错误的flag值！应该只有0或1，当前值: {flag}")

                print(f"   🎯 动作 (action): {action}")
                if action is None:
                    print("      ✅ 无动作（纯行驶）")
                else:
                    print(f"      ✅ 动作类型: {action.get('actionType', 'unknown')}")
                    print(f"         容器类型: {action.get('containerType', 'none')}")

                print(f"\n📍 轨迹点信息:")
                print(f"   起点: ({first_point.get('x', 0):.3f}, {first_point.get('y', 0):.3f}), θ={first_point.get('theta', 0):.3f}")
                print(f"   终点: ({last_point.get('x', 0):.3f}, {last_point.get('y', 0):.3f}), θ={last_point.get('theta', 0):.3f}")

            self.trajectory_count += 1
            print(f"\n📊 已接收轨迹消息数量: {self.trajectory_count}")
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

    print("🧪 Beta-3协议增强型轨迹工作流程测试")
    print("=" * 80)
    print("测试配置：")
    print(f"  轨迹0: {'启用' if ENABLE_TRAJECTORY0 else '禁用'}")
    print(f"  轨迹1: {'启用' if ENABLE_TRAJECTORY1 else '禁用'}")
    print(f"  轨迹2: {'启用' if ENABLE_TRAJECTORY2 else '禁用'}")
    print(f"  轨迹3: {'启用' if ENABLE_TRAJECTORY3 else '禁用'}")
    print(f"  Odom超时: {ODOM_TIMEOUT:.0f}秒")
    print(f"  轨迹间隔: {TRAJECTORY_INTERVAL:.0f}秒")
    print("=" * 80)

    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)

    # 初始化ROS2
    rclpy.init()

    tester = EnhancedTrajectoryTester()

    # 启动MQTT监听器
    if not tester.start_mqtt_listener():
        return

    # 等待Odom话题或超时
    tester.wait_for_odom_or_timeout()

    try:
        # 发布第零条轨迹
        if ENABLE_TRAJECTORY0:
            print("\n⏱️  准备发布第零条轨迹...")
            time.sleep(1)
            tester.publish_zero_trajectory()

            # 如果启用了第一条轨迹，等待MQTT完成信号
            if ENABLE_TRAJECTORY1:
                print(f"\n⏳ 等待MQTT轨迹完成信号（无超时限制，按Ctrl+C可中断）...")
                tester.mqtt_complete_received = False
                # 持续等待完成信号，直到收到或用户中断
                while not tester.mqtt_complete_received and rclpy.ok():
                    rclpy.spin_once(tester, timeout_sec=0.5)

                if tester.mqtt_complete_received:
                    print(f"✅ 已收到完成信号，等待{WAIT_FOR_MQTT_COMPLETE:.0f}秒后发布第一条轨迹...")
                    time.sleep(WAIT_FOR_MQTT_COMPLETE)
                    # 额外spin几次确保Odom更新到最新位置
                    for _ in range(5):
                        rclpy.spin_once(tester, timeout_sec=0.1)

        # 发布第一条轨迹
        if ENABLE_TRAJECTORY1:
            tester.publish_first_trajectory()

            # 如果启用了第二条轨迹，等待MQTT完成信号
            if ENABLE_TRAJECTORY2:
                print(f"\n⏳ 等待MQTT轨迹完成信号（无超时限制，按Ctrl+C可中断）...")
                tester.mqtt_complete_received = False
                # 持续等待完成信号，直到收到或用户中断
                while not tester.mqtt_complete_received and rclpy.ok():
                    rclpy.spin_once(tester, timeout_sec=0.5)

                if tester.mqtt_complete_received:
                    print(f"✅ 已收到完成信号，等待{WAIT_FOR_MQTT_COMPLETE:.0f}秒后发布第二条轨迹...")
                    time.sleep(WAIT_FOR_MQTT_COMPLETE)
                    # 额外spin几次确保Odom更新到最新位置
                    for _ in range(5):
                        rclpy.spin_once(tester, timeout_sec=0.1)

        # 发布第二条轨迹
        if ENABLE_TRAJECTORY2:
            tester.publish_second_trajectory()

            # 如果启用了第三条轨迹，等待MQTT完成信号
            if ENABLE_TRAJECTORY3:
                print(f"\n⏳ 等待MQTT轨迹完成信号（无超时限制，按Ctrl+C可中断）...")
                tester.mqtt_complete_received = False
                # 持续等待完成信号，直到收到或用户中断
                while not tester.mqtt_complete_received and rclpy.ok():
                    rclpy.spin_once(tester, timeout_sec=0.5)

                if tester.mqtt_complete_received:
                    print(f"✅ 已收到完成信号，等待{WAIT_FOR_MQTT_COMPLETE:.0f}秒后发布第三条轨迹...")
                    time.sleep(WAIT_FOR_MQTT_COMPLETE)
                    # 额外spin几次确保Odom更新到最新位置
                    for _ in range(5):
                        rclpy.spin_once(tester, timeout_sec=0.1)

        # 发布第三条轨迹
        if ENABLE_TRAJECTORY3:
            tester.publish_third_trajectory()

        print("\n✅ 轨迹发布完成")
        print("💡 保持运行以监听MQTT轨迹消息...")
        print("   按 Ctrl+C 停止测试\n")

        # 保持ROS2节点运行，监听MQTT消息
        while tester.running and rclpy.ok():
            rclpy.spin_once(tester, timeout_sec=1.0)

    except KeyboardInterrupt:
        signal_handler(None, None)


if __name__ == '__main__':
    main()
