#!/usr/bin/env python3
"""
Beta-3 动态轨迹工作流程测试

测试架构：
1. 监听 /Odom 话题获取当前位置
2. 发布基本路径（orientation=0.0, flag=0.0, action=null）
3. 监听 /action_command 话题接收动作指令
4. 收到动作后立即更新轨迹：
   - flag: 0 → 1 (进入分支)
   - orientation: 根据动作类型动态调整 (unload类需要掉头: 0 → 3.14)
   - action: null → 具体动作内容
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import time
import signal
import sys
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header, String
import math


class Beta3DynamicWorkflowTester(Node):
    def __init__(self):
        super().__init__('beta3_dynamic_tester')

        # MQTT客户端配置
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.robot_id = "robot-001"
        self.broker_host = "localhost"
        self.broker_port = 1883

        # ROS2发布器和订阅器
        self.path_publisher = self.create_publisher(Path, '/plans', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/Odom', self.odom_callback, 10)
        self.action_subscriber = self.create_subscription(
            String, '/action_command', self.action_callback, 10)

        # 状态管理
        self.trajectory_count = 0
        self.running = True
        self.current_pose = None
        self.current_action = None
        self.basic_path_published = False

        # 测试计数器
        self.basic_path_counter = 0

    def odom_callback(self, msg):
        """里程计回调，获取当前位置并发布基本路径"""
        self.current_pose = msg.pose.pose

        # 每隔一定时间发布一次基本路径（模拟导航系统）
        if not self.basic_path_published or self.basic_path_counter % 50 == 0:
            self.publish_basic_path()
            self.basic_path_published = True

        self.basic_path_counter += 1

    def action_callback(self, msg):
        """动作指令回调，接收动作消息并动态更新轨迹"""
        try:
            action_data = json.loads(msg.data)
            self.current_action = action_data

            print(f"\\n🎯 收到动作指令: {action_data.get('actionType', 'unknown')}")
            print(f"   容器类型: {action_data.get('containerType', 'none')}")

            # 立即发布包含动作的更新路径
            self.publish_action_updated_path()

        except json.JSONDecodeError:
            print(f"❌ 动作消息JSON解析失败: {msg.data}")
        except Exception as e:
            print(f"❌ 处理动作消息错误: {e}")

    def publish_basic_path(self):
        """发布基本路径（默认值：无动作，前向运动，非分支）"""
        if self.current_pose is None:
            return

        base_x = self.current_pose.position.x
        base_y = self.current_pose.position.y
        base_z = self.current_pose.position.z
        base_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        # 创建基本前向路径
        path = self.create_basic_forward_path(base_x, base_y, base_z, base_yaw, 0.5)

        print(f"\\n📤 发布基本路径 (#{self.basic_path_counter})")
        print(f"   位置: ({base_x:.3f}, {base_y:.3f}), 朝向: {math.degrees(base_yaw):.1f}°")
        print(f"   默认值: orientation=0.0, flag=0.0, action=null")

        self.path_publisher.publish(path)

    def publish_action_updated_path(self):
        """发布包含动作信息的更新路径"""
        if self.current_pose is None or self.current_action is None:
            return

        base_x = self.current_pose.position.x
        base_y = self.current_pose.position.y
        base_z = self.current_pose.position.z
        base_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        action_type = self.current_action.get('actionType', 'unknown')
        container_type = self.current_action.get('containerType', 'none')
        container_pose_data = self.current_action.get('containerPose', {})

        # 构建容器位姿
        container_pose = (
            container_pose_data.get('x', base_x),
            container_pose_data.get('y', base_y),
            container_pose_data.get('z', 0.1),
            container_pose_data.get('theta', 0.0),
            container_pose_data.get('width', 1.0)
        )

        # 根据动作类型决定运动模式
        if 'unload' in action_type.lower() or 'place' in action_type.lower():
            # 卸货/放置动作：需要掉头
            orientation = 3.14  # 掉头
            flag = 1.0  # 进入分支
            description = f"动作触发：{action_type}（掉头倒车）"
            path = self.create_action_path(
                base_x, base_y, base_z, base_yaw,
                action_type, container_type, container_pose,
                orientation, flag, 0.3, is_backward=True
            )
        else:
            # 取货/其他动作：前向运动但进入分支
            orientation = 0.0  # 前向
            flag = 1.0  # 进入分支
            description = f"动作触发：{action_type}（前向分支）"
            path = self.create_action_path(
                base_x, base_y, base_z, base_yaw,
                action_type, container_type, container_pose,
                orientation, flag, 0.4, is_backward=False
            )

        print(f"\\n🔄 发布动作更新路径: {description}")
        print(f"   更新后: orientation={orientation}, flag={flag}")
        print(f"   动作: {action_type}, 容器: {container_type}")

        self.path_publisher.publish(path)

    def create_basic_forward_path(self, base_x, base_y, base_z, base_yaw, distance):
        """创建基本前向运动路径（默认值）"""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        # 默认值：无动作，前向运动，非分支
        path.header.frame_id = "map|none|none|0.0|0.0|0|0|0|0|0"

        poses = []
        for i in range(3):  # 3个点
            forward_dist = i * (distance / 2)
            x = base_x + forward_dist * math.cos(base_yaw)
            y = base_y + forward_dist * math.sin(base_yaw)
            pose = self.create_pose_stamped(x, y, base_yaw)
            poses.append(pose)

        path.poses = poses
        return path

    def create_action_path(self, base_x, base_y, base_z, base_yaw,
                          action_type, container_type, container_pose,
                          orientation, flag, distance, is_backward=False):
        """创建包含动作的路径"""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()

        # 编码动作信息到frame_id
        container_x, container_y, container_z, container_theta, container_width = container_pose
        path.header.frame_id = f"map|{action_type}|{container_type}|{orientation}|{flag}|{container_x}|{container_y}|{container_z}|{container_theta}|{container_width}"

        poses = []
        if is_backward:
            # 倒车路径
            for i in range(3):
                backward_dist = i * (distance / 2)
                x = base_x - backward_dist * math.cos(base_yaw)
                y = base_y - backward_dist * math.sin(base_yaw)
                # 掉头朝向
                backward_yaw = base_yaw + math.pi
                pose = self.create_pose_stamped(x, y, backward_yaw)
                poses.append(pose)
        else:
            # 前向路径
            for i in range(3):
                forward_dist = i * (distance / 2)
                x = base_x + forward_dist * math.cos(base_yaw)
                y = base_y + forward_dist * math.sin(base_yaw)
                pose = self.create_pose_stamped(x, y, base_yaw)
                poses.append(pose)

        path.poses = poses
        return path

    def create_pose_stamped(self, x, y, theta):
        """创建姿态点"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # 将欧拉角转换为四元数
        quat = self.euler_to_quaternion(0.0, 0.0, theta)
        pose.pose.orientation = quat

        return pose

    def quaternion_to_yaw(self, q):
        """将四元数转换为yaw角度（弧度）"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw
        return quat

    # MQTT回调函数
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT连接成功")
            trajectory_topic = f"EP/{self.robot_id}/embrain/cerebellum/trajectory"
            client.subscribe(trajectory_topic)
            print(f"📡 订阅轨迹主题: {trajectory_topic}")
        else:
            print(f"❌ MQTT连接失败，返回代码: {rc}")

    def on_message(self, client, userdata, msg):
        try:
            trajectory_data = json.loads(msg.payload.decode())

            print("\\n" + "="*80)
            print("🚀 收到beta-3轨迹消息！")
            print("="*80)
            print(f"📋 轨迹ID: {trajectory_data.get('trajectoryId', 'N/A')}")
            print(f"⏰ 时间戳: {trajectory_data.get('timestamp', 'N/A')}")
            print(f"🏃 最大速度: {trajectory_data.get('maxSpeed', 'N/A')} m/s")

            # 分析轨迹点和beta-3字段
            trajectory_points = trajectory_data.get('trajectoryPoints', [])
            print(f"📍 轨迹点数量: {len(trajectory_points)}")

            if trajectory_points:
                first_point = trajectory_points[0]
                orientation = first_point.get('orientation', 'missing')
                flag = first_point.get('flag', 'missing')
                action = first_point.get('action')

                print(f"\\n🔍 Beta-3字段分析:")
                print(f"   🔄 运动方向 (orientation): {orientation}")
                if orientation == 0.0:
                    print("      → 前向运动")
                elif abs(orientation - 3.14) < 0.1 or abs(orientation + 3.14) < 0.1:
                    print("      → 倒车运动")
                else:
                    print(f"      → 其他角度 ({math.degrees(orientation):.1f}°)")

                print(f"   🌿 分支标志 (flag): {flag}")
                if flag == 0.0:
                    print("      → 非分支状态")
                elif flag == 1.0:
                    print("      → 进入分支状态")
                else:
                    print(f"      → 自定义分支状态 ({flag})")

                print(f"   🎯 动作 (action): {action}")
                if action is None:
                    print("      → 无动作（纯行驶）")
                else:
                    print(f"      → 动作类型: {action.get('actionType', 'unknown')}")
                    print(f"      → 容器类型: {action.get('containerType', 'none')}")

            self.trajectory_count += 1
            print(f"\\n📊 已接收轨迹消息数量: {self.trajectory_count}")
            print("="*80)

        except json.JSONDecodeError:
            print(f"❌ JSON解析失败: {msg.payload.decode()}")
        except Exception as e:
            print(f"❌ 消息处理错误: {e}")

    def start_mqtt_listener(self):
        """启动MQTT监听器"""
        try:
            print(f"🚀 连接到MQTT代理: {self.broker_host}:{self.broker_port}")
            self.mqtt_client.connect(self.broker_host, self.broker_port, 60)
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
        print("\\n🛑 beta-3动态测试已停止")


def signal_handler(sig, frame):
    """处理中断信号"""
    print("\\n⏹️ 收到中断信号，正在停止beta-3动态测试...")
    global tester
    if tester:
        tester.stop()
    rclpy.shutdown()
    sys.exit(0)


def main():
    global tester

    print("🧪 Beta-3 动态轨迹工作流程测试")
    print("=" * 60)
    print("测试架构：")
    print("1. 监听 /Odom 话题 → 发布基本路径（默认值）")
    print("2. 监听 /action_command 话题 → 动态更新轨迹")
    print("3. 验证 orientation 和 flag 字段的动态变化")
    print("=" * 60)

    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)

    # 初始化ROS2
    rclpy.init()

    tester = Beta3DynamicWorkflowTester()

    # 启动MQTT监听器
    if not tester.start_mqtt_listener():
        return

    print("\\n🎯 beta-3动态测试准备就绪！")
    print("💡 确保桥接器正在运行：")
    print("   ./install/ros2_zhongli_bridge_cpp/bin/zhongli_bridge_node")
    print("\\n📋 测试流程：")
    print("   1. 等待获取当前位置...")
    print("   2. 自动发布基本路径（默认值）")
    print("   3. 发送动作指令观察字段变化")

    # 等待获取当前位置
    print("\\n⏳ 等待获取当前位置...")
    while not tester.basic_path_published and rclpy.ok():
        rclpy.spin_once(tester, timeout_sec=1.0)

    if not tester.basic_path_published:
        print("\\n❌ 未能获取当前位置，请确保/Odom话题正在发布")
        return

    print("\\n✅ 已获取当前位置并开始发布基本路径")
    print("\\n💡 测试动作指令示例:")
    print('   取货动作: ros2 topic pub /action_command std_msgs/String "data: \\'{\"actionType\": \"pub_load_params\", \"containerType\": \"AGV-T300\", \"containerPose\": {\"x\": 1.0, \"y\": 1.0, \"z\": 0.1, \"theta\": 0.0, \"width\": 1.2}}\\'\"\\'\"')
    print('   卸货动作: ros2 topic pub /action_command std_msgs/String "data: \\'{\"actionType\": \"pub_unload_params\", \"containerType\": \"container\", \"containerPose\": {\"x\": 2.0, \"y\": 2.0, \"z\": 0.2, \"theta\": 1.57, \"width\": 0.8}}\\'\"\\'\"')

    try:
        print("\\n🔄 进入监听模式，等待动作指令...")
        print("   按 Ctrl+C 停止测试")

        # 保持ROS2节点运行，监听动作指令
        while tester.running and rclpy.ok():
            rclpy.spin_once(tester, timeout_sec=1.0)

    except KeyboardInterrupt:
        signal_handler(None, None)


if __name__ == '__main__':
    main()