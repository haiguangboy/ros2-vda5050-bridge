#!/usr/bin/env python3
"""
测试beta-3增强型轨迹工作流程（包含orientation和flag字段）

该脚本测试完整的beta-3轨迹数据流：
1. 发布包含新字段的ROS2路径数据
2. 桥接器接收路径并转换为TrajectoryMessage（包含orientation和flag字段）
3. TrajectoryMessage通过MQTT发布
4. 监听并验证收到的轨迹消息包含正确的新字段和动作参数
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import threading
import time
import signal
import sys
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
import math


class Beta3TrajectoryWorkflowTester(Node):
    def __init__(self):
        super().__init__('beta3_trajectory_tester')

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

        # 统计信息
        self.trajectory_count = 0
        self.running = True
        self.current_pose = None
        self.paths_published = False

        # 测试数据（将在获得位置后生成）
        self.test_paths = []

    def odom_callback(self, msg):
        """里程计回调，获取当前位置"""
        if not self.paths_published:
            self.current_pose = msg.pose.pose
            self.test_paths = self.create_test_paths_from_current_pose()
            self.paths_published = True
            self.get_logger().info('已获取当前位置，测试路径已生成')

    def create_test_paths_from_current_pose(self):
        """基于当前位置创建包含beta-3新字段的测试路径"""
        if self.current_pose is None:
            return []

        test_paths = []
        base_x = self.current_pose.position.x
        base_y = self.current_pose.position.y
        base_z = self.current_pose.position.z
        base_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        # 测试路径1：前向运动 + pub_load_params动作
        path1 = self.create_forward_path(
            base_x, base_y, base_z, base_yaw,
            distance=0.5,  # 前进0.5米
            action_type="pub_load_params",
            container_type="AGV-T300",
            container_pose=(base_x + 0.3, base_y + 0.3, 0.1, 3.14, 1.2),
            orientation=0.0,  # 前向运动
            flag=0.0  # 非进入分支
        )
        test_paths.append(("前向运动+发布取货参数", path1))

        # 测试路径2：倒车运动 + pub_unload_params动作
        path2 = self.create_backward_path(
            base_x, base_y, base_z, base_yaw,
            distance=0.3,  # 倒车0.3米
            action_type="pub_unload_params",
            container_type="container",
            container_pose=(base_x - 0.2, base_y - 0.2, 0.2, -1.57, 0.8),
            orientation=-3.14,  # 倒车
            flag=1.0  # 进入分支
        )
        test_paths.append(("倒车运动+发布放货参数", path2))

        # 测试路径3：前向转弯 + start_stacking动作
        path3 = self.create_turn_path(
            base_x, base_y, base_z, base_yaw,
            turn_angle=math.pi/4,  # 转45度
            distance=0.4,  # 转弯后前进0.4米
            action_type="start_stacking",
            container_type="pallet",
            container_pose=(base_x + 0.4, base_y + 0.4, 0.5, 1.57, 1.5),
            orientation=0.0,  # 前向运动
            flag=0.5  # 半分支状态
        )
        test_paths.append(("前向转弯+启动堆垛", path3))

        # 测试路径4：混合运动方向的路径
        path4 = self.create_mixed_motion_path(base_x, base_y, base_z, base_yaw)
        test_paths.append(("混合运动方向测试", path4))

        # 测试路径5：复杂分支标志测试
        path5 = self.create_branch_sequence_path(base_x, base_y, base_z, base_yaw)
        test_paths.append(("分支序列测试", path5))

        return test_paths

    def create_forward_path(self, base_x, base_y, base_z, base_yaw, distance,
                           action_type, container_type, container_pose, orientation, flag):
        """创建前向运动路径"""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()

        # 通过frame_id编码beta-3特定信息
        container_x, container_y, container_z, container_theta, container_width = container_pose
        path.header.frame_id = f"map|{action_type}|{container_type}|{orientation}|{flag}|{container_x}|{container_y}|{container_z}|{container_theta}|{container_width}"

        # 沿当前朝向前进
        poses = []
        for i in range(3):  # 3个点
            forward_dist = i * (distance / 2)
            x = base_x + forward_dist * math.cos(base_yaw)
            y = base_y + forward_dist * math.sin(base_yaw)
            pose = self.create_pose_stamped(x, y, base_yaw)
            poses.append(pose)

        path.poses = poses
        return path

    def create_backward_path(self, base_x, base_y, base_z, base_yaw, distance,
                            action_type, container_type, container_pose, orientation, flag):
        """创建倒车运动路径"""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()

        # 通过frame_id编码beta-3特定信息
        container_x, container_y, container_z, container_theta, container_width = container_pose
        path.header.frame_id = f"map|{action_type}|{container_type}|{orientation}|{flag}|{container_x}|{container_y}|{container_z}|{container_theta}|{container_width}"

        # 沿当前朝向反方向后退
        poses = []
        for i in range(3):  # 3个点
            backward_dist = i * (distance / 2)
            x = base_x - backward_dist * math.cos(base_yaw)
            y = base_y - backward_dist * math.sin(base_yaw)
            # 倒车时车头朝向调转180度
            backward_yaw = base_yaw + math.pi
            pose = self.create_pose_stamped(x, y, backward_yaw)
            poses.append(pose)

        path.poses = poses
        return path

    def create_turn_path(self, base_x, base_y, base_z, base_yaw, turn_angle, distance,
                        action_type, container_type, container_pose, orientation, flag):
        """创建转弯路径"""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()

        # 通过frame_id编码beta-3特定信息
        container_x, container_y, container_z, container_theta, container_width = container_pose
        path.header.frame_id = f"map|{action_type}|{container_type}|{orientation}|{flag}|{container_x}|{container_y}|{container_z}|{container_theta}|{container_width}"

        poses = []
        # 起点
        poses.append(self.create_pose_stamped(base_x, base_y, base_yaw))

        # 转弯中点
        mid_yaw = base_yaw + turn_angle / 2
        mid_x = base_x + (distance / 3) * math.cos(mid_yaw)
        mid_y = base_y + (distance / 3) * math.sin(mid_yaw)
        poses.append(self.create_pose_stamped(mid_x, mid_y, mid_yaw))

        # 终点
        end_yaw = base_yaw + turn_angle
        end_x = base_x + distance * math.cos(end_yaw)
        end_y = base_y + distance * math.sin(end_yaw)
        poses.append(self.create_pose_stamped(end_x, end_y, end_yaw))

        path.poses = poses
        return path

    def create_mixed_motion_path(self, base_x, base_y, base_z, base_yaw):
        """创建包含不同运动方向的路径"""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        # 混合运动：前向-侧向-倒车的组合
        path.header.frame_id = f"map|mixed_motion|none|0.0|0.0|0|0|0|0|0"

        poses = []
        # 前向运动点
        poses.append(self.create_pose_stamped(base_x, base_y, base_yaw))

        # 侧向运动点（90度转向）
        side_yaw = base_yaw + math.pi/2
        side_x = base_x + 0.2 * math.cos(side_yaw)
        side_y = base_y + 0.2 * math.sin(side_yaw)
        poses.append(self.create_pose_stamped(side_x, side_y, side_yaw))

        # 倒车运动点（180度转向）
        back_yaw = base_yaw + math.pi
        back_x = side_x + 0.2 * math.cos(back_yaw)
        back_y = side_y + 0.2 * math.sin(back_yaw)
        poses.append(self.create_pose_stamped(back_x, back_y, back_yaw))

        # 回到前向
        final_yaw = base_yaw
        final_x = back_x + 0.2 * math.cos(final_yaw)
        final_y = back_y + 0.2 * math.sin(final_yaw)
        poses.append(self.create_pose_stamped(final_x, final_y, final_yaw))

        path.poses = poses
        return path

    def create_branch_sequence_path(self, base_x, base_y, base_z, base_yaw):
        """创建分支序列测试路径"""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        # 分支序列：非分支->分支->非分支
        path.header.frame_id = f"map|branch_sequence|none|0.0|0.0|0|0|0|0|0"

        poses = []
        # 非分支起点
        poses.append(self.create_pose_stamped(base_x, base_y, base_yaw))

        # 分支中点
        branch_x = base_x + 0.15 * math.cos(base_yaw)
        branch_y = base_y + 0.15 * math.sin(base_yaw)
        poses.append(self.create_pose_stamped(branch_x, branch_y, base_yaw))

        # 非分支终点
        end_x = base_x + 0.3 * math.cos(base_yaw)
        end_y = base_y + 0.3 * math.sin(base_yaw)
        poses.append(self.create_pose_stamped(end_x, end_y, base_yaw))

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
            # 订阅轨迹消息主题
            trajectory_topic = f"EP/{self.robot_id}/embrain/cerebellum/trajectory"
            client.subscribe(trajectory_topic)
            print(f"📡 订阅轨迹主题: {trajectory_topic}")
        else:
            print(f"❌ MQTT连接失败，返回代码: {rc}")

    def on_message(self, client, userdata, msg):
        try:
            # 解析轨迹消息
            trajectory_data = json.loads(msg.payload.decode())

            print("\\n" + "="*80)
            print("🚀 收到beta-3轨迹消息！")
            print("="*80)
            print(f"📋 轨迹ID: {trajectory_data.get('trajectoryId', 'N/A')}")
            print(f"⏰ 时间戳: {trajectory_data.get('timestamp', 'N/A')}")
            print(f"🏃 最大速度: {trajectory_data.get('maxSpeed', 'N/A')} m/s")

            # 分析轨迹点和新字段
            trajectory_points = trajectory_data.get('trajectoryPoints', [])
            print(f"📍 轨迹点数量: {len(trajectory_points)}")

            action_points = []
            orientation_summary = {}
            flag_summary = {}

            for i, point in enumerate(trajectory_points):
                # 统计orientation和flag
                orientation = point.get('orientation', 0.0)
                flag = point.get('flag', 0.0)

                orientation_type = "前向" if orientation == 0.0 else "倒车"
                flag_type = "非分支" if flag == 0.0 else "分支"

                orientation_summary[orientation_type] = orientation_summary.get(orientation_type, 0) + 1
                flag_summary[flag_type] = flag_summary.get(flag_type, 0) + 1

                print(f"\\n  点 {i+1}: ({point.get('x', 0):.2f}, {point.get('y', 0):.2f}) "
                      f"角度: {point.get('theta', 0):.3f} 弧度")
                print(f"    🔄 运动方向: {orientation_type} (orientation={orientation})")
                print(f"    🌿 分支标志: {flag_type} (flag={flag})")

                # 检查动作参数
                if 'action' in point:
                    if point['action'] is not None:
                        action = point['action']
                        action_points.append(i+1)
                        print(f"    🎯 动作类型: {action.get('actionType', 'N/A')}")

                        if 'containerType' in action and action['containerType']:
                            print(f"    📦 容器类型: {action.get('containerType', 'N/A')}")

                        if 'containerPose' in action and action['containerPose']:
                            container_pose = action['containerPose']
                            print(f"    🏗️  容器位姿:")
                            print(f"       位置: ({container_pose.get('x', 0):.2f}, "
                                  f"{container_pose.get('y', 0):.2f}, "
                                  f"{container_pose.get('z', 0):.2f})")
                            print(f"       角度: {container_pose.get('theta', 0):.3f} 弧度 "
                                  f"({container_pose.get('theta', 0) * 180 / 3.14159:.1f}°)")
                            print(f"       宽度: {container_pose.get('width', 0):.2f} 米")
                    else:
                        print(f"    ✅ 动作: null (纯行驶)")

            # 总结beta-3新特性
            print(f"\\n🔄 运动方向统计: {orientation_summary}")
            print(f"🌿 分支标志统计: {flag_summary}")

            if action_points:
                print(f"🎬 包含动作的轨迹点: {action_points}")

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
            print("🚀 启动MQTT监听器")
            print(f"📡 连接到MQTT代理: {self.broker_host}:{self.broker_port}")

            self.mqtt_client.connect(self.broker_host, self.broker_port, 60)
            self.mqtt_client.loop_start()
            return True
        except Exception as e:
            print(f"❌ MQTT连接失败: {e}")
            return False

    def publish_test_paths(self):
        """发布测试路径"""
        if not self.test_paths:
            print("\\n❌ 无测试路径可发布，等待获取当前位置...")
            return

        print("\\n🎯 开始发布beta-3测试路径...")
        print(f"   基于当前位置: ({self.current_pose.position.x:.3f}, {self.current_pose.position.y:.3f})")
        print(f"   当前朝向: {math.degrees(self.quaternion_to_yaw(self.current_pose.orientation)):.1f}°")

        for i, (description, path) in enumerate(self.test_paths):
            print(f"\\n📤 发布测试 {i+1}/{len(self.test_paths)}: {description}")
            print(f"   路径点数量: {len(path.poses)}")
            if path.poses:
                print(f"   起点: ({path.poses[0].pose.position.x:.3f}, {path.poses[0].pose.position.y:.3f})")
                print(f"   终点: ({path.poses[-1].pose.position.x:.3f}, {path.poses[-1].pose.position.y:.3f})")
                print(f"   Frame ID: {path.header.frame_id}")

            self.path_publisher.publish(path)

            # 等待处理
            time.sleep(4)

        print("\\n✅ 所有beta-3测试路径已发布")

    def stop(self):
        """停止测试"""
        self.running = False
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        print("\\n🛑 beta-3测试已停止")


def signal_handler(sig, frame):
    """处理中断信号"""
    print("\\n⏹️  收到中断信号，正在停止beta-3测试...")
    global tester
    if tester:
        tester.stop()
    rclpy.shutdown()
    sys.exit(0)


def main():
    global tester

    print("🧪 beta-3轨迹工作流程测试")
    print("=" * 60)
    print("该测试将验证beta-3协议的新特性：")
    print("1. orientation字段（运动方向）")
    print("2. flag字段（进入分支标志位）")
    print("3. 新的动作类型（pub_load_params, pub_unload_params, start_stacking）")
    print("4. 完整的ROS2 → MQTT轨迹数据流")
    print("=" * 60)

    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)

    # 初始化ROS2
    rclpy.init()

    tester = Beta3TrajectoryWorkflowTester()

    # 启动MQTT监听器
    if not tester.start_mqtt_listener():
        return

    print("\\n🎯 beta-3测试准备就绪！")
    print("💡 确保桥接器正在运行：")
    print("   ./install/ros2_zhongli_bridge_cpp/bin/zhongli_bridge_node")
    print("\\n⏳ 5秒后开始发布beta-3测试路径...")

    # 等待获取当前位置和生成路径
    print("\\n⏳ 等待获取当前位置...")
    while not tester.paths_published and rclpy.ok():
        rclpy.spin_once(tester, timeout_sec=1.0)

    if not tester.paths_published:
        print("\\n❌ 未能获取当前位置，请确保/Odom话题正在发布")
        return

    print("\\n⏳ 5秒后开始发布beta-3测试路径...")
    time.sleep(5)

    try:
        # 发布测试路径
        tester.publish_test_paths()

        print("\\n⏳ 等待轨迹消息接收...")
        print("   按 Ctrl+C 停止测试")

        # 保持ROS2节点运行
        while tester.running and rclpy.ok():
            rclpy.spin_once(tester, timeout_sec=1.0)

    except KeyboardInterrupt:
        signal_handler(None, None)


if __name__ == '__main__':
    main()