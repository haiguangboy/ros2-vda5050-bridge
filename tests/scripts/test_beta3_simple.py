#!/usr/bin/env python3
"""
简单的beta-3协议测试
验证frame_id解析和orientation/flag字段正确性
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
import math


class SimpleBeta3Tester(Node):
    def __init__(self):
        super().__init__('simple_beta3_tester')

        # MQTT客户端配置
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.robot_id = "robot-001"
        self.broker_host = "localhost"
        self.broker_port = 1883

        # ROS2发布器
        self.path_publisher = self.create_publisher(Path, '/plans', 10)

        self.received_trajectory = None

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
            print(f"\\n🚀 收到轨迹消息: {trajectory_data.get('trajectoryId', 'N/A')}")

            # 检查轨迹点的beta-3字段
            trajectory_points = trajectory_data.get('trajectoryPoints', [])
            for i, point in enumerate(trajectory_points):
                orientation = point.get('orientation', 'missing')
                flag = point.get('flag', 'missing')
                print(f"  点 {i+1}: orientation={orientation}, flag={flag}")

            self.received_trajectory = trajectory_data

        except json.JSONDecodeError as e:
            print(f"❌ JSON解析失败: {e}")
        except Exception as e:
            print(f"❌ 消息处理错误: {e}")

    def create_test_path_with_beta3_info(self, orientation, flag):
        """创建包含beta-3信息的测试路径"""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()

        # 编码beta-3信息到frame_id
        path.header.frame_id = f"map|pub_load_params|AGV-T300|{orientation}|{flag}|1.5|1.5|0.1|3.14|1.2"

        # 创建简单的3点路径
        poses = [
            self.create_pose_stamped(0.0, 0.0, 0.0),
            self.create_pose_stamped(1.0, 1.0, 0.0),
            self.create_pose_stamped(2.0, 2.0, 0.0)
        ]
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

    def test_orientation_and_flag(self):
        """测试不同的orientation和flag值"""
        test_cases = [
            (0.0, 0.0, "前向运动，非分支"),
            (-3.14, 0.0, "倒车运动，非分支"),
            (0.0, 1.0, "前向运动，分支"),
            (-3.14, 1.0, "倒车运动，分支"),
            (3.14, 0.5, "倒车运动，半分支（边界测试）")
        ]

        for orientation, flag, description in test_cases:
            print(f"\\n🧪 测试: {description}")
            print(f"   发送 orientation={orientation}, flag={flag}")

            # 创建并发布路径
            path = self.create_test_path_with_beta3_info(orientation, flag)
            self.path_publisher.publish(path)

            # 等待接收轨迹消息
            self.received_trajectory = None
            start_time = time.time()
            while self.received_trajectory is None and (time.time() - start_time) < 3.0:
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.received_trajectory:
                # 验证接收到的值
                points = self.received_trajectory.get('trajectoryPoints', [])
                if points:
                    received_orientation = points[0].get('orientation')
                    received_flag = points[0].get('flag')

                    orientation_ok = abs(received_orientation - orientation) < 0.01
                    flag_ok = abs(received_flag - flag) < 0.01

                    if orientation_ok and flag_ok:
                        print(f"   ✅ 成功: 接收 orientation={received_orientation}, flag={received_flag}")
                    else:
                        print(f"   ❌ 失败: 期望 orientation={orientation}, flag={flag}")
                        print(f"       实际 orientation={received_orientation}, flag={received_flag}")
                else:
                    print(f"   ❌ 失败: 没有轨迹点")
            else:
                print(f"   ❌ 失败: 未收到轨迹消息")

            time.sleep(1)  # 间隔1秒

    def stop(self):
        """停止测试"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()


def main():
    print("🧪 简单beta-3协议测试")
    print("=" * 50)

    rclpy.init()
    tester = SimpleBeta3Tester()

    # 启动MQTT监听器
    if not tester.start_mqtt_listener():
        return

    print("\\n⏳ 等待3秒让系统准备...")
    time.sleep(3)

    try:
        # 运行测试
        tester.test_orientation_and_flag()
        print("\\n✅ 测试完成")

    except KeyboardInterrupt:
        print("\\n⏹️ 测试被中断")
    finally:
        tester.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()