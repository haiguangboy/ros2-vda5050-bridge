#!/usr/bin/env python3
"""
MQTT消息流诊断工具

检查：
1. MQTT broker是否可连接
2. /plans话题是否有数据
3. MQTT是否收到trajectory消息
4. MQTT是否返回trajectory_status
"""

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import json
import time


class MQTTDiagnostics:
    def __init__(self, broker="localhost", port=1883, robot_id="robot-001"):
        self.broker = broker
        self.port = port
        self.robot_id = robot_id
        self.messages_received = []

        # MQTT客户端
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"✅ MQTT连接成功: {self.broker}:{self.port}")

            # 订阅所有相关主题
            topics = [
                f"EP/{self.robot_id}/embrain/cerebellum/trajectory",
                f"EP/{self.robot_id}/cerebellum/embrain/trajectory_status",
                "EP/#"  # 订阅所有EP主题
            ]

            for topic in topics:
                client.subscribe(topic)
                print(f"📡 已订阅: {topic}")
        else:
            print(f"❌ MQTT连接失败: rc={rc}")

    def on_message(self, client, userdata, msg):
        timestamp = time.strftime("%H:%M:%S")
        print(f"\n[{timestamp}] 📩 收到MQTT消息")
        print(f"   主题: {msg.topic}")

        try:
            payload = json.loads(msg.payload.decode())
            print(f"   内容: {json.dumps(payload, indent=2, ensure_ascii=False)}")
            self.messages_received.append({
                'topic': msg.topic,
                'payload': payload,
                'timestamp': timestamp
            })
        except:
            print(f"   内容（非JSON）: {msg.payload.decode()}")

    def start(self):
        """启动MQTT监听"""
        try:
            print(f"\n{'='*80}")
            print("🔍 MQTT消息流诊断工具")
            print(f"{'='*80}")
            print(f"Broker: {self.broker}:{self.port}")
            print(f"Robot ID: {self.robot_id}\n")

            self.mqtt_client.connect(self.broker, self.port, 60)
            self.mqtt_client.loop_start()

            print("✅ 诊断工具已启动")
            print("💡 现在请在另一个终端执行以下操作：")
            print("   1. 启动轨迹规划节点: python3 test_beta4_trajectory_workflow_goal.py")
            print("   2. 发布目标点: python3 publish_goal.py --x 3.0 --y 0.0 --yaw 90")
            print("   3. 观察这里是否收到MQTT消息\n")
            print("按 Ctrl+C 停止诊断\n")

            # 持续监听
            while True:
                time.sleep(1)

        except KeyboardInterrupt:
            print("\n\n" + "="*80)
            print("📊 诊断结果汇总")
            print("="*80)
            print(f"共收到 {len(self.messages_received)} 条MQTT消息")

            if self.messages_received:
                print("\n消息列表:")
                for i, msg in enumerate(self.messages_received, 1):
                    print(f"\n  {i}. [{msg['timestamp']}] {msg['topic']}")
                    if 'trajectoryId' in msg['payload']:
                        print(f"     轨迹ID: {msg['payload'].get('trajectoryId')}")
                    if 'status' in msg['payload']:
                        print(f"     状态: {msg['payload'].get('status')}")
            else:
                print("\n⚠️  没有收到任何MQTT消息！")
                print("\n可能的原因：")
                print("  1. MQTT Bridge未运行")
                print("  2. /plans话题没有订阅器（MQTT Bridge未订阅）")
                print("  3. 话题名称不匹配")
                print("  4. MQTT broker配置不正确")

        except Exception as e:
            print(f"\n❌ 错误: {e}")
        finally:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()


class ROSPathListener(Node):
    """监听ROS2 /plans话题"""

    def __init__(self):
        super().__init__('ros_path_listener')
        self.subscription = self.create_subscription(
            Path, '/plans', self.path_callback, 10)
        self.path_received = False

        print("\n🔍 同时监听ROS2 /plans话题...")

    def path_callback(self, msg):
        if not self.path_received:
            self.path_received = True
            print(f"\n✅ 收到ROS2 /plans消息")
            print(f"   路径点数: {len(msg.poses)}")
            print(f"   Frame ID: {msg.header.frame_id}")


def main():
    # 启动MQTT诊断
    diagnostics = MQTTDiagnostics()

    # 启动ROS2监听
    rclpy.init()
    ros_listener = ROSPathListener()

    import threading

    def ros_spin():
        rclpy.spin(ros_listener)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # 运行MQTT诊断
    diagnostics.start()

    # 清理
    ros_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
