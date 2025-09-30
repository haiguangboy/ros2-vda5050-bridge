#!/usr/bin/env python3
"""
测试容器位姿完整工作流程

该脚本测试完整的容器位姿数据流：
1. 启动MQTT监听器（监听动作消息）
2. 启动容器位姿发布器
3. 验证数据从ROS2话题 -> MQTT桥接器 -> MQTT动作发布的完整流程
"""

import paho.mqtt.client as mqtt
import json
import threading
import time
import signal
import sys

class ContainerPoseWorkflowTester:
    def __init__(self):
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.robot_id = "robot-001"
        self.broker_host = "localhost"
        self.broker_port = 1883

        self.action_count = 0
        self.running = True

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT连接成功")
            # 订阅动作消息主题
            action_topic = f"EP/downstream/{self.robot_id}/embrain/cerebellum/action"
            client.subscribe(action_topic)
            print(f"📡 订阅动作主题: {action_topic}")
        else:
            print(f"❌ MQTT连接失败，返回代码: {rc}")

    def on_message(self, client, userdata, msg):
        try:
            # 解析动作消息
            action_data = json.loads(msg.payload.decode())

            print("\n" + "="*60)
            print("🎯 收到动作消息！")
            print("="*60)
            print(f"📋 动作ID: {action_data.get('actionId', 'N/A')}")
            print(f"🔧 动作类型: {action_data.get('actionType', 'N/A')}")
            print(f"📦 容器类型: {action_data.get('containerType', 'N/A')}")
            print(f"⏰ 时间戳: {action_data.get('timestamp', 'N/A')}")

            # 显示容器位姿信息
            if 'containerPose' in action_data:
                pose = action_data['containerPose']
                print("\n🏗️  容器位姿信息:")
                print(f"   位置: ({pose.get('x', 0):.2f}, {pose.get('y', 0):.2f}, {pose.get('z', 0):.2f})")
                print(f"   角度: {pose.get('theta', 0):.3f} 弧度 ({pose.get('theta', 0) * 180 / 3.14159:.1f}°)")
                print(f"   宽度: {pose.get('width', 0):.2f} 米")

            self.action_count += 1
            print(f"\n📊 已接收动作消息数量: {self.action_count}")
            print("="*60)

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

    def stop(self):
        """停止测试"""
        self.running = False
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        print("\n🛑 MQTT监听器已停止")

def signal_handler(sig, frame):
    """处理中断信号"""
    print("\n⏹️  收到中断信号，正在停止测试...")
    global tester
    if tester:
        tester.stop()
    sys.exit(0)

def main():
    global tester

    print("🧪 容器位姿工作流程测试")
    print("=" * 50)
    print("该测试将监听MQTT动作消息，验证完整的容器位姿处理流程：")
    print("1. Python发布器发布容器位姿到 /container_pose")
    print("2. 桥接器接收位姿并转换为ActionMessage")
    print("3. ActionMessage通过MQTT发布")
    print("4. 本测试监听并显示收到的动作消息")
    print("=" * 50)

    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)

    tester = ContainerPoseWorkflowTester()

    # 启动MQTT监听器
    if not tester.start_mqtt_listener():
        return

    print("\n🎯 测试准备就绪！")
    print("💡 请在另一个终端运行容器位姿发布器：")
    print("   cd tests/scripts")
    print("   python3 test_container_pose_publisher.py")
    print("\n⏳ 等待容器位姿数据和动作消息...")
    print("   按 Ctrl+C 停止测试")

    try:
        while tester.running:
            time.sleep(1)
    except KeyboardInterrupt:
        signal_handler(None, None)

if __name__ == '__main__':
    main()