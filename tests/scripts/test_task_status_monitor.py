#!/usr/bin/env python3
# test_task_status_monitor.py
#
# 测试MQTT任务状态监控功能
# 监听桥接器发布的任务状态消息
# 主题: EP/master/{robotId}/task_status
#
# ⚠️  仅用于测试目的！
# 在生产环境中，真实的调度系统会监听任务状态消息

import paho.mqtt.client as mqtt
import json
import time
from datetime import datetime

# --- Configuration ---
MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883
ROBOT_ID = "robot-001"
TASK_STATUS_TOPIC = f"EP/master/{ROBOT_ID}/task_status"

class TaskStatusMonitor:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

        self.connected = False
        self.message_count = 0

    def on_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            self.connected = True
            print(f"✅ 已连接到MQTT Broker: {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
            print(f"📥 订阅任务状态主题: {TASK_STATUS_TOPIC}")

            # 订阅任务状态主题
            result = client.subscribe(TASK_STATUS_TOPIC, qos=1)
            if result[0] == mqtt.MQTT_ERR_SUCCESS:
                print(f"✅ 成功订阅主题: {TASK_STATUS_TOPIC}")
            else:
                print(f"❌ 订阅失败，错误码: {result[0]}")
        else:
            print(f"❌ MQTT连接失败，返回码: {rc}")

    def on_message(self, client, userdata, msg):
        """接收到消息回调"""
        try:
            self.message_count += 1

            print(f"\n📩 收到任务状态消息 #{self.message_count}")
            print(f"   主题: {msg.topic}")
            print(f"   时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

            # 解析JSON消息
            try:
                message_data = json.loads(msg.payload.decode())
                print(f"   消息内容:")
                print(f"     - 时间戳: {message_data.get('timestamp', 'N/A')}")
                print(f"     - 任务ID: {message_data.get('taskId', 'N/A')}")
                print(f"     - 状态: {message_data.get('status', 'N/A')}")
                print(f"     - 完成时间: {message_data.get('finishTime', 'N/A')}")
                print(f"     - 原因: {message_data.get('reason', 'N/A')}")

                # 格式化显示JSON
                print(f"   完整JSON:")
                formatted_json = json.dumps(message_data, indent=4, ensure_ascii=False)
                for line in formatted_json.split('\n'):
                    print(f"     {line}")

            except json.JSONDecodeError as e:
                print(f"   ❌ JSON解析失败: {e}")
                print(f"   原始消息: {msg.payload.decode()}")

        except Exception as e:
            print(f"❌ 处理消息时出错: {e}")

    def on_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调"""
        self.connected = False
        print(f"🔌 MQTT连接已断开 (返回码: {rc})")

    def connect_and_start(self):
        """连接MQTT并开始监听"""
        try:
            print(f"🔌 连接到MQTT Broker: {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
            self.client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
            self.client.loop_start()

            # 等待连接建立
            timeout = 10
            while not self.connected and timeout > 0:
                time.sleep(0.1)
                timeout -= 0.1

            if not self.connected:
                print("❌ MQTT连接超时")
                return False

            return True

        except Exception as e:
            print(f"❌ MQTT连接错误: {e}")
            return False

    def stop(self):
        """停止MQTT客户端"""
        print("\n🛑 停止任务状态监控器...")
        self.client.loop_stop()
        self.client.disconnect()

def main():
    print("📊 启动MQTT任务状态监控测试")
    print("=" * 60)
    print(f"🤖 机器人ID: {ROBOT_ID}")
    print(f"📡 MQTT服务器: {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
    print(f"📥 监听主题: {TASK_STATUS_TOPIC}")
    print()

    monitor = TaskStatusMonitor()

    try:
        # 连接MQTT
        if not monitor.connect_and_start():
            print("❌ 无法建立MQTT连接，退出测试")
            return

        print("✅ 任务状态监控器已启动")
        print("🔍 等待桥接器发布任务状态消息...")
        print("💡 提示: 可以使用任务下发工具触发任务，然后观察状态更新")
        print("⏹️  按 Ctrl+C 停止")
        print()

        # 持续监听
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n🛑 用户停止任务状态监控器")
    except Exception as e:
        print(f"\n❌ 任务状态监控器错误: {e}")
    finally:
        monitor.stop()
        print("✅ 任务状态监控器已关闭")

if __name__ == '__main__':
    main()