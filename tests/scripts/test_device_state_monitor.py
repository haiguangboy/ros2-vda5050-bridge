#!/usr/bin/env python3
# test_device_state_monitor.py
#
# 测试MQTT设备状态监控功能
# 监听桥接器发布的设备状态消息
# 主题: EP/master/{robotId}/state
#
# ⚠️  仅用于测试目的！
# 在生产环境中，真实的调度系统会监听设备状态消息

import paho.mqtt.client as mqtt
import json
import time
from datetime import datetime

# --- Configuration ---
MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883
ROBOT_ID = "robot-001"
DEVICE_STATE_TOPIC = f"EP/master/{ROBOT_ID}/state"

class DeviceStateMonitor:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

        self.connected = False
        self.message_count = 0
        self.last_message_time = None

    def on_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            self.connected = True
            print(f"✅ 已连接到MQTT Broker: {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
            print(f"📥 订阅设备状态主题: {DEVICE_STATE_TOPIC}")

            # 订阅设备状态主题
            result = client.subscribe(DEVICE_STATE_TOPIC, qos=1)
            if result[0] == mqtt.MQTT_ERR_SUCCESS:
                print(f"✅ 成功订阅主题: {DEVICE_STATE_TOPIC}")
            else:
                print(f"❌ 订阅失败，错误码: {result[0]}")
        else:
            print(f"❌ MQTT连接失败，返回码: {rc}")

    def on_message(self, client, userdata, msg):
        """接收到消息回调"""
        try:
            self.message_count += 1
            current_time = datetime.now()

            # 计算消息间隔
            interval_str = ""
            if self.last_message_time:
                interval = (current_time - self.last_message_time).total_seconds()
                interval_str = f" (间隔: {interval:.1f}s)"

            print(f"\n📊 收到设备状态消息 #{self.message_count}{interval_str}")
            print(f"   主题: {msg.topic}")
            print(f"   时间: {current_time.strftime('%Y-%m-%d %H:%M:%S')}")

            # 解析JSON消息
            try:
                message_data = json.loads(msg.payload.decode())

                # 显示基本信息
                print(f"   基本信息:")
                print(f"     - 时间戳: {message_data.get('timestamp', 'N/A')}")
                print(f"     - 系统状态: {message_data.get('systemState', 'N/A')}")

                # 显示位姿信息
                pose = message_data.get('pose', {})
                if pose:
                    print(f"   位姿信息:")
                    print(f"     - 位置: x={pose.get('x', 'N/A'):.2f}, y={pose.get('y', 'N/A'):.2f}")
                    print(f"     - 角度: θ={pose.get('theta', 'N/A'):.2f}°")

                # 显示货叉状态
                forklift = message_data.get('forkliftState', {})
                if forklift:
                    print(f"   货叉状态:")
                    print(f"     - 高度: {forklift.get('height', 'N/A'):.2f}m")
                    print(f"     - 负载: {forklift.get('weight', 'N/A'):.1f}kg")
                    print(f"     - 侧移: {forklift.get('lateralShift', 'N/A'):.2f}m")
                    print(f"     - 前伸: {forklift.get('forwardExtension', 'N/A'):.2f}m")
                    print(f"     - 后倾: {forklift.get('tiltBack', 'N/A')}")
                    print(f"     - 状态: {forklift.get('status', 'N/A')}")

                # 显示电池状态
                battery = message_data.get('battery', {})
                if battery:
                    print(f"   电池状态:")
                    print(f"     - 电量: {battery.get('percentage', 'N/A'):.1f}%")
                    print(f"     - 电压: {battery.get('voltage', 'N/A'):.1f}V")
                    print(f"     - 电流: {battery.get('current', 'N/A'):.1f}A")
                    print(f"     - 温度: {battery.get('temperature', 'N/A'):.1f}°C")
                    print(f"     - 状态: {battery.get('status', 'N/A')}")

                # 显示错误信息
                errors = message_data.get('errors', [])
                if errors:
                    print(f"   错误信息 ({len(errors)}个):")
                    for i, error in enumerate(errors):
                        print(f"     [{i+1}] 代码: {error.get('errorCode', 'N/A')}")
                        print(f"         描述: {error.get('description', 'N/A')}")
                        print(f"         级别: {error.get('severity', 'N/A')}")
                else:
                    print(f"   ✅ 无错误信息")

                # 在消息数量较少时显示完整JSON
                if self.message_count <= 3:
                    print(f"   完整JSON (前3条消息):")
                    formatted_json = json.dumps(message_data, indent=4, ensure_ascii=False)
                    for line in formatted_json.split('\n'):
                        print(f"     {line}")

            except json.JSONDecodeError as e:
                print(f"   ❌ JSON解析失败: {e}")
                print(f"   原始消息: {msg.payload.decode()}")

            self.last_message_time = current_time

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
        print("\n🛑 停止设备状态监控器...")
        self.client.loop_stop()
        self.client.disconnect()

def main():
    print("📊 启动MQTT设备状态监控测试")
    print("=" * 60)
    print(f"🤖 机器人ID: {ROBOT_ID}")
    print(f"📡 MQTT服务器: {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
    print(f"📥 监听主题: {DEVICE_STATE_TOPIC}")
    print()

    monitor = DeviceStateMonitor()

    try:
        # 连接MQTT
        if not monitor.connect_and_start():
            print("❌ 无法建立MQTT连接，退出测试")
            return

        print("✅ 设备状态监控器已启动")
        print("🔍 等待桥接器发布设备状态消息...")
        print("💡 提示: 桥接器应该每隔一定时间发布设备状态")
        print("⏹️  按 Ctrl+C 停止")
        print()

        # 持续监听
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n🛑 用户停止设备状态监控器")
    except Exception as e:
        print(f"\n❌ 设备状态监控器错误: {e}")
    finally:
        monitor.stop()
        print("✅ 设备状态监控器已关闭")

if __name__ == '__main__':
    main()