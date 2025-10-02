#!/usr/bin/env python3
# test_remote_device_format.py
#
# 测试兼容远程设备消息格式的轨迹状态发送功能
# 模拟远程ARM64设备发送的实际消息格式
# 主题: EP/{robotId}/cerebellum/embrain/trajectory_status
#
# ⚠️  用于测试兼容性！
# 模拟实际远程设备发送的消息格式

import paho.mqtt.client as mqtt
import json
import time
from datetime import datetime

# --- Configuration ---
MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883
ROBOT_ID = "robot-001"
TRAJECTORY_STATUS_TOPIC = f"EP/{ROBOT_ID}/cerebellum/embrain/trajectory_status"

class RemoteDeviceFormatSender:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_publish = self.on_publish
        self.client.on_disconnect = self.on_disconnect

        self.connected = False
        self.message_count = 0

    def on_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            self.connected = True
            print(f"✅ 已连接到MQTT Broker: {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
        else:
            print(f"❌ MQTT连接失败，返回码: {rc}")

    def on_publish(self, client, userdata, mid):
        """发布消息回调"""
        print(f"📨 消息已发布 (message ID: {mid})")

    def on_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调"""
        self.connected = False
        print(f"🔌 MQTT连接已断开 (返回码: {rc})")

    def connect_and_start(self):
        """连接MQTT"""
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

    def create_remote_device_status(self, trajectory_id, status, x=0, y=0, theta=0):
        """创建远程设备格式的轨迹状态消息"""
        # 使用远程设备的实际消息格式
        message = {
            "timestamp": datetime.now().isoformat() + "Z",
            "trajectoryId": trajectory_id,
            "trajectoryStatus": status,  # 注意：使用 trajectoryStatus 而不是 status
            "currentPosition": {         # 注意：使用 currentPosition 而不是 currentPointIndex
                "x": x,
                "y": y,
                "theta": theta
            },
            "resultDescription": f"Remote device response for {status}"  # 额外字段
        }

        return message

    def send_remote_format_status(self, trajectory_id, status, x=0, y=0, theta=0):
        """发送远程设备格式的轨迹状态消息"""
        if not self.connected:
            print("❌ MQTT未连接，无法发送消息")
            return False

        # 创建消息
        message = self.create_remote_device_status(trajectory_id, status, x, y, theta)

        self.message_count += 1

        print(f"\n📤 发送远程设备格式轨迹状态 #{self.message_count}")
        print(f"   轨迹ID: {trajectory_id}")
        print(f"   状态: {status} (远程格式)")
        print(f"   当前位置: ({x}, {y}, {theta})")

        # 发布消息
        try:
            message_json = json.dumps(message, indent=2, ensure_ascii=False)
            print(f"   JSON消息:")
            for line in message_json.split('\n'):
                print(f"     {line}")

            result = self.client.publish(TRAJECTORY_STATUS_TOPIC, message_json, qos=1)

            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"✅ 远程格式轨迹状态消息发送成功")
                return True
            else:
                print(f"❌ 轨迹状态消息发送失败，错误码: {result.rc}")
                return False

        except Exception as e:
            print(f"❌ 发送消息时出错: {e}")
            return False

    def stop(self):
        """停止MQTT客户端"""
        print("\n🛑 停止远程设备格式发送器...")
        self.client.loop_stop()
        self.client.disconnect()

def main():
    print("🔄 启动远程设备消息格式兼容性测试")
    print("=" * 60)
    print(f"🤖 机器人ID: {ROBOT_ID}")
    print(f"📡 MQTT服务器: {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
    print(f"📤 发布主题: {TRAJECTORY_STATUS_TOPIC}")
    print("📋 模拟远程ARM64设备的实际消息格式:")
    print("   - 使用 'trajectoryStatus' 而不是 'status'")
    print("   - 使用 'currentPosition' 对象而不是 'currentPointIndex'")
    print("   - 包含 'resultDescription' 字段")
    print()

    sender = RemoteDeviceFormatSender()

    try:
        # 连接MQTT
        if not sender.connect_and_start():
            print("❌ 无法建立MQTT连接，退出测试")
            return

        print("✅ 远程设备格式发送器已启动")
        print("🔄 开始发送远程设备格式的轨迹状态序列...")
        print()

        # 模拟轨迹执行的完整过程，使用远程设备的格式
        trajectory_id = f"traj-robot-001-remote-{int(time.time())}"

        # 1. 轨迹开始执行 (远程设备格式)
        print("📍 Step 1: 轨迹开始执行（远程格式）...")
        sender.send_remote_format_status(trajectory_id, "RUNNING", x=0.0, y=0.0, theta=0.0)
        time.sleep(2)

        # 2. 轨迹执行中 - 位置更新
        print("📍 Step 2: 执行中，位置更新（远程格式）...")
        sender.send_remote_format_status(trajectory_id, "RUNNING", x=1.5, y=0.8, theta=0.2)
        time.sleep(2)

        # 3. 轨迹执行完成 (远程设备格式)
        print("📍 Step 3: 轨迹执行完成（远程格式）...")
        sender.send_remote_format_status(trajectory_id, "FINISHED", x=3.0, y=2.1, theta=1.57)

        print("\n✅ 远程设备格式轨迹状态序列发送完成！")

        # 等待一段时间，然后发送失败的轨迹
        time.sleep(3)
        print("\n" + "="*40)
        print("🔥 测试失败场景（远程格式）...")

        failed_trajectory_id = f"traj-robot-001-remote-failed-{int(time.time())}"

        # 1. 失败轨迹开始
        print("📍 失败轨迹: 开始执行（远程格式）...")
        sender.send_remote_format_status(failed_trajectory_id, "RUNNING", x=0.0, y=0.0, theta=0.0)
        time.sleep(2)

        # 2. 轨迹执行失败
        print("📍 失败轨迹: 执行失败（远程格式）...")
        sender.send_remote_format_status(failed_trajectory_id, "FAILED", x=0.5, y=0.2, theta=0.1)

        print("\n✅ 远程设备格式失败轨迹状态序列发送完成！")
        print("\n💡 提示: 检查桥接器是否正确解析了远程设备格式的消息")
        print("💡 提示: 桥接器应该将 'trajectoryStatus': 'FINISHED' 映射为 'status': 'completed'")
        print("💡 提示: 桥接器应该将 'trajectoryStatus': 'FAILED' 映射为 'status': 'failed'")

    except KeyboardInterrupt:
        print("\n🛑 用户停止远程设备格式发送器")
    except Exception as e:
        print(f"\n❌ 远程设备格式发送器错误: {e}")
    finally:
        sender.stop()
        print("✅ 远程设备格式发送器已关闭")

if __name__ == '__main__':
    main()