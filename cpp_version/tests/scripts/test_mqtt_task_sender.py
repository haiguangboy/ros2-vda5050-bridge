#!/usr/bin/env python3
# test_mqtt_task_sender.py
#
# 测试MQTT任务下发功能
# 模拟调度系统向桥接器发送任务消息
# 主题: EP/master/{robotId}/task
#
# ⚠️  仅用于测试目的！
# 在生产环境中，真实的调度系统会发送任务消息

import paho.mqtt.client as mqtt
import json
import time
import uuid
from datetime import datetime

# --- Configuration ---
MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883
ROBOT_ID = "robot-001"
TASK_TOPIC = f"EP/master/{ROBOT_ID}/task"

# 发布配置
PUBLISH_INTERVAL = 5.0  # 每5秒发送一个任务

class TaskSender:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_publish = self.on_publish
        self.client.on_disconnect = self.on_disconnect

        self.task_counter = 0
        self.connected = False

    def on_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            self.connected = True
            print(f"✅ 已连接到MQTT Broker: {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
            print(f"📤 将向主题发送任务: {TASK_TOPIC}")
        else:
            print(f"❌ MQTT连接失败，返回码: {rc}")

    def on_publish(self, client, userdata, mid):
        """消息发布回调"""
        print(f"📨 消息已发布 (message ID: {mid})")

    def on_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调"""
        self.connected = False
        print(f"🔌 MQTT连接已断开 (返回码: {rc})")

    def create_task_message(self, task_type="ground_pick"):
        """创建任务消息 - 符合中力协议格式"""
        task_id = f"task-{ROBOT_ID}-{datetime.now().strftime('%Y%m%d-%H%M%S')}-{self.task_counter:03d}"

        if task_type == "ground_pick":
            task_message = {
                "timestamp": datetime.now().isoformat(),
                "taskId": task_id,
                "startArea": f"AREA-{self.task_counter % 5 + 1}",
                "startAction": "ground_pick",
                "targetArea": f"STORAGE-{self.task_counter % 3 + 1}",
                "targetAction": "ground_place"
            }
        elif task_type == "load":
            task_message = {
                "timestamp": datetime.now().isoformat(),
                "taskId": task_id,
                "startArea": f"STORAGE-{self.task_counter % 3 + 1}",
                "startAction": "load",
                "targetArea": f"DELIVERY-{self.task_counter % 2 + 1}",
                "targetAction": "unload"
            }
        elif task_type == "ground_place":
            task_message = {
                "timestamp": datetime.now().isoformat(),
                "taskId": task_id,
                "startArea": f"PICKUP-{self.task_counter % 4 + 1}",
                "startAction": "ground_pick",
                "targetArea": f"AREA-{self.task_counter % 5 + 1}",
                "targetAction": "ground_place"
            }

        return task_message

    def send_task(self):
        """发送任务消息"""
        if not self.connected:
            print("❌ MQTT未连接，无法发送任务")
            return False

        # 循环不同类型的任务
        task_types = ["ground_pick", "load", "ground_place"]
        task_type = task_types[self.task_counter % len(task_types)]

        # 创建任务消息
        task_message = self.create_task_message(task_type)

        # 转换为JSON
        json_message = json.dumps(task_message, indent=2)

        print(f"\n📋 发送任务 #{self.task_counter} - 类型: {task_type}")
        print(f"   任务ID: {task_message['taskId']}")
        print(f"   起始区域: {task_message['startArea']} (动作: {task_message['startAction']})")
        print(f"   目标区域: {task_message['targetArea']} (动作: {task_message['targetAction']})")
        print(f"   消息内容:")
        print("   " + "\n   ".join(json_message.split('\n')))

        # 发布消息
        result = self.client.publish(TASK_TOPIC, json_message, qos=1)

        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"✅ 任务消息发送成功")
        else:
            print(f"❌ 任务消息发送失败，错误码: {result.rc}")
            return False

        self.task_counter += 1
        return True

    def connect_and_start(self):
        """连接MQTT并开始发送任务"""
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
        print("\n🛑 停止任务发送器...")
        self.client.loop_stop()
        self.client.disconnect()

def main():
    print("📋 启动MQTT任务下发测试")
    print("=" * 60)
    print(f"🤖 机器人ID: {ROBOT_ID}")
    print(f"📡 MQTT服务器: {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
    print(f"📤 任务主题: {TASK_TOPIC}")
    print(f"⏰ 发送间隔: {PUBLISH_INTERVAL}秒")
    print()

    sender = TaskSender()

    try:
        # 连接MQTT
        if not sender.connect_and_start():
            print("❌ 无法建立MQTT连接，退出测试")
            return

        print("✅ 任务发送器已启动")
        print("🔍 请确保桥接器正在运行以接收任务")
        print("📋 将循环发送不同类型的任务:")
        print("   - ground_pick (地面取货)")
        print("   - load (装载货物)")
        print("   - ground_place (地面放货)")
        print("⏹️  按 Ctrl+C 停止")
        print()

        # 循环发送任务
        while True:
            sender.send_task()
            time.sleep(PUBLISH_INTERVAL)

    except KeyboardInterrupt:
        print("\n🛑 用户停止任务发送器")
    except Exception as e:
        print(f"\n❌ 任务发送器错误: {e}")
    finally:
        sender.stop()
        print("✅ 任务发送器已关闭")

if __name__ == '__main__':
    main()