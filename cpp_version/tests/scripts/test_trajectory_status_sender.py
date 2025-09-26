#!/usr/bin/env python3
# test_trajectory_status_sender.py
#
# 测试MQTT轨迹状态发送功能
# 模拟车载小脑向桥接器发送轨迹执行状态
# 主题: EP/{robotId}/cerebellum/embrain/trajectory_status
#
# ⚠️  仅用于测试目的！
# 在生产环境中，真实的车载小脑会发送轨迹状态消息

import paho.mqtt.client as mqtt
import json
import time
from datetime import datetime

# --- Configuration ---
MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883
ROBOT_ID = "robot-001"
TRAJECTORY_STATUS_TOPIC = f"EP/{ROBOT_ID}/cerebellum/embrain/trajectory_status"

class TrajectoryStatusSender:
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

    def create_trajectory_status(self, trajectory_id, status, current_point=None, error_code=0, error_desc=""):
        """创建轨迹状态消息"""
        message = {
            "timestamp": datetime.now().isoformat() + "Z",
            "trajectoryId": trajectory_id,
            "status": status,  # pending/running/completed/failed
            "errorCode": error_code,
            "errorDesc": error_desc
        }

        if current_point is not None:
            message["currentPointIndex"] = current_point

        if status == "completed":
            message["finishTime"] = datetime.now().isoformat() + "Z"
        elif status == "running":
            # 估算完成时间（假设5秒后完成）
            import datetime as dt
            finish_time = dt.datetime.now() + dt.timedelta(seconds=5)
            message["estimatedFinishTime"] = finish_time.isoformat() + "Z"

        return message

    def send_trajectory_status(self, trajectory_id, status, current_point=None, error_code=0, error_desc=""):
        """发送轨迹状态消息"""
        if not self.connected:
            print("❌ MQTT未连接，无法发送消息")
            return False

        # 创建消息
        message = self.create_trajectory_status(trajectory_id, status, current_point, error_code, error_desc)

        self.message_count += 1

        print(f"\n📤 发送轨迹状态 #{self.message_count}")
        print(f"   轨迹ID: {trajectory_id}")
        print(f"   状态: {status}")
        if current_point is not None:
            print(f"   当前点: {current_point}")
        if error_code != 0:
            print(f"   错误码: {error_code}")
            print(f"   错误描述: {error_desc}")

        # 发布消息
        try:
            message_json = json.dumps(message, indent=2, ensure_ascii=False)
            print(f"   JSON消息:")
            for line in message_json.split('\n'):
                print(f"     {line}")

            result = self.client.publish(TRAJECTORY_STATUS_TOPIC, message_json, qos=1)

            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"✅ 轨迹状态消息发送成功")
                return True
            else:
                print(f"❌ 轨迹状态消息发送失败，错误码: {result.rc}")
                return False

        except Exception as e:
            print(f"❌ 发送消息时出错: {e}")
            return False

    def stop(self):
        """停止MQTT客户端"""
        print("\n🛑 停止轨迹状态发送器...")
        self.client.loop_stop()
        self.client.disconnect()

def main():
    print("🚀 启动MQTT轨迹状态发送测试")
    print("=" * 60)
    print(f"🤖 机器人ID: {ROBOT_ID}")
    print(f"📡 MQTT服务器: {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
    print(f"📤 发布主题: {TRAJECTORY_STATUS_TOPIC}")
    print()

    sender = TrajectoryStatusSender()

    try:
        # 连接MQTT
        if not sender.connect_and_start():
            print("❌ 无法建立MQTT连接，退出测试")
            return

        print("✅ 轨迹状态发送器已启动")
        print("🔄 开始发送轨迹状态序列...")
        print()

        # 模拟轨迹执行的完整过程
        trajectory_id = f"traj-{ROBOT_ID}-{int(time.time())}"

        # 1. 轨迹开始执行
        print("📍 Step 1: 轨迹开始执行...")
        sender.send_trajectory_status(trajectory_id, "running", current_point=0)
        time.sleep(2)

        # 2. 轨迹执行中 - 点1
        print("📍 Step 2: 执行到第1个点...")
        sender.send_trajectory_status(trajectory_id, "running", current_point=1)
        time.sleep(2)

        # 3. 轨迹执行中 - 点2
        print("📍 Step 3: 执行到第2个点...")
        sender.send_trajectory_status(trajectory_id, "running", current_point=2)
        time.sleep(2)

        # 4. 轨迹执行完成
        print("📍 Step 4: 轨迹执行完成...")
        sender.send_trajectory_status(trajectory_id, "completed", current_point=3)

        print("\n✅ 成功轨迹状态序列发送完成！")

        # 等待一段时间，然后发送失败的轨迹
        time.sleep(3)
        print("\n" + "="*40)
        print("🔥 测试失败场景...")

        failed_trajectory_id = f"traj-{ROBOT_ID}-failed-{int(time.time())}"

        # 1. 失败轨迹开始
        print("📍 失败轨迹: 开始执行...")
        sender.send_trajectory_status(failed_trajectory_id, "running", current_point=0)
        time.sleep(2)

        # 2. 轨迹执行失败
        print("📍 失败轨迹: 执行失败...")
        sender.send_trajectory_status(failed_trajectory_id, "failed", current_point=1,
                                    error_code=1001, error_desc="路径被阻塞，无法继续执行")

        print("\n✅ 失败轨迹状态序列发送完成！")
        print("\n💡 提示: 检查桥接器日志确认消息接收情况")
        print("💡 提示: 检查ROS2决策树是否收到状态更新")

    except KeyboardInterrupt:
        print("\n🛑 用户停止轨迹状态发送器")
    except Exception as e:
        print(f"\n❌ 轨迹状态发送器错误: {e}")
    finally:
        sender.stop()
        print("✅ 轨迹状态发送器已关闭")

if __name__ == '__main__':
    main()