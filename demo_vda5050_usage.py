#!/usr/bin/env python3
"""
VDA5050 Bridge Demo Script
演示如何使用VDA5050桥接器
"""

import json
import time
import paho.mqtt.client as mqtt
import sys
import os

# Add the package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'ros2_vda5050_bridge'))
from ros2_vda5050_bridge.vda5050_types import *

class VDA5050Demo:
    """VDA5050演示客户端"""
    
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        
        # AGV配置
        self.manufacturer = "ROS2Manufacturer"
        self.serial_number = "ROS2_AGV_001"
        self.topic_prefix = f"uagv/v2/{self.manufacturer}/{self.serial_number}"
        
        # 状态跟踪
        self.agv_states = []
        self.connected = False
    
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            print(f"✅ 连接到MQTT broker成功")
            
            # 订阅AGV状态
            state_topic = f"{self.topic_prefix}/state"
            client.subscribe(state_topic, qos=0)
            print(f"📡 订阅AGV状态: {state_topic}")
        else:
            print(f"❌ 连接失败: {rc}")
    
    def _on_message(self, client, userdata, msg):
        try:
            if msg.topic.endswith('/state'):
                state_data = json.loads(msg.payload.decode('utf-8'))
                self.agv_states.append(state_data)
                
                # 显示关键状态信息
                pos = state_data.get('agvPosition', {})
                battery = state_data.get('batteryState', {})
                
                print(f"📊 AGV状态更新:")
                print(f"   位置: ({pos.get('x', 'N/A'):.2f}, {pos.get('y', 'N/A'):.2f})")
                print(f"   电池: {battery.get('batteryCharge', 'N/A')}%")
                print(f"   驾驶: {'是' if state_data.get('driving', False) else '否'}")
                print(f"   订单: {state_data.get('orderId', '无')}")
                print()
                
        except Exception as e:
            print(f"❌ 处理消息错误: {e}")
    
    def connect(self):
        """连接到MQTT broker"""
        try:
            self.client.connect("localhost", 1883, 60)
            self.client.loop_start()
            time.sleep(2)
            return self.connected
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        self.client.loop_stop()
        self.client.disconnect()
    
    def send_simple_order(self):
        """发送简单的移动订单"""
        print("📤 发送简单移动订单...")
        
        # 创建简单的两点移动订单
        nodes = [
            Node(
                nodeId="start",
                sequenceId=0,
                released=True,
                nodePosition=NodePosition(x=0.0, y=0.0, theta=0.0, mapId="demo_map")
            ),
            Node(
                nodeId="target",
                sequenceId=2,
                released=True,
                nodePosition=NodePosition(x=3.0, y=2.0, theta=1.57, mapId="demo_map")
            )
        ]
        
        edges = [
            Edge(
                edgeId="move_to_target",
                sequenceId=1,
                released=True,
                startNodeId="start",
                endNodeId="target",
                maxSpeed=1.5
            )
        ]
        
        order = OrderMessage(
            headerId=1,
            timestamp=create_timestamp(),
            version="2.1.0",
            manufacturer=self.manufacturer,
            serialNumber=self.serial_number,
            orderId="DEMO_ORDER_001",
            orderUpdateId=0,
            nodes=nodes,
            edges=edges
        )
        
        # 发送订单
        topic = f"{self.topic_prefix}/order"
        self.client.publish(topic, order.to_json(), qos=0)
        print(f"✅ 订单已发送: 从(0,0)移动到(3,2)")
    
    def send_pickup_delivery_order(self):
        """发送取货送货订单"""
        print("📤 发送取货送货订单...")
        
        nodes = [
            Node(
                nodeId="warehouse",
                sequenceId=0,
                released=True,
                nodePosition=NodePosition(x=1.0, y=1.0, theta=0.0, mapId="demo_map"),
                actions=[
                    Action(
                        actionId="pick_item_001",
                        actionType="pick",
                        blockingType="HARD",
                        actionParameters=[
                            ActionParameter(key="loadId", value="ITEM_12345"),
                            ActionParameter(key="duration", value=5.0)
                        ]
                    )
                ]
            ),
            Node(
                nodeId="delivery_point",
                sequenceId=2,
                released=True,
                nodePosition=NodePosition(x=5.0, y=3.0, theta=3.14, mapId="demo_map"),
                actions=[
                    Action(
                        actionId="drop_item_001",
                        actionType="drop",
                        blockingType="HARD",
                        actionParameters=[
                            ActionParameter(key="loadId", value="ITEM_12345"),
                            ActionParameter(key="duration", value=3.0)
                        ]
                    )
                ]
            )
        ]
        
        edges = [
            Edge(
                edgeId="warehouse_to_delivery",
                sequenceId=1,
                released=True,
                startNodeId="warehouse",
                endNodeId="delivery_point",
                maxSpeed=1.0
            )
        ]
        
        order = OrderMessage(
            headerId=2,
            timestamp=create_timestamp(),
            version="2.1.0",
            manufacturer=self.manufacturer,
            serialNumber=self.serial_number,
            orderId="DEMO_PICKUP_001",
            orderUpdateId=0,
            nodes=nodes,
            edges=edges
        )
        
        # 发送订单
        topic = f"{self.topic_prefix}/order"
        self.client.publish(topic, order.to_json(), qos=0)
        print(f"✅ 取货送货订单已发送")
    
    def send_pause_action(self):
        """发送暂停动作"""
        print("⏸️  发送暂停指令...")
        
        instant_actions = {
            "headerId": 1,
            "timestamp": create_timestamp(),
            "version": "2.1.0",
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,
            "actions": [
                {
                    "actionId": f"pause_{int(time.time())}",
                    "actionType": "pause",
                    "blockingType": "HARD"
                }
            ]
        }
        
        topic = f"{self.topic_prefix}/instantActions"
        self.client.publish(topic, json.dumps(instant_actions), qos=0)
        print("✅ 暂停指令已发送")
    
    def send_resume_action(self):
        """发送恢复动作"""
        print("▶️  发送恢复指令...")
        
        instant_actions = {
            "headerId": 2,
            "timestamp": create_timestamp(),
            "version": "2.1.0",
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,
            "actions": [
                {
                    "actionId": f"resume_{int(time.time())}",
                    "actionType": "resume",
                    "blockingType": "HARD"
                }
            ]
        }
        
        topic = f"{self.topic_prefix}/instantActions"
        self.client.publish(topic, json.dumps(instant_actions), qos=0)
        print("✅ 恢复指令已发送")
    
    def cancel_order(self):
        """取消当前订单"""
        print("❌ 发送取消订单指令...")
        
        instant_actions = {
            "headerId": 3,
            "timestamp": create_timestamp(),
            "version": "2.1.0",
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,
            "actions": [
                {
                    "actionId": f"cancel_{int(time.time())}",
                    "actionType": "cancelOrder",
                    "blockingType": "HARD"
                }
            ]
        }
        
        topic = f"{self.topic_prefix}/instantActions"
        self.client.publish(topic, json.dumps(instant_actions), qos=0)
        print("✅ 取消订单指令已发送")

def main():
    """主演示函数"""
    print("🚀 VDA5050桥接器演示")
    print("=" * 50)
    print()
    print("这个演示将展示如何:")
    print("1. 连接到VDA5050 MQTT系统")
    print("2. 发送不同类型的订单")
    print("3. 控制AGV的行为")
    print("4. 监控AGV状态")
    print()
    
    # 检查前置条件
    print("📋 检查前置条件:")
    print("1. MQTT broker (mosquitto) 正在运行")
    print("2. ROS2 VDA5050 bridge 节点正在运行")
    print()
    
    input("按Enter键开始演示...")
    print()
    
    # 创建演示客户端
    demo = VDA5050Demo()
    
    try:
        # 连接到MQTT
        if not demo.connect():
            print("❌ 无法连接到MQTT broker")
            return
        
        print("🎯 演示菜单:")
        print("1. 发送简单移动订单")
        print("2. 发送取货送货订单") 
        print("3. 暂停AGV")
        print("4. 恢复AGV")
        print("5. 取消当前订单")
        print("6. 监控状态 (10秒)")
        print("0. 退出")
        print()
        
        while True:
            try:
                choice = input("请选择操作 (0-6): ").strip()
                
                if choice == '0':
                    break
                elif choice == '1':
                    demo.send_simple_order()
                elif choice == '2':
                    demo.send_pickup_delivery_order()
                elif choice == '3':
                    demo.send_pause_action()
                elif choice == '4':
                    demo.send_resume_action()
                elif choice == '5':
                    demo.cancel_order()
                elif choice == '6':
                    print("📊 监控AGV状态 (10秒)...")
                    start_time = time.time()
                    initial_count = len(demo.agv_states)
                    
                    while time.time() - start_time < 10:
                        time.sleep(0.5)
                    
                    new_states = len(demo.agv_states) - initial_count
                    print(f"✅ 收到 {new_states} 个状态更新")
                else:
                    print("❌ 无效选择，请重试")
                
                print()
                
            except KeyboardInterrupt:
                break
        
        print("👋 演示结束")
        
    except Exception as e:
        print(f"❌ 演示过程中出错: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        demo.disconnect()

if __name__ == "__main__":
    main()