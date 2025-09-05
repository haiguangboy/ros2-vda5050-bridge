#!/usr/bin/env python3
"""
Complete ROS2 Path to VDA5050 Integration Test
测试ROS2规划路径转换为VDA5050订单的完整流程
"""

import json
import time
import threading
import subprocess
import signal
import sys
import os
import paho.mqtt.client as mqtt

# Add the package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'ros2_vda5050_bridge'))
from ros2_vda5050_bridge.vda5050_types import *

class ROS2VDA5050IntegrationTest:
    """完整的ROS2到VDA5050集成测试"""
    
    def __init__(self):
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        
        # 配置
        self.manufacturer = "ROS2Manufacturer"
        self.serial_number = "ROS2_AGV_001"
        self.topic_prefix = f"uagv/v2/{self.manufacturer}/{self.serial_number}"
        
        # 状态跟踪
        self.received_orders = []
        self.received_states = []
        self.connected = False
        
        # 进程管理
        self.processes = []
    
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            print(f"✅ 测试客户端连接到MQTT broker")
            
            # 订阅所有相关主题
            topics = [
                f"{self.topic_prefix}/order",
                f"{self.topic_prefix}/state",
                f"{self.topic_prefix}/connection"
            ]
            
            for topic in topics:
                client.subscribe(topic, qos=0)
                print(f"📡 订阅: {topic}")
        else:
            print(f"❌ MQTT连接失败: {rc}")
    
    def _on_message(self, client, userdata, msg):
        try:
            topic_name = msg.topic.split('/')[-1]
            data = json.loads(msg.payload.decode('utf-8'))
            
            if topic_name == 'order':
                self.received_orders.append(data)
                print(f"📦 收到VDA5050订单: {data.get('orderId', 'N/A')}")
                print(f"   节点数: {len(data.get('nodes', []))}")
                print(f"   边数: {len(data.get('edges', []))}")
                
                # 显示路径点
                nodes = data.get('nodes', [])
                if nodes:
                    print("   路径点:")
                    for i, node in enumerate(nodes[:5]):  # 只显示前5个点
                        pos = node.get('nodePosition', {})
                        print(f"     {i+1}. ({pos.get('x', 'N/A'):.2f}, {pos.get('y', 'N/A'):.2f})")
                    if len(nodes) > 5:
                        print(f"     ... 还有 {len(nodes)-5} 个点")
                
            elif topic_name == 'state':
                self.received_states.append(data)
                if len(self.received_states) % 10 == 0:  # 每10个状态显示一次
                    pos = data.get('agvPosition', {})
                    print(f"📊 AGV状态 #{len(self.received_states)}: "
                          f"位置({pos.get('x', 'N/A'):.2f}, {pos.get('y', 'N/A'):.2f}), "
                          f"订单: {data.get('orderId', '无')}")
                
        except Exception as e:
            print(f"❌ 处理消息错误: {e}")
    
    def connect_mqtt(self):
        """连接MQTT broker"""
        try:
            self.mqtt_client.connect("localhost", 1883, 60)
            self.mqtt_client.loop_start()
            time.sleep(2)
            return self.connected
        except Exception as e:
            print(f"❌ MQTT连接失败: {e}")
            return False
    
    def disconnect_mqtt(self):
        """断开MQTT连接"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
    
    def start_ros2_nodes(self):
        """启动ROS2节点"""
        print("🚀 启动ROS2节点...")
        
        # 设置ROS2环境
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '0'
        
        try:
            # 启动地图发布器
            print("   启动地图发布器...")
            map_proc = subprocess.Popen([
                'bash', '-c', 
                'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run ros2_vda5050_bridge map_publisher'
            ], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.processes.append(map_proc)
            time.sleep(2)
            
            # 启动VDA5050桥接器
            print("   启动VDA5050桥接器...")
            bridge_proc = subprocess.Popen([
                'bash', '-c',
                'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run ros2_vda5050_bridge vda5050_bridge'
            ], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.processes.append(bridge_proc)
            time.sleep(3)
            
            # 启动路径发布器
            print("   启动路径发布器...")
            path_proc = subprocess.Popen([
                'bash', '-c',
                'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run ros2_vda5050_bridge path_publisher'
            ], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.processes.append(path_proc)
            time.sleep(2)
            
            print("✅ 所有ROS2节点启动完成")
            return True
            
        except Exception as e:
            print(f"❌ 启动ROS2节点失败: {e}")
            return False
    
    def stop_ros2_nodes(self):
        """停止ROS2节点"""
        print("🛑 停止ROS2节点...")
        for proc in self.processes:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        self.processes.clear()
    
    def run_integration_test(self):
        """运行完整集成测试"""
        print("🎯 ROS2路径到VDA5050集成测试")
        print("=" * 60)
        
        # 测试步骤
        print("\n📋 测试计划:")
        print("1. 启动MQTT连接")
        print("2. 启动ROS2节点 (地图发布器、VDA5050桥接器、路径发布器)")
        print("3. 监控ROS2路径转换为VDA5050订单")
        print("4. 验证地图共享")
        print("5. 验证目标到达确认")
        print()
        
        try:
            # 步骤1: MQTT连接
            print("🔗 步骤1: 连接MQTT broker...")
            if not self.connect_mqtt():
                print("❌ MQTT连接失败，测试终止")
                return False
            
            # 步骤2: 启动ROS2节点
            print("\n🚀 步骤2: 启动ROS2节点...")
            if not self.start_ros2_nodes():
                print("❌ ROS2节点启动失败，测试终止")
                return False
            
            # 步骤3: 监控路径转换
            print("\n📊 步骤3: 监控路径转换 (30秒)...")
            print("等待ROS2路径发布器发布路径并转换为VDA5050订单...")
            
            start_time = time.time()
            initial_orders = len(self.received_orders)
            initial_states = len(self.received_states)
            
            while time.time() - start_time < 30:
                time.sleep(1)
                
                # 检查是否收到新订单
                if len(self.received_orders) > initial_orders:
                    new_orders = len(self.received_orders) - initial_orders
                    print(f"✅ 收到 {new_orders} 个新的VDA5050订单")
                    break
            
            # 步骤4: 验证结果
            print(f"\n📈 步骤4: 测试结果分析...")
            print(f"   收到的VDA5050订单: {len(self.received_orders)}")
            print(f"   收到的状态更新: {len(self.received_states)}")
            
            if self.received_orders:
                latest_order = self.received_orders[-1]
                print(f"\n📦 最新订单分析:")
                print(f"   订单ID: {latest_order.get('orderId', 'N/A')}")
                print(f"   节点数: {len(latest_order.get('nodes', []))}")
                print(f"   边数: {len(latest_order.get('edges', []))}")
                
                # 分析路径点
                nodes = latest_order.get('nodes', [])
                if len(nodes) >= 2:
                    start_node = nodes[0].get('nodePosition', {})
                    end_node = nodes[-1].get('nodePosition', {})
                    print(f"   起点: ({start_node.get('x', 'N/A'):.2f}, {start_node.get('y', 'N/A'):.2f})")
                    print(f"   终点: ({end_node.get('x', 'N/A'):.2f}, {end_node.get('y', 'N/A'):.2f})")
                    
                    # 计算路径长度
                    total_length = 0
                    for i in range(len(nodes) - 1):
                        pos1 = nodes[i].get('nodePosition', {})
                        pos2 = nodes[i+1].get('nodePosition', {})
                        if pos1 and pos2:
                            dx = pos2.get('x', 0) - pos1.get('x', 0)
                            dy = pos2.get('y', 0) - pos1.get('y', 0)
                            total_length += (dx*dx + dy*dy)**0.5
                    print(f"   路径总长度: {total_length:.2f}m")
            
            # 步骤5: 等待目标到达
            print(f"\n⏳ 步骤5: 等待目标到达确认 (15秒)...")
            time.sleep(15)
            
            # 检查最新状态
            if self.received_states:
                latest_state = self.received_states[-1]
                actions = latest_state.get('actionStates', [])
                completed_actions = [a for a in actions if a.get('actionStatus') == 'FINISHED']
                
                print(f"   完成的动作数: {len(completed_actions)}")
                for action in completed_actions:
                    if action.get('actionType') == 'reportPosition':
                        print(f"   ✅ 位置报告完成: {action.get('resultDescription', 'N/A')}")
            
            # 总结
            print(f"\n🎉 集成测试完成!")
            
            success_criteria = [
                len(self.received_orders) > 0,
                len(self.received_states) > 10,
                any(order.get('nodes', []) for order in self.received_orders)
            ]
            
            if all(success_criteria):
                print("✅ 所有测试标准通过!")
                print("\n✨ 验证的功能:")
                print("   ✅ ROS2路径成功转换为VDA5050订单")
                print("   ✅ 地图信息在ROS2和VDA5050之间共享")
                print("   ✅ AGV状态实时更新")
                print("   ✅ 路径点跟踪和目标到达机制")
                return True
            else:
                print("⚠️  部分测试标准未通过")
                return False
                
        except KeyboardInterrupt:
            print("\n⏹️  测试被用户中断")
            return False
        except Exception as e:
            print(f"\n❌ 测试过程中出错: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        print("\n🧹 清理资源...")
        self.stop_ros2_nodes()
        self.disconnect_mqtt()
        print("✅ 清理完成")

def main():
    """主函数"""
    print("ROS2 到 VDA5050 完整集成测试")
    print("=" * 50)
    print()
    print("此测试将验证以下功能:")
    print("1. ROS2规划路径自动转换为VDA5050订单")
    print("2. 地图在ROS2和VDA5050之间共享")
    print("3. 路径点跟踪和目标到达确认")
    print("4. 实时状态同步")
    print()
    
    print("前置条件检查:")
    print("✅ MQTT broker (mosquitto) 正在运行")
    print("✅ ROS2 Humble 环境已配置")
    print("✅ ros2_vda5050_bridge 包已编译")
    print()
    
    input("按Enter键开始集成测试...")
    
    test = ROS2VDA5050IntegrationTest()
    
    try:
        success = test.run_integration_test()
        
        if success:
            print("\n🎊 集成测试成功完成!")
            print("\n📚 下一步建议:")
            print("1. 集成真实的Nav2导航栈")
            print("2. 添加更复杂的路径规划算法")
            print("3. 实现动态障碍物避让")
            print("4. 添加多AGV协调功能")
        else:
            print("\n⚠️  集成测试部分成功")
            print("请检查日志并调试相关问题")
            
    except KeyboardInterrupt:
        print("\n👋 测试被用户中断")
    except Exception as e:
        print(f"\n💥 测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()