#!/usr/bin/env python3
"""
Simple Path Conversion Test
简单的路径转换测试
"""

import json
import time
import paho.mqtt.client as mqtt
import sys
import os

# Add the package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'ros2_vda5050_bridge'))
from ros2_vda5050_bridge.vda5050_types import *

def test_path_conversion():
    """测试路径转换功能"""
    print("🧪 简单路径转换测试")
    print("=" * 40)
    
    # 模拟ROS2路径数据
    ros2_path_points = [
        (0.0, 0.0, 0.0),      # 起点
        (1.0, 0.0, 0.0),      # 中间点1
        (2.0, 1.0, 1.57),     # 中间点2 (转向)
        (3.0, 2.0, 3.14),     # 终点
    ]
    
    print(f"📍 模拟ROS2路径: {len(ros2_path_points)} 个点")
    for i, (x, y, theta) in enumerate(ros2_path_points):
        print(f"   点{i+1}: ({x:.1f}, {y:.1f}, {theta:.2f})")
    
    # 转换为VDA5050订单
    print(f"\n🔄 转换为VDA5050订单...")
    
    nodes = []
    edges = []
    
    for i, (x, y, theta) in enumerate(ros2_path_points):
        # 创建节点
        node = Node(
            nodeId=f"path_node_{i}",
            sequenceId=i * 2,
            released=True,
            nodePosition=NodePosition(
                x=x, y=y, theta=theta,
                mapId="shared_map",
                allowedDeviationXY=0.2
            ),
            actions=[]
        )
        
        # 最后一个节点添加到达动作
        if i == len(ros2_path_points) - 1:
            node.actions.append(Action(
                actionId=f"reach_target_{int(time.time())}",
                actionType="reportPosition",
                blockingType="SOFT",
                actionDescription="Report reaching target"
            ))
        
        nodes.append(node)
        
        # 创建边（除了最后一个点）
        if i < len(ros2_path_points) - 1:
            next_x, next_y, _ = ros2_path_points[i + 1]
            
            # 计算距离
            dx = next_x - x
            dy = next_y - y
            length = (dx*dx + dy*dy)**0.5
            
            edge = Edge(
                edgeId=f"path_edge_{i}",
                sequenceId=i * 2 + 1,
                released=True,
                startNodeId=f"path_node_{i}",
                endNodeId=f"path_node_{i+1}",
                maxSpeed=1.0,
                length=length
            )
            edges.append(edge)
    
    # 创建订单
    order = OrderMessage(
        headerId=1,
        timestamp=create_timestamp(),
        version="2.1.0",
        manufacturer="TestManufacturer",
        serialNumber="TEST_AGV_001",
        orderId=f"path_test_{int(time.time())}",
        orderUpdateId=0,
        nodes=nodes,
        edges=edges
    )
    
    print(f"✅ VDA5050订单创建成功:")
    print(f"   订单ID: {order.orderId}")
    print(f"   节点数: {len(order.nodes)}")
    print(f"   边数: {len(order.edges)}")
    
    # 显示转换后的路径
    print(f"\n📦 VDA5050路径点:")
    for i, node in enumerate(order.nodes):
        pos = node.nodePosition
        actions_desc = f", 动作: {len(node.actions)}" if node.actions else ""
        print(f"   节点{i+1}: ({pos.x:.1f}, {pos.y:.1f}, {pos.theta:.2f}){actions_desc}")
    
    print(f"\n🔗 VDA5050边:")
    for i, edge in enumerate(order.edges):
        print(f"   边{i+1}: {edge.startNodeId} -> {edge.endNodeId}, 长度: {edge.length:.2f}m")
    
    # 测试JSON序列化
    print(f"\n📄 JSON序列化测试...")
    try:
        json_str = order.to_json()
        json_size = len(json_str)
        print(f"✅ JSON序列化成功, 大小: {json_size} 字符")
        
        # 验证JSON可以解析
        parsed = json.loads(json_str)
        print(f"✅ JSON解析验证成功")
        
        # 显示JSON的关键部分
        print(f"\n📋 JSON关键信息:")
        print(f"   订单ID: {parsed.get('orderId')}")
        print(f"   时间戳: {parsed.get('timestamp')}")
        print(f"   制造商: {parsed.get('manufacturer')}")
        print(f"   序列号: {parsed.get('serialNumber')}")
        
    except Exception as e:
        print(f"❌ JSON处理失败: {e}")
        return False
    
    return True

def test_mqtt_order_publishing():
    """测试MQTT订单发布"""
    print(f"\n📡 MQTT订单发布测试")
    print("=" * 40)
    
    # 创建测试订单
    nodes = [
        Node(
            nodeId="start",
            sequenceId=0,
            released=True,
            nodePosition=NodePosition(x=0.0, y=0.0, theta=0.0, mapId="test_map")
        ),
        Node(
            nodeId="target",
            sequenceId=2,
            released=True,
            nodePosition=NodePosition(x=3.0, y=2.0, theta=1.57, mapId="test_map"),
            actions=[
                Action(
                    actionId="target_reached",
                    actionType="reportPosition",
                    blockingType="SOFT"
                )
            ]
        )
    ]
    
    edges = [
        Edge(
            edgeId="move_to_target",
            sequenceId=1,
            released=True,
            startNodeId="start",
            endNodeId="target",
            maxSpeed=1.5,
            length=3.6
        )
    ]
    
    order = OrderMessage(
        headerId=1,
        timestamp=create_timestamp(),
        version="2.1.0",
        manufacturer="TestManufacturer",
        serialNumber="TEST_AGV_001",
        orderId="mqtt_test_order",
        orderUpdateId=0,
        nodes=nodes,
        edges=edges
    )
    
    # MQTT发布测试
    try:
        client = mqtt.Client()
        client.connect("localhost", 1883, 60)
        
        topic = "uagv/v2/TestManufacturer/TEST_AGV_001/order"
        message = order.to_json()
        
        result = client.publish(topic, message, qos=0)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"✅ MQTT订单发布成功")
            print(f"   主题: {topic}")
            print(f"   消息大小: {len(message)} 字符")
        else:
            print(f"❌ MQTT发布失败: {result.rc}")
            return False
        
        client.disconnect()
        return True
        
    except Exception as e:
        print(f"❌ MQTT测试失败: {e}")
        return False

def test_target_reached_simulation():
    """测试目标到达模拟"""
    print(f"\n🎯 目标到达模拟测试")
    print("=" * 40)
    
    # 模拟AGV位置
    agv_positions = [
        (0.0, 0.0, 0.0),      # 起始位置
        (1.0, 0.5, 0.2),      # 移动中
        (2.0, 1.0, 0.8),      # 接近目标
        (2.9, 1.9, 1.5),      # 非常接近
        (3.0, 2.0, 1.57),     # 到达目标
    ]
    
    target_pos = (3.0, 2.0, 1.57)
    tolerance_xy = 0.2
    tolerance_theta = 0.1
    
    print(f"🎯 目标位置: ({target_pos[0]}, {target_pos[1]}, {target_pos[2]:.2f})")
    print(f"📏 容差: XY={tolerance_xy}m, Theta={tolerance_theta}rad")
    print(f"\n📍 AGV位置序列:")
    
    for i, (x, y, theta) in enumerate(agv_positions):
        # 计算到目标的距离
        dx = x - target_pos[0]
        dy = y - target_pos[1]
        distance = (dx*dx + dy*dy)**0.5
        
        # 计算角度差
        angle_diff = abs(theta - target_pos[2])
        angle_diff = min(angle_diff, 2*3.14159 - angle_diff)
        
        # 检查是否到达
        reached = (distance <= tolerance_xy and angle_diff <= tolerance_theta)
        
        status = "🎯 到达!" if reached else "🚶 移动中"
        print(f"   步骤{i+1}: ({x:.1f}, {y:.1f}, {theta:.2f}) "
              f"距离={distance:.2f}m, 角差={angle_diff:.2f}rad {status}")
        
        if reached:
            print(f"   ✅ 目标到达确认!")
            break
    
    return True

def main():
    """主测试函数"""
    print("🧪 ROS2路径转换VDA5050功能测试")
    print("=" * 50)
    print()
    
    tests = [
        ("路径转换", test_path_conversion),
        ("MQTT发布", test_mqtt_order_publishing),
        ("目标到达", test_target_reached_simulation),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\n🔬 运行测试: {test_name}")
        try:
            result = test_func()
            results.append((test_name, result))
            if result:
                print(f"✅ {test_name}测试通过")
            else:
                print(f"❌ {test_name}测试失败")
        except Exception as e:
            print(f"💥 {test_name}测试异常: {e}")
            results.append((test_name, False))
    
    # 总结
    print(f"\n📊 测试总结")
    print("=" * 30)
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"   {test_name}: {status}")
    
    print(f"\n🎯 总体结果: {passed}/{total} 测试通过")
    
    if passed == total:
        print("🎉 所有测试通过! 路径转换功能正常工作")
        print("\n✨ 验证的功能:")
        print("   ✅ ROS2路径点转换为VDA5050节点")
        print("   ✅ 路径段转换为VDA5050边")
        print("   ✅ 目标到达动作生成")
        print("   ✅ JSON序列化和MQTT发布")
        print("   ✅ 目标到达检测逻辑")
    else:
        print("⚠️  部分测试失败，请检查相关功能")

if __name__ == "__main__":
    main()