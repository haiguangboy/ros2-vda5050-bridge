#!/usr/bin/env python3
"""
新协议MQTT客户端测试
Test for New Protocol MQTT Client
"""

import json
import time
import threading
import sys
import os

# Add the package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'ros2_vda5050_bridge'))
from ros2_vda5050_bridge.zhongli_mqtt_client import ZhongliMQTTClient
from ros2_vda5050_bridge.new_protocol_types import *


def test_mqtt_connection():
    """测试MQTT连接"""
    print("🧪 测试MQTT连接")
    print("=" * 40)
    
    client = ZhongliMQTTClient(robot_id="test-robot-001")
    
    try:
        # 测试连接
        if client.connect():
            print("✅ MQTT连接成功")
            
            # 等待连接建立
            time.sleep(2)
            
            if client.is_connected():
                print("✅ 连接状态正常")
                client.disconnect()
                return True
            else:
                print("❌ 连接状态异常")
                return False
        else:
            print("❌ MQTT连接失败")
            return False
            
    except Exception as e:
        print(f"❌ MQTT连接异常: {e}")
        return False


def test_topic_structure():
    """测试Topic结构"""
    print(f"\n🧪 测试Topic结构")
    print("=" * 40)
    
    client = ZhongliMQTTClient(robot_id="test-robot-001")
    topics = client.get_topic_info()
    
    print("📡 Topic结构:")
    for topic_name, topic_path in topics.items():
        print(f"   {topic_name}: {topic_path}")
    
    # 验证关键Topic
    expected_topics = [
        'task_subscribe',
        'trajectory_publish',
        'action_publish',
        'state_publish'
    ]
    
    success = all(topic in topics for topic in expected_topics)
    
    if success:
        print("✅ Topic结构正确")
        return True
    else:
        print("❌ Topic结构不完整")
        return False


def test_message_publishing():
    """测试消息发布"""
    print(f"\n🧪 测试消息发布")
    print("=" * 40)
    
    client = ZhongliMQTTClient(robot_id="test-robot-001")
    
    try:
        # 连接MQTT
        if not client.connect():
            print("❌ MQTT连接失败")
            return False
        
        time.sleep(1)
        
        # 创建测试轨迹消息
        trajectory = TrajectoryMessage(
            timestamp=create_timestamp(),
            trajectoryId="test-traj-001",
            trajectoryPoints=[
                TrajectoryPoint(x=0.0, y=0.0, theta=0.0),
                TrajectoryPoint(x=5.0, y=0.0, theta=0.0),
                TrajectoryPoint(x=5.0, y=3.0, theta=90.0)
            ],
            maxSpeed=1.5
        )
        
        # 发布轨迹消息
        if client.publish_trajectory(trajectory):
            print("✅ 轨迹消息发布成功")
        else:
            print("❌ 轨迹消息发布失败")
            return False
        
        # 创建测试动作消息
        action = ActionMessage(
            timestamp=create_timestamp(),
            actionId="test-action-001",
            actionType="ground_pick",
            containerPose=ContainerPose(x=10.5, y=5.3, z=0.1, theta=180.0),
            containerType="AGV-T300"
        )
        
        # 发布动作消息
        if client.publish_action(action):
            print("✅ 动作消息发布成功")
        else:
            print("❌ 动作消息发布失败")
            return False
        
        # 创建测试设备状态消息
        device_state = DeviceStateMessage(
            timestamp=create_timestamp(),
            pose=Pose(x=10.2, y=5.8, theta=90.0),
            forkliftState=ForkliftState(
                height=0.5,
                weight=150.2,
                lateralShift=0.3,
                forwardExtension=0.8,
                tiltBack=True,
                status="ready"
            ),
            battery=BatteryState(level=85, charging=False),
            errors=[],
            systemState="running"
        )
        
        # 发布设备状态消息
        if client.publish_device_state(device_state):
            print("✅ 设备状态消息发布成功")
        else:
            print("❌ 设备状态消息发布失败")
            return False
        
        time.sleep(1)
        client.disconnect()
        
        print("✅ 所有消息发布测试通过")
        return True
        
    except Exception as e:
        print(f"❌ 消息发布测试异常: {e}")
        return False


def test_message_receiving():
    """测试消息接收"""
    print(f"\n🧪 测试消息接收")
    print("=" * 40)
    
    # 创建接收客户端
    receiver = ZhongliMQTTClient(robot_id="test-robot-001")
    
    # 消息接收记录
    received_messages = {
        'task': None,
        'trajectory_status': None,
        'action_status': None
    }
    
    # 设置消息处理器
    def handle_task(task):
        received_messages['task'] = task
        print(f"📋 收到任务: {task.taskId}")
    
    def handle_trajectory_status(status):
        received_messages['trajectory_status'] = status
        print(f"📊 收到轨迹状态: {status.trajectoryId} - {status.status}")
    
    def handle_action_status(status):
        received_messages['action_status'] = status
        print(f"🎬 收到动作状态: {status.actionId} - {status.status}")
    
    receiver.set_task_handler(handle_task)
    receiver.set_trajectory_status_handler(handle_trajectory_status)
    receiver.set_action_status_handler(handle_action_status)
    
    # 创建发送客户端
    sender = ZhongliMQTTClient(robot_id="test-robot-001")
    
    try:
        # 连接两个客户端
        if not receiver.connect():
            print("❌ 接收客户端连接失败")
            return False
        
        if not sender.connect():
            print("❌ 发送客户端连接失败")
            return False
        
        time.sleep(2)
        
        # 发送测试消息
        # 1. 发送任务消息
        task = TaskMessage(
            timestamp=create_timestamp(),
            taskId="test-task-001",
            startArea="A3 仓库区",
            startAction="ground_pick",
            targetArea="B2 车间区",
            targetAction="unload"
        )
        
        # 手动发布到任务主题（模拟调度模块）
        task_topic = f"EP/master/test-robot-001/task"
        task_json = task.to_json()
        sender.client.publish(task_topic, task_json, qos=0)
        
        # 2. 发送轨迹状态消息
        trajectory_status = TrajectoryStatusMessage(
            timestamp=create_timestamp(),
            trajectoryId="test-traj-001",
            status="running",
            currentPointIndex=1,
            errorCode=0,
            errorDesc=""
        )
        
        # 手动发布到轨迹状态主题
        traj_status_topic = f"EP/test-robot-001/cerebellum/embrain/trajectory_status"
        traj_status_json = trajectory_status.to_json()
        sender.client.publish(traj_status_topic, traj_status_json, qos=0)
        
        # 3. 发送动作状态消息
        action_status = ActionStatusMessage(
            timestamp=create_timestamp(),
            actionId="test-action-001",
            status="success",
            errorCode=0,
            errorDesc="",
            finishTime=create_timestamp()
        )
        
        # 手动发布到动作状态主题
        action_status_topic = f"EP/test-robot-001/cerebellum/embrain/action_status"
        action_status_json = action_status.to_json()
        sender.client.publish(action_status_topic, action_status_json, qos=0)
        
        # 等待消息处理
        time.sleep(2)
        
        # 验证接收结果
        success = (
            received_messages['task'] is not None and
            received_messages['trajectory_status'] is not None and
            received_messages['action_status'] is not None
        )
        
        if success:
            print("✅ 消息接收测试通过")
            print(f"   收到任务: {received_messages['task'].taskId}")
            print(f"   收到轨迹状态: {received_messages['trajectory_status'].trajectoryId}")
            print(f"   收到动作状态: {received_messages['action_status'].actionId}")
        else:
            print("❌ 消息接收测试失败")
            print(f"   任务: {'✅' if received_messages['task'] else '❌'}")
            print(f"   轨迹状态: {'✅' if received_messages['trajectory_status'] else '❌'}")
            print(f"   动作状态: {'✅' if received_messages['action_status'] else '❌'}")
        
        # 清理
        receiver.disconnect()
        sender.disconnect()
        
        return success
        
    except Exception as e:
        print(f"❌ 消息接收测试异常: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_json_serialization():
    """测试JSON序列化兼容性"""
    print(f"\n🧪 测试JSON序列化兼容性")
    print("=" * 40)
    
    try:
        # 测试轨迹消息序列化
        trajectory = TrajectoryMessage(
            timestamp=create_timestamp(),
            trajectoryId="test-traj-001",
            trajectoryPoints=[
                TrajectoryPoint(x=1.0, y=2.0, theta=90.0)
            ],
            maxSpeed=1.0
        )
        
        trajectory_json = trajectory.to_json()
        trajectory_parsed = json.loads(trajectory_json)
        
        # 验证字段
        assert 'timestamp' in trajectory_parsed
        assert 'trajectoryId' in trajectory_parsed
        assert 'trajectoryPoints' in trajectory_parsed
        assert 'maxSpeed' in trajectory_parsed
        
        print("✅ 轨迹消息JSON序列化正常")
        
        # 测试动作消息序列化
        action = ActionMessage(
            timestamp=create_timestamp(),
            actionId="test-action-001",
            actionType="ground_pick",
            containerPose=ContainerPose(x=1.0, y=2.0, z=0.1, theta=180.0)
        )
        
        action_json = action.to_json()
        action_parsed = json.loads(action_json)
        
        # 验证字段
        assert 'timestamp' in action_parsed
        assert 'actionId' in action_parsed
        assert 'actionType' in action_parsed
        assert 'containerPose' in action_parsed
        
        print("✅ 动作消息JSON序列化正常")
        
        # 测试设备状态消息序列化
        device_state = DeviceStateMessage(
            timestamp=create_timestamp(),
            pose=Pose(x=1.0, y=2.0, theta=90.0),
            forkliftState=ForkliftState(
                height=0.5,
                weight=100.0,
                lateralShift=0.0,
                forwardExtension=0.0,
                tiltBack=False,
                status="ready"
            ),
            battery=BatteryState(level=80, charging=False),
            errors=[],
            systemState="running"
        )
        
        device_json = device_state.to_json()
        device_parsed = json.loads(device_json)
        
        # 验证字段
        assert 'timestamp' in device_parsed
        assert 'pose' in device_parsed
        assert 'forkliftState' in device_parsed
        assert 'battery' in device_parsed
        assert 'systemState' in device_parsed
        
        print("✅ 设备状态消息JSON序列化正常")
        
        print("✅ JSON序列化兼容性测试通过")
        return True
        
    except Exception as e:
        print(f"❌ JSON序列化测试异常: {e}")
        return False


def main():
    """主测试函数"""
    print("🧪 中力具身机器人系统MQTT客户端测试")
    print("=" * 60)
    print()
    
    tests = [
        ("MQTT连接", test_mqtt_connection),
        ("Topic结构", test_topic_structure),
        ("JSON序列化", test_json_serialization),
        ("消息发布", test_message_publishing),
        ("消息接收", test_message_receiving),
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
            import traceback
            traceback.print_exc()
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
        print("🎉 所有测试通过! 新协议MQTT客户端功能正常")
        print("\n✨ 验证的功能:")
        print("   ✅ MQTT连接和断开")
        print("   ✅ Topic结构定义")
        print("   ✅ JSON序列化兼容性")
        print("   ✅ 消息发布功能")
        print("   ✅ 消息接收和处理")
    else:
        print("⚠️  部分测试失败，请检查相关功能")
        print("\n🔧 调试建议:")
        print("   1. 确保MQTT broker正在运行")
        print("   2. 检查网络连接")
        print("   3. 验证Topic权限")


if __name__ == "__main__":
    main()