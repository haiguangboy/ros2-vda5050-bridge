#!/usr/bin/env python3
"""
新协议数据类型测试
Test for New Protocol Data Types
"""

import json
import sys
import os

# Add the package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'ros2_vda5050_bridge'))
from ros2_vda5050_bridge.new_protocol_types import *


def test_trajectory_message():
    """测试轨迹消息"""
    print("🧪 测试轨迹消息 (TrajectoryMessage)")
    print("=" * 50)
    
    # 创建轨迹点
    points = [
        TrajectoryPoint(x=0.0, y=0.0, theta=0.0),
        TrajectoryPoint(x=5.0, y=0.0, theta=0.0),
        TrajectoryPoint(x=5.0, y=3.0, theta=90.0),
        TrajectoryPoint(x=8.0, y=3.0, theta=0.0)
    ]
    
    # 创建轨迹消息
    trajectory = TrajectoryMessage(
        timestamp=create_timestamp(),
        trajectoryId=generate_trajectory_id("robot-001"),
        trajectoryPoints=points,
        maxSpeed=1.5
    )
    
    print(f"📦 轨迹ID: {trajectory.trajectoryId}")
    print(f"📍 轨迹点数: {len(trajectory.trajectoryPoints)}")
    print(f"⚡ 最大速度: {trajectory.maxSpeed} m/s")
    
    # 显示轨迹点
    print("\n📍 轨迹点:")
    for i, point in enumerate(trajectory.trajectoryPoints):
        print(f"   {i+1}: ({point.x}, {point.y}, {point.theta}°)")
    
    # 测试JSON序列化
    print(f"\n📄 JSON序列化测试...")
    try:
        json_str = trajectory.to_json()
        print(f"✅ JSON序列化成功")
        
        # 验证JSON可以解析
        parsed = json.loads(json_str)
        print(f"✅ JSON解析验证成功")
        
        # 验证关键字段
        assert parsed['trajectoryId'] == trajectory.trajectoryId
        assert parsed['maxSpeed'] == trajectory.maxSpeed
        assert len(parsed['trajectoryPoints']) == len(points)
        
        # 测试从字典重建
        rebuilt = TrajectoryMessage.from_dict(parsed)
        assert rebuilt.trajectoryId == trajectory.trajectoryId
        assert len(rebuilt.trajectoryPoints) == len(trajectory.trajectoryPoints)
        
        print(f"✅ 轨迹消息测试通过")
        return True
        
    except Exception as e:
        print(f"❌ 轨迹消息测试失败: {e}")
        return False


def test_action_message():
    """测试动作消息"""
    print(f"\n🧪 测试动作消息 (ActionMessage)")
    print("=" * 50)
    
    # 创建动作消息
    action = ActionMessage(
        timestamp=create_timestamp(),
        actionId=generate_action_id("robot-001"),
        actionType="ground_pick",
        containerPose=ContainerPose(x=10.5, y=5.3, z=0.1, theta=180.0),
        containerType="AGV-T300"
    )
    
    print(f"🎬 动作ID: {action.actionId}")
    print(f"🔄 动作类型: {action.actionType}")
    print(f"📦 容器类型: {action.containerType}")
    
    if action.containerPose:
        pose = action.containerPose
        print(f"📍 容器位置: ({pose.x}, {pose.y}, {pose.z}, {pose.theta}°)")
    
    # 测试JSON序列化
    print(f"\n📄 JSON序列化测试...")
    try:
        json_str = action.to_json()
        print(f"✅ JSON序列化成功")
        
        # 验证JSON可以解析
        parsed = json.loads(json_str)
        print(f"✅ JSON解析验证成功")
        
        # 验证关键字段
        assert parsed['actionId'] == action.actionId
        assert parsed['actionType'] == action.actionType
        assert parsed['containerType'] == action.containerType
        
        # 测试从字典重建
        rebuilt = ActionMessage.from_dict(parsed)
        assert rebuilt.actionId == action.actionId
        assert rebuilt.actionType == action.actionType
        
        print(f"✅ 动作消息测试通过")
        return True
        
    except Exception as e:
        print(f"❌ 动作消息测试失败: {e}")
        return False


def test_device_state_message():
    """测试设备状态消息"""
    print(f"\n🧪 测试设备状态消息 (DeviceStateMessage)")
    print("=" * 50)
    
    # 创建设备状态消息
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
        errors=[
            ErrorInfo(code=2001, desc="货叉高度接近上限", timestamp=create_timestamp())
        ],
        systemState="running"
    )
    
    print(f"🤖 系统状态: {device_state.systemState}")
    print(f"📍 位置: ({device_state.pose.x}, {device_state.pose.y}, {device_state.pose.theta}°)")
    print(f"🔋 电池: {device_state.battery.level}% ({'充电中' if device_state.battery.charging else '未充电'})")
    print(f"🏗️  货叉状态: {device_state.forkliftState.status}")
    print(f"⚠️  错误数量: {len(device_state.errors)}")
    
    # 测试JSON序列化
    print(f"\n📄 JSON序列化测试...")
    try:
        json_str = device_state.to_json()
        print(f"✅ JSON序列化成功")
        
        # 验证JSON可以解析
        parsed = json.loads(json_str)
        print(f"✅ JSON解析验证成功")
        
        # 验证关键字段
        assert parsed['systemState'] == device_state.systemState
        assert parsed['pose']['x'] == device_state.pose.x
        assert parsed['battery']['level'] == device_state.battery.level
        assert len(parsed['errors']) == len(device_state.errors)
        
        print(f"✅ 设备状态消息测试通过")
        return True
        
    except Exception as e:
        print(f"❌ 设备状态消息测试失败: {e}")
        return False


def test_task_message():
    """测试任务消息"""
    print(f"\n🧪 测试任务消息 (TaskMessage)")
    print("=" * 50)
    
    # 创建任务消息
    task = TaskMessage(
        timestamp=create_timestamp(),
        taskId=generate_task_id("robot-001"),
        startArea="A3 仓库区",
        startAction="ground_pick",
        targetArea="B2 车间区",
        targetAction="unload"
    )
    
    print(f"📋 任务ID: {task.taskId}")
    print(f"🏠 起始区域: {task.startArea}")
    print(f"🔄 起始动作: {task.startAction}")
    print(f"🎯 目标区域: {task.targetArea}")
    print(f"🔄 目标动作: {task.targetAction}")
    
    # 测试JSON序列化
    print(f"\n📄 JSON序列化测试...")
    try:
        json_str = task.to_json()
        print(f"✅ JSON序列化成功")
        
        # 验证JSON可以解析
        parsed = json.loads(json_str)
        print(f"✅ JSON解析验证成功")
        
        # 验证关键字段
        assert parsed['taskId'] == task.taskId
        assert parsed['startArea'] == task.startArea
        assert parsed['targetAction'] == task.targetAction
        
        # 测试从字典重建
        rebuilt = TaskMessage.from_dict(parsed)
        assert rebuilt.taskId == task.taskId
        assert rebuilt.startAction == task.startAction
        
        print(f"✅ 任务消息测试通过")
        return True
        
    except Exception as e:
        print(f"❌ 任务消息测试失败: {e}")
        return False


def test_id_generation():
    """测试ID生成"""
    print(f"\n🧪 测试ID生成")
    print("=" * 50)
    
    robot_id = "robot-001"
    
    # 生成多个ID测试唯一性（添加延迟确保时间戳不同）
    trajectory_ids = []
    action_ids = []
    task_ids = []
    
    for i in range(10):
        trajectory_ids.append(generate_trajectory_id(robot_id))
        import time
        time.sleep(0.001)  # 1ms延迟
        action_ids.append(generate_action_id(robot_id))
        time.sleep(0.001)
        task_ids.append(generate_task_id(robot_id))
        time.sleep(0.001)
    
    print(f"🎯 机器人ID: {robot_id}")
    print(f"📊 轨迹ID数量: {len(set(trajectory_ids))} (应等于10)")
    print(f"🎬 动作ID数量: {len(set(action_ids))} (应等于10)")
    print(f"📋 任务ID数量: {len(set(task_ids))} (应等于10)")
    
    # 显示示例
    print(f"\n📝 示例ID:")
    print(f"   轨迹ID: {trajectory_ids[0]}")
    print(f"   动作ID: {action_ids[0]}")
    print(f"   任务ID: {task_ids[0]}")
    
    # 验证唯一性
    success = (
        len(set(trajectory_ids)) == 10 and
        len(set(action_ids)) == 10 and
        len(set(task_ids)) == 10
    )
    
    if success:
        print(f"✅ ID生成测试通过")
        return True
    else:
        print(f"❌ ID生成测试失败 - 存在重复ID")
        return False


def main():
    """主测试函数"""
    print("🧪 中力具身机器人系统通信协议数据类型测试")
    print("=" * 60)
    print()
    
    tests = [
        ("轨迹消息", test_trajectory_message),
        ("动作消息", test_action_message),
        ("设备状态消息", test_device_state_message),
        ("任务消息", test_task_message),
        ("ID生成", test_id_generation),
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
        print("🎉 所有测试通过! 新协议数据类型定义正确")
        print("\n✨ 验证的功能:")
        print("   ✅ 轨迹消息数据结构和JSON序列化")
        print("   ✅ 动作消息数据结构和JSON序列化")
        print("   ✅ 设备状态消息数据结构和JSON序列化")
        print("   ✅ 任务消息数据结构和JSON序列化")
        print("   ✅ ID生成和唯一性验证")
    else:
        print("⚠️  部分测试失败，请检查相关功能")


if __name__ == "__main__":
    main()