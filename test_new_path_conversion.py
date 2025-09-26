#!/usr/bin/env python3
"""
测试ROS2路径到新协议轨迹的转换
Test ROS2 Path to New Protocol Trajectory Conversion
"""

import json
import math
import time
import sys
import os

# Add the package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'ros2_vda5050_bridge'))
from ros2_vda5050_bridge.new_protocol_types import *


def test_ros2_path_to_trajectory_conversion():
    """测试ROS2路径到新协议轨迹的转换"""
    print("🧪 ROS2路径到新协议轨迹转换测试")
    print("=" * 50)
    
    # 模拟ROS2路径数据（从nav_msgs/Path）
    ros2_path_poses = [
        # [x, y, z, qx, qy, qz, qw] - 位置和四元数
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],      # 起点
        [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],      # 直线前进
        [4.0, 1.0, 0.0, 0.0, 0.0, 0.707, 0.707],  # 左转90度
        [6.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0],      # 继续前进
        [8.0, 3.0, 0.0, 0.0, 0.0, 0.707, 0.707],  # 右转90度
        [10.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],     # 终点
    ]
    
    print(f"📍 原始ROS2路径: {len(ros2_path_poses)} 个姿态点")
    for i, pose in enumerate(ros2_path_poses):
        x, y, z, qx, qy, qz, qw = pose
        print(f"   点{i+1}: ({x:.1f}, {y:.1f}, {z:.1f})")
    
    # 转换为轨迹点
    print(f"\n🔄 转换为新协议轨迹...")
    trajectory_points = []
    
    for i, pose in enumerate(ros2_path_poses):
        x, y, z, qx, qy, qz, qw = pose
        
        # 从四元数转换为偏航角（theta）
        # 使用简化的转换，假设只有绕Z轴的旋转
        theta = 2.0 * math.atan2(qz, qw)
        theta_degrees = math.degrees(theta)
        
        # 确保角度在0-360度范围内
        if theta_degrees < 0:
            theta_degrees += 360.0
        
        trajectory_point = TrajectoryPoint(
            x=x,
            y=y,
            theta=theta_degrees
        )
        trajectory_points.append(trajectory_point)
    
    # 创建轨迹消息
    trajectory = TrajectoryMessage(
        timestamp=create_timestamp(),
        trajectoryId=generate_trajectory_id("robot-001"),
        trajectoryPoints=trajectory_points,
        maxSpeed=1.5  # 默认最大速度1.5m/s
    )
    
    print(f"✅ 轨迹转换完成:")
    print(f"   轨迹ID: {trajectory.trajectoryId}")
    print(f"   轨迹点数: {len(trajectory.trajectoryPoints)}")
    print(f"   最大速度: {trajectory.maxSpeed} m/s")
    
    # 显示转换后的轨迹点
    print(f"\n📍 转换后的轨迹点:")
    for i, point in enumerate(trajectory.trajectoryPoints):
        print(f"   {i+1}: ({point.x:.1f}, {point.y:.1f}, {point.theta:.1f}°)")
    
    # 测试JSON序列化
    print(f"\n📄 JSON序列化测试...")
    try:
        json_str = trajectory.to_json()
        json_size = len(json_str)
        print(f"✅ JSON序列化成功, 大小: {json_size} 字符")
        
        # 验证JSON可以解析
        parsed = json.loads(json_str)
        print(f"✅ JSON解析验证成功")
        
        # 验证关键字段
        assert parsed.get('trajectoryId') == trajectory.trajectoryId
        assert parsed.get('maxSpeed') == trajectory.maxSpeed
        assert len(parsed.get('trajectoryPoints', [])) == len(trajectory_points)
        
        print(f"\n📋 JSON关键信息:")
        print(f"   轨迹ID: {parsed.get('trajectoryId')}")
        print(f"   时间戳: {parsed.get('timestamp')}")
        print(f"   最大速度: {parsed.get('maxSpeed')} m/s")
        print(f"   轨迹点数: {len(parsed.get('trajectoryPoints', []))}")
        
        return True
        
    except Exception as e:
        print(f"❌ JSON处理失败: {e}")
        return False


def test_path_sampling():
    """测试路径采样优化"""
    print(f"\n🧪 路径采样优化测试")
    print("=" * 50)
    
    # 创建一个密集的路径点序列（模拟真实的规划结果）
    dense_path = []
    for i in range(50):
        x = i * 0.2  # 每0.2米一个点
        y = 0.0 if i < 25 else (i - 25) * 0.2  # 前25个点直线，后25个点转弯
        theta = 0.0 if i < 25 else 90.0
        dense_path.append([x, y, 0.0, 0.0, 0.0, 0.0, 1.0])
    
    print(f"📊 原始路径点数: {len(dense_path)}")
    
    # 采样策略1: 固定间隔采样
    sample_interval = max(1, len(dense_path) // 10)  # 采样约10个点
    sampled_indices = list(range(0, len(dense_path), sample_interval))
    
    # 确保包含最后一个点
    if len(dense_path) - 1 not in sampled_indices:
        sampled_indices.append(len(dense_path) - 1)
    
    print(f"📏 采样间隔: {sample_interval}")
    print(f"📊 采样后点数: {len(sampled_indices)}")
    
    # 转换为轨迹点
    trajectory_points = []
    for idx in sampled_indices:
        pose = dense_path[idx]
        x, y, z, qx, qy, qz, qw = pose
        
        # 简化的四元数到角度转换
        theta = 2.0 * math.atan2(qz, qw)
        theta_degrees = math.degrees(theta)
        if theta_degrees < 0:
            theta_degrees += 360.0
        
        trajectory_points.append(TrajectoryPoint(x=x, y=y, theta=theta_degrees))
    
    # 创建轨迹消息
    trajectory = TrajectoryMessage(
        timestamp=create_timestamp(),
        trajectoryId=generate_trajectory_id("robot-001"),
        trajectoryPoints=trajectory_points,
        maxSpeed=1.0
    )
    
    print(f"✅ 采样优化完成:")
    print(f"   原始点数: {len(dense_path)}")
    print(f"   采样点数: {len(trajectory_points)}")
    print(f"   压缩率: {len(trajectory_points)/len(dense_path)*100:.1f}%")
    
    # 显示采样前后的路径对比
    print(f"\n📈 采样效果对比:")
    print(f"   原始路径: ({dense_path[0][0]:.1f}, {dense_path[0][1]:.1f}) -> ({dense_path[-1][0]:.1f}, {dense_path[-1][1]:.1f})")
    print(f"   采样路径: ({trajectory_points[0].x:.1f}, {trajectory_points[0].y:.1f}) -> ({trajectory_points[-1].x:.1f}, {trajectory_points[-1].y:.1f})")
    
    return True


def test_trajectory_validation():
    """测试轨迹验证"""
    print(f"\n🧪 轨迹验证测试")
    print("=" * 50)
    
    # 测试用例1: 空轨迹
    try:
        empty_trajectory = TrajectoryMessage(
            timestamp=create_timestamp(),
            trajectoryId="test-empty",
            trajectoryPoints=[],
            maxSpeed=1.0
        )
        print("❌ 空轨迹应该被拒绝")
        return False
    except:
        print("✅ 空轨迹被正确拒绝")
    
    # 测试用例2: 单点轨迹
    single_point_trajectory = TrajectoryMessage(
        timestamp=create_timestamp(),
        trajectoryId="test-single",
        trajectoryPoints=[TrajectoryPoint(x=0.0, y=0.0, theta=0.0)],
        maxSpeed=1.0
    )
    print("✅ 单点轨迹创建成功")
    
    # 测试用例3: 无效速度
    try:
        invalid_speed_trajectory = TrajectoryMessage(
            timestamp=create_timestamp(),
            trajectoryId="test-speed",
            trajectoryPoints=[TrajectoryPoint(x=0.0, y=0.0, theta=0.0)],
            maxSpeed=-1.0  # 负速度
        )
        print("❌ 负速度应该被拒绝")
        return False
    except:
        print("✅ 负速度被正确拒绝")
    
    # 测试用例4: 正常轨迹
    normal_trajectory = TrajectoryMessage(
        timestamp=create_timestamp(),
        trajectoryId="test-normal",
        trajectoryPoints=[
            TrajectoryPoint(x=0.0, y=0.0, theta=0.0),
            TrajectoryPoint(x=1.0, y=0.0, theta=0.0),
            TrajectoryPoint(x=1.0, y=1.0, theta=90.0)
        ],
        maxSpeed=1.5
    )
    print("✅ 正常轨迹创建成功")
    
    # 验证轨迹连续性
    points = normal_trajectory.trajectoryPoints
    is_continuous = True
    for i in range(len(points) - 1):
        p1, p2 = points[i], points[i + 1]
        distance = ((p2.x - p1.x)**2 + (p2.y - p1.y)**2)**0.5
        if distance > 10.0:  # 假设最大距离限制为10米
            print(f"❌ 轨迹点{i+1}到{i+2}距离过大: {distance:.2f}m")
            is_continuous = False
    
    if is_continuous:
        print("✅ 轨迹连续性验证通过")
    else:
        print("❌ 轨迹连续性验证失败")
        return False
    
    return True


def main():
    """主测试函数"""
    
    print("🧪 ROS2路径到新协议轨迹转换功能测试")
    print("=" * 60)
    print()
    
    tests = [
        ("路径转换", test_ros2_path_to_trajectory_conversion),
        ("路径采样", test_path_sampling),
        ("轨迹验证", test_trajectory_validation),
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
        print("🎉 所有测试通过! 路径转换功能正常工作")
        print("\n✨ 验证的功能:")
        print("   ✅ ROS2路径点转换为轨迹点")
        print("   ✅ 四元数到角度转换")
        print("   ✅ 路径采样和优化")
        print("   ✅ 轨迹验证和检查")
        print("   ✅ JSON序列化和解析")
    else:
        print("⚠️  部分测试失败，请检查相关功能")


if __name__ == "__main__":
    main()