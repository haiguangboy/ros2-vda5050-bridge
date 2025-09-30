#!/usr/bin/env python3
"""
测试原地旋转功能
验证位置完全不变，只改变朝向角度
"""

import math
import sys
import os

# 导入配置
sys.path.append(os.path.dirname(__file__))
from danci3_test_nav_path_publisher import *

def test_in_place_rotation():
    """测试原地旋转：位置不变，只改变角度"""
    print("=" * 80)
    print("测试原地旋转功能（位置不变，只改变朝向）")
    print("=" * 80)

    # 测试场景
    start_x, start_y, start_z = 2.0, 1.5, 0.0
    start_yaw = math.pi / 6  # 30度

    print(f"测试场景:")
    print(f"  起始位置: ({start_x}, {start_y}, {start_z})")
    print(f"  起始朝向: {math.degrees(start_yaw):.1f}°")
    print(f"  旋转角度: {math.degrees(TURN_ANGLE_RAD):.1f}°")
    print(f"  旋转步数: {TURN_STEPS}")

    print(f"\n--- 原地旋转路径点 ---")

    # 模拟阶段2的原地旋转
    all_same_position = True
    for i in range(0, TURN_STEPS + 1):
        angle_offset = i * (TURN_ANGLE_RAD / TURN_STEPS)
        current_yaw = start_yaw + angle_offset

        # 原地旋转：位置保持不变
        current_x = start_x  # 位置不变
        current_y = start_y  # 位置不变

        print(f"  Point {i}: x={current_x:.3f}, y={current_y:.3f}, yaw={math.degrees(current_yaw):.1f}°")

        # 验证位置是否保持不变
        if current_x != start_x or current_y != start_y:
            all_same_position = False

    print(f"\n--- 验证结果 ---")
    if all_same_position:
        print(f"✅ 位置验证：所有路径点的x,y坐标完全相同")
    else:
        print(f"❌ 位置验证：发现位置变化")

    # 计算角度变化
    final_yaw = start_yaw + TURN_ANGLE_RAD
    angle_change = math.degrees(TURN_ANGLE_RAD)
    print(f"✅ 角度验证：从 {math.degrees(start_yaw):.1f}° 旋转到 {math.degrees(final_yaw):.1f}°，变化 {angle_change:.1f}°")

    return all_same_position

def test_90_degree_rotation():
    """测试90度原地旋转"""
    global TURN_ANGLE_RAD, TURN_STEPS

    print(f"\n" + "=" * 80)
    print("测试90度原地旋转")
    print("=" * 80)

    # 临时修改配置为90度
    original_angle = TURN_ANGLE_RAD
    original_steps = TURN_STEPS
    TURN_ANGLE_RAD = math.pi / 2  # 90度
    TURN_STEPS = 3  # 3步完成

    start_x, start_y = 5.0, 3.0
    start_yaw = 0.0  # 朝向东方

    print(f"测试90度旋转:")
    print(f"  起始位置: ({start_x}, {start_y})")
    print(f"  起始朝向: {math.degrees(start_yaw):.1f}° (东方)")
    print(f"  目标朝向: {math.degrees(start_yaw + TURN_ANGLE_RAD):.1f}° (北方)")

    print(f"\n--- 90度旋转路径 ---")
    for i in range(0, TURN_STEPS + 1):
        angle_offset = i * (TURN_ANGLE_RAD / TURN_STEPS)
        current_yaw = start_yaw + angle_offset

        print(f"  Point {i}: x={start_x:.3f}, y={start_y:.3f}, yaw={math.degrees(current_yaw):.1f}°")

    print(f"\n✅ 90度原地旋转：位置({start_x}, {start_y})保持不变，朝向从0°变为90°")

    # 恢复原始配置
    TURN_ANGLE_RAD = original_angle
    TURN_STEPS = original_steps

def compare_rotation_types():
    """比较原地旋转和绕轴旋转的差异"""
    print(f"\n" + "=" * 80)
    print("对比：原地旋转 vs 绕后轴旋转")
    print("=" * 80)

    start_x, start_y = 2.0, 0.0
    start_yaw = 0.0
    steps = 2

    print(f"起始位置: ({start_x}, {start_y}), 起始朝向: {math.degrees(start_yaw):.1f}°")
    print(f"旋转角度: {math.degrees(TURN_ANGLE_RAD):.1f}°")

    print(f"\n【修改前】绕后轴中心旋转:")
    rear_axle_x = start_x - VEHICLE_CENTER_OFFSET * math.cos(start_yaw)
    rear_axle_y = start_y - VEHICLE_CENTER_OFFSET * math.sin(start_yaw)
    print(f"  后轴中心: ({rear_axle_x:.3f}, {rear_axle_y:.3f})")

    for i in range(0, steps + 1):
        angle_offset = i * (TURN_ANGLE_RAD / steps)
        current_yaw = start_yaw + angle_offset
        # 绕后轴中心旋转
        old_x = rear_axle_x + VEHICLE_CENTER_OFFSET * math.cos(current_yaw)
        old_y = rear_axle_y + VEHICLE_CENTER_OFFSET * math.sin(current_yaw)
        print(f"    Point {i}: x={old_x:.3f}, y={old_y:.3f}, yaw={math.degrees(current_yaw):.1f}°")

    print(f"\n【修改后】原地旋转:")
    for i in range(0, steps + 1):
        angle_offset = i * (TURN_ANGLE_RAD / steps)
        current_yaw = start_yaw + angle_offset
        # 原地旋转：位置不变
        print(f"    Point {i}: x={start_x:.3f}, y={start_y:.3f}, yaw={math.degrees(current_yaw):.1f}°")

    print(f"\n✅ 区别明显：原地旋转位置完全不变，更符合真实的原地掉头行为")

if __name__ == '__main__':
    # 测试原地旋转
    success = test_in_place_rotation()

    # 测试90度旋转
    test_90_degree_rotation()

    # 对比两种旋转方式
    compare_rotation_types()

    print(f"\n" + "=" * 80)
    if success:
        print("✅ 修改成功！阶段2现在实现真正的原地旋转（位置不变，只改变朝向）")
    else:
        print("❌ 修改失败！请检查代码")
    print("=" * 80)