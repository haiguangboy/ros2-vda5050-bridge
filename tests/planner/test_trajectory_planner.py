#!/usr/bin/env python3
"""
轨迹规划器测试脚本
测试 SimpleTrajectoryPlanner 的各种场景
"""

import math
from geometry_msgs.msg import Pose, Point, Quaternion
from trajectory_planner import SimpleTrajectoryPlanner


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    将yaw角度转换为四元数

    Args:
        yaw: yaw角度（弧度）

    Returns:
        geometry_msgs/Quaternion
    """
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


def create_pose(x: float, y: float, yaw: float) -> Pose:
    """
    创建Pose消息

    Args:
        x, y: 位置坐标
        yaw: 朝向角度（弧度）

    Returns:
        geometry_msgs/Pose
    """
    pose = Pose()
    pose.position = Point(x=x, y=y, z=0.0)
    pose.orientation = yaw_to_quaternion(yaw)
    return pose


def test_scenario_1():
    """
    测试场景1: 直线前进 + 目标朝向改变
    起点: (0, 0, 0°)
    终点: (2, 0, 90°)
    """
    print("\n" + "="*80)
    print("测试场景1: 直线前进 + 旋转到新朝向")
    print("="*80)

    planner = SimpleTrajectoryPlanner(step_size=0.15)

    start_pose = create_pose(0.0, 0.0, 0.0)
    goal_pose = create_pose(2.0, 0.0, math.radians(90))

    waypoints = planner.plan_from_pose(start_pose, goal_pose)
    planner.print_waypoints(waypoints)

    return waypoints


def test_scenario_2():
    """
    测试场景2: 需要先旋转朝向目标
    起点: (0, 0, 0°)
    终点: (1, 0.3 90°
    """
    print("\n" + "="*80)
    print("测试场景2: 先旋转朝向目标 → 移动 → 旋转到目标朝向")
    print("="*80)

    planner = SimpleTrajectoryPlanner(step_size=0.15)

    start_pose = create_pose(0.0, 0.0, 0.0)
    goal_pose = create_pose(1.0, 0.3, math.radians(90))

    waypoints = planner.plan_from_pose(start_pose, goal_pose)
    planner.print_waypoints(waypoints)

    return waypoints


def test_scenario_3():
    """
    测试场景3: 仅原地旋转（位置不变）
    起点: (1, 1, 0°)
    终点: (1, 1, 180°)
    """
    print("\n" + "="*80)
    print("测试场景3: 仅原地旋转180°")
    print("="*80)

    planner = SimpleTrajectoryPlanner(step_size=0.15)

    start_pose = create_pose(1.0, 1.0, 0.0)
    goal_pose = create_pose(1.0, 1.0, math.radians(180))

    waypoints = planner.plan_from_pose(start_pose, goal_pose)
    planner.print_waypoints(waypoints)

    return waypoints


def test_scenario_4():
    """
    测试场景4: 后退场景（目标在后方）
    起点: (0, 0, 0°)
    终点: (-1.5, 0, 180°)
    """
    print("\n" + "="*80)
    print("测试场景4: 后退场景（目标在后方）")
    print("="*80)

    planner = SimpleTrajectoryPlanner(step_size=0.15)

    start_pose = create_pose(0.0, 0.0, 0.0)
    goal_pose = create_pose(-1.5, 0.0, math.radians(180))

    waypoints = planner.plan_from_pose(start_pose, goal_pose)
    planner.print_waypoints(waypoints)

    return waypoints


def test_scenario_5():
    """
    测试场景5: 长距离移动 + 大角度旋转
    起点: (0, 0, 0°)
    终点: (3, 4, -90°)
    """
    print("\n" + "="*80)
    print("测试场景5: 长距离移动(5m) + 大角度旋转")
    print("="*80)

    planner = SimpleTrajectoryPlanner(step_size=0.15)

    start_pose = create_pose(0.0, 0.0, 0.0)
    goal_pose = create_pose(3.0, 4.0, math.radians(-90))

    waypoints = planner.plan_from_pose(start_pose, goal_pose)
    # 路径点太多，只打印前10个和最后10个
    planner.print_waypoints(waypoints, max_points=10)

    return waypoints


def test_scenario_6():
    """
    测试场景6: 已经面向目标位置和朝向（几乎无需调整）
    起点: (0, 0, 45°)
    终点: (1.414, 1.414, 45°)
    """
    print("\n" + "="*80)
    print("测试场景6: 已经正确朝向，直接移动即可")
    print("="*80)

    planner = SimpleTrajectoryPlanner(step_size=0.15)

    start_pose = create_pose(0.0, 0.0, math.radians(45))
    goal_pose = create_pose(1.414, 1.414, math.radians(45))

    waypoints = planner.plan_from_pose(start_pose, goal_pose)
    planner.print_waypoints(waypoints)

    return waypoints


def main():
    print("\n" + "🚀"*40)
    print("🧪 轨迹规划器测试程序")
    print("🚀"*40)

    # 运行所有测试场景
    test_scenario_1()
    test_scenario_2()
    test_scenario_3()
    test_scenario_4()
    test_scenario_5()
    test_scenario_6()

    print("\n" + "="*80)
    print("✅ 所有测试场景完成")
    print("="*80 + "\n")


if __name__ == "__main__":
    main()
