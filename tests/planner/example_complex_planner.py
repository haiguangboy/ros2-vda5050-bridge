#!/usr/bin/env python3
"""
ComplexTrajectoryPlanner 使用示例

演示如何使用ComplexTrajectoryPlanner规划复杂多阶段轨迹
模拟 test_beta4_trajectory_workflow_goal.py 中的 Traj1+Traj2 和 Traj3
"""

import math
from geometry_msgs.msg import Pose, Quaternion
from trajectory_planner import ComplexTrajectoryPlanner


def euler_to_quaternion(roll, pitch, yaw):
    """欧拉角转四元数"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def main():
    print("=" * 80)
    print("🎯 ComplexTrajectoryPlanner 使用示例")
    print("=" * 80)

    # 创建规划器
    planner = ComplexTrajectoryPlanner(forward_step=0.15, backward_step=0.15)

    # ==================== 场景1: 前向轨迹（组合Traj1 + Traj2） ====================
    print("\n【场景1】前向轨迹：左转90° → 前进0.5m → 左转90°")
    print("-" * 80)

    # 创建起始位姿（假设从原点开始，朝向0度）
    start_pose = Pose()
    start_pose.position.x = 0.0
    start_pose.position.y = 0.0
    start_pose.position.z = 0.0
    start_pose.orientation = euler_to_quaternion(0.0, 0.0, 0.0)  # 朝向0度

    # 规划参数（对应原始测试中的Traj1 + Traj2）
    first_turn = math.pi / 2    # 左转90度
    forward_dist = 0.5          # 前进0.5米
    second_turn = math.pi / 2   # 再左转90度

    # 执行规划
    forward_waypoints = planner.plan_forward_with_turns(
        start_pose,
        first_turn,
        forward_dist,
        second_turn
    )

    # 打印路径点
    planner.print_waypoints(forward_waypoints, max_points=10)

    # ==================== 场景2: 后向轨迹（Traj3） ====================
    print("\n【场景2】后向轨迹：倒车0.3m")
    print("-" * 80)

    # 使用前向轨迹的终点作为后向轨迹的起点
    # （实际应用中应该从Odom获取当前位置）
    backward_start_pose = Pose()
    end_x, end_y, end_yaw = forward_waypoints[-1]
    backward_start_pose.position.x = end_x
    backward_start_pose.position.y = end_y
    backward_start_pose.position.z = 0.0
    backward_start_pose.orientation = euler_to_quaternion(0.0, 0.0, end_yaw)

    # 规划参数（对应原始测试中的Traj3）
    backward_dist = 0.3  # 倒车0.3米

    # 执行规划
    backward_waypoints = planner.plan_backward(
        backward_start_pose,
        backward_dist
    )

    # 打印路径点
    planner.print_waypoints(backward_waypoints)

    # ==================== 总结 ====================
    print("\n" + "=" * 80)
    print("📊 规划总结")
    print("=" * 80)
    print(f"前向轨迹点数: {len(forward_waypoints)}")
    print(f"后向轨迹点数: {len(backward_waypoints)}")
    print()
    print("💡 使用说明:")
    print("1. 前向轨迹对应 test_beta4 中的 Traj1 + Traj2（flag=0, orientation=0.0）")
    print("2. 后向轨迹对应 test_beta4 中的 Traj3（flag=1, orientation=3.14）")
    print("3. 在实际使用时，应该从 /Odom 话题获取当前位置作为起点")
    print("=" * 80)


if __name__ == '__main__':
    main()
