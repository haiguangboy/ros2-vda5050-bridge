#!/usr/bin/env python3
"""
简单路径规划器
提供从起点到目标点的路径规划功能
"""

import math
from typing import List, Tuple
from geometry_msgs.msg import Pose, Quaternion


class SimpleTrajectoryPlanner:
    """
    简单路径规划器
    策略：先旋转朝向目标位置 → 直线移动到目标位置 → 旋转到目标朝向
    """

    def __init__(self, step_size: float = 0.15):
        """
        初始化规划器

        Args:
            step_size: 路径点间距（米）
        """
        self.step_size = step_size

    def plan_from_pose(self, start_pose: Pose, goal_pose: Pose) -> List[Tuple[float, float, float]]:
        """
        基于ROS2 Pose消息规划路径

        Args:
            start_pose: 起点Pose（geometry_msgs/Pose）
            goal_pose: 目标点Pose（geometry_msgs/Pose）

        Returns:
            路径点列表 [(x, y, yaw), ...]
        """
        # 提取起点信息
        start_x = start_pose.position.x
        start_y = start_pose.position.y
        start_yaw = self._quaternion_to_yaw(start_pose.orientation)

        # 提取目标点信息
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        goal_yaw = self._quaternion_to_yaw(goal_pose.orientation)

        return self.plan(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw)

    def plan(self, start_x: float, start_y: float, start_yaw: float,
             goal_x: float, goal_y: float, goal_yaw: float) -> List[Tuple[float, float, float]]:
        """
        规划从起点到终点的路径

        Args:
            start_x, start_y, start_yaw: 起点位置和朝向（弧度）
            goal_x, goal_y, goal_yaw: 目标位置和朝向（弧度）

        Returns:
            路径点列表 [(x, y, yaw), ...]
        """
        waypoints = []

        # 计算到目标位置的距离和方向
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.sqrt(dx**2 + dy**2)

        print(f"\n📋 路径规划开始:")
        print(f"   起点: ({start_x:.3f}, {start_y:.3f}), yaw={start_yaw:.3f} ({math.degrees(start_yaw):.1f}°)")
        print(f"   终点: ({goal_x:.3f}, {goal_y:.3f}), yaw={goal_yaw:.3f} ({math.degrees(goal_yaw):.1f}°)")
        print(f"   直线距离: {distance:.3f}m")

        # 特殊情况：起点和终点位置重合，只需旋转
        if distance < 0.01:
            print(f"   策略: 仅原地旋转")
            waypoints = self._generate_rotation_path(
                start_x, start_y, start_yaw, goal_yaw
            )
            return waypoints

        # 计算移动到目标位置所需的朝向角度
        target_angle = math.atan2(dy, dx)

        # 阶段1：旋转朝向目标位置（如果需要）
        angle_to_target = self._normalize_angle(target_angle - start_yaw)
        current_x, current_y, current_yaw = start_x, start_y, start_yaw

        if abs(angle_to_target) > 0.1:  # 阈值约5.7度
            print(f"   阶段1: 原地旋转 {math.degrees(angle_to_target):.1f}° 朝向目标位置")
            rotation_points = self._generate_rotation_path(
                current_x, current_y, current_yaw, target_angle
            )
            waypoints.extend(rotation_points)
            current_yaw = target_angle
        else:
            # 角度差很小，直接添加起点
            waypoints.append((current_x, current_y, current_yaw))

        # 阶段2：直线移动到目标位置
        num_points = int(distance / self.step_size) + 1
        print(f"   阶段2: 直线移动 {distance:.3f}m (点间距{self.step_size}m, {num_points}个点)")

        for i in range(1, num_points + 1):
            t = i / num_points
            x = start_x + dx * t
            y = start_y + dy * t
            waypoints.append((x, y, current_yaw))

        current_x, current_y = goal_x, goal_y

        # 阶段3：旋转到目标朝向（如果需要）
        final_angle_diff = self._normalize_angle(goal_yaw - current_yaw)

        if abs(final_angle_diff) > 0.1:
            print(f"   阶段3: 原地旋转 {math.degrees(final_angle_diff):.1f}° 到目标朝向")
            rotation_points = self._generate_rotation_path(
                current_x, current_y, current_yaw, goal_yaw
            )
            # 去掉第一个点避免重复
            waypoints.extend(rotation_points[1:])

        print(f"   ✅ 规划完成: 共 {len(waypoints)} 个路径点\n")

        return waypoints

    def _generate_rotation_path(self, x: float, y: float,
                                start_yaw: float, end_yaw: float) -> List[Tuple[float, float, float]]:
        """
        生成原地旋转路径（只包含起点和终点）

        Args:
            x, y: 旋转中心位置
            start_yaw: 起始朝向（弧度）
            end_yaw: 目标朝向（弧度）

        Returns:
            旋转路径点列表 [(x, y, yaw), ...] - 只包含起点和终点
        """
        # 原地旋转只需要两个点：起始朝向和目标朝向
        return [(x, y, start_yaw), (x, y, end_yaw)]

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """
        将角度归一化到 [-pi, pi]

        Args:
            angle: 输入角度（弧度）

        Returns:
            归一化后的角度（弧度）
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def _quaternion_to_yaw(q: Quaternion) -> float:
        """
        将四元数转换为yaw角度（弧度）

        Args:
            q: geometry_msgs/Quaternion

        Returns:
            yaw角度（弧度）
        """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def print_waypoints(self, waypoints: List[Tuple[float, float, float]], max_points: int = None):
        """
        打印路径点信息（用于调试）

        Args:
            waypoints: 路径点列表
            max_points: 最多打印多少个点，None表示全部打印
        """
        if not waypoints:
            print("⚠️  路径为空")
            return

        print(f"\n{'='*80}")
        print(f"📍 路径点详情（共 {len(waypoints)} 个点）")
        print(f"{'='*80}")

        points_to_print = waypoints if max_points is None else waypoints[:max_points]

        for i, (x, y, yaw) in enumerate(points_to_print, 1):
            print(f"  点{i:3d}: x={x:8.3f}, y={y:8.3f}, yaw={yaw:7.3f} ({math.degrees(yaw):6.1f}°)")

        if max_points and len(waypoints) > max_points:
            print(f"  ... (省略 {len(waypoints) - max_points} 个点)")
            print(f"\n  最后一个点:")
            x, y, yaw = waypoints[-1]
            print(f"  点{len(waypoints):3d}: x={x:8.3f}, y={y:8.3f}, yaw={yaw:7.3f} ({math.degrees(yaw):6.1f}°)")

        print(f"{'='*80}\n")
