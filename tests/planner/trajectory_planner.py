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


class ComplexTrajectoryPlanner:
    """
    复杂路径规划器
    用于模拟多阶段轨迹规划，支持前向和后向运动

    设计目标：
    - 前向轨迹：左转90° → 前进 → 左转90°（模拟Traj1 + Traj2）
    - 后向轨迹：倒车指定距离（模拟Traj3，flag=1, orientation=3.14）
    """

    def __init__(self, forward_step: float = 0.15, backward_step: float = 0.15):
        """
        初始化复杂路径规划器

        Args:
            forward_step: 前向移动路径点间距（米）
            backward_step: 后向移动路径点间距（米）
        """
        self.forward_step = forward_step
        self.backward_step = backward_step

    def plan_forward_with_turns(self, start_pose: Pose,
                                first_turn_angle: float,
                                forward_distance: float,
                                second_turn_angle: float) -> List[Tuple[float, float, float]]:
        """
        规划前向轨迹：左转 → 前进 → 左转
        （组合Trajectory 1 + Trajectory 2）

        Args:
            start_pose: 起点Pose（geometry_msgs/Pose）
            first_turn_angle: 第一次转弯角度（弧度，正值为左转）
            forward_distance: 前进距离（米）
            second_turn_angle: 第二次转弯角度（弧度，正值为左转）

        Returns:
            路径点列表 [(x, y, yaw), ...]
        """
        waypoints = []

        # 提取起点信息
        start_x = start_pose.position.x
        start_y = start_pose.position.y
        start_yaw = self._quaternion_to_yaw(start_pose.orientation)

        print(f"\n📋 复杂前向轨迹规划:")
        print(f"   起点: ({start_x:.3f}, {start_y:.3f}), yaw={start_yaw:.3f} ({math.degrees(start_yaw):.1f}°)")
        print(f"   第一次左转: {math.degrees(first_turn_angle):.1f}°")
        print(f"   前进距离: {forward_distance:.3f}m")
        print(f"   第二次左转: {math.degrees(second_turn_angle):.1f}°")

        current_x = start_x
        current_y = start_y
        current_yaw = start_yaw

        # 阶段1: 第一次左转（原地旋转，只生成起点和终点）
        print(f"   阶段1: 原地左转 {math.degrees(first_turn_angle):.1f}°")
        yaw_after_first_turn = start_yaw + first_turn_angle
        waypoints.append((current_x, current_y, current_yaw))  # 起点
        waypoints.append((current_x, current_y, yaw_after_first_turn))  # 转弯后
        current_yaw = yaw_after_first_turn

        # 阶段2: 前进
        num_forward_points = int(forward_distance / self.forward_step) + 1
        print(f"   阶段2: 前进 {forward_distance:.3f}m (点间距{self.forward_step}m, {num_forward_points}个点)")

        for i in range(1, num_forward_points + 1):
            dist = i * self.forward_step
            if dist > forward_distance:
                dist = forward_distance
            x = current_x + dist * math.cos(current_yaw)
            y = current_y + dist * math.sin(current_yaw)
            waypoints.append((x, y, current_yaw))

        # 更新当前位置到前进终点
        current_x = current_x + forward_distance * math.cos(current_yaw)
        current_y = current_y + forward_distance * math.sin(current_yaw)

        # 阶段3: 第二次左转（原地旋转，只生成终点）
        print(f"   阶段3: 原地左转 {math.degrees(second_turn_angle):.1f}°")
        yaw_after_second_turn = current_yaw + second_turn_angle
        waypoints.append((current_x, current_y, yaw_after_second_turn))

        print(f"   ✅ 前向轨迹规划完成: 共 {len(waypoints)} 个路径点")
        print(f"   终点: ({current_x:.3f}, {current_y:.3f}), yaw={yaw_after_second_turn:.3f} ({math.degrees(yaw_after_second_turn):.1f}°)\n")

        return waypoints

    def plan_backward(self, start_pose: Pose, backward_distance: float) -> List[Tuple[float, float, float]]:
        """
        规划后向轨迹：沿y轴方向移动指定距离（支持正负距离）
        （对应Trajectory 3，flag=1, orientation=3.14）

        Args:
            start_pose: 起点Pose（geometry_msgs/Pose）
            backward_distance: 倒车距离（米），负数表示沿-y方向移动

        Returns:
            路径点列表 [(x, y, yaw), ...] - 朝向保持不变，位置沿y轴移动
        """
        waypoints = []

        # 提取起点信息
        start_x = start_pose.position.x
        start_y = start_pose.position.y
        start_yaw = self._quaternion_to_yaw(start_pose.orientation)

        print(f"\n📋 后向轨迹规划 (倒车):")
        print(f"   起点: ({start_x:.3f}, {start_y:.3f}), yaw={start_yaw:.3f} ({math.degrees(start_yaw):.1f}°)")
        print(f"   Y方向位移: {backward_distance:.3f}m")

        # 计算倒车路径点数量（使用绝对值）
        abs_distance = abs(backward_distance)
        num_backward_points = int(abs_distance / self.backward_step) + 1
        print(f"   生成路径点: {num_backward_points}个 (点间距{self.backward_step}m)")

        # 生成倒车路径点（沿y轴方向移动，朝向保持不变）
        # backward_distance为负时，沿-y方向移动
        for i in range(num_backward_points):
            dist = i * self.backward_step
            if dist > abs_distance:
                dist = abs_distance

            # 沿y轴方向移动：如果backward_distance为负，则向-y移动
            if backward_distance >= 0:
                y = start_y + dist
            else:
                y = start_y - dist

            waypoints.append((start_x, y, start_yaw))  # x保持不变，yaw保持不变

        # 计算并打印终点
        end_x = start_x
        end_y = start_y + backward_distance

        print(f"   ✅ 后向轨迹规划完成: 共 {len(waypoints)} 个路径点")
        print(f"   终点: ({end_x:.3f}, {end_y:.3f}), yaw={start_yaw:.3f} ({math.degrees(start_yaw):.1f}°)\n")

        return waypoints

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
