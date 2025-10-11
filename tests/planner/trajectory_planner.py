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
        # 策略：最后一个点必须是精确目标点，如果最后一段 < 0.05m 则合并到目标点
        num_steps = int(distance / self.step_size)
        remaining_distance = distance - num_steps * self.step_size

        # 生成中间点（按step_size间隔）
        for i in range(1, num_steps + 1):
            dist = i * self.step_size
            x = start_x + (dx / distance) * dist
            y = start_y + (dy / distance) * dist
            waypoints.append((x, y, current_yaw))

        # 处理最后一段距离
        if remaining_distance > 0.001:  # 有剩余距离
            if remaining_distance < 0.05:  # 剩余距离太小，去掉最后一个中间点
                if len(waypoints) > 1:  # 确保有点可以去掉
                    waypoints.pop()  # 去掉倒数第二个点
                    print(f"   阶段2: 直线移动 {distance:.3f}m (点间距{self.step_size}m, {num_steps-1}个点 + 终点, 最后{remaining_distance:.3f}m < 0.05m已合并)")
                else:
                    print(f"   阶段2: 直线移动 {distance:.3f}m (点间距{self.step_size}m, 终点)")
            else:
                print(f"   阶段2: 直线移动 {distance:.3f}m (点间距{self.step_size}m, {num_steps}个点 + 终点)")
            # 添加精确的目标点
            waypoints.append((goal_x, goal_y, current_yaw))
        else:
            print(f"   阶段2: 直线移动 {distance:.3f}m (点间距{self.step_size}m, {num_steps}个点)")

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

    def plan_from_pose_curve(self, start_pose: Pose, goal_pose: Pose) -> List[Tuple[float, float, float]]:
        """
        基于ROS2 Pose消息规划曲线路径

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

        return self.plan_curve(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw)

    def plan_curve(self, start_x: float, start_y: float, start_yaw: float,
             goal_x: float, goal_y: float, goal_yaw: float) -> List[Tuple[float, float, float]]:
        """
        基于搜索算法的曲线路径规划

        适用场景：目标点和起始点角度差 ≤ 45°

        算法特点：
        - 每步固定距离 0.15m
        - 相邻步yaw变化 ≤ 6°（0.105 rad）
        - 必须到达目标位置（位置通过搜索算法）
        - 使用贪心搜索 + 启发式引导

        分3个阶段：
        - 阶段1+2合并：使用搜索算法生成曲线路径到达目标位置
        - 阶段3：原地旋转到目标yaw（保留原有逻辑）

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

        # 计算移动到目标位置所需的朝向角度
        target_angle = math.atan2(dy, dx)

        print(f"\n📋 曲线路径规划开始:")
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

        # 阶段1+2（合并）：使用搜索算法生成曲线路径到达目标位置
        print(f"   阶段1+2: 搜索曲线路径到达目标位置")
        curve_waypoints = self._search_curve_to_position(
            start_x, start_y, start_yaw,
            goal_x, goal_y
        )

        if not curve_waypoints:
            print(f"   ⚠️  曲线搜索失败，回退到原有plan()方法")
            # 回退到原始的三阶段规划
            return self.plan(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw)

        waypoints.extend(curve_waypoints)

        # 更新当前位置（曲线终点）
        current_x, current_y, current_yaw = waypoints[-1]

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

    def _search_curve_to_position(self, start_x: float, start_y: float, start_yaw: float,
                                  goal_x: float, goal_y: float) -> List[Tuple[float, float, float]]:
        """
        搜索从起点到目标位置的曲线路径（贪心搜索 + 启发式）

        算法核心：
        1. 从起点开始迭代
        2. 每步生成多个候选角度（在±6°范围内，满足相邻约束）
        3. 计算每个候选的下一个位置（由运动学约束决定）：
           next_x = current_x + step_size * cos(candidate_yaw)
           next_y = current_y + step_size * sin(candidate_yaw)
        4. 评估代价：distance_to_goal（只考虑位置，不考虑yaw）
        5. 选择最优候选
        6. 重复直到到达目标位置

        参数：
        - start_x, start_y, start_yaw: 起点位置和姿态
        - goal_x, goal_y: 目标位置（不包含目标yaw）

        返回：
        - waypoints: [(x, y, yaw), ...] 或 None（搜索失败）
        """
        # 搜索参数
        MAX_ANGLE_CHANGE = 0.08  # 6° = 0.105 rad
        POSITION_TOLERANCE = 0.08  # 位置容差8cm
        MAX_ITERATIONS = 200  # 最大迭代次数

        print(f"      🔍 开始搜索曲线路径...")

        waypoints = [(start_x, start_y, start_yaw)]
        current_x, current_y, current_yaw = start_x, start_y, start_yaw

        # 记录最小距离，用于检测是否在绕圈
        min_dist_seen = float('inf')
        iterations_since_improvement = 0

        # 主搜索循环
        for iteration in range(MAX_ITERATIONS):
            # 检查是否到达目标位置
            dist_to_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)

            if dist_to_goal < POSITION_TOLERANCE:
                # 到达目标位置！如果有误差，添加最终点
                if dist_to_goal > 0.01:
                    waypoints.append((goal_x, goal_y, current_yaw))
                print(f"      ✅ 搜索成功！迭代次数: {iteration + 1}")
                print(f"         最终位置误差: {dist_to_goal*1000:.1f}mm")
                return waypoints

            # 检测是否在改进（避免绕圈）
            if dist_to_goal < min_dist_seen - 0.01:  # 有明显改进（>1cm）
                min_dist_seen = dist_to_goal
                iterations_since_improvement = 0
            else:
                iterations_since_improvement += 1

            # 如果连续50步没有改进，可能陷入绕圈，提前退出
            if iterations_since_improvement > 50:
                print(f"      ⚠️  连续{iterations_since_improvement}步未改进，可能陷入局部最优")
                print(f"         最小距离: {min_dist_seen:.3f}m, 当前距离: {dist_to_goal:.3f}m")
                # 如果曾经接近过目标（<2倍容差），返回路径
                if min_dist_seen < POSITION_TOLERANCE * 2:
                    print(f"         曾接近目标，返回当前路径")
                    waypoints.append((goal_x, goal_y, current_yaw))
                    return waypoints
                else:
                    return None

            # 生成候选角度 - 灵活策略
            # 核心思想：优先尝试指向目标的角度，同时在允许范围内生成多个备选
            angle_to_goal_pos = math.atan2(goal_y - current_y, goal_x - current_x)
            ideal_angle_change = self._normalize_angle(angle_to_goal_pos - current_yaw)

            candidates = []

            # 1. 理想候选：尽量指向目标
            if abs(ideal_angle_change) <= MAX_ANGLE_CHANGE:
                # 理想角度在范围内，直接使用
                candidates.append(angle_to_goal_pos)
            else:
                # 超出范围，使用朝向目标方向的边界角度
                candidates.append(current_yaw + math.copysign(MAX_ANGLE_CHANGE, ideal_angle_change))

            # 2. 备选候选：在允许范围内均匀分布（提供多样性）
            # 使用更密集的采样：-100%, -75%, -50%, -25%, 0%, +25%, +50%, +75%, +100%
            for ratio in [-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0]:
                candidate = current_yaw + ratio * MAX_ANGLE_CHANGE
                # 去重：如果与已有候选接近（<1°），跳过
                is_duplicate = any(abs(self._normalize_angle(candidate - existing)) < 0.017
                                  for existing in candidates)
                if not is_duplicate:
                    candidates.append(candidate)

            # 评估每个候选
            best_candidate = None
            best_cost = float('inf')

            for candidate_yaw in candidates:
                # 检查相邻角度约束
                angle_change = abs(self._normalize_angle(candidate_yaw - current_yaw))
                if angle_change > MAX_ANGLE_CHANGE + 0.001:  # 允许小量数值误差
                    continue

                # 计算下一个位置（由运动学约束决定）
                next_x = current_x + self.step_size * math.cos(candidate_yaw)
                next_y = current_y + self.step_size * math.sin(candidate_yaw)

                # 计算到目标的距离
                dist_to_goal_next = math.sqrt((goal_x - next_x)**2 + (goal_y - next_y)**2)

                # 改进的启发式代价：考虑方向对齐度
                # 计算下一步的朝向与"指向目标"的夹角
                if dist_to_goal_next > 0.01:  # 避免除零
                    angle_to_goal_from_next = math.atan2(goal_y - next_y, goal_x - next_x)
                    direction_alignment = abs(self._normalize_angle(angle_to_goal_from_next - candidate_yaw))
                else:
                    direction_alignment = 0.0

                # 组合代价：距离 + 方向偏差惩罚
                # 当距离较远时，主要考虑距离；接近目标时，增加方向对齐的权重
                if dist_to_goal > 0.5:
                    # 远离目标：主要看距离
                    cost = dist_to_goal_next + 0.1 * direction_alignment
                else:
                    # 接近目标：增加方向对齐的重要性，避免"冲过头"
                    cost = dist_to_goal_next + 0.5 * direction_alignment

                if cost < best_cost:
                    best_cost = cost
                    best_candidate = (next_x, next_y, candidate_yaw)

            if best_candidate is None:
                print(f"      ❌ 搜索失败：第{iteration + 1}步无法找到满足约束的候选")
                return None

            # 更新当前位置
            current_x, current_y, current_yaw = best_candidate
            waypoints.append(best_candidate)

            # 每10步打印进度
            if (iteration + 1) % 10 == 0:
                print(f"         迭代 {iteration + 1}: 距离目标 {dist_to_goal:.3f}m")

        # 达到最大迭代次数
        print(f"      ⚠️  达到最大迭代次数 {MAX_ITERATIONS}，搜索终止")
        print(f"         当前位置误差: {dist_to_goal:.3f}m")

        # 如果接近目标（容差放宽2倍），仍返回结果
        if dist_to_goal < POSITION_TOLERANCE * 2:
            print(f"         接近目标，返回当前路径")
            waypoints.append((goal_x, goal_y, current_yaw))
            return waypoints
        else:
            return None

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

        # 阶段1: 第一次左转（原地旋转）
        yaw_after_first_turn = start_yaw + first_turn_angle

        # 只有转弯角度大于阈值时才添加转弯点
        if abs(first_turn_angle) > 0.01:  # 约0.57度
            print(f"   阶段1: 原地左转 {math.degrees(first_turn_angle):.1f}°")
            waypoints.append((current_x, current_y, current_yaw))  # 起点
            waypoints.append((current_x, current_y, yaw_after_first_turn))  # 转弯后
        else:
            # 角度太小，只添加起点
            print(f"   阶段1: 转弯角度很小({math.degrees(first_turn_angle):.1f}°)，只添加起点")
            waypoints.append((current_x, current_y, yaw_after_first_turn))  # 起点（使用转弯后的yaw）

        current_yaw = yaw_after_first_turn

        # 阶段2: 前进
        # 策略：最后一个点必须是精确目标点，如果最后一段 < 0.05m 则合并到目标点
        num_steps = int(forward_distance / self.forward_step)
        remaining_distance = forward_distance - num_steps * self.forward_step

        # 生成中间点（按forward_step间隔）
        for i in range(1, num_steps + 1):
            dist = i * self.forward_step
            x = current_x + dist * math.cos(current_yaw)
            y = current_y + dist * math.sin(current_yaw)
            waypoints.append((x, y, current_yaw))

        # 计算目标位置
        target_x = current_x + forward_distance * math.cos(current_yaw)
        target_y = current_y + forward_distance * math.sin(current_yaw)

        # 处理最后一段距离
        if remaining_distance > 0.001:  # 有剩余距离
            if remaining_distance < 0.05:  # 剩余距离太小，去掉最后一个中间点
                if len(waypoints) > 2:  # 确保有点可以去掉（前面有起点和转弯点）
                    waypoints.pop()  # 去掉倒数第二个点
                    print(f"   阶段2: 前进 {forward_distance:.3f}m (点间距{self.forward_step}m, {num_steps-1}个点 + 终点, 最后{remaining_distance:.3f}m < 0.05m已合并)")
                else:
                    print(f"   阶段2: 前进 {forward_distance:.3f}m (点间距{self.forward_step}m, 终点)")
            else:
                print(f"   阶段2: 前进 {forward_distance:.3f}m (点间距{self.forward_step}m, {num_steps}个点 + 终点)")
            # 添加精确的目标点
            waypoints.append((target_x, target_y, current_yaw))
        else:
            print(f"   阶段2: 前进 {forward_distance:.3f}m (点间距{self.forward_step}m, {num_steps}个点)")

        # 更新当前位置到前进终点
        current_x = target_x
        current_y = target_y

        # 阶段3: 第二次左转（原地旋转，只在有转弯时添加）
        yaw_after_second_turn = current_yaw + second_turn_angle

        # 只有转弯角度大于阈值时才添加新点
        if abs(second_turn_angle) > 0.01:  # 约0.57度
            print(f"   阶段3: 原地左转 {math.degrees(second_turn_angle):.1f}°")
            waypoints.append((current_x, current_y, yaw_after_second_turn))
        else:
            # 角度太小，不添加新点，但更新最后一个点的yaw
            if waypoints:
                last_x, last_y, _ = waypoints[-1]
                waypoints[-1] = (last_x, last_y, yaw_after_second_turn)
            print(f"   阶段3: 转弯角度很小({math.degrees(second_turn_angle):.1f}°)，合并到前一点")

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
        # 策略：最后一个点必须是精确目标点，如果最后一段 < 0.05m 则合并到目标点
        abs_distance = abs(backward_distance)
        num_steps = int(abs_distance / self.backward_step)
        remaining_distance = abs_distance - num_steps * self.backward_step

        # 生成中间点（按backward_step间隔）
        for i in range(num_steps):
            dist = (i + 1) * self.backward_step

            # 沿y轴方向移动：如果backward_distance为负，则向-y移动
            if backward_distance >= 0:
                y = start_y + dist
            else:
                y = start_y - dist

            waypoints.append((start_x, y, start_yaw))  # x保持不变，yaw保持不变

        # 计算目标位置
        end_x = start_x
        end_y = start_y + backward_distance

        # 处理最后一段距离
        if remaining_distance > 0.001:  # 有剩余距离
            if remaining_distance < 0.05:  # 剩余距离太小，去掉最后一个中间点
                if len(waypoints) > 0:  # 确保有点可以去掉
                    waypoints.pop()  # 去掉倒数第二个点
                    print(f"   生成路径点: {num_steps-1}个 (点间距{self.backward_step}m) + 终点, 最后{remaining_distance:.3f}m < 0.05m已合并")
                else:
                    print(f"   生成路径点: 终点")
            else:
                print(f"   生成路径点: {num_steps}个 (点间距{self.backward_step}m) + 终点")
            # 添加精确的目标点
            waypoints.append((end_x, end_y, start_yaw))
        else:
            print(f"   生成路径点: {num_steps}个 (点间距{self.backward_step}m)")

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
