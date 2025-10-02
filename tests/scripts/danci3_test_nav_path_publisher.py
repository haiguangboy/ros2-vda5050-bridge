# test_nav_path_publisher.py
#
# ⚠️  仅用于测试目的！
# 在生产环境中，真实的Nav2会自动发布/plan话题，无需此模拟器
# This is ONLY for testing purposes!
# In production, real Nav2 will publish /plan topic automatically

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import math # 仅用于可能需要计算角度的场景，如果你直接用四元数则无需

# --- Configuration ---
PUBLISH_TOPIC = '/plans' # Nav2 通常发布到 /plan, 但也可以是其他自定义话题
ODOM_TOPIC = '/Odom'   # 里程计话题，获取当前位置
PATH_FRAME_ID = 'map' # 路径的参考系，通常是 'map'

# 阶段开关
ENABLE_STAGE1 = True    # 是否生成直线路径（阶段1）
ENABLE_STAGE2 = False #True    # 是否生成掉头路径（阶段2）
ENABLE_STAGE3 = False    # 是否生成曲线路径（阶段3）

# 阶段1参数：直线路径
NUM_PATH_POINTS = 2   # 生成的路径点数量
FORWARD_DISTANCE = 0.15  # 沿着朝向前进的距离（米）

# 阶段2参数：原地掉头
TURN_ANGLE_RAD = math.pi    # 掉头角度（弧度制），π表示180度掉头
TURN_STEPS = 2             # 掉头分几步完成，步数越多转向越平滑

# 阶段3参数：曲线路径
CURVE_ANGLE_RAD = math.pi / 3    # 曲线转弯角度（弧度制），π/4表示45度转弯
CURVE_POINT_DISTANCE = 0.15      # 曲线路径点间距（米），15cm
CURVE_RADIUS = 2.0               # 曲线转弯半径（米）

# 车辆参数（三轮叉车）
WHEELBASE = 1.47              # 轴距 1470mm
MIN_TURN_RADIUS = 1.743       # 最小转弯半径 1743mm
VEHICLE_CENTER_OFFSET = WHEELBASE / 2  # 车辆中心到后轴距离

# --- End Configuration ---

class SimpleNavPathPublisher(Node):

    def __init__(self):
        super().__init__('simple_nav_path_publisher_node')

        # 创建发布器和订阅器
        self.publisher_ = self.create_publisher(Path, PUBLISH_TOPIC, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            10
        )

        # 标志变量，确保只发布一次
        self.path_published = False
        self.current_pose = None

        self.get_logger().info(f'Path Publisher started. Waiting for odom data from "{ODOM_TOPIC}"...')

    def odom_callback(self, msg):
        # 获取当前位置，如果还没有发布过路径，则发布一次
        if not self.path_published:
            self.current_pose = msg.pose.pose
            self.generate_and_publish_path()
            self.path_published = True
            self.get_logger().info('Path published once. Node will continue running for visualization.')

    def quaternion_to_yaw(self, q):
        """将四元数转换为yaw角度（弧度）"""
        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def yaw_to_quaternion(self, yaw):
        """将yaw角度（弧度）转换为四元数"""
        # 仅绕z轴旋转，所以roll=0, pitch=0
        roll = 0.0
        pitch = 0.0

        # 计算四元数
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

    def generate_and_publish_path(self):
        if self.current_pose is None:
            self.get_logger().warn('No odom data received yet.')
            return

        # 创建路径消息
        path_msg = Path()
        path_msg.header.frame_id = PATH_FRAME_ID
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # 获取当前位置作为基准
        base_x = self.current_pose.position.x
        base_y = self.current_pose.position.y
        base_z = self.current_pose.position.z
        base_orientation = self.current_pose.orientation

        # 计算基准yaw角
        base_yaw = self.quaternion_to_yaw(base_orientation)

        # 存储路径点信息用于打印
        path_points = []

        # 阶段1：生成直线路径点
        if ENABLE_STAGE1:
            stage1_points = self.generate_stage1_points(base_x, base_y, base_z, base_yaw, path_msg.header.stamp)
            path_msg.poses.extend(stage1_points)
            path_points.extend(self.extract_point_info(stage1_points, 'stage1'))

        # 阶段2：生成原地掉头路径点
        if ENABLE_STAGE2:
            # 确定掉头起点（如果阶段1开启，则从最后一点开始；否则从当前位置开始）
            if ENABLE_STAGE1 and path_msg.poses:
                last_pose = path_msg.poses[-1]
                turn_start_x = last_pose.pose.position.x
                turn_start_y = last_pose.pose.position.y
                turn_start_z = last_pose.pose.position.z
                turn_start_yaw = base_yaw  # 直线阶段保持同一朝向
            else:
                turn_start_x = base_x
                turn_start_y = base_y
                turn_start_z = base_z
                turn_start_yaw = base_yaw

            stage2_points = self.generate_stage2_points(turn_start_x, turn_start_y, turn_start_z, turn_start_yaw, path_msg.header.stamp)

            # 如果阶段1已启用，跳过阶段2的第一个点（避免重复）
            if ENABLE_STAGE1 and stage2_points:
                stage2_points = stage2_points[1:]  # 跳过第一个点

            path_msg.poses.extend(stage2_points)
            path_points.extend(self.extract_point_info(stage2_points, 'stage2'))

        # 阶段3：生成曲线路径点
        if ENABLE_STAGE3:
            # 确定曲线起点（按优先级：阶段2终点 > 阶段1终点 > 当前位置）
            if path_msg.poses:
                last_pose = path_msg.poses[-1]
                curve_start_x = last_pose.pose.position.x
                curve_start_y = last_pose.pose.position.y
                curve_start_z = last_pose.pose.position.z
                # 获取前一阶段的终点朝向
                curve_start_yaw = self.quaternion_to_yaw(last_pose.pose.orientation)
            else:
                curve_start_x = base_x
                curve_start_y = base_y
                curve_start_z = base_z
                curve_start_yaw = base_yaw

            stage3_points = self.generate_stage3_points(curve_start_x, curve_start_y, curve_start_z, curve_start_yaw, path_msg.header.stamp)

            # 如果前面有阶段，跳过阶段3的第一个点（避免重复）
            if path_msg.poses and stage3_points:
                stage3_points = stage3_points[1:]  # 跳过第一个点

            path_msg.poses.extend(stage3_points)
            path_points.extend(self.extract_point_info(stage3_points, 'stage3'))

        # 发布路径
        self.publisher_.publish(path_msg)

        # 打印详细的路径信息
        self.print_path_information(path_points, base_yaw, len(path_msg.poses))

    def generate_stage1_points(self, base_x, base_y, base_z, base_yaw, stamp):
        """生成阶段1直线路径点"""
        stage1_points = []

        for i in range(NUM_PATH_POINTS):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = PATH_FRAME_ID
            pose_stamped.header.stamp = stamp

            # 计算沿着当前朝向前进的距离
            forward_dist = i * FORWARD_DISTANCE

            # 根据yaw角计算新的位置
            current_x = base_x + forward_dist * math.cos(base_yaw)
            current_y = base_y + forward_dist * math.sin(base_yaw)

            pose_stamped.pose.position = Point(x=current_x, y=current_y, z=base_z)
            pose_stamped.pose.orientation = self.yaw_to_quaternion(base_yaw)

            stage1_points.append(pose_stamped)

        return stage1_points

    def generate_stage2_points(self, start_x, start_y, start_z, start_yaw, stamp):
        """生成阶段2原地掉头路径点（原地旋转，位置不变）"""
        stage2_points = []

        # 生成原地旋转路径点 - 位置保持不变，只改变朝向
        for i in range(0, TURN_STEPS + 1):  # 从0开始，包含起始位置
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = PATH_FRAME_ID
            pose_stamped.header.stamp = stamp

            # 计算当前步骤的yaw角（逐步旋转）
            angle_offset = i * (TURN_ANGLE_RAD / TURN_STEPS)
            current_yaw = start_yaw + angle_offset

            # 原地旋转：x,y位置保持不变，只改变朝向
            pose_stamped.pose.position = Point(x=start_x, y=start_y, z=start_z)
            pose_stamped.pose.orientation = self.yaw_to_quaternion(current_yaw)

            stage2_points.append(pose_stamped)

        return stage2_points

    def generate_stage3_points(self, start_x, start_y, start_z, start_yaw, stamp):
        """生成阶段3曲线路径点"""
        stage3_points = []

        # 计算弧线总长度
        arc_length = CURVE_ANGLE_RAD * CURVE_RADIUS

        # 根据点间距计算需要多少个点
        num_points = int(arc_length / CURVE_POINT_DISTANCE) + 1

        # 计算转弯中心位置（假设向左转弯）
        # 转弯中心位于车辆当前位置的左侧，距离为转弯半径
        center_x = start_x - CURVE_RADIUS * math.sin(start_yaw)
        center_y = start_y + CURVE_RADIUS * math.cos(start_yaw)

        # 计算起始角度（从转弯中心看向起始点的角度）
        start_angle = math.atan2(start_y - center_y, start_x - center_x)

        # 生成曲线路径点 - 包含起始位置作为第一个点
        for i in range(0, num_points + 1):  # 从0开始，包含起始位置
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = PATH_FRAME_ID
            pose_stamped.header.stamp = stamp

            # 计算当前点在弧线上的位置
            distance_along_arc = i * CURVE_POINT_DISTANCE
            angle_offset = distance_along_arc / CURVE_RADIUS

            # 限制角度偏移不超过总转弯角度
            if angle_offset > CURVE_ANGLE_RAD:
                angle_offset = CURVE_ANGLE_RAD

            # 计算当前点的位置
            current_angle = start_angle + angle_offset
            current_x = center_x + CURVE_RADIUS * math.cos(current_angle)
            current_y = center_y + CURVE_RADIUS * math.sin(current_angle)

            # 计算当前点的朝向（切线方向）
            current_yaw = start_yaw + angle_offset

            pose_stamped.pose.position = Point(x=current_x, y=current_y, z=start_z)
            pose_stamped.pose.orientation = self.yaw_to_quaternion(current_yaw)

            stage3_points.append(pose_stamped)

            # 如果已经完成指定角度的转弯，停止生成
            if angle_offset >= CURVE_ANGLE_RAD:
                break

        return stage3_points

    def extract_point_info(self, poses, stage_name):
        """提取路径点信息用于打印"""
        point_info = []
        for i, pose in enumerate(poses):
            yaw = self.quaternion_to_yaw(pose.pose.orientation)
            point_info.append({
                'stage': stage_name,
                'index': i,
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z,
                'yaw': yaw,
                'quaternion': {
                    'x': pose.pose.orientation.x,
                    'y': pose.pose.orientation.y,
                    'z': pose.pose.orientation.z,
                    'w': pose.pose.orientation.w
                }
            })
        return point_info

    def print_path_information(self, path_points, base_yaw, total_poses):
        """打印路径信息"""
        print("=" * 80)
        print("THREE-STAGE PATH INFORMATION")
        print("=" * 80)

        # 打印阶段状态
        print(f"阶段状态 (Stage Status):")
        print(f"  阶段1 (直线): {'ENABLED' if ENABLE_STAGE1 else 'DISABLED'}")
        print(f"  阶段2 (掉头): {'ENABLED' if ENABLE_STAGE2 else 'DISABLED'}")
        print(f"  阶段3 (曲线): {'ENABLED' if ENABLE_STAGE3 else 'DISABLED'}")
        print(f"  总路径点数: {total_poses}")

        if not path_points:
            print("\n无路径点生成")
            return

        # 按阶段分组打印
        stage1_points = [p for p in path_points if p.get('stage') == 'stage1']
        stage2_points = [p for p in path_points if p.get('stage') == 'stage2']
        stage3_points = [p for p in path_points if p.get('stage') == 'stage3']

        # 打印阶段1信息
        if stage1_points:
            print(f"\n=== 阶段1: 直线路径 ({len(stage1_points)} 点) ===")
            start_point = stage1_points[0]
            end_point = stage1_points[-1]

            print(f"起点: x={start_point['x']:.3f}, y={start_point['y']:.3f}, yaw={math.degrees(start_point['yaw']):.1f}°")
            if len(stage1_points) > 1:
                print(f"终点: x={end_point['x']:.3f}, y={end_point['y']:.3f}, yaw={math.degrees(end_point['yaw']):.1f}°")
                distance = math.sqrt((end_point['x'] - start_point['x'])**2 + (end_point['y'] - start_point['y'])**2)
                print(f"移动距离: {distance:.3f} m")

        # 打印阶段2信息
        if stage2_points:
            print(f"\n=== 阶段2: 原地掉头 ({len(stage2_points)} 点) ===")
            start_point = stage2_points[0]
            end_point = stage2_points[-1]

            print(f"掉头起点: x={start_point['x']:.3f}, y={start_point['y']:.3f}, yaw={math.degrees(start_point['yaw']):.1f}°")
            print(f"掉头终点: x={end_point['x']:.3f}, y={end_point['y']:.3f}, yaw={math.degrees(end_point['yaw']):.1f}°")

            angle_change = end_point['yaw'] - start_point['yaw']
            # 处理角度跨越2π的情况
            if angle_change > math.pi:
                angle_change -= 2 * math.pi
            elif angle_change < -math.pi:
                angle_change += 2 * math.pi

            print(f"角度变化: {math.degrees(angle_change):.1f}°")

            # 原地旋转：位置不变，无移动距离
            print(f"移动距离: 0.000 m (原地旋转)")

        # 打印阶段3信息
        if stage3_points:
            print(f"\n=== 阶段3: 曲线路径 ({len(stage3_points)} 点) ===")
            start_point = stage3_points[0]
            end_point = stage3_points[-1]

            print(f"曲线起点: x={start_point['x']:.3f}, y={start_point['y']:.3f}, yaw={math.degrees(start_point['yaw']):.1f}°")
            print(f"曲线终点: x={end_point['x']:.3f}, y={end_point['y']:.3f}, yaw={math.degrees(end_point['yaw']):.1f}°")

            angle_change = end_point['yaw'] - start_point['yaw']
            # 处理角度跨越2π的情况
            if angle_change > math.pi:
                angle_change -= 2 * math.pi
            elif angle_change < -math.pi:
                angle_change += 2 * math.pi

            print(f"角度变化: {math.degrees(angle_change):.1f}°")
            print(f"转弯半径: {CURVE_RADIUS:.3f} m")
            print(f"点间距: {CURVE_POINT_DISTANCE:.3f} m")

            # 计算实际弧长
            curve_arc_length = abs(angle_change) * CURVE_RADIUS
            print(f"弧长: {curve_arc_length:.3f} m")

        # 打印车辆参数和配置
        print(f"\n=== 车辆参数 ===")
        print(f"轴距: {WHEELBASE:.3f} m")
        print(f"车辆中心到后轴距离: {VEHICLE_CENTER_OFFSET:.3f} m")
        print(f"最小转弯半径: {MIN_TURN_RADIUS:.3f} m")

        print(f"\n=== 路径配置 ===")
        print(f"阶段1 - 直线距离: {FORWARD_DISTANCE:.1f} m, 点数: {NUM_PATH_POINTS}")
        print(f"阶段2 - 掉头角度: {math.degrees(TURN_ANGLE_RAD):.1f}°, 步数: {TURN_STEPS}")
        print(f"阶段3 - 转弯角度: {math.degrees(CURVE_ANGLE_RAD):.1f}°, 转弯半径: {CURVE_RADIUS:.1f} m, 点间距: {CURVE_POINT_DISTANCE:.2f} m")

        # 打印所有路径点详细信息
        print(f"\n=== 所有路径点详细信息 ===")
        print("-" * 80)
        for point in path_points:
            stage_map = {'stage1': '直线', 'stage2': '掉头', 'stage3': '曲线'}
            stage_label = stage_map.get(point.get('stage'), '未知')
            print(f"[{stage_label}] Point {point['index']}: x={point['x']:.3f}, y={point['y']:.3f}, "
                  f"yaw={point['yaw']:.3f} rad ({math.degrees(point['yaw']):.1f}°)")

        print("=" * 80)

        self.get_logger().info(f'Published three-stage path with {total_poses} poses. '
                              f'Stage1: {"ON" if ENABLE_STAGE1 else "OFF"}, '
                              f'Stage2: {"ON" if ENABLE_STAGE2 else "OFF"}, '
                              f'Stage3: {"ON" if ENABLE_STAGE3 else "OFF"}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # 允许Ctrl+C退出
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
