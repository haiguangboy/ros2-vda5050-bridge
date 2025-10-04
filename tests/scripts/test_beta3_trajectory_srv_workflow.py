#!/usr/bin/env python3
"""
Beta-3协议：基于服务目标位姿的直线轨迹测试

流程：
- 从 `/Odom` 订阅起点（超时则使用默认值）
- 通过 `/nav/go_to_pose` 调用 `GoToPose` 服务发布目标位姿（MODE_NORMAL）
- 使用起点+终点计算一条直线，每隔 0.15m 采样，生成 `Path` 并发布到 `/plans`

参考：forklift_testbench 服务端与 forklift_fsm 客户端调用方式
"""

import math
import time
import signal
import sys
import os
import json
import argparse
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from std_msgs.msg import Header

# Optional service import; fallback to CLI/file when unavailable
SERVICE_AVAILABLE = True
try:
    from forklift_interfaces.srv import GoToPose  # type: ignore
except Exception:
    GoToPose = None  # type: ignore
    SERVICE_AVAILABLE = False


# =============== 配置 ===============
ODOM_TOPIC = "/Odom"  # 按现有脚本约定使用大写 O
PATH_TOPIC = "/plans"
ODOM_TIMEOUT = 10.0
STEP_M = 0.15

# 默认目标（也可改为命令行或参数）
DEFAULT_TARGET_X = 3.0
DEFAULT_TARGET_Y = 0.0
DEFAULT_TARGET_YAW = 0.0

# 默认起点
DEFAULT_X = 0.0
DEFAULT_Y = 0.0
DEFAULT_YAW = 0.0


class TrajectorySrvTester(Node):
    def __init__(self, args):
        super().__init__("trajectory_srv_tester")
        # ROS2 pub/sub
        self.path_pub = self.create_publisher(Path, PATH_TOPIC, 10)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        # GoToPose client (only if type is available)
        self.cli_nav = None
        if SERVICE_AVAILABLE and GoToPose is not None:
            self.cli_nav = self.create_client(GoToPose, "/nav/go_to_pose")
        else:
            print("⚠️ GoToPose 类型不可用；将使用CLI/文件目标作为替代")

        self.current_pose = None
        self.odom_received = False
        self.running = True
        self.args = args

    # ---------------- helpers ----------------
    def on_odom(self, msg: Odometry):
        if not self.odom_received:
            self.current_pose = msg.pose.pose
            self.odom_received = True
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            yaw = self.q_to_yaw(self.current_pose.orientation)
            print(f"✅ 已接收 /Odom 起点: ({x:.3f}, {y:.3f}), yaw={math.degrees(yaw):.1f}°")

    def wait_for_odom_or_default(self):
        print(f"⏳ 等待 {ODOM_TOPIC}（最多 {ODOM_TIMEOUT:.0f}s）...")
        start = time.time()
        while not self.odom_received and (time.time() - start) < ODOM_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.5)
        if not self.odom_received:
            print("⚠️ Odom超时，使用默认起点")
            p = Pose()
            p.position.x = DEFAULT_X
            p.position.y = DEFAULT_Y
            p.position.z = 0.0
            p.orientation = self.euler_to_q(0.0, 0.0, DEFAULT_YAW)
            self.current_pose = p

    def send_nav_goal(self, x: float, y: float, yaw: float) -> bool:
        if self.cli_nav is None:
            print("ℹ️ 无服务客户端；跳过调用")
            return False
        if not self.cli_nav.wait_for_service(timeout_sec=3.0):
            print("⚠️ GoToPose 服务不可用，继续仅发布路径")
            return False
        req = GoToPose.Request()
        req.mode = GoToPose.Request.MODE_NORMAL
        req.target = PoseStamped()
        req.target.header.stamp = self.get_clock().now().to_msg()
        req.target.header.frame_id = "map"
        req.target.pose.position.x = x
        req.target.pose.position.y = y
        req.target.pose.position.z = 0.0
        req.target.pose.orientation = self.euler_to_q(0.0, 0.0, yaw)
        req.timeout_sec = 30.0

        print(f"🚀 调用 GoToPose: 目标=({x:.3f}, {y:.3f}), yaw={math.degrees(yaw):.1f}°")
        fut = self.cli_nav.async_send_request(req)
        try:
            resp = fut.result(timeout=2.0)
            print(f"📨 GoToPose 响应: arrived={resp.arrived}, message='{resp.message}'")
            return bool(resp.arrived)
        except Exception:
            print("⚠️ GoToPose 响应超时或失败")
            return False

    def build_and_publish_straight_path(self, start: Pose, target_x: float, target_y: float, target_yaw: float):
        sx, sy = start.position.x, start.position.y
        # 直线方向由起点->终点决定，路径点朝向统一使用目标朝向
        dx = target_x - sx
        dy = target_y - sy
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            print("❌ 起点与终点重合，跳过发布")
            return
        steps = max(2, int(math.ceil(dist / STEP_M)) + 1)

        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        # Beta-3 header: orientation=0.0（前向），flag=0（非分支），其他留空
        path.header.frame_id = "map|none|none|0.0|0|0|0|0|0|0"

        poses = []
        for i in range(steps):
            t = i / (steps - 1)
            x = sx + t * dx
            y = sy + t * dy
            poses.append(self.make_pose_stamped(x, y, target_yaw))

        path.poses = poses
        self.path_pub.publish(path)
        print(f"📡 已发布直线路径到 {PATH_TOPIC}，点数={len(poses)}，长度≈{dist:.2f}m")

    # ---------------- utils ----------------
    def make_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation = self.euler_to_q(0.0, 0.0, yaw)
        return ps

    def q_to_yaw(self, q: Quaternion) -> float:
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def euler_to_q(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    # ---------------- lifecycle ----------------
    def stop(self):
        self.running = False
        print("🛑 测试停止")


tester = None


def signal_handler(sig, frame):
    print("\n⏹️ 收到中断信号，正在停止...")
    global tester
    if tester:
        tester.stop()
    rclpy.shutdown()
    sys.exit(0)


def main():
    global tester
    parser = argparse.ArgumentParser(description='Beta-3: 服务目标位姿直线路径测试')
    parser.add_argument('--target-x', type=float, default=DEFAULT_TARGET_X)
    parser.add_argument('--target-y', type=float, default=DEFAULT_TARGET_Y)
    parser.add_argument('--target-yaw', type=float, default=DEFAULT_TARGET_YAW, help='yaw (rad)')
    parser.add_argument('--target-file', type=str, default='', help='JSON file with {"x":..., "y":..., "yaw":...}')
    args = parser.parse_args()

    print("🧪 Beta-3：服务目标位姿直线轨迹测试")
    print("=" * 80)
    print(f"  Odom: {ODOM_TOPIC}, Path: {PATH_TOPIC}, 采样间距: {STEP_M}m")
    print("=" * 80)

    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init()
    tester = TrajectorySrvTester(args)

    # 1) 等待起点
    tester.wait_for_odom_or_default()

    # 2) 解析目标（优先文件，其次CLI，若服务可用则也调用）
    target_x = args.target_x
    target_y = args.target_y
    target_yaw = args.target_yaw
    if args.target_file and os.path.exists(args.target_file):
        try:
            with open(args.target_file, 'r') as f:
                data = json.load(f)
            target_x = float(data.get('x', target_x))
            target_y = float(data.get('y', target_y))
            target_yaw = float(data.get('yaw', target_yaw))
            print(f"📄 使用文件目标: ({target_x:.3f}, {target_y:.3f}), yaw={math.degrees(target_yaw):.1f}°")
        except Exception as e:
            print(f"⚠️ 目标文件解析失败: {e}，使用CLI参数")

    tester.send_nav_goal(target_x, target_y, target_yaw)

    # 3) 计算直线路径并发布
    tester.build_and_publish_straight_path(tester.current_pose, target_x, target_y, target_yaw)

    print("\n✅ 已完成路径发布，保持节点运行以便调试（Ctrl+C 退出）\n")
    while tester.running and rclpy.ok():
        rclpy.spin_once(tester, timeout_sec=0.5)


if __name__ == "__main__":
    main()
