#!/usr/bin/env python3
# test_map_publisher.py
#
# 测试ROS2-VDA5050桥接器的地图订阅功能
# 模拟发布地图数据到/map话题，验证桥接器是否正确接收和处理
#
# ⚠️  仅用于测试目的！
# 在生产环境中，真实的SLAM或地图服务器会自动发布/map话题

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import time

# --- Configuration ---
PUBLISH_TOPIC = '/map'
PUBLISH_RATE_HZ = 0.5  # 每2秒发布一次（地图更新不需要太频繁）
MAP_FRAME_ID = 'map'

# --- Map Data Configuration ---
MAP_WIDTH = 20        # 地图宽度 (格子数)
MAP_HEIGHT = 20       # 地图高度 (格子数)
MAP_RESOLUTION = 0.1  # 每个格子的实际尺寸 (米)
MAP_ORIGIN_X = -1.0   # 地图原点X坐标
MAP_ORIGIN_Y = -1.0   # 地图原点Y坐标

class MapPublisher(Node):
    def __init__(self):
        super().__init__('test_map_publisher')

        # 创建发布器
        self.publisher = self.create_publisher(OccupancyGrid, PUBLISH_TOPIC, 10)

        # 创建定时器
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_map)

        # 计数器
        self.counter = 0

        self.get_logger().info(f'🗺️  地图发布器已启动')
        self.get_logger().info(f'   发布话题: {PUBLISH_TOPIC}')
        self.get_logger().info(f'   发布频率: {PUBLISH_RATE_HZ} Hz')
        self.get_logger().info(f'   地图尺寸: {MAP_WIDTH}x{MAP_HEIGHT}')
        self.get_logger().info(f'   地图分辨率: {MAP_RESOLUTION}m/格子')
        self.get_logger().info('⚠️  这是测试工具，生产环境中应使用真实的SLAM/地图服务器')

    def create_test_map(self, scenario=0):
        """创建测试地图数据"""
        # 初始化地图 (-1=未知, 0=自由空间, 100=障碍物)
        map_data = np.full((MAP_HEIGHT, MAP_WIDTH), -1, dtype=np.int8)

        if scenario == 0:
            # 场景0: 简单的走廊地图
            # 创建一个中央走廊
            map_data[8:12, 2:18] = 0  # 走廊
            map_data[6:14, 2] = 100   # 左墙
            map_data[6:14, 17] = 100  # 右墙
            map_data[6, 2:18] = 100   # 上墙
            map_data[13, 2:18] = 100  # 下墙

        elif scenario == 1:
            # 场景1: 房间地图
            # 创建几个房间
            map_data[5:15, 5:15] = 0   # 主房间
            map_data[5:15, 5] = 100    # 左墙
            map_data[5:15, 14] = 100   # 右墙
            map_data[5, 5:15] = 100    # 上墙
            map_data[14, 5:15] = 100   # 下墙
            map_data[9:11, 14] = 0     # 门

        elif scenario == 2:
            # 场景2: 迷宫地图
            map_data[2:18, 2:18] = 0   # 全部自由
            # 添加一些障碍物
            map_data[5:8, 7:10] = 100
            map_data[12:15, 5:8] = 100
            map_data[8:11, 12:15] = 100

        # 转换为一维数组 (ROS要求)
        map_data_flat = map_data.flatten().tolist()

        return map_data_flat

    def publish_map(self):
        """发布地图消息"""
        msg = OccupancyGrid()

        # 设置消息头
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = MAP_FRAME_ID

        # 设置地图元数据
        msg.info.map_load_time = self.get_clock().now().to_msg()
        msg.info.resolution = MAP_RESOLUTION
        msg.info.width = MAP_WIDTH
        msg.info.height = MAP_HEIGHT

        # 设置地图原点
        msg.info.origin.position.x = MAP_ORIGIN_X
        msg.info.origin.position.y = MAP_ORIGIN_Y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # 根据计数器选择不同场景
        scenario = self.counter % 3
        msg.data = self.create_test_map(scenario)

        # 发布消息
        self.publisher.publish(msg)

        # 日志输出
        scenario_names = ["走廊地图", "房间地图", "迷宫地图"]
        self.get_logger().info(f'📤 发布地图 #{self.counter} - {scenario_names[scenario]}')
        self.get_logger().info(f'   尺寸: {MAP_WIDTH}x{MAP_HEIGHT}, 分辨率: {MAP_RESOLUTION}m')
        self.get_logger().info(f'   原点: ({MAP_ORIGIN_X}, {MAP_ORIGIN_Y})')

        self.counter += 1

def main():
    print("🗺️  启动地图发布器测试")
    print("=" * 50)

    rclpy.init()

    try:
        node = MapPublisher()

        print("✅ 地图发布器已启动，等待桥接器连接...")
        print("🔍 请在另一个终端启动桥接器来观察地图接收情况")
        print("⏹️  按 Ctrl+C 停止")

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n🛑 用户停止地图发布器")
    except Exception as e:
        print(f"❌ 地图发布器错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ 地图发布器已关闭")

if __name__ == '__main__':
    main()