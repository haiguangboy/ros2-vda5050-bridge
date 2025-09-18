#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('ros2_zhongli_bridge_cpp')
    config_file = os.path.join(pkg_path, 'config', 'bridge_config.yaml')

    # 声明启动参数
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot-001',
        description='Robot ID for MQTT topics'
    )

    mqtt_host_arg = DeclareLaunchArgument(
        'mqtt_host',
        default_value='localhost',
        description='MQTT broker host address'
    )

    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )

    # 桥接器节点
    bridge_node = Node(
        package='ros2_zhongli_bridge_cpp',
        executable='zhongli_bridge_node',
        name='zhongli_bridge',
        parameters=[
            config_file,
            {
                'robot_id': LaunchConfiguration('robot_id'),
                'mqtt_broker_host': LaunchConfiguration('mqtt_host'),
                'mqtt_broker_port': LaunchConfiguration('mqtt_port'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_id_arg,
        mqtt_host_arg,
        mqtt_port_arg,
        bridge_node,
    ])