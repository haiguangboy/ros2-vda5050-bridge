#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ros2_vda5050_bridge')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'bridge_config.yaml')
    
    # Declare launch arguments
    mqtt_host_arg = DeclareLaunchArgument(
        'mqtt_host',
        default_value='localhost',
        description='MQTT broker hostname'
    )
    
    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )
    
    manufacturer_arg = DeclareLaunchArgument(
        'manufacturer',
        default_value='ROS2Manufacturer',
        description='AGV manufacturer name'
    )
    
    serial_number_arg = DeclareLaunchArgument(
        'serial_number',
        default_value='ROS2_AGV_001',
        description='AGV serial number'
    )
    
    # VDA5050 Bridge Node
    bridge_node = Node(
        package='ros2_vda5050_bridge',
        executable='vda5050_bridge',
        name='vda5050_bridge',
        parameters=[
            config_file,
            {
                'mqtt_broker_host': LaunchConfiguration('mqtt_host'),
                'mqtt_broker_port': LaunchConfiguration('mqtt_port'),
                'manufacturer': LaunchConfiguration('manufacturer'),
                'serial_number': LaunchConfiguration('serial_number'),
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        mqtt_host_arg,
        mqtt_port_arg,
        manufacturer_arg,
        serial_number_arg,
        bridge_node
    ])