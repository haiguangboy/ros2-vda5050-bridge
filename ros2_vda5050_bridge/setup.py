from setuptools import setup

package_name = 'ros2_vda5050_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bridge_launch.py']),
        ('share/' + package_name + '/config', ['config/bridge_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Bridge between ROS2 navigation and VDA5050 protocol',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vda5050_bridge = ros2_vda5050_bridge.vda5050_bridge:main',
            'test_mqtt_client = ros2_vda5050_bridge.test_mqtt_client:main',
            'path_publisher = ros2_vda5050_bridge.path_publisher:main',
            'map_publisher = ros2_vda5050_bridge.map_publisher:main',
        ],
    },
)