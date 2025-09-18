#!/bin/bash
# launch_with_config.sh
# 使用配置文件启动桥接器的脚本

set -e

echo "🚀 启动中力桥接器（带配置文件）"
echo "=================================="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2环境未设置，尝试加载ROS2 Humble..."
    source /opt/ros/humble/setup.bash || {
        echo "❌ 无法加载ROS2环境，请确保已安装ROS2 Humble"
        exit 1
    }
fi

echo "✅ ROS2环境: $ROS_DISTRO"

# 检查配置文件是否存在
CONFIG_FILE="config/bridge_config.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

echo "📋 配置文件: $CONFIG_FILE"
echo "📋 配置文件内容:"
echo "=================="
cat "$CONFIG_FILE"
echo "=================="
echo ""

# 检查可执行文件是否存在
EXECUTABLE="install/bin/zhongli_bridge_node"
if [ ! -f "$EXECUTABLE" ]; then
    echo "❌ 可执行文件不存在: $EXECUTABLE"
    echo "请先运行编译脚本"
    exit 1
fi

echo "🎯 启动桥接器..."
echo "使用ROS2参数文件启动方式"

# 使用ROS2参数文件启动方式
ros2 run zhongli_bridge_pkg zhongli_bridge_node --ros-args --params-file "$CONFIG_FILE" 2>/dev/null || {
    echo "📝 标准ROS2启动方式失败，尝试直接启动..."
    # 备用方案：直接启动（但参数可能不会加载）
    "$EXECUTABLE" --ros-args --params-file "$CONFIG_FILE"
}