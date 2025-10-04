#!/bin/bash

# MQTT Bridge启动脚本

echo "=================================="
echo "🚀 启动MQTT Bridge"
echo "=================================="

# 检查可执行文件是否存在
BRIDGE_BIN="/home/yhg/Documents/ep-embodied/mqtt_bridge/install/bin/zhongli_bridge_node"
CONFIG_FILE="/home/yhg/Documents/ep-embodied/mqtt_bridge/config/bridge_config.yaml"

if [ ! -f "$BRIDGE_BIN" ]; then
    echo "❌ MQTT Bridge可执行文件不存在: $BRIDGE_BIN"
    echo "   请先编译: cd /home/yhg/Documents/ep-embodied/mqtt_bridge && colcon build"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

echo "✅ 可执行文件: $BRIDGE_BIN"
echo "✅ 配置文件: $CONFIG_FILE"
echo ""

# Source ROS2环境
echo "📦 加载ROS2环境..."
source /opt/ros/humble/setup.bash
source /home/yhg/Documents/ep-embodied/mqtt_bridge/install/setup.bash

# 运行MQTT Bridge
echo "🔌 启动MQTT Bridge..."
echo "   (按 Ctrl+C 停止)"
echo ""

$BRIDGE_BIN --ros-args --params-file $CONFIG_FILE
