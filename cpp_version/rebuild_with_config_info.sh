#!/bin/bash
# rebuild_with_config_info.sh
# 重新编译带有配置信息打印的版本

set -e

echo "🔄 重新编译带有详细配置信息的桥接器"
echo "===================================="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2环境未设置，尝试加载ROS2 Humble..."
    source /opt/ros/humble/setup.bash || {
        echo "❌ 无法加载ROS2环境，请确保已安装ROS2 Humble"
        exit 1
    }
fi

echo "✅ ROS2环境: $ROS_DISTRO"

# 清理之前的编译
echo "🧹 清理之前的编译..."
rm -rf build

# 重新编译
echo "🔨 重新编译项目..."
mkdir build && cd build
cmake ..
make -j$(nproc)
make install

if [ $? -eq 0 ]; then
    echo "✅ 编译成功！"
    echo ""
    echo "🎯 现在运行桥接器会显示详细的配置信息："
    echo "   - 机器人ID"
    echo "   - MQTT Broker地址和端口"
    echo "   - 所有MQTT主题"
    echo "   - 其他配置参数"
    echo ""
    echo "🚀 运行桥接器:"
    echo "   cd .."
    echo "   ./install/bin/zhongli_bridge_node"
    echo ""
    echo "📋 当前配置文件内容:"
    echo "=================================="
    if [ -f "../config/bridge_config.yaml" ]; then
        cat ../config/bridge_config.yaml
    else
        echo "❌ 配置文件不存在"
    fi
else
    echo "❌ 编译失败"
    exit 1
fi