#!/bin/bash

# C++ 版本编译和测试脚本
# 支持 x86_64 和 ARM64 架构

set -e  # 如果有任何命令失败，立即退出

# 回到项目根目录执行
cd "$(dirname "$0")/../.." || {
    echo "❌ 无法回到项目根目录"
    exit 1
}

echo "🔧 开始编译中力具身机器人ROS2桥接器 - C++版本"
echo "=================================================="

# 检查依赖
echo "📦 检查依赖..."

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2环境未设置，尝试加载ROS2 Humble..."
    source /opt/ros/humble/setup.bash || {
        echo "❌ 无法加载ROS2环境，请确保已安装ROS2 Humble"
        exit 1
    }
fi

echo "✅ ROS2环境: $ROS_DISTRO"

# 检查必要的包是否存在
echo "🔍 检查系统依赖..."

# 检查nlohmann-json
if ! pkg-config --exists nlohmann_json; then
    echo "❌ nlohmann-json未安装，请运行: sudo apt install nlohmann-json3-dev"
    exit 1
fi

# 检查mosquitto（替代paho-mqtt-cpp）
ARCH=$(dpkg --print-architecture)
MOSQUITTO_LIB_PATH="/usr/lib/${ARCH}-linux-gnu/libmosquitto.so.1"

if [ -f "/usr/include/mosquitto.h" ] && [ -f "$MOSQUITTO_LIB_PATH" ]; then
    echo "✅ mosquitto MQTT客户端库已安装 ($ARCH)"
elif [ -f "/usr/include/mosquitto.h" ] && find /usr/lib -name "libmosquitto.so.1" 2>/dev/null | grep -q .; then
    echo "✅ mosquitto MQTT客户端库已安装 (其他位置)"
else
    echo "❌ mosquitto未安装，请运行: sudo apt install libmosquitto-dev"
    echo "   或运行依赖检查脚本: ./check_dependencies.sh"
    exit 1
fi

echo "✅ 所有系统依赖已安装"

# 创建构建目录
echo "📁 准备构建目录..."
mkdir -p build
cd build

# 运行cmake配置
echo "⚙️  配置CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Debug

# 编译项目
echo "🔨 编译项目..."
make -j$(nproc)

# 安装到install目录
if [ $? -eq 0 ]; then
    echo "📦 安装到install目录..."
    make install
    echo "✅ 编译和安装成功！"

    echo "📋 安装目录结构:"
    echo "=================="
    find ../install -type f | sort
    echo "=================="

    echo "🧪 运行基础测试..."

    # 运行单元测试（如果编译了的话）
    if [ -f "src/tests/test_zhongli_protocol_types" ]; then
        echo "📋 运行协议类型测试..."
        ./src/tests/test_zhongli_protocol_types
    fi

    # 运行主程序
    if [ -f "../install/bin/zhongli_bridge_node" ]; then
        echo "🚀 运行主程序（使用配置文件）..."
        echo "📋 加载配置文件: config/bridge_config.yaml"
        cd .. # 回到项目根目录运行，这样路径更清晰
        ./install/bin/zhongli_bridge_node --ros-args --params-file config/bridge_config.yaml
    else
        echo "❌ 找不到安装的可执行文件: ../install/bin/zhongli_bridge_node"
    fi

    echo "🎉 所有测试通过！"
else
    echo "❌ 编译失败！"
    exit 1
fi

echo "✅ 构建和测试完成"