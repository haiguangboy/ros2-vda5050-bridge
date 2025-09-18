#!/bin/bash
# check_dependencies.sh
# 中力协议C++桥接器依赖检查脚本
# 支持 x86_64 和 ARM64 架构

echo "🔍 中力协议C++桥接器依赖检查"
echo "=================================="

# 检查ROS2环境
echo ""
echo "📋 检查ROS2环境..."
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未设置"
    echo "   请运行: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo "✅ ROS2 $ROS_DISTRO"
fi

# 检查基础构建工具
echo ""
echo "🔧 检查构建工具..."
if command -v cmake &> /dev/null; then
    CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
    echo "✅ CMake $CMAKE_VERSION"
else
    echo "❌ 缺少cmake"
    echo "   请运行: sudo apt install cmake"
    exit 1
fi

if command -v make &> /dev/null; then
    echo "✅ make"
else
    echo "❌ 缺少make"
    echo "   请运行: sudo apt install build-essential"
    exit 1
fi

# 检查C++编译器
echo ""
echo "🔨 检查C++编译器..."
if command -v g++ &> /dev/null; then
    GCC_VERSION=$(g++ --version | head -n1 | cut -d' ' -f3)
    echo "✅ GCC $GCC_VERSION"
else
    echo "❌ 缺少g++"
    echo "   请运行: sudo apt install build-essential"
    exit 1
fi

# 检查mosquitto开发库
echo ""
echo "📡 检查MQTT客户端库..."
if [ -f "/usr/include/mosquitto.h" ]; then
    echo "✅ mosquitto.h 开发头文件"
else
    echo "❌ 缺少libmosquitto-dev"
    echo "   请运行: sudo apt install libmosquitto-dev"
    exit 1
fi

# 检测系统架构并查找mosquitto运行库
ARCH=$(dpkg --print-architecture)
MOSQUITTO_LIB_PATH="/usr/lib/${ARCH}-linux-gnu/libmosquitto.so.1"

if [ -f "$MOSQUITTO_LIB_PATH" ]; then
    echo "✅ libmosquitto.so.1 运行库 ($ARCH)"
elif find /usr/lib -name "libmosquitto.so.1" 2>/dev/null | grep -q .; then
    echo "✅ libmosquitto.so.1 运行库 (找到在其他位置)"
else
    echo "❌ 缺少libmosquitto运行库"
    echo "   请运行: sudo apt install libmosquitto-dev"
    exit 1
fi

# 检查nlohmann/json
echo ""
echo "📄 检查JSON处理库..."
if dpkg -l | grep -q nlohmann-json3-dev; then
    echo "✅ nlohmann/json3 开发包"
elif [ -f "/usr/include/nlohmann/json.hpp" ]; then
    echo "✅ nlohmann/json.hpp"
else
    echo "❌ 缺少nlohmann-json3-dev"
    echo "   请运行: sudo apt install nlohmann-json3-dev"
    exit 1
fi

# 检查ROS2包
echo ""
echo "🤖 检查ROS2依赖包..."

# 使用兼容sh的方式检查ROS2包
check_ros2_package() {
    package=$1
    if dpkg -l | grep -q "ros-humble-${package}"; then
        echo "✅ ros-humble-${package}"
        return 0
    else
        echo "❌ 缺少ros-humble-${package}"
        echo "   请运行: sudo apt install ros-humble-${package}"
        return 1
    fi
}

# 检查各个包
check_ros2_package "ament-cmake" || exit 1
check_ros2_package "nav-msgs" || exit 1
check_ros2_package "geometry-msgs" || exit 1
check_ros2_package "nav2-msgs" || exit 1
check_ros2_package "tf2-ros" || exit 1
check_ros2_package "tf2-geometry-msgs" || exit 1

# 检查MQTT代理
echo ""
echo "🌐 检查MQTT代理..."
if netstat -tlpn 2>/dev/null | grep -q ":1883"; then
    echo "✅ MQTT代理运行中 (端口1883)"

    # 尝试检测是什么类型的MQTT代理
    if pgrep -f "emqx" > /dev/null; then
        echo "   检测到: EMQX"
    elif pgrep -f "mosquitto" > /dev/null; then
        echo "   检测到: Mosquitto"
    else
        echo "   检测到: 未知MQTT代理"
    fi
else
    echo "⚠️  MQTT代理未运行 (端口1883)"
    echo "   如需测试，可启动mosquitto: sudo systemctl start mosquitto"
    echo "   或者配置桥接器连接到远程EMQX服务器"
fi

# 检查mosquitto客户端工具
echo ""
echo "🛠️  检查MQTT测试工具..."
if command -v mosquitto_pub &> /dev/null && command -v mosquitto_sub &> /dev/null; then
    echo "✅ mosquitto客户端工具"
else
    echo "⚠️  缺少mosquitto客户端工具（可选）"
    echo "   用于测试MQTT通信: sudo apt install mosquitto-clients"
fi

# 检查colcon构建工具
echo ""
echo "🏗️  检查ROS2构建工具..."
if command -v colcon &> /dev/null; then
    echo "✅ colcon构建工具"
else
    echo "⚠️  缺少colcon构建工具（可选）"
    echo "   用于ROS2包构建: sudo apt install python3-colcon-common-extensions"
fi

# 总结
echo ""
echo "📊 依赖检查完成"
echo "=================================="
echo "✅ 所有核心依赖已满足"
echo ""
echo "📝 下一步:"
echo "   1. cd cpp_version"
echo "   2. mkdir build && cd build"
echo "   3. cmake .."
echo "   4. make -j\$(nproc)"
echo "   5. make install"
echo ""
echo "🚀 然后运行:"
echo "   ./install/bin/zhongli_bridge_node"