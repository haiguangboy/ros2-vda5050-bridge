#!/bin/bash
# install_deps.sh
# 中力协议C++桥接器依赖安装脚本

echo "🚀 中力协议C++桥接器依赖安装"
echo "=================================="

# 检查是否为Ubuntu系统
if [ ! -f /etc/os-release ] || ! grep -q "Ubuntu" /etc/os-release; then
    echo "❌ 此脚本仅支持Ubuntu系统"
    exit 1
fi

# 获取Ubuntu版本
UBUNTU_VERSION=$(lsb_release -rs)
echo "📋 检测到Ubuntu $UBUNTU_VERSION"

# 检查是否为22.04（推荐版本）
if [ "$UBUNTU_VERSION" != "22.04" ]; then
    echo "⚠️  推荐使用Ubuntu 22.04 LTS，当前版本为 $UBUNTU_VERSION"
    read -p "继续安装？(y/N): " confirm
    if [[ ! $confirm =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "📦 更新软件包列表..."
sudo apt update

echo ""
echo "🔧 安装基础构建工具..."
sudo apt install -y build-essential cmake pkg-config

echo ""
echo "🤖 检查ROS2 Humble安装..."
if [ -d "/opt/ros/humble" ]; then
    echo "✅ ROS2 Humble已安装"
else
    echo "📥 安装ROS2 Humble..."
    # 添加ROS2软件源
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y

    # 添加ROS2密钥
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    # 添加ROS2软件源
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # 更新并安装ROS2
    sudo apt update
    sudo apt install -y ros-humble-desktop

    echo "✅ ROS2 Humble安装完成"
fi

echo ""
echo "🛠️  安装ROS2开发工具..."
sudo apt install -y \
    python3-colcon-common-extensions \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-gtest

echo ""
echo "📦 安装ROS2导航相关包..."
sudo apt install -y \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav2-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs

echo ""
echo "📄 安装JSON处理库..."
sudo apt install -y nlohmann-json3-dev

echo ""
echo "📡 安装MQTT客户端库..."
sudo apt install -y libmosquitto-dev mosquitto-clients

echo ""
echo "🌐 配置MQTT代理（可选）..."
if ! systemctl is-active --quiet mosquitto; then
    echo "安装本地mosquitto代理（用于测试）..."
    sudo apt install -y mosquitto
    sudo systemctl enable mosquitto
    sudo systemctl start mosquitto
    echo "✅ 本地mosquitto代理已启动"
else
    echo "✅ mosquitto代理已运行"
fi

echo ""
echo "🔍 验证安装..."

# 检查关键依赖
echo "检查关键依赖:"

# ROS2
if [ -d "/opt/ros/humble" ]; then
    echo "  ✅ ROS2 Humble"
else
    echo "  ❌ ROS2 Humble"
fi

# mosquitto开发库
if [ -f "/usr/include/mosquitto.h" ]; then
    echo "  ✅ mosquitto开发库"
else
    echo "  ❌ mosquitto开发库"
fi

# nlohmann/json
if dpkg -l | grep -q nlohmann-json3-dev; then
    echo "  ✅ nlohmann/json"
else
    echo "  ❌ nlohmann/json"
fi

# CMake
if command -v cmake &> /dev/null; then
    echo "  ✅ CMake"
else
    echo "  ❌ CMake"
fi

echo ""
echo "📝 设置环境变量..."
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export MQTT_BROKER_HOST=localhost" >> ~/.bashrc
echo "export MQTT_BROKER_PORT=1883" >> ~/.bashrc

echo ""
echo "🎉 依赖安装完成!"
echo "=================================="
echo ""
echo "📋 下一步:"
echo "   1. 重新打开终端或运行: source ~/.bashrc"
echo "   2. cd到cpp_version目录"
echo "   3. 运行编译: mkdir build && cd build && cmake .. && make -j\$(nproc) && make install"
echo "   4. 启动桥接器: ./install/bin/zhongli_bridge_node"
echo ""
echo "🧪 测试MQTT通信:"
echo "   mosquitto_sub -h localhost -p 1883 -t 'EP/robot-001/embrain/cerebellum/trajectory'"
echo ""
echo "⚠️  注意: 如果使用远程EMQX服务器，请修改config/bridge_config.yaml中的MQTT配置"