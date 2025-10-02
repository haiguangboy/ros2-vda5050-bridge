#!/bin/bash
# fix_ros2_deps.sh
# 修复ROS2依赖问题

echo "🔧 修复ROS2依赖问题"
echo "=================="

echo "📦 检查当前ROS2安装状态..."

# 检查缺失的包
missing_packages=()

# 检查rcl包
if ! dpkg -l | grep -q "ros-humble-rcl-"; then
    missing_packages+=("ros-humble-rcl")
fi

# 检查rosidl运行时包
if ! dpkg -l | grep -q "ros-humble-rosidl-runtime-c"; then
    missing_packages+=("ros-humble-rosidl-runtime-c")
fi

if ! dpkg -l | grep -q "ros-humble-rosidl-runtime-cpp"; then
    missing_packages+=("ros-humble-rosidl-runtime-cpp")
fi

# 检查其他核心开发包
core_packages=(
    "ros-humble-rcl-interfaces"
    "ros-humble-rclcpp"
    "ros-humble-rosidl-default-generators"
    "ros-humble-rosidl-default-runtime"
    "ros-humble-rmw"
    "ros-humble-rmw-implementation"
    "ros-humble-rcutils"
    "ros-humble-fastrtps"
    "ros-humble-fastrtps-cmake-module"
)

for package in "${core_packages[@]}"; do
    if ! dpkg -l | grep -q "$package"; then
        missing_packages+=("$package")
    fi
done

if [ ${#missing_packages[@]} -eq 0 ]; then
    echo "✅ 所有核心ROS2包已安装"
else
    echo "❌ 发现缺失的包: ${#missing_packages[@]} 个"
    echo ""
    echo "📋 缺失的包列表:"
    for package in "${missing_packages[@]}"; do
        echo "  - $package"
    done
    echo ""

    echo "📥 安装缺失的包..."
    sudo apt update
    sudo apt install -y "${missing_packages[@]}"

    if [ $? -eq 0 ]; then
        echo "✅ 成功安装缺失的包"
    else
        echo "❌ 安装失败，请检查网络连接和权限"
        exit 1
    fi
fi

# 重新安装完整的ROS2开发环境（推荐）
echo ""
echo "🚀 安装完整的ROS2开发环境..."
sudo apt install -y ros-humble-desktop-full ros-humble-ros-base ros-humble-development

# 验证关键头文件
echo ""
echo "🔍 验证关键头文件..."

header_files=(
    "/opt/ros/humble/include/rcl/guard_condition.h"
    "/opt/ros/humble/include/rosidl_runtime_c/message_initialization.h"
    "/opt/ros/humble/include/rclcpp/rclcpp.hpp"
)

all_headers_found=true
for header in "${header_files[@]}"; do
    if [ -f "$header" ]; then
        echo "  ✅ $header"
    else
        echo "  ❌ $header (缺失)"
        all_headers_found=false
    fi
done

if [ "$all_headers_found" = true ]; then
    echo ""
    echo "🎉 所有必要的头文件都已找到！"
    echo "现在可以重新编译项目了"
    echo ""
    echo "🔧 建议的下一步操作："
    echo "  1. source /opt/ros/humble/setup.bash"
    echo "  2. rm -rf build"
    echo "  3. mkdir build && cd build"
    echo "  4. cmake .."
    echo "  5. make -j\$(nproc)"
else
    echo ""
    echo "⚠️  仍有头文件缺失，可能需要重新安装ROS2"
    echo ""
    echo "🔄 尝试完全重新安装ROS2 Humble:"
    echo "  sudo apt remove ros-humble-* && sudo apt autoremove"
    echo "  sudo apt install ros-humble-desktop-full"
fi

echo ""
echo "📝 创建环境检查脚本..."
cat > check_ros2_headers.sh << 'EOF'
#!/bin/bash
# 快速检查ROS2头文件

echo "🔍 ROS2头文件检查"
echo "================"

headers=(
    "/opt/ros/humble/include/rcl/guard_condition.h"
    "/opt/ros/humble/include/rosidl_runtime_c/message_initialization.h"
    "/opt/ros/humble/include/rclcpp/rclcpp.hpp"
    "/opt/ros/humble/include/nav_msgs/msg/path.hpp"
    "/opt/ros/humble/include/geometry_msgs/msg/pose_stamped.hpp"
)

for header in "${headers[@]}"; do
    if [ -f "$header" ]; then
        echo "✅ $(basename $header)"
    else
        echo "❌ $(basename $header) - $header"
    fi
done
EOF

chmod +x check_ros2_headers.sh
echo "✅ 已创建 check_ros2_headers.sh 用于快速检查"