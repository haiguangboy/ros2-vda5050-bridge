#!/bin/bash
# test_real_path_fixed.sh
# 修复版本的路径转换测试脚本

set -e

echo "🧪 修复版本：真实Nav2路径转换测试"
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

# 检查是否有缺失的ROS2依赖
echo "🔍 检查ROS2依赖..."
missing_headers=()

critical_headers=(
    "/opt/ros/humble/include/rcl/guard_condition.h"
    "/opt/ros/humble/include/rosidl_runtime_c/message_initialization.h"
    "/opt/ros/humble/include/rclcpp/rclcpp.hpp"
    "/opt/ros/humble/include/nav_msgs/msg/path.hpp"
)

for header in "${critical_headers[@]}"; do
    if [ ! -f "$header" ]; then
        missing_headers+=("$(basename $header)")
        echo "❌ 缺失: $(basename $header)"
    else
        echo "✅ 找到: $(basename $header)"
    fi
done

if [ ${#missing_headers[@]} -ne 0 ]; then
    echo ""
    echo "❌ 发现缺失的ROS2头文件!"
    echo "请运行修复脚本: bash fix_ros2_deps.sh"
    exit 1
fi

echo "✅ 所有必需的ROS2头文件都存在"

# 使用CMake方式编译（推荐）
echo ""
echo "🔨 使用CMake编译路径转换测试..."

# 创建临时CMake项目
mkdir -p build_test
cd build_test

cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.16)
project(test_path_conversion)

set(CMAKE_CXX_STANDARD 17)

# 查找ROS2包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# 包含源代码目录
include_directories(../../src/include)

# 创建测试可执行文件
add_executable(test_path_conversion
    ../cpp/test_real_path_conversion.cpp
    ../../src/lib/zhongli_protocol_types.cpp
    ../../src/lib/path_converter.cpp
)

# 链接ROS2库
ament_target_dependencies(test_path_conversion
    rclcpp
    nav_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
)

target_link_libraries(test_path_conversion
    nlohmann_json::nlohmann_json
)
EOF

# CMake配置
echo "⚙️  CMake配置..."
cmake . -DCMAKE_BUILD_TYPE=Debug

# 编译
echo "🔨 编译..."
make -j$(nproc)

if [ $? -eq 0 ]; then
    echo "✅ 编译成功！"
    echo ""
    echo "🚀 运行路径转换测试..."
    echo "====================="

    # 运行测试
    ./test_path_conversion

    if [ $? -eq 0 ]; then
        echo ""
        echo "🎉 路径转换测试通过！"
        echo ""
        echo "📋 测试总结:"
        echo "  ✅ ROS2环境正确加载"
        echo "  ✅ 所有依赖库正确链接"
        echo "  ✅ 路径转换逻辑正常"
        echo "  ✅ JSON序列化成功"
        echo ""
        echo "🎯 修复版本测试成功！"
    else
        echo "❌ 测试运行失败"
        exit 1
    fi
else
    echo "❌ 编译失败"
    echo ""
    echo "💡 建议的解决步骤："
    echo "  1. 运行: bash fix_ros2_deps.sh"
    echo "  2. 重新source ROS2环境"
    echo "  3. 重新运行此测试脚本"
    exit 1
fi

# 清理
cd ..
rm -rf build_test

echo "✅ 修复版本路径测试完成"