#!/bin/bash

# 真实Nav2路径转换测试脚本

echo "🧪 编译和运行真实Nav2路径转换测试"
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

# 创建build目录
mkdir -p build

echo "🔨 编译真实路径转换测试..."

# 编译测试程序
g++ -std=c++17 \
    -I/opt/ros/humble/include/rclcpp \
    -I/opt/ros/humble/include/nav_msgs \
    -I/opt/ros/humble/include/geometry_msgs \
    -I/opt/ros/humble/include/std_msgs \
    -I/opt/ros/humble/include/builtin_interfaces \
    -I/opt/ros/humble/include/rosidl_runtime_cpp \
    -I/opt/ros/humble/include/rcutils \
    -I/opt/ros/humble/include/rmw \
    -I/opt/ros/humble/include/tf2 \
    -I/opt/ros/humble/include/tf2_geometry_msgs \
    -I/opt/ros/humble/include/tf2_ros \
    -Isrc/include \
    test_real_path_conversion.cpp \
    src/lib/zhongli_protocol_types.cpp \
    src/lib/path_converter.cpp \
    -L/opt/ros/humble/lib \
    -lrclcpp \
    -lnav_msgs__rosidl_typesupport_cpp \
    -lgeometry_msgs__rosidl_typesupport_cpp \
    -lstd_msgs__rosidl_typesupport_cpp \
    -ltf2 \
    -ltf2_ros \
    -o build/test_real_path_conversion

if [ $? -eq 0 ]; then
    echo "✅ 编译成功！"

    echo "🚀 运行真实路径转换测试..."
    echo "============================="

    # 运行测试
    ./build/test_real_path_conversion

    if [ $? -eq 0 ]; then
        echo ""
        echo "🎉 真实路径转换测试通过！"
        echo ""
        echo "📋 测试总结:"
        echo "  ✅ 四元数到角度转换精确"
        echo "  ✅ 真实Nav2路径数据解析正确"
        echo "  ✅ ROS2路径到中力协议轨迹转换成功"
        echo "  ✅ JSON序列化输出正常"
        echo "  ✅ 轨迹验证通过"
        echo ""
        echo "🎯 C++版本桥接器可以正确处理真实的Nav2路径数据！"
    else
        echo "❌ 测试运行失败"
        exit 1
    fi
else
    echo "❌ 编译失败"
    echo ""
    echo "💡 可能的解决方案："
    echo "  1. 确保已安装ROS2 Humble完整开发包"
    echo "  2. 检查所有依赖库是否正确安装"
    echo "  3. 确保src/include和src/lib目录存在且包含必要文件"
    exit 1
fi

echo "✅ 真实路径测试完成"