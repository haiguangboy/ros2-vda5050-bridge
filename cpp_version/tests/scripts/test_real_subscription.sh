#!/bin/bash

# 规范的真实/plan话题订阅测试脚本

echo "🧪 真实Nav2 /plan话题订阅测试"
echo "============================="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2环境未设置，尝试加载ROS2 Humble..."
    source /opt/ros/humble/setup.bash || {
        echo "❌ 无法加载ROS2环境，请确保已安装ROS2 Humble"
        exit 1
    }
fi

echo "✅ ROS2环境: $ROS_DISTRO"

# 检查/plan话题是否存在
echo "🔍 检查/plan话题状态..."
if ros2 topic list | grep -q "/plan"; then
    echo "✅ /plan话题已存在"
    echo "📡 话题信息:"
    ros2 topic info /plan
else
    echo "⚠️  /plan话题不存在"
    echo "💡 请先启动路径发布器: python3 test_nav_path_publisher.py"
    exit 1
fi

echo ""
echo "🔨 在cpp_version目录下构建和安装..."

# 确保在cpp_version目录下
if [ "$(basename $(pwd))" != "cpp_version" ]; then
    echo "❌ 请在cpp_version目录下运行此脚本"
    exit 1
fi

# 创建build目录并构建
mkdir -p build
cd build

# 使用cmake构建
cmake .. -DBUILD_TESTING=ON
make -j$(nproc)

if [ $? -eq 0 ]; then
    echo "✅ 构建成功！"

    # 安装到cpp_version/install目录
    make install

    echo "✅ 安装完成！"

    # 回到cpp_version目录
    cd ..

    echo ""
    echo "🚀 运行真实订阅测试..."
    echo "=============================="
    echo "使用install/bin目录中的可执行文件"
    echo "=============================="

    # 检查可执行文件是否存在
    TEST_EXEC="install/bin/test_real_plan_subscription"

    if [ -f "$TEST_EXEC" ]; then
        echo "✅ 找到测试可执行文件: $TEST_EXEC"
        echo "🚀 启动测试，按Ctrl+C停止..."
        echo ""

        # 运行安装的测试可执行文件
        ./$TEST_EXEC

    else
        echo "❌ 测试可执行文件不存在: $TEST_EXEC"
        echo "💡 查看install目录结构："
        find install/ -type f 2>/dev/null || echo "  install目录为空"
        exit 1
    fi

else
    echo "❌ 构建失败"
    echo "💡 请检查依赖项是否正确安装"
    exit 1
fi

echo ""
echo "✅ 测试完成"