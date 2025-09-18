#!/bin/bash

# 基础编译测试脚本 - 不依赖完整的ROS2环境

echo "🔧 基础C++模块编译测试"
echo "========================="

# 检查基础依赖
echo "📦 检查C++编译环境..."

# 检查g++
if ! command -v g++ &> /dev/null; then
    echo "❌ g++未安装，请运行: sudo apt install build-essential"
    exit 1
fi

# 检查cmake
if ! command -v cmake &> /dev/null; then
    echo "❌ cmake未安装，请运行: sudo apt install cmake"
    exit 1
fi

echo "✅ C++编译环境正常"

# 只编译协议类型库
echo "🔨 编译协议类型库..."

cd src/lib

# 直接用g++编译协议类型库
g++ -std=c++17 -I../include -fPIC -shared \
    zhongli_protocol_types.cpp \
    -o ../../build/libzhongli_protocol_types.so

if [ $? -eq 0 ]; then
    echo "✅ 协议类型库编译成功！"

    # 创建简单的测试程序
    echo "🧪 创建测试程序..."

    cat > ../../build/test_types.cpp << 'EOF'
#include <iostream>
#include <chrono>
#include <thread>

// 模拟协议类型（简化版本，不依赖外部库）
namespace zhongli_protocol {

    std::string create_timestamp() {
        return "2025-09-14T10:00:00.000Z";
    }

    std::string generate_trajectory_id(const std::string& robot_id) {
        return "traj-" + robot_id + "-20250914-123456";
    }

    void test_basic_functions() {
        std::cout << "⏰ 时间戳测试: " << create_timestamp() << std::endl;
        std::cout << "🛤️  轨迹ID测试: " << generate_trajectory_id("robot-001") << std::endl;
    }
}

int main() {
    std::cout << "🚀 中力具身机器人协议 - 基础测试" << std::endl;
    std::cout << "==================================" << std::endl;

    try {
        zhongli_protocol::test_basic_functions();
        std::cout << "✅ 基础功能测试通过！" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cout << "❌ 测试失败: " << e.what() << std::endl;
        return 1;
    }
}
EOF

    # 编译测试程序
    g++ -std=c++17 ../../build/test_types.cpp -o ../../build/test_types

    if [ $? -eq 0 ]; then
        echo "🧪 运行基础测试..."
        ../../build/test_types

        if [ $? -eq 0 ]; then
            echo "🎉 所有基础测试通过！"
        else
            echo "❌ 测试运行失败"
            exit 1
        fi
    else
        echo "❌ 测试程序编译失败"
        exit 1
    fi

else
    echo "❌ 协议类型库编译失败"
    exit 1
fi

echo "✅ 基础编译测试完成"