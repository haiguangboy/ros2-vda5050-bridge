#!/bin/bash
# test_mosquitto_detect.sh
# 测试mosquitto库检测是否在ARM64系统上正常工作

echo "🔍 测试mosquitto库检测 (多架构支持)"
echo "==========================================="

# 检测系统架构
ARCH=$(dpkg --print-architecture)
echo "📋 系统架构: $ARCH"

# 查找mosquitto头文件
echo ""
echo "🔍 查找mosquitto头文件..."
if [ -f "/usr/include/mosquitto.h" ]; then
    echo "✅ 找到 /usr/include/mosquitto.h"
else
    echo "❌ 未找到 /usr/include/mosquitto.h"
fi

# 查找mosquitto库文件（多架构支持）
echo ""
echo "🔍 查找mosquitto库文件..."
MOSQUITTO_LIB_PATH="/usr/lib/${ARCH}-linux-gnu/libmosquitto.so.1"
echo "预期路径: $MOSQUITTO_LIB_PATH"

if [ -f "$MOSQUITTO_LIB_PATH" ]; then
    echo "✅ 找到 $MOSQUITTO_LIB_PATH"
    echo "📁 库文件详情:"
    ls -la "$MOSQUITTO_LIB_PATH"
else
    echo "❌ 未找到 $MOSQUITTO_LIB_PATH"
    echo "🔍 搜索其他位置..."
    find /usr/lib -name "*mosquitto*" 2>/dev/null || echo "   未找到任何mosquitto库文件"
fi

# 测试pkg-config
echo ""
echo "🔍 测试pkg-config..."
if command -v pkg-config &> /dev/null; then
    echo "✅ pkg-config 可用"
    if pkg-config --exists libmosquitto; then
        echo "✅ pkg-config 找到 libmosquitto"
        echo "   版本: $(pkg-config --modversion libmosquitto)"
        echo "   库路径: $(pkg-config --libs libmosquitto)"
        echo "   头文件路径: $(pkg-config --cflags libmosquitto)"
    else
        echo "❌ pkg-config 未找到 libmosquitto"
    fi
else
    echo "❌ pkg-config 不可用"
fi

# 检查CMake能否找到
echo ""
echo "🔍 测试CMake检测..."
cat > /tmp/test_mosquitto_cmake.txt << 'EOF'
cmake_minimum_required(VERSION 3.16)
project(test_mosquitto)

# 检测架构
execute_process(COMMAND dpkg --print-architecture
                OUTPUT_VARIABLE SYSTEM_ARCH
                OUTPUT_STRIP_TRAILING_WHITESPACE)

set(MOSQUITTO_LIB_PATH "/usr/lib/${SYSTEM_ARCH}-linux-gnu/libmosquitto.so.1")
find_path(MOSQUITTO_INCLUDE_DIR mosquitto.h PATHS /usr/include /usr/local/include)

if(EXISTS ${MOSQUITTO_LIB_PATH} AND MOSQUITTO_INCLUDE_DIR)
    message(STATUS "SUCCESS: Found mosquitto library: ${MOSQUITTO_LIB_PATH}")
    message(STATUS "SUCCESS: Found mosquitto headers: ${MOSQUITTO_INCLUDE_DIR}")
else()
    message(WARNING "FAILED: mosquitto not found")
    message(STATUS "Searched: ${MOSQUITTO_LIB_PATH}")
    message(STATUS "Headers: ${MOSQUITTO_INCLUDE_DIR}")
endif()
EOF

echo "运行CMake测试..."
cd /tmp && cmake -S . -B build_test_mosquitto -P test_mosquitto_cmake.txt 2>&1 | grep -E "(SUCCESS|FAILED|Found|WARNING)"

echo ""
echo "🎯 检测总结:"
echo "============"
if [ -f "/usr/include/mosquitto.h" ] && [ -f "$MOSQUITTO_LIB_PATH" ]; then
    echo "✅ mosquitto库和头文件都已正确安装"
    echo "✅ 支持 $ARCH 架构"
    echo "✅ 可以开始编译C++桥接器"
else
    echo "❌ mosquitto安装不完整"
    echo "建议重新安装: sudo apt install libmosquitto-dev"
fi

# 清理临时文件
rm -f /tmp/test_mosquitto_cmake.txt
rm -rf /tmp/build_test_mosquitto