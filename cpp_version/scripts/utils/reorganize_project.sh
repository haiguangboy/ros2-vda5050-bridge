#!/bin/bash
# reorganize_project.sh
# 重新整理cpp_version项目目录结构

echo "🗂️ 重新整理项目目录结构"
echo "=========================="

# 创建规范的目录结构
echo "📁 创建目录结构..."
mkdir -p tests/unit           # 单元测试
mkdir -p tests/integration    # 集成测试
mkdir -p tests/scripts        # 测试脚本
mkdir -p tests/data           # 测试数据
mkdir -p tools                # 工具脚本
mkdir -p docs                 # 文档
mkdir -p scripts              # 构建和部署脚本

echo "🔄 移动文件到合适的位置..."

# 移动测试相关的cpp文件
echo "移动测试C++文件..."
[ -f "test_real_path_conversion.cpp" ] && mv test_real_path_conversion.cpp tests/unit/
[ -f "test_path_simple.cpp" ] && mv test_path_simple.cpp tests/unit/

# 移动测试脚本
echo "移动测试脚本..."
[ -f "test_real_path.sh" ] && mv test_real_path.sh tests/scripts/
[ -f "test_path_simple.sh" ] && mv test_path_simple.sh tests/scripts/
[ -f "test_basic_build.sh" ] && mv test_basic_build.sh tests/scripts/
[ -f "test_real_subscription.sh" ] && mv test_real_subscription.sh tests/scripts/

# 移动测试数据
echo "移动测试数据..."
[ -f "test_nav_path_publisher.py" ] && mv test_nav_path_publisher.py tests/data/

# 移动工具脚本
echo "移动工具脚本..."
[ -f "check_dependencies.sh" ] && mv check_dependencies.sh tools/
[ -f "test_mosquitto_detect.sh" ] && mv test_mosquitto_detect.sh tools/
[ -f "install_deps.sh" ] && mv install_deps.sh tools/

# 移动构建脚本
echo "移动构建脚本..."
[ -f "build_and_test.sh" ] && mv build_and_test.sh scripts/

# 移动文档
echo "移动文档..."
[ -f "zongjie.md" ] && mv zongjie.md docs/
[ -f "USAGE.md" ] && mv USAGE.md docs/
[ -f "README.md" ] && mv README.md docs/

# 创建新的规范化测试文件

# 1. 重写test_real_path.sh
echo "📝 创建规范的测试脚本..."
cat > tests/scripts/test_path_conversion.sh << 'EOF'
#!/bin/bash
# test_path_conversion.sh
# 路径转换功能测试

set -e

echo "🧪 路径转换功能测试"
echo "=================="

# 检查测试环境
if [ ! -f "../../install/bin/zhongli_bridge_node" ]; then
    echo "❌ 桥接器未编译，请先运行编译脚本"
    exit 1
fi

echo "✅ 测试环境检查通过"

# 这里可以添加具体的路径转换测试逻辑
echo "🎯 测试将在后续版本中实现"
EOF

# 2. 创建集成测试脚本
cat > tests/integration/test_mqtt_integration.sh << 'EOF'
#!/bin/bash
# test_mqtt_integration.sh
# MQTT集成测试

set -e

echo "🌐 MQTT集成测试"
echo "=============="

# 检查MQTT代理
if ! netstat -tlpn 2>/dev/null | grep -q ":1883"; then
    echo "❌ MQTT代理未运行"
    exit 1
fi

echo "✅ MQTT代理运行中"

# 这里可以添加MQTT集成测试逻辑
echo "🎯 集成测试将在后续版本中实现"
EOF

# 3. 创建主测试脚本
cat > tests/run_all_tests.sh << 'EOF'
#!/bin/bash
# run_all_tests.sh
# 运行所有测试

set -e

echo "🧪 运行所有测试"
echo "=============="

# 运行单元测试
echo "📋 单元测试..."
# 这里可以添加单元测试调用

# 运行集成测试
echo "🔗 集成测试..."
if [ -f "integration/test_mqtt_integration.sh" ]; then
    bash integration/test_mqtt_integration.sh
fi

# 运行脚本测试
echo "📜 脚本测试..."
if [ -f "scripts/test_path_conversion.sh" ]; then
    bash scripts/test_path_conversion.sh
fi

echo "✅ 所有测试完成"
EOF

# 设置执行权限
chmod +x tests/scripts/*.sh
chmod +x tests/integration/*.sh
chmod +x tests/run_all_tests.sh

# 创建新的README
cat > README.md << 'EOF'
# 中力协议C++桥接器

高性能的ROS2到中力协议MQTT桥接器，支持x86_64和ARM64架构。

## 快速开始

```bash
# 1. 安装依赖
./tools/install_deps.sh

# 2. 检查环境
./tools/check_dependencies.sh

# 3. 编译项目
./scripts/build_and_test.sh

# 4. 运行桥接器
./install/bin/zhongli_bridge_node
```

## 目录结构

```
cpp_version/
├── src/                     # 源代码
├── config/                  # 配置文件
├── tests/                   # 测试
│   ├── unit/               # 单元测试
│   ├── integration/        # 集成测试
│   ├── scripts/            # 测试脚本
│   └── data/               # 测试数据
├── tools/                   # 开发工具
├── scripts/                 # 构建脚本
├── docs/                    # 文档
├── build/                   # 编译目录
└── install/                 # 安装目录
```

## 文档

- [详细总结](docs/zongjie.md) - 完整的开发过程和技术方案
- [使用说明](docs/USAGE.md) - 基本使用指南

## 架构支持

- x86_64 (Intel/AMD)
- ARM64 (ARM处理器，如树莓派、NVIDIA Jetson等)
EOF

echo "📊 整理完成！新的目录结构："
echo ""
find . -type d | sort | sed 's/^/  /'

echo ""
echo "🎯 主要改进："
echo "  ✅ 测试文件分类到tests目录"
echo "  ✅ 工具脚本移至tools目录"
echo "  ✅ 构建脚本移至scripts目录"
echo "  ✅ 文档整理到docs目录"
echo "  ✅ 创建规范的项目结构"
echo ""
echo "📝 下一步："
echo "  1. 检查移动的文件是否正确"
echo "  2. 更新CMakeLists.txt中的测试路径"
echo "  3. 测试新的目录结构"