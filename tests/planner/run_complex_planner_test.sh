#!/bin/bash
# ComplexTrajectoryPlanner 一键测试脚本

echo "=================================="
echo "🧪 ComplexTrajectoryPlanner 测试"
echo "=================================="
echo ""

# 获取当前脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 检查MQTT Bridge是否运行
echo "📋 检查环境..."
if pgrep -f "zhongli_bridge_node" > /dev/null; then
    echo "✅ MQTT Bridge 正在运行"
else
    echo "⚠️  MQTT Bridge 未运行"
    echo ""
    echo "请先在另一个终端启动MQTT Bridge："
    echo "  cd $SCRIPT_DIR"
    echo "  ./start_mqtt_bridge.sh"
    echo ""
    read -p "是否继续测试（不连MQTT）？[y/N] " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "=================================="
echo "🚀 启动测试节点"
echo "=================================="
echo ""
echo "测试文件: test_complex_planner_workflow.py"
echo ""
echo "💡 提示："
echo "  - 测试会自动规划并发布前向和后向轨迹"
echo "  - 如果没有Odom数据，会使用默认位置(0, 0)"
echo "  - 按 Ctrl+C 停止测试"
echo ""
echo "开始测试..."
echo ""

# 运行测试
python3 test_complex_planner_workflow.py

echo ""
echo "=================================="
echo "测试结束"
echo "=================================="
