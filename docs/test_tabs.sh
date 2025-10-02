#!/bin/bash

# test_tabs.sh
# 测试在gnome-terminal中创建多个标签页

echo "测试方案1: 使用gnome-terminal直接创建多标签页"
gnome-terminal --tab --title="Tab 1" --tab --title="Tab 2" --tab --title="Tab 3" &
sleep 2
echo "方案1执行完毕，检查是否创建了3个标签页"
echo ""

sleep 3

echo "测试方案2: 使用xdotool模拟按键创建标签页"
if command -v xdotool &> /dev/null; then
    echo "启动gnome-terminal..."
    gnome-terminal &
    sleep 2

    echo "创建第二个标签页 (Ctrl+Shift+T)..."
    xdotool search --onlyvisible --class "gnome-terminal" windowactivate
    sleep 0.5
    xdotool key ctrl+shift+t
    sleep 1

    echo "创建第三个标签页 (Ctrl+Shift+T)..."
    xdotool key ctrl+shift+t
    sleep 1

    echo "方案2执行完毕"
else
    echo "xdotool未安装，跳过方案2"
    echo "安装命令: sudo apt install xdotool"
fi

echo ""
echo "测试完成！请检查终端窗口是否正确创建了标签页"