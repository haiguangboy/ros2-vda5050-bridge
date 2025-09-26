#!/bin/bash

# zmq_mqtt_start_unit.sh
# 启动3个独立的终端窗口执行不同任务

echo "启动3个独立的终端窗口..."

# 第一个终端：MQTT Build
echo "启动终端1: MQTT Build"
gnome-terminal --title="MQTT Build" --geometry=80x24+100+100 -- bash -c 'cd ~/Documents/mqtt_ws/cpp_version; echo "=== MQTT Build Terminal ==="; echo "Current directory: $(pwd)"; bash build.sh; echo "Build completed. Press Enter to close or continue..."; read; exec bash' &

sleep 1

# 第二个终端：Test Scripts
echo "启动终端2: Test Scripts"
gnome-terminal --title="Test Scripts" --geometry=80x24+600+100 -- bash -c 'cd ~/Documents/mqtt_ws/cpp_version/tests/scripts; echo "=== Test Scripts Terminal ==="; echo "Current directory: $(pwd)"; echo "Ready for testing commands..."; exec bash' &

sleep 1

# 第三个终端：ZMQ Production
echo "启动终端3: ZMQ Production"
gnome-terminal --title="ZMQ Production" --geometry=80x24+1100+100 -- bash -c 'cd ~/Documents/zmq_ws/cpp_version; echo "=== ZMQ Production Terminal ==="; echo "Current directory: $(pwd)"; bash start_production.sh; echo "Production script completed. Press Enter to close or continue..."; read; exec bash' &

sleep 2

echo ""
echo "完成！启动了3个独立的终端窗口："
echo "终端1: MQTT Build (~/Documents/mqtt_ws/cpp_version) - 自动执行 build.sh"
echo "终端2: Test Scripts (~/Documents/mqtt_ws/cpp_version/tests/scripts/) - 等待手动输入"
echo "终端3: ZMQ Production (~/Documents/zmq_ws/cpp_version) - 自动执行 start_production.sh"
echo ""
echo "每个终端都是独立运行，互不干扰"