#!/bin/bash

# 简单版本：使用 gnome-terminal 创建多个标签页
gnome-terminal \
    --tab --title="MQTT Build" --working-directory="$HOME/Documents/mqtt_ws/cpp_version" -- bash -c 'bash build.sh; exec bash' \
    --tab --title="Test Scripts" --working-directory="$HOME/Documents/mqtt_ws/cpp_version/tests/scripts" \
    --tab --title="ZMQ Production" --working-directory="$HOME/Documents/zmq_ws/cpp_version" -- bash -c 'bash start_production.sh; exec bash'

echo "Started gnome-terminal with 3 tabs"
echo "Tab 1: MQTT Build (~/Documents/mqtt_ws/cpp_version)"
echo "Tab 2: Test Scripts (~/Documents/mqtt_ws/cpp_version/tests/scripts/)"
echo "Tab 3: ZMQ Production (~/Documents/zmq_ws/cpp_version)"