# 故障排查指南

## 问题0: 轨迹已发布但查询不到状态（最常见）⚠️

### 症状
```bash
# 轨迹规划节点显示：
📤 轨迹已发布到 /plans
📋 轨迹ID: goal_traj_1759581173984
⏳ 等待MQTT轨迹完成信号（按Ctrl+C可中断）...
# 一直等待，没有收到状态

# 查询状态显示：
⚠️  暂无状态数据
```

### 原因
**MQTT Bridge没有运行！**

轨迹发布到 `/plans` 话题后，需要MQTT Bridge来：
1. 订阅 `/plans` 话题
2. 转换为MQTT消息发送到EMQX
3. 从EMQX接收状态消息
4. 将状态反馈给轨迹规划节点

### 解决方案 ✅

**步骤1: 检查MQTT Bridge是否运行**
```bash
ps aux | grep zhongli_bridge_node | grep -v grep
```

如果没有输出，说明Bridge没有运行。

**步骤2: 启动MQTT Bridge**

**方法A: 使用启动脚本（推荐）**
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
./start_mqtt_bridge.sh
```

**方法B: 手动启动**
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge

# Source环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 运行Bridge
./install/bin/zhongli_bridge_node --ros-args \
  --params-file config/bridge_config.yaml
```

**步骤3: 验证Bridge正常工作**

在另一个终端运行诊断工具：
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
python3 diagnose_mqtt.py
```

然后发布目标点，应该能看到MQTT消息。

### 完整测试流程

**终端1: 启动MQTT Bridge**
```bash
./start_mqtt_bridge.sh
```

**终端2: 启动轨迹规划节点**
```bash
python3 test_beta4_trajectory_workflow_goal.py
```

**终端3: 发布目标点**
```bash
python3 publish_goal.py --x 3.0 --y 0.0 --yaw 90
```

**终端4（可选）: 诊断MQTT消息流**
```bash
python3 diagnose_mqtt.py
```

如果一切正常，应该看到：
- 终端1（Bridge）: 接收到 /plans，发送MQTT消息
- 终端2（规划节点）: 收到MQTT状态，service可查询
- 终端4（诊断）: 看到MQTT trajectory和trajectory_status消息

## 问题1: 发布目标点后，轨迹规划节点没有收到

### 症状
```bash
# 终端1：轨迹规划节点
⏳ 等待目标点 /nav_goal 话题数据...
   无超时限制（按Ctrl+C可中断）
# 一直等待，没有反应

# 终端2：发布目标点
✅ 已发布目标点到 /nav_goal:
   位置: (3.000, 0.000)
   朝向: 90.0° (1.571 rad)
# 立即退出
```

### 原因
ROS2的发布器和订阅器需要时间建立连接（DDS发现机制）。如果发布器发布消息后立即退出，订阅器可能还没来得及建立连接。

### 解决方案 ✅

**方法1: 使用改进后的发布器（推荐）**

`publish_goal.py` 已修改为持续发布2秒（20次），确保订阅器能收到消息：

```bash
python3 publish_goal.py --x 3.0 --y 0.0 --yaw 90
```

输出：
```
✅ 准备发布目标点到 /nav_goal:
   位置: (3.000, 0.000)
   朝向: 90.0° (1.571 rad)
   发布频率: 10 Hz，持续2秒

📤 已发布 5/20 次...
📤 已发布 10/20 次...
📤 已发布 15/20 次...
📤 已发布 20/20 次...
✅ 已发布 20 次，完成
```

**方法2: 使用 `ros2 topic pub`（调试用）**

```bash
ros2 topic pub /nav_goal geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 3.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}
  }
}" --once
```

### 验证连接

**步骤1: 检查topic是否存在**
```bash
ros2 topic list | grep nav_goal
```

应该看到：
```
/nav_goal
```

**步骤2: 检查订阅器是否在线**
```bash
ros2 topic info /nav_goal
```

应该看到：
```
Subscription count: 1
```

**步骤3: 使用测试脚本验证**

终端1：
```bash
python3 test_goal_connection.py
```

终端2：
```bash
python3 publish_goal.py --x 3.0 --y 0.0 --yaw 90
```

如果连接正常，终端1应该显示：
```
📩 收到目标点 #1:
   位置: (3.000, 0.000)
   朝向: 90.0° (1.571 rad)
   Frame: map
```

## 问题2: /Odom话题超时

### 症状
```
⏳ 等待 /Odom 话题数据（最多等待 10 秒）...
⚠️  10秒内未收到 /Odom 话题，使用默认位置数据
   默认位置: (0.000, 0.000), 朝向: 0.0°
```

### 解决方案

**方法1: 使用mock服务器发布Odom**

```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/scripts
python3 mock_nav_goal_server.py --odom-x 1.0 --odom-y 0.0 --odom-yaw 0.3
```

**方法2: 手动发布Odom（测试用）**

```bash
ros2 topic pub /Odom nav_msgs/msg/Odometry "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 1.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}" --rate 10
```

**方法3: 修改默认位置**

编辑 `test_beta4_trajectory_workflow_goal.py`：
```python
# 默认位置（Odom超时时使用）
DEFAULT_X = 1.0      # 修改为你的起始X
DEFAULT_Y = 0.0      # 修改为你的起始Y
DEFAULT_YAW = 0.3    # 修改为你的起始朝向（弧度）
```

## 问题3: MQTT连接失败

### 症状
```
❌ MQTT连接失败: [Errno 111] Connection refused
```

### 解决方案

**检查MQTT broker是否运行：**

```bash
# 检查EMQX是否在运行
systemctl status emqx

# 或检查端口
netstat -tuln | grep 1883
```

**修改MQTT配置：**

编辑 `test_beta4_trajectory_workflow_goal.py`：
```python
# MQTT配置
MQTT_BROKER = "localhost"  # 或实际的broker地址
MQTT_PORT = 1883
```

**启动EMQX（如果未运行）：**

```bash
sudo systemctl start emqx
# 或
emqx start
```

## 问题4: Service查询返回no_data

### 症状
```bash
python3 query_trajectory_status.py

⚠️  暂无状态数据
   message: {"status": "no_data", ...}
```

### 原因
1. 还没有发布过轨迹
2. MQTT还没有收到状态反馈

### 解决方案

**确保完整流程：**

1. 启动轨迹规划节点
2. 发布目标点
3. 等待轨迹执行
4. MQTT返回状态
5. 查询service

**查看MQTT消息：**

```bash
# 订阅所有EP主题
mosquitto_sub -h localhost -t "EP/#" -v

# 或只订阅状态主题
mosquitto_sub -h localhost -t "EP/robot-001/cerebellum/embrain/trajectory_status" -v
```

## 调试技巧

### 1. 查看所有话题
```bash
ros2 topic list
```

### 2. 监听话题消息
```bash
# 监听目标点
ros2 topic echo /nav_goal

# 监听Odom
ros2 topic echo /Odom

# 监听规划路径
ros2 topic echo /plans
```

### 3. 查看节点信息
```bash
# 查看所有节点
ros2 node list

# 查看节点详情
ros2 node info /goal_based_trajectory_tester
```

### 4. 查看服务列表
```bash
# 查看所有服务
ros2 service list

# 查看服务类型
ros2 service type /trajectory_status

# 调用服务
ros2 service call /trajectory_status example_interfaces/srv/Trigger
```

### 5. 日志级别调整

在Python代码中添加：
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

或运行时：
```bash
ros2 run --ros-args --log-level DEBUG
```

## 常见配置检查清单

- [ ] MQTT broker运行正常 (`systemctl status emqx`)
- [ ] /Odom话题有数据发布 (`ros2 topic echo /Odom`)
- [ ] 目标点发布器持续发布 (使用新版 `publish_goal.py`)
- [ ] 轨迹规划节点正在运行
- [ ] 网络配置正确（localhost vs IP地址）
- [ ] ROS2环境已source (`source install/setup.bash`)
