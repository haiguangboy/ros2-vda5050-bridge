# ROS2 Zhongli Protocol Bridge (C++)

中力具身装卸机器人系统 ROS2 通信桥接器 - C++实现

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Protocol](https://img.shields.io/badge/Protocol-Beta--3-green)](./beta3_changes.md)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

## 📋 项目简介

本项目是中力具身装卸机器人系统的ROS2通信桥接器，采用C++实现，负责在ROS2导航系统和中力专有协议之间进行双向通信。支持最新的**Beta-3协议版本**，具有高性能、低延迟的特点。

### 主要功能

- ✅ ROS2路径规划结果转换为中力轨迹协议
- ✅ MQTT协议通信（轨迹、动作、任务消息）
- ✅ 实时状态反馈和里程计集成
- ✅ Beta-3协议完整支持（orientation、flag字段）
- ✅ 动作驱动的轨迹参数动态更新
- ✅ 容器位姿参数传递

## 🚀 快速开始

### 环境要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS版本**: ROS2 Humble
- **编译器**: GCC 11+ (支持C++17)
- **依赖库**:
  - nlohmann-json (≥3.10)
  - paho-mqtt-cpp
  - ROS2 core packages (rclcpp, nav_msgs, tf2, etc.)

### 安装依赖

```bash
# ROS2 Humble (如未安装)
sudo apt update
sudo apt install ros-humble-desktop

# 系统依赖
sudo apt install nlohmann-json3-dev libpaho-mqttpp-dev

# ROS2 依赖包
sudo apt install ros-humble-nav2-msgs ros-humble-tf2-geometry-msgs
```

### 编译项目

```bash
# 1. 进入项目目录
cd /path/to/ros2_zhongli_bridge

# 2. Source ROS2环境
source /opt/ros/humble/setup.bash

# 3. 编译
colcon build --packages-select ros2_zhongli_bridge_cpp --symlink-install

# 4. Source工作空间
source install/setup.bash
```

### 运行桥接器

```bash
# 启动MQTT broker (如未运行)
sudo systemctl start mosquitto

# 运行桥接器节点
./install/ros2_zhongli_bridge_cpp/bin/zhongli_bridge_node \
  --ros-args --params-file config/bridge_config.yaml
```

## 📐 系统架构

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────────┐
│  ROS2 Nav Stack │ ◄─────► │  Zhongli Bridge  │ ◄─────► │  MQTT Broker    │
│  (/plans, /Odom)│         │  (C++ Node)      │         │  (Mosquitto)    │
└─────────────────┘         └──────────────────┘         └─────────────────┘
                                     │
                                     ▼
                            ┌─────────────────┐
                            │  Zhongli Robot  │
                            │  Control System │
                            └─────────────────┘
```

### 核心组件

| 组件 | 功能 | 文件 |
|------|------|------|
| **Bridge Node** | ROS2主节点，协调所有通信 | `ros2_zhongli_bridge.cpp` |
| **Path Converter** | ROS2路径→中力轨迹转换 | `path_converter.cpp` |
| **MQTT Client** | MQTT通信管理 | `mqtt_client.cpp` |
| **Protocol Types** | Beta-3协议数据结构 | `zhongli_protocol_types.hpp` |

## 🔧 配置说明

### 配置文件: `config/bridge_config.yaml`

```yaml
zhongli_bridge:
  ros__parameters:
    # MQTT配置
    mqtt_broker_host: "localhost"
    mqtt_broker_port: 1883
    robot_id: "robot-001"

    # 路径转换参数
    default_max_speed: 1.5  # 默认最大速度 (m/s)

    # 目标容差
    goal_tolerance_distance: 0.1  # 位置容差 (m)
    goal_tolerance_theta: 0.1     # 角度容差 (rad)
```

## 📡 Beta-3 协议支持

### 新增字段

本项目完整支持Beta-3协议的所有新特性：

#### 1. Orientation（运动方向）
- `0.0`: 前向运动
- `3.14` / `-3.14`: 倒车运动

#### 2. Flag（分支标志位）
- `0`: 非进入分支状态
- `1`: 进入分支状态

#### 3. 新增动作类型
- `pub_load_params`: 发布取货参数
- `pub_unload_params`: 发布放货参数
- `start_stacking`: 启动堆垛

### 协议编码示例

轨迹消息通过ROS2 Path的`frame_id`字段编码Beta-3参数：

```
frame_id = "map|action_type|container_type|orientation|flag|x|y|z|theta|width"
```

示例：
```
"map|pub_unload_params|AGV-T300|3.14|1|4.5|1.0|0.1|0.0|1.2"
```

详细说明请参考：[beta3_changes.md](./beta3_changes.md)

## 🧪 测试

### 运行测试脚本

```bash
cd tests/scripts

# 基本功能测试
python3 test_beta3_simple.py

# 完整工作流程测试
python3 test_beta3_trajectory_workflow.py

# 动态轨迹更新测试
python3 test_beta3_dynamic_workflow.py
```

### 测试文件说明

| 测试文件 | 功能 | 配置 |
|---------|------|------|
| `test_beta3_simple.py` | 基本字段验证 | 固定路径 |
| `test_beta3_trajectory_workflow.py` | 完整轨迹流程 | 可配置参数 |
| `test_beta3_dynamic_workflow.py` | 动态参数更新 | 真实Odom |

## 📚 项目文档

- **[Beta-3协议更新记录](./beta3_changes.md)** - 协议变化和实现详情
- **[代码改进记录](./BETA3_CODE_IMPROVEMENTS.md)** - 代码优化和枚举类型
- **[项目状态](./PROJECT_STATUS.md)** - 当前开发状态和重启指南
- **[使用说明](./USAGE.md)** - 详细使用方法

## 🔍 关键特性

### 1. 枚举类型支持

严格的类型定义，避免协议违规：

```cpp
enum class Orientation : int {
    FORWARD = 0,
    BACKWARD_NEG = -314,
    BACKWARD_POS = 314
};

enum class BranchFlag : int {
    NON_BRANCH = 0,
    ENTER_BRANCH = 1
};
```

### 2. 参数化测试配置

测试脚本支持灵活配置：

```python
# 轨迹开关
ENABLE_TRAJECTORY1 = True
ENABLE_TRAJECTORY2 = True

# 轨迹参数
TRAJ1_FORWARD_DISTANCE = 3.0
TRAJ1_FORWARD_POINTS = 10
TRAJ1_RIGHT_TURN_ANGLE = -math.pi / 2
```

### 3. 路径点优化

原地转弯只发布必要的路径点（起点+终点），大幅减少数据量：
- 第一条轨迹: 27点 → **17点** (减少37%)
- 第二条轨迹: 11点 → **5点** (减少54%)

## 🛠️ 开发指南

### 项目结构

```
.
├── CMakeLists.txt          # CMake构建配置
├── package.xml             # ROS2包定义
├── config/                 # 配置文件
│   └── bridge_config.yaml
├── src/
│   ├── include/            # 头文件
│   │   ├── zhongli_protocol_types.hpp
│   │   ├── path_converter.hpp
│   │   └── ros2_zhongli_bridge.hpp
│   └── lib/                # 实现文件
│       ├── path_converter.cpp
│       ├── mqtt_client.cpp
│       └── ros2_zhongli_bridge.cpp
├── tests/
│   └── scripts/            # Python测试脚本
└── docs/                   # 文档
```

### 添加新功能

1. 更新协议定义 (`zhongli_protocol_types.hpp`)
2. 实现转换逻辑 (`path_converter.cpp`)
3. 添加测试用例 (`tests/scripts/`)
4. 更新文档 (`beta3_changes.md`)

## 🐛 故障排查

### 常见问题

**Q: MQTT连接失败**
```bash
# 检查mosquitto服务
sudo systemctl status mosquitto

# 重启服务
sudo systemctl restart mosquitto
```

**Q: TF2变换错误**
```bash
# 检查TF树
ros2 run tf2_ros tf2_echo map base_link

# 发布静态变换（测试用）
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```

**Q: 编译错误**
```bash
# 清理重建
rm -rf build install log
colcon build --packages-select ros2_zhongli_bridge_cpp --symlink-install
```

## 🤝 贡献

欢迎提交Issue和Pull Request！

## 📄 许可证

MIT License

## 📮 联系方式

项目维护者：Developer
- Email: dev@example.com
- 文档问题：请提交Issue

---

**最后更新**: 2025-10-02
**协议版本**: Beta-3
**ROS2版本**: Humble
