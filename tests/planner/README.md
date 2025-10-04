# 基于目标点的轨迹规划系统

## 系统架构

```
调度器 (publish_goal.py) ─────> /nav_goal ─────> 轨迹规划节点 (test_beta4_trajectory_workflow_goal.py)
                                                           │
                                                           ├──> 订阅 /Odom (当前位置)
                                                           ├──> 使用 SimpleTrajectoryPlanner 规划路径
                                                           ├──> 发布 /plans (轨迹路径)
                                                           │         │
                                                           │         └──> MQTT Bridge ──> EMQX ──> 底层控制器
                                                           │                                           │
                                                           │         轨迹状态反馈 <────────────────────┘
                                                           │         │
                                                           └──> 提供 /trajectory_status Service (查询状态)
                                                                      │
状态机 (query_trajectory_status.py) ──────────────────────────────────┘
```

## 文件说明

- **trajectory_planner.py** - 路径规划器核心模块
  - `SimpleTrajectoryPlanner` 类：实现三阶段规划（旋转→直线→旋转）
  - 输入：起点Pose + 目标点Pose
  - 输出：路径点列表 `[(x, y, yaw), ...]`

- **test_trajectory_planner.py** - 规划器单元测试
  - 测试6种场景：直线、转弯、原地旋转、后退等
  - 打印所有路径点用于验证

- **test_beta4_trajectory_workflow_goal.py** - 完整工作流程测试
  - 订阅 `/Odom` 获取当前位置
  - 订阅 `/nav_goal` 获取目标点
  - 规划路径并发布到 `/plans`
  - 监听MQTT轨迹完成状态

- **publish_goal.py** - 目标点发布器（用于测试）
  - 模拟调度器发布导航目标

- **query_trajectory_status.py** - 轨迹状态查询客户端
  - 调用 `/trajectory_status` service 查询最新轨迹状态
  - 用于状态机监控轨迹执行情况

## ROS2 Service接口

### `/trajectory_status` (example_interfaces/srv/Trigger)

**作用：** 状态机查询当前轨迹执行状态

**Request:**
```
# 空请求
```

**Response:**
```python
{
  "success": bool,           # true表示有状态数据，false表示无数据
  "message": str             # JSON格式的状态信息
}
```

**Response message (JSON格式):**
```json
{
  "trajectory_id": "goal_traj_1759568808486",
  "status": "completed",           # pending | running | completed | failed | no_data
  "timestamp": 1759568808486,      # 时间戳（毫秒）
  "message": "轨迹执行完成"
}
```

## 使用方法

### 1. 测试轨迹规划器

```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
python3 test_trajectory_planner.py
```

### 2. 运行完整工作流程

**⚠️ 重要：必须先启动MQTT Bridge！**

**终端1：启动MQTT Bridge**
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
./start_mqtt_bridge.sh
```

**终端2：启动轨迹规划节点**
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
python3 test_beta4_trajectory_workflow_goal.py
```

**终端3：发布目标点**
```bash
# 发布目标点: (3.0, 0.0), 朝向90度
python3 publish_goal.py --x 3.0 --y 0.0 --yaw 90

# 或自定义目标点
python3 publish_goal.py --x 5.0 --y 0.0 --yaw 180
```

**终端3（可选）：模拟Odom发布器**
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/scripts
python3 mock_nav_goal_server.py --odom-x 1.0 --odom-y 0.0 --odom-yaw 0.3
```

### 3. 查询轨迹状态（状态机使用）

**在任意终端：**
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
python3 query_trajectory_status.py
```

**示例输出：**
```
📞 调用 /trajectory_status service...

================================================================================
📊 轨迹状态查询结果
================================================================================
✅ 查询成功

状态信息:
  轨迹ID: goal_traj_1759568808486
  状态: completed
  时间戳: 1759568808486
  消息: 轨迹执行完成

✅ 轨迹已完成
================================================================================
```

## 配置参数

编辑 `test_beta4_trajectory_workflow_goal.py` 修改配置：

```python
# MQTT配置
MQTT_BROKER = "192.168.1.102"
MQTT_PORT = 1883
ROBOT_ID = "robot-001"

# ROS2话题
ODOM_TOPIC = "/Odom"
PATH_TOPIC = "/plans"
GOAL_TOPIC = "/nav_goal"

# 路径规划配置
PLANNER_STEP_SIZE = 0.15  # 路径点间距（米）
```

## 路径规划策略

`SimpleTrajectoryPlanner` 采用三阶段规划：

1. **阶段1：旋转朝向目标位置**
   - 如果角度差 > 0.1 rad (5.7°)，原地旋转朝向目标
   - 生成2个点：起始朝向 → 目标方向

2. **阶段2：直线移动到目标位置**
   - 按固定步长（默认0.15m）生成路径点
   - 保持朝向不变

3. **阶段3：旋转到目标朝向**
   - 如果角度差 > 0.1 rad，原地旋转到目标朝向
   - 生成2个点：当前朝向 → 目标朝向

## 示例输出

```
📋 路径规划开始:
   起点: (1.000, 0.000), yaw=0.300 (17.2°)
   终点: (3.000, 2.000), yaw=1.571 (90.0°)
   直线距离: 2.236m

   阶段1: 原地旋转 46.0° 朝向目标位置
   阶段2: 直线移动 2.236m (点间距0.15m, 15个点)
   阶段3: 原地旋转 27.0° 到目标朝向
   ✅ 规划完成: 共 18 个路径点
```

## MQTT消息流

1. **轨迹发布** → `EP/{robot_id}/embrain/cerebellum/trajectory`
2. **轨迹状态** ← `EP/{robot_id}/cerebellum/embrain/trajectory_status`
   - `status`: `pending` | `running` | `completed` | `failed`

### 工作流程

```
1. 状态机发布目标点 → /nav_goal
2. 轨迹规划节点接收 → 规划路径 → 发布到 /plans
3. MQTT Bridge 转发 → EMQX → 底层控制器
4. 底层控制器执行轨迹 → 返回状态到 EMQX
5. MQTT Bridge 接收状态 → 轨迹规划节点存储状态
6. 状态机调用 /trajectory_status service → 查询轨迹状态
```

## 注意事项

1. 确保MQTT broker (EMQX) 运行在 `192.168.1.102:1883`
2. `/Odom` 话题必须持续发布当前位置
3. 目标点通过 `/nav_goal` 话题发布（`geometry_msgs/PoseStamped`）
4. 原地旋转只生成起点和终点，中间不插值
5. 路径点间距可通过 `PLANNER_STEP_SIZE` 调整
