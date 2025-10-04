# Service模式对比：我们的实现 vs 标准GoToPose

## 概览对比

| 特性 | 我们的实现 | 标准 GoToPose (forklift_interfaces) |
|------|------------|-------------------------------------|
| **目标点传递** | 通过话题 `/nav_goal` | 通过service请求参数 `target` |
| **通信模式** | Pub/Sub (异步) | Request/Response (同步) |
| **状态查询** | Service `/trajectory_status` | Service response `arrived` |
| **调用方式** | 发布器 → 订阅器 → 规划 | 客户端调用 → 服务器执行 → 返回结果 |
| **适用场景** | 持续规划、多目标点 | 单次导航、等待完成 |

---

## 详细对比

### 1. 标准 GoToPose Service

**定义文件：** `forklift_interfaces/srv/GoToPose.srv`

```
# Request（客户端发送）
uint8 MODE_NORMAL=0
uint8 MODE_FORK=1
uint8 mode                        # 导航模式
geometry_msgs/PoseStamped target  # 目标点（在这里！）
float32 timeout_sec               # 超时时间
geometry_msgs/Pose pallet_pose    # 托盘位置（可选）
geometry_msgs/Vector3 pallet_size # 托盘尺寸（可选）

---
# Response（服务器返回）
bool arrived                      # 是否到达
string message                    # 状态消息
```

**使用方式：**
```python
# 状态机（客户端）
client = node.create_client(GoToPose, '/nav/go_to_pose')

request = GoToPose.Request()
request.mode = GoToPose.Request.MODE_NORMAL
request.target.pose.position.x = 3.0
request.target.pose.position.y = 0.0
request.timeout_sec = 60.0

# 同步调用，阻塞等待
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)

response = future.result()
if response.arrived:
    print("导航成功！")
else:
    print(f"导航失败: {response.message}")
```

**特点：**
- ✅ **目标点在request中** - 客户端明确指定去哪里
- ✅ **同步等待结果** - 阻塞直到到达或失败
- ✅ **简单直接** - 一次调用，一个目标
- ❌ **阻塞式** - 客户端需要等待完成
- ❌ **不适合多目标** - 需要多次调用

---

### 2. 我们的实现（话题 + Service）

**目标点发布（话题）：** `/nav_goal`
```python
# 发布器（调度器）
publisher = node.create_publisher(PoseStamped, '/nav_goal', 10)

goal = PoseStamped()
goal.pose.position.x = 3.0
goal.pose.position.y = 0.0
goal.pose.orientation = ...
publisher.publish(goal)  # 发布后立即返回，不等待
```

**状态查询（Service）：** `/trajectory_status`
```
# Request（空请求）
---
# Response
bool success              # 是否有状态数据
string message            # JSON格式状态信息
{
  "trajectory_id": "...",
  "status": "completed",  # pending/running/completed/failed
  "timestamp": 123456,
  "message": "..."
}
```

**使用方式：**
```python
# 1. 发布目标点（非阻塞）
publisher.publish(goal_pose)

# 2. 定时查询状态
def check_status():
    request = Trigger.Request()
    future = status_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response = future.result()
    if response.success:
        status = json.loads(response.message)
        if status['status'] == 'completed':
            print("轨迹完成！")
```

**特点：**
- ✅ **异步非阻塞** - 发布后立即返回
- ✅ **适合多目标** - 可连续发布多个目标点
- ✅ **状态可查询** - 随时查询执行状态
- ✅ **解耦设计** - 发布和查询分离
- ❌ **需要轮询** - 状态机需要定时查询
- ❌ **复杂度高** - 需要维护状态

---

## 架构对比图

### 标准 GoToPose 模式
```
状态机
  │
  ├─> [Service Call] GoToPose
  │        │
  │        ├─ Request: target pose
  │        │
  │        └─ Response: arrived (阻塞等待)
  │
  └─> 导航执行器
          │
          └─> 底层控制器
```

### 我们的模式（话题 + Service）
```
状态机
  │
  ├─> [Publish] /nav_goal ──────> 轨迹规划节点
  │                                    │
  │                                    ├─> 规划路径
  │                                    │
  │                                    └─> [Publish] /plans
  │                                             │
  │                                             └─> MQTT Bridge
  │                                                      │
  │                                                      └─> EMQX → 底层控制器
  │                                                              │
  │                                                              └─> 状态反馈
  │                                                                      │
  ├─> [定时查询] /trajectory_status Service <─────────────────────────┘
  │        │
  │        └─ Response: JSON status
  │
  └─> 根据状态决定下一步
```

---

## 为什么我们不用标准GoToPose？

### 原因1：架构不同
- **标准模式**：状态机直接控制导航
- **我们的模式**：状态机 → 规划节点 → MQTT Bridge → 底层控制器

### 原因2：异步需求
- 标准GoToPose是**同步阻塞**的，状态机会一直等待
- 我们需要**异步**，状态机可以做其他事情（如监控、日志）

### 原因3：多目标点支持
- 标准模式一次一个目标
- 我们可以连续发布多个目标点，规划节点排队处理

### 原因4：状态监控
- 标准模式只有 `arrived` (bool)
- 我们有详细状态：`pending`, `running`, `completed`, `failed`

---

## 改进建议：创建自己的GoToPose Service

### 方案A：扩展标准GoToPose（推荐）

创建 `/home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner/GoToPoseAsync.srv`

```
# Request - 发起导航请求
uint8 mode
geometry_msgs/PoseStamped target
float32 timeout_sec
---
# Response - 立即返回轨迹ID
bool accepted              # 是否接受请求
string trajectory_id       # 轨迹ID（用于后续查询）
string message             # 接受/拒绝的原因
```

然后保留现有的状态查询Service：
```
# GetTrajectoryStatus.srv
string trajectory_id       # 要查询的轨迹ID
---
bool found
string status              # pending/running/completed/failed
uint64 timestamp
string message
```

**使用方式：**
```python
# 1. 发起导航（非阻塞）
request = GoToPoseAsync.Request()
request.target = goal_pose
response = client.call(request)

if response.accepted:
    trajectory_id = response.trajectory_id

    # 2. 定时查询状态
    while True:
        status = query_status(trajectory_id)
        if status == 'completed':
            break
        time.sleep(0.5)
```

### 方案B：保持现有模式（简单）

**优点：**
- 已经实现，无需修改
- 简单明了
- 话题发布器更灵活

**缺点：**
- 不符合ROS2常见模式
- 需要解释为什么不用标准GoToPose

---

## 最佳实践建议

### 如果你的系统需要：

**使用标准GoToPose（同步）：**
- ✅ 状态机需要等待导航完成
- ✅ 一次一个目标点
- ✅ 简单的到达/失败判断
- ✅ 符合ROS2标准

**使用话题+Service（异步）：**
- ✅ 状态机需要并行处理
- ✅ 连续多个目标点
- ✅ 详细的执行状态
- ✅ 解耦的架构

### 我们的推荐：

**短期**：保持现有模式，工作正常
**长期**：实现 `GoToPoseAsync.srv`，同时保留状态查询Service

这样既符合ROS2习惯，又保持异步特性！

---

## 总结表格

| 对比项 | 标准 GoToPose | 我们的实现 |
|--------|---------------|------------|
| 目标点传递 | Service Request | Topic Publish |
| 通信模式 | 同步阻塞 | 异步非阻塞 |
| 状态反馈 | Response.arrived | Service查询 |
| 多目标支持 | ❌ 需多次调用 | ✅ 连续发布 |
| 状态详细度 | 简单（bool） | 详细（JSON） |
| 复杂度 | 低 | 中 |
| ROS2标准性 | ✅ 标准 | ⚠️ 自定义 |
| 适用场景 | 单次导航 | 持续规划 |

---

## 代码示例对比

### 标准模式（同步）
```python
# 状态机代码
def navigate_to_goal(x, y, yaw):
    request = GoToPose.Request()
    request.target.pose.position.x = x
    request.target.pose.position.y = y
    request.target.pose.orientation = yaw_to_quaternion(yaw)

    # 阻塞等待
    response = client.call(request)
    return response.arrived  # True/False
```

### 我们的模式（异步）
```python
# 状态机代码
def navigate_to_goal(x, y, yaw):
    # 1. 发布目标（立即返回）
    goal = PoseStamped()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation = yaw_to_quaternion(yaw)
    goal_publisher.publish(goal)

    # 2. 返回，状态机继续做其他事
    return True

def check_navigation_status():
    # 定时器回调
    response = status_client.call(Trigger.Request())
    if response.success:
        status = json.loads(response.message)
        if status['status'] == 'completed':
            on_navigation_complete()
```

**结论：两种模式都有各自的优势，选择取决于你的系统需求！**
