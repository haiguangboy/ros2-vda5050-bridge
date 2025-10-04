# 导航服务集成说明

## 功能概述

本集成为 `test_beta3_trajectory_workflow_king.py` 添加了以下功能：

1. **GoToPose 服务客户端**：订阅外部发布的导航目标点
2. **TrajectoryComplete 服务发布器**：在每条轨迹完成后发布通知

## 工作流程

### 轨迹规划流程

1. **第0条轨迹**：
   - 起点：从 `/Odom` 话题获取当前位置
   - 终点：从第1个 GoToPose 服务调用获取目标点
   - 生成从起点到终点的直线路径

2. **第1条轨迹**：
   - 起点：从 `/Odom` 话题获取当前位置（第0条轨迹执行完后的新位置）
   - 终点：从第2个 GoToPose 服务调用获取目标点
   - 生成从起点到终点的直线路径

3. **第2条轨迹**：
   - 保持原有逻辑（倒车）
   - 起点：从 `/Odom` 话题获取当前位置

### 轨迹完成通知

每条轨迹执行完成后（收到MQTT的completed状态），会通过以下方式发布完成通知：
- 话题：`/trajectory/complete`
- 消息类型：`std_msgs/String` (JSON格式)
- 消息内容：`{"trajectory_id": "xxx", "trajectory_index": 0, "timestamp": 123456.789}`

## 服务定义

### TrajectoryComplete.srv

```
# Request
string trajectory_id
uint32 trajectory_index
---
# Response
bool acknowledged
string message
```

## 使用方法

### 运行测试脚本

```bash
python3 test_beta3_trajectory_workflow_king.py
```

### 配置GoToPose服务

脚本会尝试调用 `/nav/go_to_pose` 服务来获取目标点。如果服务不可用，将使用默认配置生成轨迹。

可以使用 `mock_nav_goal_server.py` 来模拟GoToPose服务器：

```bash
python3 mock_nav_goal_server.py --target-x 3.0 --target-y 0.0 --target-yaw 0.0
```

## 注意事项

1. 当前 `TrajectoryComplete` 通知使用 `std_msgs/String` 话题模拟，实际使用时需要：
   - 将 `TrajectoryComplete.srv` 添加到 `forklift_interfaces` 包中
   - 编译服务接口：`colcon build --packages-select forklift_interfaces`
   - 修改代码使用真正的服务调用

2. 每条轨迹发布前会自动更新当前位置（从 `/Odom` 话题）

3. GoToPose 服务调用超时时间为5秒，可在代码中调整
