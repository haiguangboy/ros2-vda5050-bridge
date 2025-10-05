# ComplexTrajectoryPlanner 使用指南

## 概述

`ComplexTrajectoryPlanner` 是专门设计用于处理复杂多阶段轨迹的规划器，主要用于模拟和实现类似 `test_beta4_trajectory_workflow_goal.py` 中的复杂轨迹模式。

## 设计目标

基于原始测试文件中的轨迹模式：
- **Trajectory 1**: 左转90° → 前进0.5m（flag=0, orientation=0.0）
- **Trajectory 2**: 左转90°（flag=0, orientation=0.0）
- **Trajectory 3**: 倒车0.3m（flag=1, orientation=3.14）

由于控制器执行的原因，需要将规划结果拆分为：
1. **前向轨迹**：Traj1 + Traj2 合并为一条（两次左转 + 前进）
2. **后向轨迹**：Traj3 单独一条（倒车，flag=1）

## 核心功能

### 1. 前向轨迹规划 `plan_forward_with_turns()`

**功能**：规划"左转 → 前进 → 左转"的组合轨迹

**参数**：
```python
def plan_forward_with_turns(
    start_pose: Pose,           # 起点位姿（从Odom获取）
    first_turn_angle: float,    # 第一次转弯角度（弧度，正值=左转）
    forward_distance: float,    # 前进距离（米）
    second_turn_angle: float    # 第二次转弯角度（弧度，正值=左转）
) -> List[Tuple[float, float, float]]
```

**返回**：路径点列表 `[(x, y, yaw), ...]`

**轨迹构成**：
- 阶段1：原地左转（2个点：起点 + 转弯后）
- 阶段2：直线前进（根据步长生成多个点）
- 阶段3：原地左转（1个点：终点朝向）

**示例**：
```python
from trajectory_planner import ComplexTrajectoryPlanner
import math

planner = ComplexTrajectoryPlanner(forward_step=0.15)

# 模拟Traj1 + Traj2: 左转90° → 前进0.5m → 左转90°
waypoints = planner.plan_forward_with_turns(
    start_pose=current_pose,      # 从Odom获取
    first_turn_angle=math.pi/2,   # 90度
    forward_distance=0.5,         # 0.5米
    second_turn_angle=math.pi/2   # 90度
)
```

### 2. 后向轨迹规划 `plan_backward()`

**功能**：规划倒车轨迹（对应Beta-3协议的flag=1, orientation=3.14）

**参数**：
```python
def plan_backward(
    start_pose: Pose,           # 起点位姿（从Odom获取）
    backward_distance: float    # 倒车距离（米）
) -> List[Tuple[float, float, float]]
```

**返回**：路径点列表 `[(x, y, yaw), ...]`

**特点**：
- 朝向保持不变
- 位置沿当前朝向的反方向移动
- 用于Beta-3协议的倒车操作（flag=1）

**示例**：
```python
# 模拟Traj3: 倒车0.3m
waypoints = planner.plan_backward(
    start_pose=current_pose,  # 从Odom获取
    backward_distance=0.3     # 0.3米
)
```

## 使用流程

### 完整示例：基于Odom的实时规划

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from trajectory_planner import ComplexTrajectoryPlanner
import math

class ComplexPlannerNode(Node):
    def __init__(self):
        super().__init__('complex_planner_node')

        # 创建规划器
        self.planner = ComplexTrajectoryPlanner(
            forward_step=0.15,
            backward_step=0.15
        )

        # 订阅Odom话题
        self.odom_sub = self.create_subscription(
            Odometry, '/Odom', self.odom_callback, 10
        )

        self.current_pose = None

    def odom_callback(self, msg):
        """持续更新当前位姿"""
        self.current_pose = msg.pose.pose

    def plan_forward_trajectory(self):
        """规划前向轨迹（Traj1 + Traj2）"""
        if self.current_pose is None:
            print("⚠️  等待Odom数据...")
            return None

        # 规划：左转90° → 前进0.5m → 左转90°
        waypoints = self.planner.plan_forward_with_turns(
            start_pose=self.current_pose,
            first_turn_angle=math.pi / 2,   # 90度
            forward_distance=0.5,
            second_turn_angle=math.pi / 2   # 90度
        )

        return waypoints

    def plan_backward_trajectory(self):
        """规划后向轨迹（Traj3）"""
        if self.current_pose is None:
            print("⚠️  等待Odom数据...")
            return None

        # 规划：倒车0.3m
        waypoints = self.planner.plan_backward(
            start_pose=self.current_pose,
            backward_distance=0.3
        )

        return waypoints

def main():
    rclpy.init()
    node = ComplexPlannerNode()

    # 等待Odom数据
    rclpy.spin_once(node, timeout_sec=1.0)

    # 规划前向轨迹
    print("规划前向轨迹...")
    forward_waypoints = node.plan_forward_trajectory()

    # 这里应该发布轨迹并等待完成...

    # 规划后向轨迹
    print("规划后向轨迹...")
    backward_waypoints = node.plan_backward_trajectory()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 与SimpleTrajectoryPlanner的对比

| 特性 | SimpleTrajectoryPlanner | ComplexTrajectoryPlanner |
|------|------------------------|--------------------------|
| **适用场景** | 点对点导航 | 多阶段复杂轨迹 |
| **输入方式** | 起点 + 目标点 | 起点 + 运动参数 |
| **规划策略** | 自动计算最优路径 | 按指定模式规划 |
| **前向运动** | ✅ 支持 | ✅ 支持（可组合多个阶段） |
| **后向运动** | ❌ 不支持 | ✅ 支持（倒车） |
| **Beta-3协议** | flag=0, orientation自动 | 可指定flag=0/1, orientation=0.0/3.14 |

## 实际应用场景

### 场景1：货架搬运（对应原始测试）

```python
# 第1步：前向接近货架（Traj1 + Traj2）
forward_waypoints = planner.plan_forward_with_turns(
    start_pose=current_pose,
    first_turn_angle=math.pi/2,   # 左转90°对准货架
    forward_distance=0.5,         # 前进到货架前
    second_turn_angle=math.pi/2   # 再左转90°对准取货位置
)
# 发布轨迹（flag=0, orientation=0.0）

# 等待完成并更新Odom...

# 第2步：倒车带货架离开（Traj3）
backward_waypoints = planner.plan_backward(
    start_pose=current_pose,
    backward_distance=0.3  # 倒车0.3m
)
# 发布轨迹（flag=1, orientation=3.14）
```

### 场景2：自定义多阶段轨迹

```python
# 示例：L形路径（左转45° → 前进2m → 左转45°）
waypoints = planner.plan_forward_with_turns(
    start_pose=current_pose,
    first_turn_angle=math.pi/4,   # 45度
    forward_distance=2.0,         # 2米
    second_turn_angle=math.pi/4   # 45度
)
```

## 参数配置

### 初始化参数

```python
planner = ComplexTrajectoryPlanner(
    forward_step=0.15,    # 前向路径点间距（米）
    backward_step=0.15    # 后向路径点间距（米）
)
```

### 路径点间距建议

| 场景 | 建议步长 | 说明 |
|------|---------|------|
| 高速场景 | 0.20-0.30m | 减少路径点数量 |
| 标准场景 | 0.15m | 平衡精度和性能 |
| 精确场景 | 0.05-0.10m | 提高路径精度 |

## 与Beta-3协议的对应关系

### 前向轨迹（flag=0, orientation=0.0）

```python
# 规划前向轨迹
waypoints = planner.plan_forward_with_turns(...)

# 发布时设置Beta-3参数
path.header.frame_id = f"map|none|none|0.0|0|0|0|0|0|0|{trajectory_id}"
#                                        ^^^  ^
#                                orientation  flag
```

### 后向轨迹（flag=1, orientation=3.14）

```python
# 规划后向轨迹
waypoints = planner.plan_backward(...)

# 发布时设置Beta-3参数
path.header.frame_id = f"map|pub_unload_params|{container_type}|3.14|1|..."
#                                                                ^^^^  ^
#                                                            orientation  flag
```

## 调试工具

### 打印路径点

```python
# 打印所有路径点
planner.print_waypoints(waypoints)

# 只打印前10个点
planner.print_waypoints(waypoints, max_points=10)
```

### 可视化路径

```python
import matplotlib.pyplot as plt

def plot_trajectory(waypoints):
    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]

    plt.figure(figsize=(10, 10))
    plt.plot(xs, ys, 'b-o', label='Trajectory')
    plt.scatter(xs[0], ys[0], c='green', s=100, label='Start')
    plt.scatter(xs[-1], ys[-1], c='red', s=100, label='End')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# 使用
plot_trajectory(forward_waypoints)
```

## 常见问题

### Q1: 为什么要分成两条轨迹发布？

**A**: 由于控制器执行的原因，前向运动（flag=0）和倒车运动（flag=1）需要分开处理。前向轨迹可以组合多个阶段，但倒车必须单独一条。

### Q2: 如何确保Odom数据是最新的？

**A**: 在规划前多次调用 `rclpy.spin_once()` 确保Odom更新：

```python
# 主动获取最新Odom
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.05)

# 然后规划
waypoints = planner.plan_forward_with_turns(...)
```

### Q3: 转弯角度为负数代表什么？

**A**:
- 正值：左转（逆时针）
- 负值：右转（顺时针）

```python
# 右转90度
first_turn_angle = -math.pi/2
```

### Q4: 如何调整路径点密度？

**A**: 修改初始化参数：

```python
# 密集路径（0.05m间距）
planner = ComplexTrajectoryPlanner(forward_step=0.05)

# 稀疏路径（0.30m间距）
planner = ComplexTrajectoryPlanner(forward_step=0.30)
```

## 完整工作流程示例

```bash
# 1. 启动MQTT Bridge
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
./start_mqtt_bridge.sh

# 2. 启动规划节点（使用ComplexTrajectoryPlanner）
python3 your_complex_planner_node.py

# 3. 查看轨迹状态
python3 query_trajectory_status.py
```

## 文件清单

- `trajectory_planner.py` - 规划器实现（包含SimpleTrajectoryPlanner和ComplexTrajectoryPlanner）
- `example_complex_planner.py` - 使用示例
- `COMPLEX_PLANNER_GUIDE.md` - 本文档
- `test_beta4_trajectory_workflow_goal.py` - 原始参考实现

## 总结

`ComplexTrajectoryPlanner` 提供了专门的复杂轨迹规划能力：

✅ **前向组合轨迹**：支持"转弯 → 前进 → 转弯"模式
✅ **后向倒车轨迹**：支持Beta-3协议的倒车操作
✅ **灵活参数配置**：可调整步长和运动参数
✅ **Beta-3协议兼容**：正确设置flag和orientation

根据你的具体需求选择合适的规划器！🎯
