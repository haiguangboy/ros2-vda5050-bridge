# ComplexTrajectoryPlanner - 完整说明

## 📋 快速开始

### 最简单的测试方式（推荐）

**选项1：完整集成测试（需要MQTT Bridge）**

```bash
# 终端1：启动MQTT Bridge
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
./start_mqtt_bridge.sh

# 终端2：运行测试
./run_complex_planner_test.sh
```

**选项2：纯规划器测试（不需要MQTT）**

```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
python3 example_complex_planner.py
```

---

## 🎯 功能概述

`ComplexTrajectoryPlanner` 是专门用于复杂多阶段轨迹规划的工具，设计目标是模拟和实现类似原始测试文件中的复杂轨迹模式。

### 核心能力

1. **前向组合轨迹**：左转 → 前进 → 左转（对应Traj1 + Traj2）
2. **后向倒车轨迹**：沿当前朝向反方向倒车（对应Traj3）
3. **Beta-3协议兼容**：正确设置flag和orientation

### 与原始测试的对应关系

| 原始测试 | ComplexTrajectoryPlanner | Beta-3参数 |
|---------|-------------------------|-----------|
| Trajectory 1 + 2 | `plan_forward_with_turns()` | flag=0, orientation=0.0 |
| Trajectory 3 | `plan_backward()` | flag=1, orientation=3.14 |

---

## 📁 文件结构

```
mqtt_bridge/tests/planner/
├── trajectory_planner.py              # 规划器实现（包含2个规划器）
│   ├── SimpleTrajectoryPlanner        # 简单点对点规划器
│   └── ComplexTrajectoryPlanner       # 复杂多阶段规划器 ⭐
│
├── test_complex_planner_workflow.py   # 完整工作流程测试 ⭐
├── example_complex_planner.py         # 简单示例（纯规划测试）
├── run_complex_planner_test.sh        # 一键测试脚本 ⭐
│
├── COMPLEX_PLANNER_GUIDE.md           # 详细使用指南
├── TEST_COMPLEX_PLANNER.md            # 测试指南
└── README_COMPLEX_PLANNER.md          # 本文件
```

---

## 🚀 使用方法

### 方法1：在代码中使用

```python
from trajectory_planner import ComplexTrajectoryPlanner
import math

# 创建规划器
planner = ComplexTrajectoryPlanner(
    forward_step=0.15,     # 前向步长
    backward_step=0.15     # 后向步长
)

# 规划前向轨迹
forward_waypoints = planner.plan_forward_with_turns(
    start_pose=current_pose,        # 从Odom获取
    first_turn_angle=math.pi/2,     # 左转90°
    forward_distance=0.5,           # 前进0.5m
    second_turn_angle=math.pi/2     # 再左转90°
)

# 规划后向轨迹
backward_waypoints = planner.plan_backward(
    start_pose=current_pose,        # 从Odom获取
    backward_distance=0.3           # 倒车0.3m
)
```

### 方法2：运行测试节点

```python
# test_complex_planner_workflow.py 会自动：
# 1. 订阅 /Odom 获取当前位置
# 2. 规划前向轨迹
# 3. 发布到 /plans 话题
# 4. 等待MQTT完成信号
# 5. 规划后向轨迹
# 6. 发布到 /plans 话题
# 7. 提供 /trajectory_status service
```

---

## 📊 测试场景

### 场景1：完整工作流程测试

**目标**：测试从规划到MQTT的完整流程

**步骤**：
```bash
# 1. 启动MQTT Bridge
./start_mqtt_bridge.sh

# 2. 启动测试节点
./run_complex_planner_test.sh

# 3. （可选）查询状态
python3 query_trajectory_status.py
```

**预期结果**：
- ✅ 规划生成7个前向路径点
- ✅ 规划生成3个后向路径点
- ✅ 轨迹成功发布到/plans
- ✅ MQTT Bridge收到并转发
- ✅ 可查询轨迹状态

### 场景2：纯规划器测试

**目标**：快速验证规划算法

**步骤**：
```bash
python3 example_complex_planner.py
```

**预期结果**：
- ✅ 打印前向轨迹规划详情
- ✅ 打印后向轨迹规划详情
- ✅ 显示所有路径点坐标

### 场景3：自定义参数测试

**修改参数**：编辑 `test_complex_planner_workflow.py`

```python
# 修改这些参数来测试不同的轨迹
FIRST_TURN_ANGLE = math.pi / 4    # 改为45度
FORWARD_DISTANCE = 1.0            # 改为1米
SECOND_TURN_ANGLE = math.pi / 4   # 改为45度
BACKWARD_DISTANCE = 0.5           # 改为0.5米
```

---

## 🔧 API文档

### ComplexTrajectoryPlanner 类

#### 初始化

```python
planner = ComplexTrajectoryPlanner(
    forward_step=0.15,    # 前向路径点间距（米）
    backward_step=0.15    # 后向路径点间距（米）
)
```

#### 方法1: plan_forward_with_turns()

**功能**：规划"左转 → 前进 → 左转"组合轨迹

**签名**：
```python
def plan_forward_with_turns(
    start_pose: Pose,           # 起点位姿
    first_turn_angle: float,    # 第一次转弯角度（弧度，正=左转）
    forward_distance: float,    # 前进距离（米）
    second_turn_angle: float    # 第二次转弯角度（弧度，正=左转）
) -> List[Tuple[float, float, float]]
```

**返回**：`[(x, y, yaw), ...]` - 路径点列表

**示例**：
```python
# 左转90° → 前进0.5m → 左转90°
waypoints = planner.plan_forward_with_turns(
    start_pose=current_pose,
    first_turn_angle=math.pi/2,
    forward_distance=0.5,
    second_turn_angle=math.pi/2
)
```

#### 方法2: plan_backward()

**功能**：规划倒车轨迹

**签名**：
```python
def plan_backward(
    start_pose: Pose,           # 起点位姿
    backward_distance: float    # 倒车距离（米）
) -> List[Tuple[float, float, float]]
```

**返回**：`[(x, y, yaw), ...]` - 路径点列表

**示例**：
```python
# 倒车0.3m
waypoints = planner.plan_backward(
    start_pose=current_pose,
    backward_distance=0.3
)
```

#### 方法3: print_waypoints()

**功能**：打印路径点详情（调试用）

**签名**：
```python
def print_waypoints(
    waypoints: List[Tuple[float, float, float]],
    max_points: int = None    # 最多打印几个点，None=全部
)
```

---

## 📖 与SimpleTrajectoryPlanner对比

| 特性 | SimpleTrajectoryPlanner | ComplexTrajectoryPlanner |
|------|------------------------|--------------------------|
| **输入方式** | 起点 + 目标点 | 起点 + 运动参数 |
| **规划策略** | 自动计算最优路径 | 按指定模式规划 |
| **前向运动** | ✅ 支持（自动优化） | ✅ 支持（固定模式） |
| **后向运动** | ❌ 不支持 | ✅ 支持（倒车） |
| **多阶段** | ❌ 单次规划 | ✅ 支持组合 |
| **适用场景** | 点对点导航 | 复杂多阶段动作 |
| **Beta-3协议** | flag=0, orientation自动 | 可指定flag=0/1 |

**选择建议**：
- **使用SimpleTrajectoryPlanner**：当你有明确的目标点，希望系统自动规划最优路径
- **使用ComplexTrajectoryPlanner**：当你需要执行特定的动作序列（如接近货架、倒车等）

---

## 🐛 故障排查

### 问题1: "ModuleNotFoundError: No module named 'trajectory_planner'"

**原因**：不在正确的目录

**解决**：
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
python3 test_complex_planner_workflow.py
```

### 问题2: MQTT连接失败

**原因**：EMQX未启动或MQTT_BROKER配置错误

**解决**：
```bash
# 检查EMQX
sudo systemctl status emqx

# 启动EMQX
sudo systemctl start emqx

# 或修改配置
# 编辑 test_complex_planner_workflow.py
MQTT_BROKER = "192.168.1.102"  # 改为你的EMQX地址
```

### 问题3: 查询状态返回 no_data

**原因**：MQTT Bridge未运行

**解决**：
```bash
# 检查
ps aux | grep zhongli_bridge_node | grep -v grep

# 启动
./start_mqtt_bridge.sh
```

### 问题4: 轨迹发布但底层不执行

**原因**：这是测试环境的正常现象（没有真实底层控制器）

**解决**：
- 测试环境可忽略
- 生产环境需连接真实控制器

---

## 📚 相关文档

- **COMPLEX_PLANNER_GUIDE.md** - 详细使用指南（包含完整示例代码）
- **TEST_COMPLEX_PLANNER.md** - 测试指南（详细测试步骤）
- **ARCHITECTURE_SUMMARY.md** - 架构总结（系统设计说明）
- **SERVICE_COMPARISON.md** - Service模式对比
- **QUICK_START.md** - 快速启动指南（SimpleTrajectoryPlanner）

---

## 🎓 学习路径

### 初学者

1. ✅ 运行简单示例：`python3 example_complex_planner.py`
2. ✅ 阅读输出，理解路径点生成逻辑
3. ✅ 修改参数，观察路径点变化
4. ✅ 阅读 `COMPLEX_PLANNER_GUIDE.md`

### 进阶用户

1. ✅ 启动完整测试：`./run_complex_planner_test.sh`
2. ✅ 观察MQTT消息流
3. ✅ 查询轨迹状态
4. ✅ 修改测试参数
5. ✅ 集成到自己的节点

### 专家用户

1. ✅ 阅读源码：`trajectory_planner.py`
2. ✅ 自定义规划器
3. ✅ 集成到状态机
4. ✅ 连接真实控制器

---

## 💡 最佳实践

### 1. 路径点密度

```python
# 高速场景：减少路径点
planner = ComplexTrajectoryPlanner(forward_step=0.30)

# 标准场景：平衡精度和性能
planner = ComplexTrajectoryPlanner(forward_step=0.15)

# 精确场景：提高路径精度
planner = ComplexTrajectoryPlanner(forward_step=0.05)
```

### 2. 获取最新Odom

```python
# 规划前主动获取最新Odom
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.05)

# 然后规划
waypoints = planner.plan_forward_with_turns(...)
```

### 3. 错误处理

```python
try:
    waypoints = planner.plan_forward_with_turns(...)
    if len(waypoints) == 0:
        print("警告：路径为空")
except Exception as e:
    print(f"规划失败: {e}")
```

### 4. 调试技巧

```python
# 打印路径点验证
planner.print_waypoints(waypoints, max_points=10)

# 可视化（需要matplotlib）
import matplotlib.pyplot as plt
xs, ys = zip(*[(p[0], p[1]) for p in waypoints])
plt.plot(xs, ys, 'b-o')
plt.show()
```

---

## 🔄 版本历史

- **v1.0** (2025-10) - 初始版本
  - 实现 `ComplexTrajectoryPlanner` 类
  - 支持前向组合轨迹和后向倒车轨迹
  - 提供完整测试框架
  - Beta-3协议兼容

---

## 📞 获取帮助

如果遇到问题：

1. 查看 `TEST_COMPLEX_PLANNER.md` 的故障排查章节
2. 查看 `COMPLEX_PLANNER_GUIDE.md` 的常见问题
3. 运行诊断工具：`python3 diagnose_mqtt.py`
4. 检查系统日志

---

## 🎯 总结

ComplexTrajectoryPlanner 提供了：

✅ **专门的复杂轨迹规划能力**（组合转弯+前进+转弯）
✅ **后向倒车支持**（Beta-3协议flag=1）
✅ **完整的测试框架**（一键测试脚本）
✅ **详细的文档**（使用指南 + 测试指南）
✅ **灵活的参数配置**（可调步长和运动参数）

开始你的测试吧！🚀

```bash
./run_complex_planner_test.sh
```
