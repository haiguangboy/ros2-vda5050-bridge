# 轨迹规划器测试指南

本文档介绍如何使用测试脚本对轨迹规划器进行独立测试和完整流程测试。

## 📁 文件说明

### 核心文件
- `unified_planner_workflow.py` - 统一轨迹规划器主程序
- `trajectory_planner.py` - 轨迹规划算法实现

### 测试文件
- `test_independent.py` - **推荐** 独立测试脚本，支持单独测试各个轨迹
- `test_unload.py` - 完整流程测试（观察点 → 取货点 → 卸货点）
- `test_goto_service.py` - 基础测试（观察点 → 取货点）
- `test_goto_service_3.py` - 三点测试（观察点1 → 观察点2 → 取货点）

---

## 🚀 快速开始

### 1. 启动规划器
首先在终端1启动统一轨迹规划器：
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
python3 unified_planner_workflow.py
```

启动后会显示配置信息：
```
🚀 统一轨迹规划器
================================================================================
📋 轨迹开关配置：
  观察点轨迹: ✅ 启用
  取货轨迹:   ✅ 启用
    ├─ 误差消除: ✅ 启用
    └─ 倒车距离: 0.6米
  卸货轨迹:   ✅ 启用
    └─ 主干道y: 0.0米
================================================================================
```

### 2. 运行测试
在终端2运行测试脚本（根据需要选择）：

#### 方式1：独立测试（推荐）
```bash
# 只测试观察点轨迹（最快）
python3 test_independent.py observation

# 只测试取货点轨迹
python3 test_independent.py pickup

# 只测试卸货点轨迹（需先完成取货点测试）
python3 test_independent.py unload

# 完整流程测试
python3 test_independent.py full
# 或直接运行（默认为完整流程）
python3 test_independent.py
```

#### 方式2：传统测试脚本
```bash
# 基础测试：观察点 + 取货点
python3 test_goto_service.py

# 完整流程：观察点 + 取货点 + 卸货点
python3 test_unload.py

# 三点测试：观察点1 + 观察点2 + 取货点
python3 test_goto_service_3.py
```

---

## 🎯 独立测试详解（test_independent.py）

### 测试模式

| 模式 | 命令 | 测试内容 | 依赖 |
|------|------|----------|------|
| `observation` | `python3 test_independent.py observation` | 观察点轨迹<br>SimpleTrajectoryPlanner | 无 |
| `pickup` | `python3 test_independent.py pickup` | 取货点轨迹<br>ComplexTrajectoryPlanner<br>误差消除轨迹 | 无 |
| `unload` | `python3 test_independent.py unload` | 卸货点轨迹（3段）<br>- 向前回主干道<br>- 右转+沿主干道前进<br>- 左转+倒车到卸货点 | **需要先完成 pickup 测试** |
| `full` | `python3 test_independent.py full`<br>或<br>`python3 test_independent.py` | 完整流程<br>观察点 → 取货点 → 卸货点 | 无 |

### 测试点位配置

测试点位在 `test_independent.py` 顶部的 `TEST_POINTS` 字典中配置：

```python
TEST_POINTS = {
    'observation': {
        'x': 3.0,
        'y': 0.0,
        'yaw_deg': -90,
        'mode': 'NORMAL',
        'description': '观察点'
    },
    'pickup': {
        'x': 4.0,
        'y': -1.0,
        'yaw_deg': 90,
        'mode': 'FORK',
        'description': '取货点'
    },
    'unload': {
        'x': 1.0,
        'y': 2.0,
        'yaw_deg': -90,
        'mode': 'NORMAL',
        'description': '卸货点'
    }
}
```

### 典型使用场景

#### 场景1：开发/调试观察点轨迹
```bash
# 1. 修改 unified_planner_workflow.py 中的配置
#    ENABLE_OBSERVATION_TRAJECTORY = True
#    ENABLE_PICKUP_TRAJECTORY = False      # 禁用取货，节省时间
#    ENABLE_UNLOAD_TRAJECTORY = False      # 禁用卸货

# 2. 启动规划器
python3 unified_planner_workflow.py

# 3. 运行测试
python3 test_independent.py observation
```

#### 场景2：开发/调试取货点轨迹
```bash
# 1. 修改配置
#    ENABLE_OBSERVATION_TRAJECTORY = False  # 可选
#    ENABLE_PICKUP_TRAJECTORY = True
#    ENABLE_CORRECTION_TRAJECTORY = True    # 测试误差消除
#    ENABLE_UNLOAD_TRAJECTORY = False

# 2. 启动规划器
python3 unified_planner_workflow.py

# 3. 运行测试
python3 test_independent.py pickup
```

#### 场景3：开发/调试卸货点轨迹
```bash
# 1. 修改配置
#    ENABLE_PICKUP_TRAJECTORY = True        # 必须启用，用于设置上下文
#    ENABLE_UNLOAD_TRAJECTORY = True

# 2. 启动规划器
python3 unified_planner_workflow.py

# 3. 先运行取货点测试（设置 last_mode）
python3 test_independent.py pickup

# 4. 取货完成后，运行卸货点测试
python3 test_independent.py unload
```

#### 场景4：完整集成测试
```bash
# 1. 启用所有轨迹
#    ENABLE_OBSERVATION_TRAJECTORY = True
#    ENABLE_PICKUP_TRAJECTORY = True
#    ENABLE_CORRECTION_TRAJECTORY = True
#    ENABLE_UNLOAD_TRAJECTORY = True

# 2. 启动规划器
python3 unified_planner_workflow.py

# 3. 运行完整流程测试
python3 test_independent.py full
# 或
python3 test_unload.py
```

---

## ⚙️ 配置说明

### unified_planner_workflow.py 配置参数

在文件顶部的配置区域：

```python
# ==================== 轨迹开关配置 ====================
# 观察点轨迹配置
ENABLE_OBSERVATION_TRAJECTORY = True  # 是否启用观察点轨迹（SimpleTrajectoryPlanner）

# 取货轨迹配置
ENABLE_PICKUP_TRAJECTORY = True  # 是否启用取货轨迹（ComplexTrajectoryPlanner）
ENABLE_CORRECTION_TRAJECTORY = True  # 是否启用误差消除轨迹（观察点完成后回正+倒车）
CORRECTION_BACKWARD_DISTANCE = 0.6   # 误差消除轨迹的倒车距离（米）

# 卸货轨迹配置
ENABLE_UNLOAD_TRAJECTORY = True  # 是否启用卸货轨迹（叉取完成后返回主干道并送到卸货点）
MAIN_ROAD_Y = 0.0  # 主干道的y坐标（米）

# 默认位置（Odom超时时使用）
DEFAULT_X = 0.0
DEFAULT_Y = 0.0
DEFAULT_YAW = 0.0

# 等待的时间
WAIT_TIME = 0.1
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `ENABLE_OBSERVATION_TRAJECTORY` | bool | True | 启用/禁用观察点轨迹 |
| `ENABLE_PICKUP_TRAJECTORY` | bool | True | 启用/禁用取货轨迹 |
| `ENABLE_CORRECTION_TRAJECTORY` | bool | True | 启用/禁用误差消除轨迹（取货的子功能） |
| `CORRECTION_BACKWARD_DISTANCE` | float | 0.6 | 误差消除时的倒车距离（米） |
| `ENABLE_UNLOAD_TRAJECTORY` | bool | True | 启用/禁用卸货轨迹 |
| `MAIN_ROAD_Y` | float | 0.0 | 主干道的y坐标（米） |
| `WAIT_TIME` | float | 0.1 | 轨迹段之间的等待时间（秒） |

---

## 📊 轨迹类型详解

### 1. 观察点轨迹（MODE_NORMAL）

**规划器**: SimpleTrajectoryPlanner

**策略**:
- 阶段1: 原地旋转朝向目标位置
- 阶段2: 直线移动到目标位置
- 阶段3: 原地旋转到目标朝向

**测试命令**:
```bash
python3 test_independent.py observation
```

**日志输出示例**:
```
✅ 接受为观察点（MODE_NORMAL）
规划策略: SimpleTrajectoryPlanner

📋 路径规划开始:
   起点: (0.000, 0.000), yaw=0.000 (0.0°)
   终点: (3.000, 0.000), yaw=-1.571 (-90.0°)
   直线距离: 3.000m
   阶段1: 原地旋转 0.0° 朝向目标位置
   阶段2: 直线移动 3.000m
   阶段3: 原地旋转 -90.0° 到目标朝向
   ✅ 规划完成: 共 22 个路径点
```

---

### 2. 取货点轨迹（MODE_FORK）

**规划器**: ComplexTrajectoryPlanner

**策略（启用误差消除）**:
- **第1段**: 回正 + 倒车0.6米（误差消除）
- **第2段**: 转弯 + 前进 + 转弯 + 倒车到取货点

**策略（禁用误差消除）**:
- **单段**: 转弯 + 前进 + 转弯 + 倒车到取货点

**测试命令**:
```bash
python3 test_independent.py pickup
```

**日志输出示例**:
```
✅ 接受为叉取点（MODE_FORK）
规划策略: 误差消除轨迹 + ComplexTrajectoryPlanner

📋 误差消除轨迹规划:
   当前位置: (3.000, 0.000), yaw=-1.571 (-90.0°)
   步骤1: 回正到 yaw=0.000 (0.0°)
   步骤2: 倒车 0.600m
   ✅ 误差消除轨迹完成

📤 第1段轨迹（误差消除）已发布
⏳ 等待MQTT完成信号...

📋 复杂前向轨迹规划:
   起点: (3.000, -0.600), yaw=0.000 (0.0°)
   第一次左转: 90.0°
   前进距离: 1.600m
   第二次左转: 0.0°
   ✅ 规划完成

📤 第2段轨迹（前向）已发布
📤 第3段轨迹（后向）已发布
```

---

### 3. 卸货点轨迹（MODE_NORMAL，取货后）

**规划器**: 组合使用 ComplexTrajectoryPlanner 和 plan_forward_with_turns

**策略（3段）**:

#### 第1段：向前行驶回主干道
- 从取货点 (4, -1, yaw=90°) 沿车头方向前进到主干道 (4, 0)
- 使用 `plan_forward_with_turns(0, forward_distance, 0)`
- orientation=0.0（前向模式）

#### 第2段：右转 + 沿主干道前进
- 原地右转90°（yaw: 90° → 180°）
- 沿-x方向前进到卸货点的x坐标
- 使用 `plan_forward_with_turns(π/2, forward_distance, 0)`
- orientation=0.0（前向模式）

#### 第3段：左转 + 倒车到卸货点
- 原地左转90°（yaw: 180° → -90°）
- 倒车到卸货点 (1, 2, yaw=-90°)
- 使用 `plan_backward(backward_distance)`
- orientation=3.14（倒车模式）+ flag=1（卸货动作）

**测试命令**:
```bash
# 必须先完成取货点测试
python3 test_independent.py pickup
# 然后测试卸货点
python3 test_independent.py unload
```

**日志输出示例**:
```
✅ 检测到取货后的目标点，触发卸货轨迹（3段）
规划策略: 卸货轨迹

🚛 规划卸货轨迹 - 第1段：向前行驶回主干道
📐 向前行驶距离: 1.000米
   起点: (4.000, -1.000)
   终点: (4.000, 0.000)
✅ 第1段轨迹生成完成

🚛 规划卸货轨迹 - 第2段：右转 + 沿主干道前进
📐 右转90° + 沿主干道前进 3.000米
✅ 第2段轨迹生成完成

🚛 规划卸货轨迹 - 第3段：左转 + 倒车到卸货点
📐 倒车距离: 2.000米
✅ 第3段轨迹生成完成

🎉 卸货轨迹全部完成！
```

---

## ⚠️ 注意事项

### 1. 卸货轨迹的上下文依赖
卸货轨迹的触发条件是：
```python
mode == MODE_NORMAL and last_mode == MODE_FORK
```

这意味着：
- 必须先完成取货点测试（MODE_FORK）
- 然后才能测试卸货点（MODE_NORMAL）
- 如果直接测试卸货点，会被当作普通观察点处理

### 2. 测试环境 vs 生产环境
测试环境中，`/Odom` 是通过轨迹终点自动更新的（模拟）：
```python
# TODO: 生产环境有真实Odom时，注释掉下面这行
if hasattr(self, 'unload_stage1_waypoints'):
    self.update_odom_from_trajectory_end(self.unload_stage1_waypoints)
```

生产环境中，应该使用真实的 `/Odom` 话题数据。

### 3. 配置同步
建议测试脚本中的配置与 `unified_planner_workflow.py` 保持一致：
- `ENABLE_CORRECTION_TRAJECTORY`
- `ENABLE_UNLOAD_TRAJECTORY`

### 4. 坐标系约定
- 右手坐标系，z轴向上
- 右转：yaw + π/2
- 角度范围：[-π, π]

---

## 🐛 常见问题

### Q1: 运行测试时提示 "Service 未就绪"
**A**: 需要先在另一个终端启动 `unified_planner_workflow.py`

### Q2: 卸货点测试失败，被当作观察点处理
**A**: 需要先运行取货点测试以设置 `last_mode = MODE_FORK`，或者使用完整流程测试

### Q3: 轨迹执行超时
**A**: 检查：
1. MQTT 连接是否正常
2. `/Odom` 话题是否有数据
3. 超时时间是否足够（可在测试脚本中调整 `timeout_sec`）

### Q4: 如何修改测试点位
**A**: 编辑 `test_independent.py` 顶部的 `TEST_POINTS` 字典

### Q5: 如何禁用某个轨迹
**A**: 在 `unified_planner_workflow.py` 中设置对应的开关为 False：
```python
ENABLE_OBSERVATION_TRAJECTORY = False  # 禁用观察点
ENABLE_PICKUP_TRAJECTORY = False       # 禁用取货点
ENABLE_UNLOAD_TRAJECTORY = False       # 禁用卸货点
```

---

## 📝 日志解读

### 成功的轨迹执行日志
```
📤 发送目标点: (3.0, 0.0, -90°)
   模式: NORMAL
   超时: 600.0秒

================================================================================
📥 收到响应
================================================================================
arrived: True
message: 观察点已到达
================================================================================
```

### 失败的轨迹执行日志
```
arrived: False
message: 观察点轨迹未启用
```
或
```
arrived: False
message: 观察点执行超时
```

### 调试信息
启动时会显示调试信息：
```
🔍 调试信息:
   当前模式: 0 (NORMAL)
   上一次模式: None
   卸货轨迹启用: True
```

---

## 🔧 高级用法

### 自定义测试点位
创建自己的测试脚本，例如 `test_custom.py`：

```python
import rclpy
from test_independent import GoToPoseClient, GoToPose

rclpy.init()
client = GoToPoseClient()

# 自定义测试点
response = client.send_goal(
    x=5.0,
    y=2.0,
    yaw_deg=45,
    mode=GoToPose.Request.MODE_NORMAL,
    timeout_sec=300.0
)

print(f"测试结果: {'成功' if response.arrived else '失败'}")

client.destroy_node()
rclpy.shutdown()
```

### 批量测试
创建测试脚本循环测试多个点位：

```bash
#!/bin/bash
# batch_test.sh

echo "开始批量测试..."

# 测试观察点
python3 test_independent.py observation
if [ $? -ne 0 ]; then
    echo "观察点测试失败"
    exit 1
fi

# 测试取货点
python3 test_independent.py pickup
if [ $? -ne 0 ]; then
    echo "取货点测试失败"
    exit 1
fi

# 测试卸货点
python3 test_independent.py unload
if [ $? -ne 0 ]; then
    echo "卸货点测试失败"
    exit 1
fi

echo "所有测试通过！"
```

---

## 📚 相关文档

- `unified_planner_workflow.py` - 主程序实现
- `trajectory_planner.py` - 轨迹规划算法
- `forklift_interfaces/srv/GoToPose.srv` - Service 接口定义

---

## 📧 反馈与贡献

如有问题或建议，请联系开发团队或提交 Issue。

**最后更新**: 2025-10-09
