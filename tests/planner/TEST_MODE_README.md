# TEST_MODE 配置说明

## ⚠️ 重要：测试环境 vs 生产环境

### 问题背景
在开发过程中发现一个重要问题：

- **测试环境**：没有真实机器人，需要手动模拟 `/Odom` 更新
- **生产环境**：有真实机器人，会自动发布真实的 `/Odom` 数据

如果两种模式混用，会导致 Odom 数据冲突！

### 解决方案
在 `unified_planner_workflow.py` 中添加 `TEST_MODE` 配置开关：

```python
# ==================== 环境配置 ====================
# 测试环境 vs 生产环境
TEST_MODE = True  # True=测试环境（模拟Odom更新），False=生产环境（使用真实Odom）
```

---

## 🧪 测试模式（TEST_MODE = True）

### 适用场景
- 本地开发测试
- 没有真实机器人
- 使用测试脚本验证轨迹规划逻辑

### 工作原理
轨迹完成后，会调用 `update_odom_from_trajectory_end()` 方法：
1. 读取轨迹终点位置 `(x, y, yaw)`
2. 创建 Odometry 消息
3. 发布到 `/Odom` 话题
4. 下一段轨迹从这个位置开始规划

### 代码示例
```python
# 测试环境：模拟Odom更新
if TEST_MODE and hasattr(self, 'unload_stage1_waypoints'):
    self.update_odom_from_trajectory_end(self.unload_stage1_waypoints)
```

### 日志输出
```
✅ 卸货第1段（向前行驶回主干道）已完成
📡 更新/Odom: (4.000, 0.000), yaw=1.571 (90.0°)
   (测试模式：轨迹终点 → /Odom)
```

---

## 🏭 生产模式（TEST_MODE = False）

### 适用场景
- 实际部署到机器人
- 有真实的 `/Odom` 话题发布者（如底盘驱动节点）
- 需要使用真实的机器人位置信息

### 工作原理
1. **不会**调用 `update_odom_from_trajectory_end()`
2. 完全依赖真实的 `/Odom` 话题数据
3. 每段轨迹规划时从当前真实位置读取 `self.current_odom`

### 代码示例
```python
# 测试环境：模拟Odom更新
if TEST_MODE and hasattr(self, 'unload_stage1_waypoints'):
    self.update_odom_from_trajectory_end(self.unload_stage1_waypoints)
    # 在生产模式下，这段代码不会执行
```

### 注意事项
⚠️ **必须确保有真实的 /Odom 发布者**，否则：
- 初始化时会超时，使用默认位置 `(0, 0, 0)`
- 多段轨迹规划可能失败（起点不正确）

---

## 🔄 切换环境

### 从测试切换到生产

**步骤 1**: 修改配置
```python
# 在 unified_planner_workflow.py 顶部
TEST_MODE = False  # 改为 False
```

**步骤 2**: 确认真实 Odom 可用
```bash
# 检查 /Odom 话题
ros2 topic list | grep Odom
ros2 topic echo /Odom --once
```

**步骤 3**: 重启规划器
```bash
python3 unified_planner_workflow.py
```

**步骤 4**: 验证启动日志
```
🚀 统一轨迹规划器
================================================================================
📋 环境配置：
  运行模式: 🏭 生产模式（使用真实Odom）
  说明: 从真实/Odom话题订阅机器人位置
================================================================================
```

### 从生产切换回测试

**步骤 1**: 修改配置
```python
TEST_MODE = True  # 改回 True
```

**步骤 2**: 重启规划器

---

## 📊 对比表格

| 特性 | 测试模式 | 生产模式 |
|------|---------|---------|
| **Odom 来源** | 模拟（轨迹终点） | 真实（底盘驱动） |
| **update_odom_from_trajectory_end** | ✅ 会调用 | ❌ 不调用 |
| **适用场景** | 本地开发/测试 | 实际部署 |
| **需要真实机器人** | ❌ 不需要 | ✅ 需要 |
| **Odom 发布** | 程序自己发布 | 外部节点发布 |
| **位置准确性** | 理论位置（轨迹终点） | 实际位置（传感器反馈） |

---

## 🐛 常见问题

### Q1: 生产模式下轨迹规划失败
**症状**: 第2段轨迹起点不正确，从 (0, 0) 开始

**原因**: 没有真实的 /Odom 发布者

**解决**:
1. 检查 `/Odom` 话题是否存在
2. 确认底盘驱动节点是否启动
3. 或临时切换回测试模式

### Q2: 测试模式下 Odom 不更新
**症状**: 所有轨迹都从同一个点开始

**原因**: `TEST_MODE = False` 但没有真实 Odom

**解决**: 将 `TEST_MODE` 改为 `True`

### Q3: 两个节点同时发布 /Odom 冲突
**症状**: Odom 数据跳变

**原因**: 
- 生产环境中 `TEST_MODE = True`（错误）
- 导致程序和底盘都发布 Odom

**解决**: 在生产环境确保 `TEST_MODE = False`

---

## 💡 最佳实践

### 开发阶段
```python
TEST_MODE = True                        # 使用模拟Odom
ENABLE_OBSERVATION_TRAJECTORY = True    # 根据需要调整
ENABLE_PICKUP_TRAJECTORY = True
ENABLE_UNLOAD_TRAJECTORY = True
```

### 测试阶段
```python
TEST_MODE = True                        # 仍使用模拟
# 确保所有轨迹都测试通过
```

### 部署阶段
```python
TEST_MODE = False                       # 切换到真实Odom
# 确认真实 /Odom 话题可用
```

### 紧急调试
```python
# 如果生产环境出问题，可以临时切换回测试模式
TEST_MODE = True
# 但要注意：这只是临时方案，不能代替真实Odom
```

---

## 📝 代码变更记录

### Before（旧代码 - 有冲突风险）
```python
# TODO: 生产环境有真实Odom时，注释掉下面这行
if hasattr(self, 'unload_stage1_waypoints'):
    self.update_odom_from_trajectory_end(self.unload_stage1_waypoints)
```
❌ 问题：需要手动注释，容易忘记，导致冲突

### After（新代码 - 使用配置开关）
```python
# 测试环境：模拟Odom更新
if TEST_MODE and hasattr(self, 'unload_stage1_waypoints'):
    self.update_odom_from_trajectory_end(self.unload_stage1_waypoints)
```
✅ 优点：通过配置控制，清晰明确，不会冲突

---

## 🔗 相关文件

- `unified_planner_workflow.py` - 主程序（包含 TEST_MODE 配置）
- `test_independent.py` - 独立测试脚本
- `test_unload.py` - 完整流程测试

---

**最后更新**: 2025-10-09  
**变更原因**: 解决测试环境和生产环境 Odom 冲突问题
