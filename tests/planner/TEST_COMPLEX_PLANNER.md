# ComplexTrajectoryPlanner 测试指南

## 快速测试（3步启动）

### 步骤1: 启动MQTT Bridge（必须！）

**终端1：**
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
./start_mqtt_bridge.sh
```

**期望输出：**
```
==================================
🚀 启动MQTT Bridge
==================================
✅ 中力MQTT客户端已创建 - Robot ID: robot-001
✅ 已连接到EMQX服务器
📡 已订阅主题: EP/master/robot-001/task
```

⚠️ **保持这个终端运行！**

---

### 步骤2: 启动ComplexTrajectoryPlanner测试节点

**新开终端2：**
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
python3 test_complex_planner_workflow.py
```

**期望输出：**
```
🧪 ComplexTrajectoryPlanner 完整工作流程测试
================================================================================
测试配置：
  第一次左转: 90.0°
  前进距离: 0.5m
  第二次左转: 90.0°
  倒车距离: 0.3m
================================================================================

✅ ComplexPlannerTester 节点已启动
   规划器: ComplexTrajectoryPlanner
   Service: /trajectory_status

🚀 连接到MQTT代理: localhost:1883
✅ MQTT连接成功
📡 订阅轨迹状态主题: EP/robot-001/cerebellum/embrain/trajectory_status

⏳ 等待 /Odom 话题数据（最多等待 10 秒）...
⚠️  10秒内未收到 /Odom 话题，使用默认位置数据
   默认位置: (0.000, 0.000), 朝向: 0.0°

⏱️  准备发布前向轨迹...

================================================================================
📤 规划并发布前向轨迹（Traj1 + Traj2 组合）
================================================================================
📡 获取最新Odom数据...

📋 复杂前向轨迹规划:
   起点: (0.000, 0.000), yaw=0.000 (0.0°)
   第一次左转: 90.0°
   前进距离: 0.500m
   第二次左转: 90.0°
   阶段1: 原地左转 90.0°
   阶段2: 前进 0.500m (点间距0.15m, 4个点)
   阶段3: 原地左转 90.0°
   ✅ 前向轨迹规划完成: 共 7 个路径点
   终点: (0.000, 0.500), yaw=3.142 (180.0°)

✅ 前向轨迹生成完成，共 7 个路径点

路径点详情（前5个和后2个）:
  点1: x=0.000, y=0.000, yaw=0.000 (0.0°)
  点2: x=0.000, y=0.000, yaw=1.571 (90.0°)
  点3: x=0.000, y=0.150, yaw=1.571 (90.0°)
  点4: x=0.000, y=0.300, yaw=1.571 (90.0°)
  点5: x=0.000, y=0.450, yaw=1.571 (90.0°)
  点6: x=0.000, y=0.500, yaw=1.571 (90.0°)
  点7: x=0.000, y=0.500, yaw=3.142 (180.0°)

Beta-3参数: orientation=0.0, flag=0 (前向运动)
================================================================================
📤 轨迹已发布到 /plans
📋 轨迹ID: complex_forward_XXXXXXXXXX

⏳ 等待MQTT轨迹完成信号（按Ctrl+C可中断）...
```

**终端1（MQTT Bridge）应该显示：**
```
📤 发布轨迹指令到: EP/robot-001/embrain/cerebellum/trajectory
   轨迹ID: complex_forward_XXXXXXXXXX
   轨迹点数: 7
```

---

### 步骤3: 查询轨迹状态（可选）

**新开终端3：**
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
python3 query_trajectory_status.py
```

**期望输出：**
```
📞 调用 /trajectory_status service...

================================================================================
📊 轨迹状态查询结果
================================================================================
✅ 查询成功

状态信息:
  轨迹ID: complex_forward_XXXXXXXXXX
  状态: running/completed
  时间戳: XXXXXXXXXX
  消息: ...
================================================================================
```

---

## 完整测试流程

### 自动化测试流程

测试节点会自动执行以下步骤：

1. ✅ **等待Odom数据**（10秒超时，超时使用默认位置）
2. ✅ **规划前向轨迹**（左转90° → 前进0.5m → 左转90°）
3. ✅ **发布前向轨迹**（flag=0, orientation=0.0）
4. ⏳ **等待MQTT完成信号**
5. ✅ **规划后向轨迹**（倒车0.3m）
6. ✅ **发布后向轨迹**（flag=1, orientation=3.14）
7. ⏳ **等待MQTT完成信号**
8. 🔁 **持续监听MQTT状态**

### 期望的完整输出

```
================================================================================
📊 收到MQTT轨迹状态消息
================================================================================
📋 轨迹ID: complex_forward_XXXXXXXXXX
📍 状态: completed
⏰ 时间戳: XXXXXXXXXX

✅ 轨迹已完成！
================================================================================

✅ 前向轨迹已完成，等待3秒后发布后向轨迹...

================================================================================
📤 规划并发布后向轨迹（Traj3 倒车）
================================================================================
📡 获取最新Odom数据...

📋 后向轨迹规划 (倒车):
   起点: (0.000, 0.500), yaw=3.142 (180.0°)
   倒车距离: 0.300m
   生成路径点: 3个 (点间距0.15m)
   ✅ 后向轨迹规划完成: 共 3 个路径点
   终点: (0.300, 0.500), yaw=3.142 (180.0°)

✅ 后向轨迹生成完成，共 3 个路径点

所有路径点:
  点1: x=0.000, y=0.500, yaw=3.142 (180.0°)
  点2: x=0.150, y=0.500, yaw=3.142 (180.0°)
  点3: x=0.300, y=0.500, yaw=3.142 (180.0°)

Beta-3参数: orientation=3.14, flag=1 (倒车运动)
容器信息: type=AGV-T300, pos=(1.000, 1.500)
================================================================================
📤 轨迹已发布到 /plans
📋 轨迹ID: complex_backward_XXXXXXXXXX

✅ 所有轨迹发布完成
💡 保持运行以监听MQTT轨迹消息...
   按 Ctrl+C 停止测试
```

---

## 单独测试（不需要MQTT）

如果只想测试规划器本身，不需要MQTT Bridge：

```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
python3 example_complex_planner.py
```

这个命令会：
- ✅ 测试前向轨迹规划
- ✅ 测试后向轨迹规划
- ✅ 打印所有路径点
- ❌ 不发布到ROS2话题
- ❌ 不连接MQTT

---

## 对比测试

### 与SimpleTrajectoryPlanner对比

**测试SimpleTrajectoryPlanner：**
```bash
python3 test_beta4_trajectory_workflow_goal.py
```

**测试ComplexTrajectoryPlanner：**
```bash
python3 test_complex_planner_workflow.py
```

### 主要差异

| 特性 | SimpleTrajectoryPlanner | ComplexTrajectoryPlanner |
|------|------------------------|--------------------------|
| 输入方式 | 起点 + 目标点（自动规划） | 起点 + 运动参数（指定模式） |
| 前向轨迹 | 自动优化路径 | 固定"转弯→前进→转弯"模式 |
| 后向轨迹 | 不支持 | 支持倒车 |
| 适用场景 | 点对点导航 | 多阶段复杂动作 |

---

## 验证清单

测试成功的标志：

- [ ] MQTT Bridge成功连接
- [ ] 收到Odom数据（或使用默认位置）
- [ ] 前向轨迹规划生成7个路径点
- [ ] 前向轨迹成功发布到/plans话题
- [ ] MQTT Bridge收到轨迹并转发
- [ ] 收到MQTT完成信号（status=completed）
- [ ] 后向轨迹规划生成3个路径点
- [ ] 后向轨迹成功发布到/plans话题
- [ ] /trajectory_status service可查询

---

## 故障排查

### 问题1: 查询状态返回 no_data

**原因**：MQTT Bridge未运行或未收到轨迹状态

**解决**：
```bash
# 检查MQTT Bridge是否运行
ps aux | grep zhongli_bridge_node | grep -v grep

# 如果没有输出，重新启动
./start_mqtt_bridge.sh
```

### 问题2: Odom超时

**原因**：没有发布/Odom话题

**解决**：使用默认位置继续测试（程序会自动处理）

### 问题3: MQTT连接失败

**原因**：EMQX未启动

**解决**：
```bash
# 检查EMQX状态
sudo systemctl status emqx

# 启动EMQX
sudo systemctl start emqx
```

### 问题4: 轨迹发布但未收到完成信号

**原因**：底层控制器未运行或未返回状态

**解决**：
- 这是正常的（测试环境可能没有真实控制器）
- 按Ctrl+C中断等待，手动继续测试

---

## 修改测试参数

编辑 `test_complex_planner_workflow.py`：

```python
# 修改轨迹规划参数
FIRST_TURN_ANGLE = math.pi / 2   # 第一次左转角度（改为45度：math.pi/4）
FORWARD_DISTANCE = 0.5           # 前进距离（改为1米）
SECOND_TURN_ANGLE = math.pi / 2  # 第二次左转角度
BACKWARD_DISTANCE = 0.3          # 倒车距离（改为0.5米）

# 修改路径点密度
planner = ComplexTrajectoryPlanner(
    forward_step=0.15,    # 改为0.10（更密集）或0.30（更稀疏）
    backward_step=0.15
)
```

---

## 下一步

测试成功后，可以：

1. **集成到状态机**：将ComplexTrajectoryPlanner集成到你的状态机节点
2. **实际Odom数据**：连接真实的/Odom话题
3. **真实控制器**：连接底层运动控制器
4. **可视化**：使用RViz可视化规划的路径

---

## 相关文件

- `trajectory_planner.py` - 规划器实现
- `test_complex_planner_workflow.py` - 完整工作流程测试
- `example_complex_planner.py` - 简单示例（不需要ROS2/MQTT）
- `query_trajectory_status.py` - 状态查询工具
- `start_mqtt_bridge.sh` - MQTT Bridge启动脚本
- `COMPLEX_PLANNER_GUIDE.md` - 详细使用指南

---

## 总结

✅ 使用 `test_complex_planner_workflow.py` 进行完整集成测试
✅ 使用 `example_complex_planner.py` 进行快速规划验证
✅ 使用 `query_trajectory_status.py` 查询轨迹执行状态

祝测试顺利！🎯
