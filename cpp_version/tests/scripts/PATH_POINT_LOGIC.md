# 路径点生成逻辑说明

## 原地转弯的正确实现

### ❌ 错误做法（之前的实现）
```python
# 错误：原地转弯生成5个中间点
num_turn_points = 5
for i in range(1, num_turn_points + 1):
    angle_offset = -(i / num_turn_points) * (math.pi / 2)
    current_yaw = base_yaw + angle_offset
    pose = self.create_pose_stamped(end_x, end_y, current_yaw)
    poses.append(pose)
```

**问题**: 生成了5个中间点，但x和y坐标都相同，只是逐渐改变yaw角度

### ✅ 正确做法（修复后）
```python
# 正确：原地转弯只需起点和终点
# 起点
pose_start = self.create_pose_stamped(x, y, yaw_before)
poses.append(pose_start)

# 终点（x,y相同，只有yaw变化）
yaw_after = yaw_before + math.pi / 2  # 左转90度
pose_end = self.create_pose_stamped(x, y, yaw_after)
poses.append(pose_end)
```

**原理**: 
- 原地转弯意味着位置(x,y)不变
- 只需要两个点：转弯前和转弯后
- 两点的x和y坐标完全相同
- 只有yaw角度发生变化

## 第一条轨迹路径点详情

### 完整流程
1. **直行3米**: 11个点（起点 + 10个中间点）
2. **原地右转90度**: 1个点（终点，x,y不变）
3. **停顿2秒**: 1个点（x,y,yaw都不变）
4. **原地左转90度回正**: 1个点（终点，x,y不变）
5. **直行0.5米**: 3个点

**总共**: 11 + 1 + 1 + 1 + 3 = **17个路径点**

### 路径点示例

```python
# 假设起始位置 (0, 0, yaw=0)

# 1. 直行3米 (11个点)
点1:  x=0.0,   y=0.0,   yaw=0.0   (起点)
点2:  x=0.3,   y=0.0,   yaw=0.0
点3:  x=0.6,   y=0.0,   yaw=0.0
...
点11: x=3.0,   y=0.0,   yaw=0.0   (直行终点)

# 2. 原地右转90度 (1个点)
点12: x=3.0,   y=0.0,   yaw=-π/2  (右转后，x,y不变！)

# 3. 停顿2秒 (1个点)
点13: x=3.0,   y=0.0,   yaw=-π/2  (停顿，x,y,yaw都不变)

# 4. 原地左转90度回正 (1个点)
点14: x=3.0,   y=0.0,   yaw=0.0   (左转后回正，x,y不变！)

# 5. 直行0.5米 (3个点)
点15: x=3.167, y=0.0,   yaw=0.0
点16: x=3.333, y=0.0,   yaw=0.0
点17: x=3.5,   y=0.0,   yaw=0.0   (最终终点)
```

## 第二条轨迹路径点详情

### 完整流程
1. **原地左转90度**: 2个点（起点 + 终点）
2. **倒车0.5米**: 3个点

**总共**: 2 + 3 = **5个路径点**

### 路径点示例

```python
# 假设起始位置 (3.5, 0, yaw=0)

# 1. 原地左转90度 (2个点)
点1: x=3.5,   y=0.0,   yaw=0.0     (转弯前)
点2: x=3.5,   y=0.0,   yaw=π/2     (转弯后，x,y不变！)

# 2. 倒车0.5米 (3个点)
# 注意：倒车是沿当前朝向的反方向移动，但车头朝向不变
# 当前朝向是π/2（朝向Y轴正方向），倒车就是沿Y轴负方向移动
点3: x=3.5,   y=-0.167, yaw=π/2   (车头仍朝向Y轴正方向)
点4: x=3.5,   y=-0.333, yaw=π/2
点5: x=3.5,   y=-0.5,   yaw=π/2   (倒车终点)
```

## 关键点总结

### 1. 原地转弯特征
- ✅ 只需要**起点和终点**两个点
- ✅ 两点的 **x 和 y 坐标完全相同**
- ✅ 只有 **yaw 角度发生变化**
- ❌ 不需要生成多个中间角度的点

### 2. 直行特征
- 多个路径点
- x 和/或 y 坐标线性变化
- yaw 保持不变（直线行驶）

### 3. 倒车特征
- 沿当前朝向的反方向移动位置
- 车头朝向（yaw）保持不变
- 通过 orientation=3.14 告知是倒车运动

## 修复前后对比

### 第一条轨迹路径点数量
- ❌ 修复前: 27个点（过多的中间点）
- ✅ 修复后: 17个点（精简合理）

### 第二条轨迹路径点数量
- ❌ 修复前: 11个点（过多的中间点）
- ✅ 修复后: 5个点（精简合理）

## Beta-3 协议参数

### 第一条轨迹
```
orientation: 0.0   (前向运动)
flag: 0            (非分支)
action: null       (无动作)
```

### 第二条轨迹
```
orientation: 3.14  (倒车运动)
flag: 1            (进入分支)
action: pub_unload_params
containerType: AGV-T300
containerPose: {x, y, z, theta, width}
```

## 验证方法

运行测试程序后，检查输出：

```bash
cd /home/yhg/Documents/docs/zhongli/cpp_version/tests/scripts
python3 test_beta3_trajectory_workflow.py
```

查看原地转弯的路径点，确认：
1. 转弯前后的x坐标相同 ✓
2. 转弯前后的y坐标相同 ✓
3. 只有yaw角度发生变化 ✓
4. 没有多余的中间点 ✓

## 配置参数说明（2025-10-02更新）

测试程序已参数化，所有配置项位于文件顶部：

### 轨迹控制开关
```python
ENABLE_TRAJECTORY1 = True    # 是否发布第一条轨迹
ENABLE_TRAJECTORY2 = True    # 是否发布第二条轨迹
```

### 第一条轨迹参数
```python
TRAJ1_FORWARD_DISTANCE = 3.0     # 直行距离
TRAJ1_FORWARD_POINTS = 10        # 直行点数
TRAJ1_RIGHT_TURN_STEPS = 2       # 右转点数（含起点终点）
TRAJ1_LEFT_TURN_STEPS = 2        # 左转点数（含起点终点）
TRAJ1_FINAL_FORWARD_POINTS = 3   # 最后直行点数
```

**注意**: `TURN_STEPS = 2` 表示原地转弯只有起点和终点两个点，符合正确实现。

### 灵活使用示例

**仅测试第一条轨迹**:
```python
ENABLE_TRAJECTORY1 = True
ENABLE_TRAJECTORY2 = False
```

**修改直行距离**:
```python
TRAJ1_FORWARD_DISTANCE = 5.0  # 改为5米
```

**增加转弯平滑度**（如需插值点）:
```python
TRAJ1_RIGHT_TURN_STEPS = 5  # 原地转弯分5步完成
```
但根据Beta-3协议，推荐保持为2（只有起点和终点）。
