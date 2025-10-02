# Beta-3 协议更新和实现记录

## 概述

本文档记录了中力具身装卸机器人系统通信协议从之前版本升级到 Beta-3 版本的所有修改和实现细节。

## 协议变化分析

### 1. 核心变化：轨迹指令消息（4.2.6节）

**新增字段：**
- `orientation` (Number): 运动方向
  - `0.0`: 前向运动
  - `-3.14`: 倒车运动
  - `3.14`: 倒车运动
- `flag` (Number): 进入分支标志位
  - `0.0`: 非进入分支
  - `1.0`: 进行分支

**新增动作类型：**
- `pub_load_params`: 发布取货参数
- `pub_unload_params`: 发布放货参数
- `start_stacking`: 启动堆垛

## 代码实现修改

### 1. 数据结构更新

#### zhongli_protocol_types.hpp
```cpp
struct TrajectoryPoint {
    double x;                                              ///< X坐标，单位：米
    double y;                                              ///< Y坐标，单位：米
    double theta;                                          ///< 轨迹点航向角，单位：弧度（范围：-π到+π）
    double orientation;                                    ///< 运动方向，0:前向运动 -3.14:倒车 3.14:倒车
    double flag;                                           ///< 进入分支标志位，0:非进入分支 1:进行分支
    std::optional<TrajectoryAction> action;                ///< 该轨迹点上的动作（可选）

    // JSON序列化支持新字段
    nlohmann::json to_json() const {
        nlohmann::json j = {
            {"x", x},
            {"y", y},
            {"theta", theta},
            {"orientation", orientation},  // 新增
            {"flag", flag}                 // 新增
        };
        // ... action处理
        return j;
    }

    // JSON反序列化支持新字段
    static TrajectoryPoint from_json(const nlohmann::json& j) {
        TrajectoryPoint point;
        // ... 基本字段
        point.orientation = j.value("orientation", 0.0);  // 新增，默认值0.0
        point.flag = j.value("flag", 0.0);                // 新增，默认值0.0
        // ... action处理
        return point;
    }
};
```

### 2. 路径转换器更新

#### path_converter.hpp
```cpp
class PathConverter {
private:
    /**
     * @brief Beta-3协议特定信息结构
     */
    struct Beta3Info {
        double orientation;              ///< 运动方向
        double flag;                    ///< 分支标志
        std::string action_type;        ///< 动作类型
        std::string container_type;     ///< 容器类型
    };

    /**
     * @brief 从frame_id解析beta-3特定信息
     */
    Beta3Info parse_beta3_info_from_frame_id(const std::string& frame_id);
};
```

#### path_converter.cpp
```cpp
zhongli_protocol::TrajectoryMessage PathConverter::convert_path_to_trajectory(
    const nav_msgs::msg::Path& ros_path) {

    // 解析frame_id中的beta-3信息
    auto beta3_info = parse_beta3_info_from_frame_id(ros_path.header.frame_id);

    // 转换所有路径点
    std::vector<zhongli_protocol::TrajectoryPoint> trajectory_points;
    for (const auto& pose_stamped : ros_path.poses) {
        auto point = convert_pose_to_trajectory_point(pose_stamped);
        // 应用beta-3信息
        point.orientation = beta3_info.orientation;
        point.flag = beta3_info.flag;
        trajectory_points.push_back(point);
    }

    // ... 其余代码
}

PathConverter::Beta3Info PathConverter::parse_beta3_info_from_frame_id(const std::string& frame_id) {
    Beta3Info info;
    info.orientation = 0.0;  // 默认前向
    info.flag = 0.0;         // 默认非分支

    // 解析格式: "map|action_type|container_type|orientation|flag|container_x|container_y|container_z|container_theta|container_width"
    std::vector<std::string> parts;
    std::stringstream ss(frame_id);
    std::string item;

    while (std::getline(ss, item, '|')) {
        parts.push_back(item);
    }

    if (parts.size() >= 5) {
        try {
            info.action_type = parts[1];
            info.container_type = parts[2];
            info.orientation = std::stod(parts[3]);
            info.flag = std::stod(parts[4]);
        } catch (const std::exception& e) {
            // 解析失败，使用默认值
        }
    }

    return info;
}
```

### 3. 测试架构设计

#### 动态测试架构 (test_beta3_dynamic_workflow.py)

**设计理念：**
1. **基本路径发布**：监听 `/Odom` 话题，发布默认值路径
   - `orientation = 0.0` (前向运动)
   - `flag = 0.0` (非分支)
   - `action = null` (无动作)

2. **动作指令订阅**：监听 `/action_command` 话题接收动作指令
3. **动态字段更新**：收到动作后立即更新轨迹参数

**关键实现：**
```python
def action_callback(self, msg):
    """动作指令回调，接收动作消息并动态更新轨迹"""
    action_data = json.loads(msg.data)

    # 根据动作类型决定运动模式
    if 'unload' in action_type.lower() or 'place' in action_type.lower():
        # 卸货/放置动作：需要掉头
        orientation = 3.14  # 掉头
        flag = 1.0  # 进入分支
    else:
        # 取货/其他动作：前向运动但进入分支
        orientation = 0.0  # 前向
        flag = 1.0  # 进入分支

    # 立即发布包含动作的更新路径
    self.publish_action_updated_path()
```

## 测试验证

### 1. 基本功能测试

**简单测试 (test_beta3_simple.py)：**
- 验证不同 orientation 和 flag 值的正确传递
- 测试用例：
  - (0.0, 0.0): 前向运动，非分支
  - (-3.14, 0.0): 倒车运动，非分支
  - (0.0, 1.0): 前向运动，分支
  - (-3.14, 1.0): 倒车运动，分支

### 2. 动态工作流程测试

**动态测试 (test_beta3_dynamic_workflow.py)：**
1. 自动发布基本路径（默认值）
2. 通过 `/action_command` 话题发送动作指令
3. 观察字段动态变化：
   - 取货：`orientation=0.0, flag=1.0`
   - 卸货：`orientation=3.14, flag=1.0`

**测试命令示例：**
```bash
# 取货动作
ros2 topic pub /action_command std_msgs/String "data: '{\"actionType\": \"pub_load_params\", \"containerType\": \"AGV-T300\", \"containerPose\": {\"x\": 1.0, \"y\": 1.0, \"z\": 0.1, \"theta\": 0.0, \"width\": 1.2}}'"

# 卸货动作
ros2 topic pub /action_command std_msgs/String "data: '{\"actionType\": \"pub_unload_params\", \"containerType\": \"container\", \"containerPose\": {\"x\": 2.0, \"y\": 2.0, \"z\": 0.2, \"theta\": 1.57, \"width\": 0.8}}'"
```

### 3. 真实路径生成

**参考 danci3_test_nav_path_publisher.py 的正确做法：**
- 监听实际的里程计数据 (`/Odom`)
- 基于当前位置和朝向计算前向路径
- 沿着车辆当前朝向方向生成路径点
- 确保生成的是真正的"前向运动"

## 构建和部署

### 构建命令
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_zhongli_bridge_cpp --symlink-install
```

### 运行测试
```bash
# 启动桥接器
./install/ros2_zhongli_bridge_cpp/bin/zhongli_bridge_node --ros-args --params-file config/bridge_config.yaml

# 运行动态测试
cd tests/scripts && python3 test_beta3_dynamic_workflow.py
```

## 兼容性

### 向后兼容性
- 新字段有默认值：`orientation=0.0`, `flag=0.0`
- 旧的轨迹点结构仍然有效
- 无 action 信息时自动设置为 `null`

### 扩展性
- frame_id 编码支持完整的容器位姿信息
- 支持未来新增的动作类型
- JSON 序列化/反序列化自动处理新字段

## 关键改进点

1. **问题修复**：原测试文件只是创建固定坐标点，现在基于实际车辆位置和朝向生成真实导航路径

2. **架构优化**：实现了动态更新机制，模拟真实系统中动作指令对轨迹参数的影响

3. **测试完善**：提供了多层次的测试：
   - 基本字段验证
   - 动态工作流程
   - 真实路径生成

4. **协议解析**：通过 frame_id 编码机制实现了 ROS2 到 Beta-3 协议的信息传递

## 文件清单

### 修改的文件
- `src/include/zhongli_protocol_types.hpp` - 数据结构更新
- `src/include/path_converter.hpp` - 添加 Beta3Info 结构和解析方法
- `src/lib/path_converter.cpp` - 实现 frame_id 解析和字段应用

### 新增的文件
- `tests/scripts/test_beta3_simple.py` - 基本功能测试
- `tests/scripts/test_beta3_dynamic_workflow.py` - 动态工作流程测试

### 修正的文件
- `tests/scripts/test_beta3_trajectory_workflow.py` - 修正为基于实际位置的路径生成

## 总结

Beta-3 协议更新成功实现了：

1. ✅ **协议兼容**：完全支持新的 orientation 和 flag 字段
2. ✅ **动态更新**：实现了动作触发的轨迹参数动态变化
3. ✅ **真实测试**：基于实际车辆位置生成的导航路径
4. ✅ **向后兼容**：保持与旧版本的兼容性
5. ✅ **完整验证**：提供了多层次的测试验证机制

系统现在能够正确处理 Beta-3 协议的所有新特性，并提供了完整的测试框架来验证功能正确性。