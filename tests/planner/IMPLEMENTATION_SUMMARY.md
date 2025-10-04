# 实现总结：MQTT轨迹状态 → ROS2 Service

## ✅ 已完成功能

### 1. **轨迹状态存储**
- MQTT接收到的轨迹状态自动存储到内存
- 使用字典 `trajectory_status_dict` 按ID索引
- 维护 `latest_status` 记录最新状态

### 2. **ROS2 Service接口**
- Service名称: `/trajectory_status`
- Service类型: `example_interfaces/srv/Trigger`
- 功能: 状态机可随时查询最新轨迹状态

### 3. **状态机集成**
- 状态机通过调用service获取轨迹执行状态
- 无需直接订阅MQTT，降低耦合
- JSON格式返回完整状态信息

## 📋 核心代码修改

### `test_beta4_trajectory_workflow_goal.py`

#### 1. 添加Service服务器
```python
# 在 __init__ 中
self.status_service = self.create_service(
    Trigger, '/trajectory_status', self.handle_status_query)

# 状态存储
self.trajectory_status_dict = {}
self.latest_status = None
```

#### 2. Service处理函数
```python
def handle_status_query(self, request, response):
    """处理轨迹状态查询service请求"""
    if self.latest_status:
        response.success = True
        status_json = json.dumps({
            'trajectory_id': self.latest_status.get('trajectoryId', ''),
            'status': self.latest_status.get('status', ''),
            'timestamp': self.latest_status.get('timestamp', 0),
            'message': self.latest_status.get('message', '')
        })
        response.message = status_json
    else:
        response.success = False
        response.message = json.dumps({...})
    return response
```

#### 3. MQTT消息处理（存储状态）
```python
def on_message(self, client, userdata, msg):
    if 'trajectory_status' in msg.topic:
        # ... 解析消息 ...

        # 存储到字典
        self.trajectory_status_dict[trajectory_id] = {
            'status': status,
            'timestamp': trajectory_data.get('timestamp'),
            'message': trajectory_data.get('message', '')
        }

        # 更新最新状态
        self.latest_status = trajectory_data
```

## 🔧 状态机使用方式

### Python代码示例
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
import json

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.client = self.create_client(Trigger, '/trajectory_status')

    def check_trajectory_status(self):
        """查询轨迹状态"""
        request = Trigger.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            status = json.loads(future.result().message)
            return status['status']  # 'pending', 'running', 'completed', 'failed'
        return 'no_data'
```

### 命令行测试
```bash
# 查询当前状态
python3 query_trajectory_status.py

# 或使用ros2 service命令
ros2 service call /trajectory_status example_interfaces/srv/Trigger
```

## 📊 数据流

```
MQTT (EMQX)
    │
    │ trajectory_status (JSON)
    ↓
test_beta4_trajectory_workflow_goal.py
    │
    ├─> trajectory_status_dict[id] = {...}   # 按ID存储
    ├─> latest_status = {...}                # 最新状态
    │
    └─> /trajectory_status Service
           │
           └─> 状态机查询 ──> JSON响应
```

## 🎯 状态类型

| 状态 | 含义 | 状态机行为 |
|------|------|------------|
| `pending` | 轨迹等待执行 | 继续等待 |
| `running` | 轨迹正在执行 | 监控进度 |
| `completed` | 轨迹已完成 | 进入下一状态 |
| `failed` | 轨迹执行失败 | 错误处理 |
| `no_data` | 暂无状态数据 | 等待首次状态 |

## 🚀 测试流程

### 终端1：启动轨迹规划节点
```bash
cd /home/yhg/Documents/ep-embodied/mqtt_bridge/tests/planner
python3 test_beta4_trajectory_workflow_goal.py
```

### 终端2：发布目标点
```bash
python3 publish_goal.py --x 3.0 --y 0.0 --yaw 90
```

### 终端3：查询状态（模拟状态机）
```bash
# 等待轨迹发布后
python3 query_trajectory_status.py

# 预期输出：
# 状态: pending/running/completed
```

## ⚙️ 配置说明

### Service配置
- Service名称: `/trajectory_status`（可在代码中修改）
- 返回格式: JSON字符串
- 超时处理: 无超时，即时返回当前状态

### MQTT配置
```python
MQTT_BROKER = "localhost"  # EMQX地址
MQTT_PORT = 1883
ROBOT_ID = "robot-001"
```

## 📝 注意事项

1. **状态持久化**: 当前状态仅存储在内存中，节点重启后丢失
2. **并发安全**: 单线程访问，无需额外加锁
3. **状态覆盖**: `latest_status` 始终保存最新接收的状态
4. **Service类型**: 使用标准 `Trigger` 类型，无需编译自定义srv

## 🔮 未来改进

1. 添加状态历史记录（保存最近N条状态）
2. 支持按轨迹ID查询特定轨迹状态
3. 添加状态变化回调通知
4. 实现状态持久化（数据库/文件）
5. 添加状态超时检测（轨迹卡死检测）
