#!/usr/bin/env python3
"""
实时状态反馈测试 - 验证控制器执行进度反馈给ROS2
Real-time Status Feedback Test - Verify controller execution progress feedback to ROS2
"""

import json
import time
import threading
import math
import sys
import os

# Add the package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'ros2_vda5050_bridge'))
from ros2_vda5050_bridge.zhongli_mqtt_client import ZhongliMQTTClient
from ros2_vda5050_bridge.new_protocol_types import *


class RealtimeStatusFeedbackTest:
    """实时状态反馈测试类"""
    
    def __init__(self):
        self.robot_id = "robot-001"
        self.trajectory_status_history = []
        self.action_status_history = []
        self.max_history_size = 20
        
        # 创建桥接器端MQTT客户端（模拟ROS2端）
        self.ros2_client = ZhongliMQTTClient(robot_id=self.robot_id)
        
        # 创建控制器端MQTT客户端（模拟车载小脑）
        self.controller_client = ZhongliMQTTClient(robot_id=self.robot_id)
        
        # 设置状态处理器
        self.setup_status_handlers()
        
    def setup_status_handlers(self):
        """设置状态处理器"""
        def handle_trajectory_status(trajectory_status):
            """处理轨迹状态反馈"""
            self.trajectory_status_history.append(trajectory_status)
            
            # 限制历史记录大小
            if len(self.trajectory_status_history) > self.max_history_size:
                self.trajectory_status_history.pop(0)
            
            print(f"📊 [轨迹状态] ID: {trajectory_status.trajectoryId}")
            print(f"     状态: {trajectory_status.status}")
            if trajectory_status.currentPointIndex is not None:
                print(f"     当前进度: 点 {trajectory_status.currentPointIndex + 1}")
            if trajectory_status.estimatedFinishTime:
                print(f"     预计完成: {trajectory_status.estimatedFinishTime}")
            print(f"     错误码: {trajectory_status.errorCode}")
            if trajectory_status.errorDesc:
                print(f"     错误描述: {trajectory_status.errorDesc}")
            print()
            
        def handle_action_status(action_status):
            """处理动作状态反馈"""
            self.action_status_history.append(action_status)
            
            # 限制历史记录大小
            if len(self.action_status_history) > self.max_history_size:
                self.action_status_history.pop(0)
            
            print(f"🎬 [动作状态] ID: {action_status.actionId}")
            print(f"     状态: {action_status.status}")
            print(f"     错误码: {action_status.errorCode}")
            if action_status.errorDesc:
                print(f"     错误描述: {action_status.errorDesc}")
            if action_status.finishTime:
                print(f"     完成时间: {action_status.finishTime}")
            print()
        
        self.ros2_client.set_trajectory_status_handler(handle_trajectory_status)
        self.ros2_client.set_action_status_handler(handle_action_status)
    
    def simulate_trajectory_execution(self):
        """模拟轨迹执行过程"""
        print("🚀 开始模拟轨迹执行过程...")
        print("=" * 50)
        
        # 创建测试轨迹
        trajectory_points = [
            TrajectoryPoint(x=0.0, y=0.0, theta=0.0),
            TrajectoryPoint(x=2.0, y=0.0, theta=0.0),
            TrajectoryPoint(x=4.0, y=2.0, theta=90.0),
            TrajectoryPoint(x=6.0, y=4.0, theta=90.0),
            TrajectoryPoint(x=8.0, y=6.0, theta=180.0),
        ]
        
        trajectory = TrajectoryMessage(
            timestamp=create_timestamp(),
            trajectoryId=generate_trajectory_id(self.robot_id),
            trajectoryPoints=trajectory_points,
            maxSpeed=1.5
        )
        
        print(f"📋 测试轨迹信息:")
        print(f"   轨迹ID: {trajectory.trajectoryId}")
        print(f"   总点数: {len(trajectory_points)}")
        print(f"   最大速度: {trajectory.maxSpeed} m/s")
        print()
        
        # 模拟执行过程
        execution_steps = [
            # (状态, 当前点索引, 错误码, 错误描述, 预计完成时间)
            ("pending", 0, 0, "", create_timestamp()),
            ("running", 0, 0, "", self._add_seconds_to_timestamp(create_timestamp(), 5)),
            ("running", 1, 0, "", self._add_seconds_to_timestamp(create_timestamp(), 8)),
            ("running", 2, 0, "", self._add_seconds_to_timestamp(create_timestamp(), 12)),
            ("running", 3, 0, "", self._add_seconds_to_timestamp(create_timestamp(), 15)),
            ("running", 4, 0, "", self._add_seconds_to_timestamp(create_timestamp(), 18)),
            ("completed", 4, 0, "", "", create_timestamp()),
        ]
        
        print("📈 执行进度模拟:")
        print("-" * 30)
        
        for i, (status, point_index, error_code, error_desc, est_time, *_) in enumerate(execution_steps):
            # 创建轨迹状态消息
            trajectory_status = TrajectoryStatusMessage(
                timestamp=create_timestamp(),
                trajectoryId=trajectory.trajectoryId,
                status=status,
                currentPointIndex=point_index,
                errorCode=error_code,
                errorDesc=error_desc,
                estimatedFinishTime=est_time if est_time else None,
                finishTime=create_timestamp() if status == "completed" else None
            )
            
            # 发布状态（控制器 -> ROS2）
            topic = f"EP/{self.robot_id}/cerebellum/embrain/trajectory_status"
            json_msg = trajectory_status.to_json()
            self.controller_client.client.publish(topic, json_msg, qos=0)
            
            # 显示进度
            progress_percent = (point_index + 1) / len(trajectory_points) * 100 if status != "pending" else 0
            status_emoji = {"pending": "⏳", "running": "🏃", "completed": "✅", "failed": "❌"}.get(status, "❓")
            
            print(f"步骤{i+1}: {status_emoji} {status.upper()}")
            print(f"   进度: {progress_percent:.1f}% ({point_index + 1}/{len(trajectory_points)} 点)")
            if est_time and status == "running":
                print(f"   预计剩余: {self._calculate_remaining_time(est_time)}秒")
            print()
            
            # 等待一段时间，模拟实时反馈
            time.sleep(1)
        
        return trajectory.trajectoryId
    
    def simulate_action_execution(self):
        """模拟动作执行过程"""
        print("🎬 开始模拟动作执行过程...")
        print("=" * 50)
        
        # 创建测试动作
        action = ActionMessage(
            timestamp=create_timestamp(),
            actionId=generate_action_id(self.robot_id),
            actionType="ground_pick",
            containerPose=ContainerPose(x=5.0, y=3.0, z=0.1, theta=90.0),
            containerType="AGV-T300"
        )
        
        print(f"🎭 测试动作信息:")
        print(f"   动作ID: {action.actionId}")
        print(f"   动作类型: {action.actionType}")
        print(f"   容器位置: ({action.containerPose.x}, {action.containerPose.y}, {action.containerPose.z})")
        print()
        
        # 模拟执行过程
        execution_steps = [
            # (状态, 错误码, 错误描述)
            ("running", 0, ""),
            ("running", 0, ""),
            ("success", 0, ""),
        ]
        
        print("🎭 动作执行进度:")
        print("-" * 30)
        
        for i, (status, error_code, error_desc) in enumerate(execution_steps):
            # 创建动作状态消息
            action_status = ActionStatusMessage(
                timestamp=create_timestamp(),
                actionId=action.actionId,
                status=status,
                errorCode=error_code,
                errorDesc=error_desc,
                finishTime=create_timestamp() if status == "success" else None
            )
            
            # 发布状态（控制器 -> ROS2）
            topic = f"EP/{self.robot_id}/cerebellum/embrain/action_status"
            json_msg = action_status.to_json()
            self.controller_client.client.publish(topic, json_msg, qos=0)
            
            # 显示进度
            status_emoji = {"running": "🔄", "success": "✅", "failed": "❌"}.get(status, "❓")
            
            print(f"步骤{i+1}: {status_emoji} {status.upper()}")
            if error_desc:
                print(f"   描述: {error_desc}")
            print()
            
            # 等待一段时间，模拟实时反馈
            time.sleep(1.5)
        
        return action.actionId
    
    def test_realtime_feedback_latency(self):
        """测试实时反馈延迟"""
        print("⏱️ 测试实时反馈延迟...")
        print("=" * 50)
        
        latency_results = []
        
        for i in range(5):
            # 创建测试状态消息
            trajectory_status = TrajectoryStatusMessage(
                timestamp=create_timestamp(),
                trajectoryId=f"test-latency-{i}",
                status="running",
                currentPointIndex=i,
                errorCode=0,
                errorDesc=""
            )
            
            # 记录发送时间
            send_time = time.time()
            
            # 发布消息
            topic = f"EP/{self.robot_id}/cerebellum/embrain/trajectory_status"
            json_msg = trajectory_status.to_json()
            self.controller_client.client.publish(topic, json_msg, qos=0)
            
            # 等待接收
            received = False
            start_wait = time.time()
            
            while time.time() - start_wait < 2.0:  # 最多等待2秒
                if self.trajectory_status_history:
                    latest = self.trajectory_status_history[-1]
                    if latest.trajectoryId == f"test-latency-{i}":
                        received = True
                        break
                time.sleep(0.01)
            
            # 计算延迟
            if received:
                latency = (time.time() - send_time) * 1000  # 转换为毫秒
                latency_results.append(latency)
                print(f"测试{i+1}: ✅ 延迟 {latency:.2f} ms")
            else:
                print(f"测试{i+1}: ❌ 超时")
                latency_results.append(None)
            
            time.sleep(0.5)
        
        # 分析结果
        valid_results = [r for r in latency_results if r is not None]
        if valid_results:
            avg_latency = sum(valid_results) / len(valid_results)
            max_latency = max(valid_results)
            min_latency = min(valid_results)
            
            print(f"\n📊 延迟统计:")
            print(f"   平均延迟: {avg_latency:.2f} ms")
            print(f"   最大延迟: {max_latency:.2f} ms")
            print(f"   最小延迟: {min_latency:.2f} ms")
            print(f"   成功率: {len(valid_results)}/5 ({len(valid_results)*20}%)")
            
            # 评估延迟性能
            if avg_latency < 50:
                print("   ✅ 延迟性能优秀 (< 50ms)")
            elif avg_latency < 100:
                print("   ✅ 延迟性能良好 (< 100ms)")
            elif avg_latency < 200:
                print("   ⚠️  延迟性能一般 (< 200ms)")
            else:
                print("   ❌ 延迟性能较差 (> 200ms)")
            
            return avg_latency < 100  # 100ms以内认为合格
        else:
            print("   ❌ 所有测试都失败了")
            return False
    
    def test_status_history_tracking(self):
        """测试状态历史跟踪"""
        print("📚 测试状态历史跟踪...")
        print("=" * 50)
        
        # 清空历史记录
        self.trajectory_status_history.clear()
        self.action_status_history.clear()
        
        # 模拟一些状态更新
        for i in range(3):
            trajectory_status = TrajectoryStatusMessage(
                timestamp=create_timestamp(),
                trajectoryId=f"test-history-{i}",
                status="running",
                currentPointIndex=i,
                errorCode=0,
                errorDesc=""
            )
            
            topic = f"EP/{self.robot_id}/cerebellum/embrain/trajectory_status"
            json_msg = trajectory_status.to_json()
            self.controller_client.client.publish(topic, json_msg, qos=0)
            time.sleep(0.1)
        
        # 验证历史记录
        trajectory_count = len(self.trajectory_status_history)
        action_count = len(self.action_status_history)
        
        print(f"📊 历史记录统计:")
        print(f"   轨迹状态记录: {trajectory_count}")
        print(f"   动作状态记录: {action_count}")
        
        # 验证记录内容
        if trajectory_count >= 3:
            latest = self.trajectory_status_history[-1]
            print(f"   最新轨迹状态: {latest.status} (点 {latest.currentPointIndex + 1})")
            print("   ✅ 历史跟踪功能正常")
            return True
        else:
            print("   ❌ 历史跟踪功能异常")
            return False
    
    def _add_seconds_to_timestamp(self, timestamp: str, seconds: int) -> str:
        """给时间戳增加秒数"""
        from datetime import datetime, timedelta
        dt = datetime.strptime(timestamp, '%Y-%m-%dT%H:%M:%S.%fZ')
        dt += timedelta(seconds=seconds)
        return dt.strftime('%Y-%m-%dT%H:%M:%S.%fZ')[:-3] + 'Z'
    
    def _calculate_remaining_time(self, estimated_finish: str) -> int:
        """计算剩余时间（秒）"""
        from datetime import datetime
        now = datetime.utcnow()
        finish_time = datetime.strptime(estimated_finish, '%Y-%m-%dT%H:%M:%S.%fZ')
        remaining = int((finish_time - now).total_seconds())
        return max(0, remaining)
    
    def run_comprehensive_test(self):
        """运行综合测试"""
        print("🧪 实时状态反馈综合测试")
        print("=" * 60)
        print()
        print("此测试验证以下关键功能:")
        print("1. 控制器执行进度实时反馈")
        print("2. 动作执行状态实时反馈")
        print("3. 状态反馈延迟性能")
        print("4. 状态历史跟踪功能")
        print()
        
        try:
            # 连接MQTT
            print("🔗 连接MQTT服务器...")
            if not self.ros2_client.connect():
                print("❌ ROS2客户端连接失败")
                return False
            
            if not self.controller_client.connect():
                print("❌ 控制器客户端连接失败")
                return False
            
            time.sleep(2)
            print("✅ MQTT连接成功")
            print()
            
            # 测试1: 轨迹执行状态反馈
            print("📊 测试1: 轨迹执行状态反馈")
            trajectory_id = self.simulate_trajectory_execution()
            
            # 验证轨迹状态接收
            trajectory_statuses = [s for s in self.trajectory_status_history if s.trajectoryId == trajectory_id]
            if trajectory_statuses:
                print(f"✅ 轨迹状态反馈正常，收到 {len(trajectory_statuses)} 条状态更新")
            else:
                print("❌ 未收到轨迹状态反馈")
                return False
            
            print()
            
            # 测试2: 动作执行状态反馈
            print("🎭 测试2: 动作执行状态反馈")
            action_id = self.simulate_action_execution()
            
            # 验证动作状态接收
            action_statuses = [s for s in self.action_status_history if s.actionId == action_id]
            if action_statuses:
                print(f"✅ 动作状态反馈正常，收到 {len(action_statuses)} 条状态更新")
            else:
                print("❌ 未收到动作状态反馈")
                return False
            
            print()
            
            # 测试3: 延迟性能
            print("⏱️ 测试3: 实时反馈延迟性能")
            latency_ok = self.test_realtime_feedback_latency()
            
            print()
            
            # 测试4: 历史跟踪
            print("📚 测试4: 状态历史跟踪")
            history_ok = self.test_status_history_tracking()
            
            print()
            
            # 总结
            print("📊 测试总结")
            print("=" * 30)
            
            success_criteria = [
                len(trajectory_statuses) > 0,  # 轨迹状态反馈
                len(action_statuses) > 0,     # 动作状态反馈
                latency_ok,                  # 延迟性能
                history_ok                    # 历史跟踪
            ]
            
            criteria_names = ["轨迹状态反馈", "动作状态反馈", "延迟性能", "历史跟踪"]
            
            for i, (name, success) in enumerate(zip(criteria_names, success_criteria)):
                status = "✅ 通过" if success else "❌ 失败"
                print(f"   {name}: {status}")
            
            overall_success = all(success_criteria)
            print(f"\n🎯 总体结果: {'✅ 通过' if overall_success else '❌ 失败'}")
            
            if overall_success:
                print("\n🎉 实时状态反馈功能测试通过!")
                print("\n✨ 验证的关键能力:")
                print("   ✅ 控制器执行进度实时反馈给ROS2")
                print("   ✅ 动作执行状态实时同步")
                print("   ✅ 低延迟的状态传输 (< 100ms)")
                print("   ✅ 完整的状态历史跟踪")
                print("   ✅ 支持NAV2决策树的状态感知")
                
                print("\n🔬 关键指标:")
                print(f"   轨迹状态更新: {len(trajectory_statuses)} 次")
                print(f"   动作状态更新: {len(action_statuses)} 次")
                print(f"   状态历史记录: {len(self.trajectory_status_history)} 条")
                
            else:
                print("\n⚠️  实时状态反馈功能需要改进")
                
            return overall_success
            
        except Exception as e:
            print(f"\n💥 测试过程中出现异常: {e}")
            import traceback
            traceback.print_exc()
            return False
        
        finally:
            # 清理资源
            print("\n🧹 清理资源...")
            self.ros2_client.disconnect()
            self.controller_client.disconnect()
            print("✅ 清理完成")


def main():
    """主函数"""
    test = RealtimeStatusFeedbackTest()
    
    try:
        success = test.run_comprehensive_test()
        
        if success:
            print("\n🚀 实时状态反馈功能已就绪，可以为ROS2 NAV2提供决策支持!")
        else:
            print("\n🔧 实时状态反馈功能需要进一步调试")
            
    except KeyboardInterrupt:
        print("\n👋 测试被用户中断")
    except Exception as e:
        print(f"\n💥 测试失败: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()