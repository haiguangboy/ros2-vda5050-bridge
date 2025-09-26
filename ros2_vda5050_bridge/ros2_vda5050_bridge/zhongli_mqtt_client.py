#!/usr/bin/env python3
"""
中力具身机器人系统MQTT客户端
Zhongli Embodied Robot System MQTT Client
"""

import json
import logging
from typing import Callable, Optional, Dict, Any
import paho.mqtt.client as mqtt
from .new_protocol_types import (
    TrajectoryMessage, TrajectoryStatusMessage, ActionMessage, ActionStatusMessage,
    TaskMessage, TaskStatusMessage, DeviceStateMessage, create_timestamp
)


class ZhongliMQTTClient:
    """中力具身机器人系统MQTT客户端"""
    
    def __init__(self, 
                 robot_id: str = "robot-001",
                 broker_host: str = "localhost",
                 broker_port: int = 1883,
                 protocol_version: str = "new"):
        """
        初始化MQTT客户端
        
        Args:
            robot_id: 机器人ID
            broker_host: MQTT broker地址
            broker_port: MQTT broker端口
            protocol_version: 协议版本 ("new" 或 "vda5050")
        """
        self.robot_id = robot_id
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.protocol_version = protocol_version
        
        # MQTT客户端
        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect
        
        # 连接状态
        self.connected = False
        
        # 消息处理器 - 新协议
        self.task_handler: Optional[Callable[[TaskMessage], None]] = None
        self.trajectory_status_handler: Optional[Callable[[TrajectoryStatusMessage], None]] = None
        self.action_status_handler: Optional[Callable[[ActionStatusMessage], None]] = None
        
        # 消息ID计数器
        self.message_ids = {
            'trajectory': 0,
            'action': 0,
            'device_state': 0
        }
        
        # 设置日志
        self.logger = logging.getLogger(__name__)
        
        # Topic定义 - 新协议
        self.topics = {
            'task_subscribe': f"EP/master/{self.robot_id}/task",
            'task_status_publish': f"EP/master/{self.robot_id}/task_status",
            'trajectory_publish': f"EP/{self.robot_id}/embrain/cerebellum/trajectory",
            'trajectory_status_subscribe': f"EP/{self.robot_id}/cerebellum/embrain/trajectory_status",
            'action_publish': f"EP/{self.robot_id}/embrain/cerebellum/action",
            'action_status_subscribe': f"EP/{self.robot_id}/cerebellum/embrain/action_status",
            'state_publish': f"EP/master/{self.robot_id}/state"
        }
    
    def _on_connect(self, client, userdata, flags, rc):
        """连接回调"""
        if rc == 0:
            self.connected = True
            self.logger.info(f"Connected to MQTT broker at {self.broker_host}:{self.broker_port}")
            
            # 订阅相关主题
            subscribe_topics = [
                (self.topics['task_subscribe'], 0),
                (self.topics['trajectory_status_subscribe'], 0),
                (self.topics['action_status_subscribe'], 0)
            ]
            
            for topic, qos in subscribe_topics:
                client.subscribe(topic, qos=qos)
                self.logger.info(f"Subscribed to: {topic}")
                
        else:
            self.logger.error(f"Failed to connect to MQTT broker, return code {rc}")
            self.connected = False
    
    def _on_message(self, client, userdata, msg):
        """消息接收回调"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            
            self.logger.debug(f"Received message on topic: {topic}")
            
            # 根据主题分发消息
            if topic == self.topics['task_subscribe']:
                self._handle_task_message(data)
            elif topic == self.topics['trajectory_status_subscribe']:
                self._handle_trajectory_status_message(data)
            elif topic == self.topics['action_status_subscribe']:
                self._handle_action_status_message(data)
            else:
                self.logger.warning(f"Unhandled topic: {topic}")
                
        except Exception as e:
            self.logger.error(f"Error processing message: {e}")
    
    def _on_disconnect(self, client, userdata, rc):
        """断开连接回调"""
        self.connected = False
        if rc != 0:
            self.logger.warning("Unexpected disconnection from MQTT broker")
        else:
            self.logger.info("Disconnected from MQTT broker")
    
    def _handle_task_message(self, data: Dict[str, Any]):
        """处理任务消息"""
        try:
            task = TaskMessage.from_dict(data)
            self.logger.info(f"Received task: {task.taskId}")
            
            if self.task_handler:
                self.task_handler(task)
            else:
                self.logger.warning("No task handler registered")
                
        except Exception as e:
            self.logger.error(f"Error handling task message: {e}")
    
    def _handle_trajectory_status_message(self, data: Dict[str, Any]):
        """处理轨迹状态消息"""
        try:
            trajectory_status = TrajectoryStatusMessage(
                timestamp=data['timestamp'],
                trajectoryId=data['trajectoryId'],
                status=data['status'],
                currentPointIndex=data.get('currentPointIndex'),
                errorCode=data.get('errorCode', 0),
                errorDesc=data.get('errorDesc', ''),
                estimatedFinishTime=data.get('estimatedFinishTime'),
                finishTime=data.get('finishTime')
            )
            
            self.logger.info(f"Received trajectory status: {trajectory_status.trajectoryId} - {trajectory_status.status}")
            
            if self.trajectory_status_handler:
                self.trajectory_status_handler(trajectory_status)
                
        except Exception as e:
            self.logger.error(f"Error handling trajectory status message: {e}")
    
    def _handle_action_status_message(self, data: Dict[str, Any]):
        """处理动作状态消息"""
        try:
            action_status = ActionStatusMessage(
                timestamp=data['timestamp'],
                actionId=data['actionId'],
                status=data['status'],
                errorCode=data.get('errorCode', 0),
                errorDesc=data.get('errorDesc', ''),
                finishTime=data.get('finishTime')
            )
            
            self.logger.info(f"Received action status: {action_status.actionId} - {action_status.status}")
            
            if self.action_status_handler:
                self.action_status_handler(action_status)
                
        except Exception as e:
            self.logger.error(f"Error handling action status message: {e}")
    
    def connect(self) -> bool:
        """连接到MQTT broker"""
        try:
            # 设置最后遗嘱消息（可选）
            # 在新协议中，可以设置一个离线状态消息
            
            # 连接到broker
            self.client.connect(self.broker_host, self.broker_port, 60)
            self.client.loop_start()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect to MQTT broker: {e}")
            return False
    
    def disconnect(self):
        """断开MQTT连接"""
        if self.connected:
            # 可以发送离线状态消息
            self.client.loop_stop()
            self.client.disconnect()
    
    def _get_next_message_id(self, message_type: str) -> int:
        """获取下一个消息ID"""
        self.message_ids[message_type] += 1
        return self.message_ids[message_type]
    
    def publish_trajectory(self, trajectory: TrajectoryMessage) -> bool:
        """发布轨迹指令"""
        if not self.connected:
            self.logger.warning("Not connected to MQTT broker")
            return False
        
        try:
            json_msg = trajectory.to_json()
            result = self.client.publish(self.topics['trajectory_publish'], json_msg, qos=0)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.logger.info(f"Published trajectory: {trajectory.trajectoryId}")
                return True
            else:
                self.logger.error(f"Failed to publish trajectory: {result.rc}")
                return False
                
        except Exception as e:
            self.logger.error(f"Error publishing trajectory: {e}")
            return False
    
    def publish_action(self, action: ActionMessage) -> bool:
        """发布动作指令"""
        if not self.connected:
            self.logger.warning("Not connected to MQTT broker")
            return False
        
        try:
            json_msg = action.to_json()
            result = self.client.publish(self.topics['action_publish'], json_msg, qos=0)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.logger.info(f"Published action: {action.actionId}")
                return True
            else:
                self.logger.error(f"Failed to publish action: {result.rc}")
                return False
                
        except Exception as e:
            self.logger.error(f"Error publishing action: {e}")
            return False
    
    def publish_task_status(self, task_status: TaskStatusMessage) -> bool:
        """发布任务状态"""
        if not self.connected:
            self.logger.warning("Not connected to MQTT broker")
            return False
        
        try:
            json_msg = task_status.to_json()
            result = self.client.publish(self.topics['task_status_publish'], json_msg, qos=0)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.logger.info(f"Published task status: {task_status.taskId} - {task_status.status}")
                return True
            else:
                self.logger.error(f"Failed to publish task status: {result.rc}")
                return False
                
        except Exception as e:
            self.logger.error(f"Error publishing task status: {e}")
            return False
    
    def publish_device_state(self, device_state: DeviceStateMessage) -> bool:
        """发布设备状态"""
        if not self.connected:
            self.logger.warning("Not connected to MQTT broker")
            return False
        
        try:
            json_msg = device_state.to_json()
            result = self.client.publish(self.topics['state_publish'], json_msg, qos=0)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.logger.debug(f"Published device state")
                return True
            else:
                self.logger.error(f"Failed to publish device state: {result.rc}")
                return False
                
        except Exception as e:
            self.logger.error(f"Error publishing device state: {e}")
            return False
    
    # 设置消息处理器
    def set_task_handler(self, handler: Callable[[TaskMessage], None]):
        """设置任务消息处理器"""
        self.task_handler = handler
        self.logger.info("Task handler registered")
    
    def set_trajectory_status_handler(self, handler: Callable[[TrajectoryStatusMessage], None]):
        """设置轨迹状态消息处理器"""
        self.trajectory_status_handler = handler
        self.logger.info("Trajectory status handler registered")
    
    def set_action_status_handler(self, handler: Callable[[ActionStatusMessage], None]):
        """设置动作状态消息处理器"""
        self.action_status_handler = handler
        self.logger.info("Action status handler registered")
    
    def is_connected(self) -> bool:
        """检查连接状态"""
        return self.connected
    
    def get_topic_info(self) -> Dict[str, str]:
        """获取Topic信息"""
        return self.topics.copy()