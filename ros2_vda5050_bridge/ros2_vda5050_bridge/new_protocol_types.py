#!/usr/bin/env python3
"""
中力具身机器人系统通信协议数据类型定义
Zhongli Embodied Robot System Communication Protocol Data Types
"""

import time
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from datetime import datetime
import json


@dataclass
class Pose:
    """机器人位姿信息"""
    x: float
    y: float
    theta: float  # 单位：度
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta
        }


@dataclass
class ContainerPose:
    """容器/货物姿态"""
    x: float
    y: float
    z: float
    theta: float  # 单位：度
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "theta": self.theta
        }


@dataclass
class ForkliftState:
    """货叉状态信息"""
    height: float  # 货叉高度，单位：米
    weight: float  # 货叉负载重量，单位：千克
    lateralShift: float  # 货叉侧移量，单位：米
    forwardExtension: float  # 货叉前伸量，单位：米
    tiltBack: bool  # 货叉是否后倾
    status: str  # 货叉工作状态: ready/moving/error
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "height": self.height,
            "weight": self.weight,
            "lateralShift": self.lateralShift,
            "forwardExtension": self.forwardExtension,
            "tiltBack": self.tiltBack,
            "status": self.status
        }


@dataclass
class BatteryState:
    """电池状态信息"""
    level: int  # 电池剩余电量，百分比 0-100
    charging: bool  # 是否正在充电
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "level": self.level,
            "charging": self.charging
        }


@dataclass
class ErrorInfo:
    """错误信息"""
    code: int
    desc: str
    timestamp: str
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "code": self.code,
            "desc": self.desc,
            "timestamp": self.timestamp
        }


@dataclass
class TrajectoryPoint:
    """轨迹点"""
    x: float
    y: float
    theta: float  # 单位：度
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta
        }


@dataclass
class TrajectoryMessage:
    """具身大脑下发轨迹指令消息"""
    timestamp: str
    trajectoryId: str
    trajectoryPoints: List[TrajectoryPoint]
    maxSpeed: float  # 单位：米/秒
    
    def to_json(self) -> str:
        """转换为JSON字符串"""
        data = {
            "timestamp": self.timestamp,
            "trajectoryId": self.trajectoryId,
            "trajectoryPoints": [point.to_dict() for point in self.trajectoryPoints],
            "maxSpeed": self.maxSpeed
        }
        return json.dumps(data, indent=2, ensure_ascii=False)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TrajectoryMessage':
        """从字典创建轨迹消息"""
        points = [TrajectoryPoint(**point) for point in data.get('trajectoryPoints', [])]
        return cls(
            timestamp=data['timestamp'],
            trajectoryId=data['trajectoryId'],
            trajectoryPoints=points,
            maxSpeed=data['maxSpeed']
        )


@dataclass
class TrajectoryStatusMessage:
    """车载小脑反馈轨迹状态消息"""
    timestamp: str
    trajectoryId: str
    status: str  # pending/running/completed/failed
    currentPointIndex: Optional[int] = None
    errorCode: int = 0
    errorDesc: str = ""
    estimatedFinishTime: Optional[str] = None
    finishTime: Optional[str] = None
    
    def to_json(self) -> str:
        """转换为JSON字符串"""
        data = {
            "timestamp": self.timestamp,
            "trajectoryId": self.trajectoryId,
            "status": self.status,
            "errorCode": self.errorCode,
            "errorDesc": self.errorDesc
        }
        
        if self.currentPointIndex is not None:
            data["currentPointIndex"] = self.currentPointIndex
        if self.estimatedFinishTime:
            data["estimatedFinishTime"] = self.estimatedFinishTime
        if self.finishTime:
            data["finishTime"] = self.finishTime
            
        return json.dumps(data, indent=2, ensure_ascii=False)


@dataclass
class ActionMessage:
    """具身大脑下发动作指令消息"""
    timestamp: str
    actionId: str
    actionType: str  # ground_pick/ground_place/load/unload
    containerPose: Optional[ContainerPose] = None
    containerType: Optional[str] = None
    
    def to_json(self) -> str:
        """转换为JSON字符串"""
        data = {
            "timestamp": self.timestamp,
            "actionId": self.actionId,
            "actionType": self.actionType
        }
        
        if self.containerPose:
            data["containerPose"] = self.containerPose.to_dict()
        if self.containerType:
            data["containerType"] = self.containerType
            
        return json.dumps(data, indent=2, ensure_ascii=False)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ActionMessage':
        """从字典创建动作消息"""
        container_pose = None
        if 'containerPose' in data:
            pose_data = data['containerPose']
            container_pose = ContainerPose(**pose_data)
        
        return cls(
            timestamp=data['timestamp'],
            actionId=data['actionId'],
            actionType=data['actionType'],
            containerPose=container_pose,
            containerType=data.get('containerType')
        )


@dataclass
class ActionStatusMessage:
    """车载小脑反馈动作状态消息"""
    timestamp: str
    actionId: str
    status: str  # success/failed
    errorCode: int = 0
    errorDesc: str = ""
    finishTime: Optional[str] = None
    
    def to_json(self) -> str:
        """转换为JSON字符串"""
        data = {
            "timestamp": self.timestamp,
            "actionId": self.actionId,
            "status": self.status,
            "errorCode": self.errorCode,
            "errorDesc": self.errorDesc
        }
        
        if self.finishTime:
            data["finishTime"] = self.finishTime
            
        return json.dumps(data, indent=2, ensure_ascii=False)


@dataclass
class TaskMessage:
    """调度下发任务消息"""
    timestamp: str
    taskId: str
    startArea: str
    startAction: str  # ground_pick/ground_place/load/unload
    targetArea: str
    targetAction: str  # ground_pick/ground_place/load/unload
    
    def to_json(self) -> str:
        """转换为JSON字符串"""
        data = {
            "timestamp": self.timestamp,
            "taskId": self.taskId,
            "startArea": self.startArea,
            "startAction": self.startAction,
            "targetArea": self.targetArea,
            "targetAction": self.targetAction
        }
        return json.dumps(data, indent=2, ensure_ascii=False)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TaskMessage':
        """从字典创建任务消息"""
        return cls(
            timestamp=data['timestamp'],
            taskId=data['taskId'],
            startArea=data['startArea'],
            startAction=data['startAction'],
            targetArea=data['targetArea'],
            targetAction=data['targetAction']
        )


@dataclass
class TaskStatusMessage:
    """机器人上报任务状态消息"""
    timestamp: str
    taskId: str
    status: str  # success/failed
    finishTime: str
    reason: str = ""  # 失败原因
    
    def to_json(self) -> str:
        """转换为JSON字符串"""
        data = {
            "timestamp": self.timestamp,
            "taskId": self.taskId,
            "status": self.status,
            "finishTime": self.finishTime,
            "reason": self.reason
        }
        return json.dumps(data, indent=2, ensure_ascii=False)


@dataclass
class DeviceStateMessage:
    """车载网关上报设备状态消息"""
    timestamp: str
    pose: Pose
    forkliftState: ForkliftState
    battery: BatteryState
    errors: List[ErrorInfo]
    systemState: str  # idle/running/paused/fault
    
    def to_json(self) -> str:
        """转换为JSON字符串"""
        data = {
            "timestamp": self.timestamp,
            "pose": self.pose.to_dict(),
            "forkliftState": self.forkliftState.to_dict(),
            "battery": self.battery.to_dict(),
            "errors": [error.to_dict() for error in self.errors],
            "systemState": self.systemState
        }
        return json.dumps(data, indent=2, ensure_ascii=False)


def create_timestamp() -> str:
    """创建ISO8601 UTC时间戳"""
    return datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%S.%fZ')[:-3] + 'Z'


def generate_trajectory_id(robot_id: str) -> str:
    """生成轨迹ID"""
    timestamp = datetime.utcnow().strftime('%Y%m%d')
    # 使用时间戳确保唯一性
    sequence = int(time.time() * 1000) % 1000000
    return f"traj-{robot_id}-{timestamp}-{sequence:06d}"


def generate_action_id(robot_id: str) -> str:
    """生成动作ID"""
    timestamp = datetime.utcnow().strftime('%Y%m%d')
    # 使用时间戳确保唯一性
    sequence = int(time.time() * 1000) % 1000000
    return f"action-{robot_id}-{timestamp}-{sequence:06d}"


def generate_task_id(robot_id: str) -> str:
    """生成任务ID"""
    timestamp = datetime.utcnow().strftime('%Y%m%d')
    # 使用时间戳确保唯一性
    sequence = int(time.time() * 1000) % 1000000
    return f"task-{robot_id}-{timestamp}-{sequence:06d}"