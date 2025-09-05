#!/usr/bin/env python3
"""
VDA5050 Protocol Data Types and Message Structures
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from datetime import datetime
import json


@dataclass
class ActionParameter:
    """VDA5050 Action Parameter"""
    key: str
    value: Any


@dataclass
class Action:
    """VDA5050 Action"""
    actionId: str
    actionType: str
    blockingType: str  # NONE, SOFT, HARD
    actionDescription: Optional[str] = None
    actionParameters: List[ActionParameter] = field(default_factory=list)


@dataclass
class NodePosition:
    """VDA5050 Node Position"""
    x: float
    y: float
    mapId: str
    theta: Optional[float] = None
    allowedDeviationXY: Optional[float] = None
    allowedDeviationTheta: Optional[float] = None
    mapDescription: Optional[str] = None


@dataclass
class Node:
    """VDA5050 Node"""
    nodeId: str
    sequenceId: int
    released: bool
    actions: List[Action] = field(default_factory=list)
    nodeDescription: Optional[str] = None
    nodePosition: Optional[NodePosition] = None


@dataclass
class ControlPoint:
    """VDA5050 NURBS Control Point"""
    x: float
    y: float
    weight: float = 1.0


@dataclass
class Trajectory:
    """VDA5050 NURBS Trajectory"""
    degree: int
    knotVector: List[float]
    controlPoints: List[ControlPoint]


@dataclass
class Edge:
    """VDA5050 Edge"""
    edgeId: str
    sequenceId: int
    released: bool
    startNodeId: str
    endNodeId: str
    actions: List[Action] = field(default_factory=list)
    edgeDescription: Optional[str] = None
    maxSpeed: Optional[float] = None
    orientation: Optional[float] = None
    orientationType: Optional[str] = "TANGENTIAL"
    rotationAllowed: Optional[bool] = None
    maxRotationSpeed: Optional[float] = None
    length: Optional[float] = None
    trajectory: Optional[Trajectory] = None


@dataclass
class OrderMessage:
    """VDA5050 Order Message"""
    headerId: int
    timestamp: str
    version: str
    manufacturer: str
    serialNumber: str
    orderId: str
    orderUpdateId: int
    nodes: List[Node]
    edges: List[Edge]
    zoneSetId: Optional[str] = None

    def to_json(self) -> str:
        """Convert to JSON string"""
        return json.dumps(self, default=lambda o: o.__dict__, indent=2)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'OrderMessage':
        """Create OrderMessage from dictionary"""
        # Convert nodes
        nodes = []
        for node_data in data.get('nodes', []):
            node_pos = None
            if 'nodePosition' in node_data:
                pos_data = node_data['nodePosition']
                node_pos = NodePosition(**pos_data)
            
            actions = []
            for action_data in node_data.get('actions', []):
                params = [ActionParameter(**p) for p in action_data.get('actionParameters', [])]
                actions.append(Action(
                    actionId=action_data['actionId'],
                    actionType=action_data['actionType'],
                    blockingType=action_data['blockingType'],
                    actionDescription=action_data.get('actionDescription'),
                    actionParameters=params
                ))
            
            nodes.append(Node(
                nodeId=node_data['nodeId'],
                sequenceId=node_data['sequenceId'],
                released=node_data['released'],
                nodeDescription=node_data.get('nodeDescription'),
                nodePosition=node_pos,
                actions=actions
            ))
        
        # Convert edges
        edges = []
        for edge_data in data.get('edges', []):
            actions = []
            for action_data in edge_data.get('actions', []):
                params = [ActionParameter(**p) for p in action_data.get('actionParameters', [])]
                actions.append(Action(
                    actionId=action_data['actionId'],
                    actionType=action_data['actionType'],
                    blockingType=action_data['blockingType'],
                    actionDescription=action_data.get('actionDescription'),
                    actionParameters=params
                ))
            
            trajectory = None
            if 'trajectory' in edge_data:
                traj_data = edge_data['trajectory']
                control_points = [ControlPoint(**cp) for cp in traj_data['controlPoints']]
                trajectory = Trajectory(
                    degree=traj_data['degree'],
                    knotVector=traj_data['knotVector'],
                    controlPoints=control_points
                )
            
            edges.append(Edge(
                edgeId=edge_data['edgeId'],
                sequenceId=edge_data['sequenceId'],
                released=edge_data['released'],
                startNodeId=edge_data['startNodeId'],
                endNodeId=edge_data['endNodeId'],
                edgeDescription=edge_data.get('edgeDescription'),
                maxSpeed=edge_data.get('maxSpeed'),
                orientation=edge_data.get('orientation'),
                orientationType=edge_data.get('orientationType', 'TANGENTIAL'),
                rotationAllowed=edge_data.get('rotationAllowed'),
                maxRotationSpeed=edge_data.get('maxRotationSpeed'),
                length=edge_data.get('length'),
                trajectory=trajectory,
                actions=actions
            ))
        
        return cls(
            headerId=data['headerId'],
            timestamp=data['timestamp'],
            version=data['version'],
            manufacturer=data['manufacturer'],
            serialNumber=data['serialNumber'],
            orderId=data['orderId'],
            orderUpdateId=data['orderUpdateId'],
            zoneSetId=data.get('zoneSetId'),
            nodes=nodes,
            edges=edges
        )


@dataclass
class AGVPosition:
    """VDA5050 AGV Position"""
    x: float
    y: float
    theta: float
    mapId: str
    positionInitialized: bool
    mapDescription: Optional[str] = None
    localizationScore: Optional[float] = None
    deviationRange: Optional[float] = None


@dataclass
class Velocity:
    """VDA5050 Velocity"""
    vx: Optional[float] = None
    vy: Optional[float] = None
    omega: Optional[float] = None


@dataclass
class BatteryState:
    """VDA5050 Battery State"""
    batteryCharge: float
    charging: bool
    batteryVoltage: Optional[float] = None
    batteryHealth: Optional[float] = None
    reach: Optional[float] = None


@dataclass
class SafetyState:
    """VDA5050 Safety State"""
    eStop: str  # AUTOACK, MANUAL, REMOTE, NONE
    fieldViolation: bool


@dataclass
class ActionState:
    """VDA5050 Action State"""
    actionId: str
    actionStatus: str  # WAITING, INITIALIZING, RUNNING, PAUSED, FINISHED, FAILED
    actionType: Optional[str] = None
    actionDescription: Optional[str] = None
    resultDescription: Optional[str] = None


@dataclass
class NodeState:
    """VDA5050 Node State"""
    nodeId: str
    sequenceId: int
    released: bool
    nodeDescription: Optional[str] = None
    nodePosition: Optional[NodePosition] = None


@dataclass
class EdgeState:
    """VDA5050 Edge State"""
    edgeId: str
    sequenceId: int
    released: bool
    edgeDescription: Optional[str] = None
    trajectory: Optional[Trajectory] = None


@dataclass
class StateMessage:
    """VDA5050 State Message"""
    headerId: int
    timestamp: str
    version: str
    manufacturer: str
    serialNumber: str
    orderId: str
    orderUpdateId: int
    lastNodeId: str
    lastNodeSequenceId: int
    driving: bool
    actionStates: List[ActionState]
    batteryState: BatteryState
    operatingMode: str  # AUTOMATIC, SEMIAUTOMATIC, MANUAL, SERVICE, TEACHIN
    errors: List[Dict[str, Any]]
    safetyState: SafetyState
    nodeStates: List[NodeState] = field(default_factory=list)
    edgeStates: List[EdgeState] = field(default_factory=list)
    agvPosition: Optional[AGVPosition] = None
    velocity: Optional[Velocity] = None
    paused: Optional[bool] = None
    newBaseRequest: Optional[bool] = None
    distanceSinceLastNode: Optional[float] = None

    def to_json(self) -> str:
        """Convert to JSON string"""
        return json.dumps(self, default=lambda o: o.__dict__, indent=2)


def create_timestamp() -> str:
    """Create ISO8601 timestamp"""
    return datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%S.%fZ')[:-3] + 'Z'