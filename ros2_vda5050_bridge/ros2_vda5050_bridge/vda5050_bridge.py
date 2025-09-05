#!/usr/bin/env python3
"""
ROS2 to VDA5050 Bridge Node
"""

import rclpy
from rclpy.node import Node as ROS2Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool
import json
import math
import threading
import time
from typing import Optional, List

from .mqtt_client import VDA5050MQTTClient
from .vda5050_types import *


class VDA5050Bridge(ROS2Node):
    """Bridge between ROS2 Navigation and VDA5050 Protocol"""
    
    def __init__(self):
        super().__init__('vda5050_bridge')
        
        # Declare parameters
        self.declare_parameter('mqtt_broker_host', 'localhost')
        self.declare_parameter('mqtt_broker_port', 1883)
        self.declare_parameter('manufacturer', 'ROS2Manufacturer')
        self.declare_parameter('serial_number', 'ROS2_AGV_001')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('state_publish_rate', 2.0)  # Hz
        
        # Get parameters
        self.mqtt_host = self.get_parameter('mqtt_broker_host').value
        self.mqtt_port = self.get_parameter('mqtt_broker_port').value
        self.manufacturer = self.get_parameter('manufacturer').value
        self.serial_number = self.get_parameter('serial_number').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.state_rate = self.get_parameter('state_publish_rate').value
        
        self.get_logger().info(f"Starting VDA5050 Bridge for {self.manufacturer}/{self.serial_number}")
        
        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ROS2 Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cancel_nav_pub = self.create_publisher(Bool, '/cancel_navigation', 10)
        
        # ROS2 Subscribers
        self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.amcl_pose_callback, 
            10
        )
        
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscribe to ROS2 planned path from Nav2
        self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        # Subscribe to map data
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Subscribe to navigation result
        self.create_subscription(
            String,
            '/navigation_result',
            self.navigation_result_callback,
            10
        )
        
        # VDA5050 MQTT Client
        self.mqtt_client = VDA5050MQTTClient(
            broker_host=self.mqtt_host,
            broker_port=self.mqtt_port,
            manufacturer=self.manufacturer,
            serial_number=self.serial_number
        )
        
        # Set MQTT handlers
        self.mqtt_client.set_order_handler(self.handle_vda5050_order)
        self.mqtt_client.set_instant_actions_handler(self.handle_instant_actions)
        
        # State variables
        self.current_pose: Optional[PoseWithCovarianceStamped] = None
        self.current_velocity: Optional[Twist] = None
        self.current_order: Optional[OrderMessage] = None
        self.current_node_index = 0
        self.driving = False
        self.battery_charge = 100.0  # Simulated battery
        self.errors = []
        self.action_states = []
        
        # Path and map management
        self.current_path: Optional[Path] = None
        self.current_map: Optional[OccupancyGrid] = None
        self.path_converted_to_vda5050 = False
        self.target_reached = False
        self.current_target_node: Optional[Node] = None
        
        # Goal tolerance for checking if target is reached
        self.goal_tolerance_xy = 0.2  # meters
        self.goal_tolerance_theta = 0.1  # radians
        
        # Connect to MQTT
        if self.mqtt_client.connect():
            self.get_logger().info("Connected to VDA5050 MQTT broker")
        else:
            self.get_logger().error("Failed to connect to VDA5050 MQTT broker")
            return
        
        # Start state publishing timer
        self.state_timer = self.create_timer(
            1.0 / self.state_rate, 
            self.publish_vda5050_state
        )
        
        self.get_logger().info("VDA5050 Bridge initialized successfully")
    
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle AMCL pose updates"""
        self.current_pose = msg
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands"""
        self.current_velocity = msg
        # Update driving state based on velocity
        self.driving = (abs(msg.linear.x) > 0.01 or 
                       abs(msg.linear.y) > 0.01 or 
                       abs(msg.angular.z) > 0.01)
        
        # Check if we've reached the target node
        self.check_target_reached()
    
    def path_callback(self, msg: Path):
        """Handle ROS2 planned path - convert to VDA5050 order"""
        self.get_logger().info(f"Received ROS2 path with {len(msg.poses)} waypoints")
        self.current_path = msg
        
        # Convert ROS2 path to VDA5050 order
        if not self.path_converted_to_vda5050:
            self.convert_ros2_path_to_vda5050_order(msg)
            self.path_converted_to_vda5050 = True
    
    def map_callback(self, msg: OccupancyGrid):
        """Handle map updates - shared between ROS2 and VDA5050"""
        self.get_logger().info(f"Received map update: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}")
        self.current_map = msg
        
        # Publish map info to VDA5050 if needed
        self.publish_map_info_to_vda5050(msg)
    
    def navigation_result_callback(self, msg: String):
        """Handle navigation result from Nav2"""
        result = msg.data
        self.get_logger().info(f"Navigation result: {result}")
        
        if result == "SUCCEEDED":
            self.target_reached = True
            self.report_target_reached()
        elif result == "FAILED":
            self.report_navigation_failed()
    
    def convert_ros2_path_to_vda5050_order(self, path: Path):
        """Convert ROS2 Path to VDA5050 Order"""
        if not path.poses:
            self.get_logger().warning("Empty path received")
            return
        
        self.get_logger().info(f"Converting ROS2 path to VDA5050 order with {len(path.poses)} waypoints")
        
        # Sample waypoints from the path (don't use every single point)
        sample_interval = max(1, len(path.poses) // 10)  # Sample ~10 points
        sampled_poses = path.poses[::sample_interval]
        
        # Always include the last pose
        if path.poses[-1] not in sampled_poses:
            sampled_poses.append(path.poses[-1])
        
        nodes = []
        edges = []
        
        for i, pose_stamped in enumerate(sampled_poses):
            pose = pose_stamped.pose
            
            # Convert quaternion to yaw
            quat = pose.orientation
            yaw = math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            )
            
            # Create VDA5050 node
            node = Node(
                nodeId=f"path_node_{i}",
                sequenceId=i * 2,  # Even numbers for nodes
                released=True,
                nodePosition=NodePosition(
                    x=pose.position.x,
                    y=pose.position.y,
                    theta=yaw,
                    mapId="shared_map",
                    allowedDeviationXY=0.3
                ),
                actions=[]
            )
            
            # Add special action for the last node
            if i == len(sampled_poses) - 1:
                node.actions.append(Action(
                    actionId=f"reach_target_{int(time.time())}",
                    actionType="reportPosition",
                    blockingType="SOFT",
                    actionDescription="Report reaching target position"
                ))
            
            nodes.append(node)
            
            # Create edge to next node (except for the last node)
            if i < len(sampled_poses) - 1:
                next_pose = sampled_poses[i + 1].pose
                
                # Calculate distance for edge length
                dx = next_pose.position.x - pose.position.x
                dy = next_pose.position.y - pose.position.y
                length = math.sqrt(dx*dx + dy*dy)
                
                edge = Edge(
                    edgeId=f"path_edge_{i}",
                    sequenceId=i * 2 + 1,  # Odd numbers for edges
                    released=True,
                    startNodeId=f"path_node_{i}",
                    endNodeId=f"path_node_{i+1}",
                    maxSpeed=1.0,  # Default speed
                    length=length,
                    actions=[]
                )
                edges.append(edge)
        
        # Create VDA5050 order
        order = OrderMessage(
            headerId=1,
            timestamp=create_timestamp(),
            version="2.1.0",
            manufacturer=self.manufacturer,
            serialNumber=self.serial_number,
            orderId=f"ros2_path_{int(time.time())}",
            orderUpdateId=0,
            nodes=nodes,
            edges=edges
        )
        
        # Store as current order and publish to VDA5050
        self.current_order = order
        self.current_target_node = nodes[-1] if nodes else None
        
        self.get_logger().info(f"Created VDA5050 order with {len(nodes)} nodes and {len(edges)} edges")
        
        # Publish the order via MQTT (simulate external VDA5050 master control)
        self.publish_vda5050_order_externally(order)
    
    def publish_map_info_to_vda5050(self, map_msg: OccupancyGrid):
        """Publish map information to VDA5050 system"""
        # In a real implementation, this would publish map data to VDA5050
        # For now, we just log the map info
        map_info = {
            "mapId": "shared_map",
            "width": map_msg.info.width,
            "height": map_msg.info.height,
            "resolution": map_msg.info.resolution,
            "origin": {
                "x": map_msg.info.origin.position.x,
                "y": map_msg.info.origin.position.y,
                "theta": 0.0
            }
        }
        
        self.get_logger().info(f"Map info for VDA5050: {map_info}")
    
    def publish_vda5050_order_externally(self, order: OrderMessage):
        """Simulate external VDA5050 master control publishing order"""
        # This simulates an external VDA5050 master control system
        # In a real system, this would be done by the actual master control
        
        if self.mqtt_client.is_connected():
            topic = f"uagv/v2/{self.manufacturer}/{self.serial_number}/order"
            order_json = order.to_json()
            
            # Use the internal MQTT client to simulate external order
            self.mqtt_client.client.publish(topic, order_json, qos=0)
            self.get_logger().info(f"Published VDA5050 order externally: {order.orderId}")
    
    def check_target_reached(self):
        """Check if AGV has reached the current target node"""
        if not self.current_target_node or not self.current_target_node.nodePosition:
            return
        
        current_pos = self.get_current_position()
        if not current_pos:
            return
        
        target_pos = self.current_target_node.nodePosition
        
        # Calculate distance to target
        dx = current_pos.x - target_pos.x
        dy = current_pos.y - target_pos.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate angular difference
        angle_diff = abs(current_pos.theta - (target_pos.theta or 0.0))
        angle_diff = min(angle_diff, 2*math.pi - angle_diff)  # Normalize to [0, pi]
        
        # Check if within tolerance
        if (distance <= self.goal_tolerance_xy and 
            angle_diff <= self.goal_tolerance_theta and
            not self.target_reached):
            
            self.target_reached = True
            self.report_target_reached()
    
    def report_target_reached(self):
        """Report that target has been reached"""
        if self.current_target_node:
            self.get_logger().info(f"Target reached: {self.current_target_node.nodeId}")
            
            # Execute target node actions
            for action in self.current_target_node.actions:
                if action.actionType == "reportPosition":
                    # Mark action as completed
                    action_state = ActionState(
                        actionId=action.actionId,
                        actionStatus="FINISHED",
                        actionType=action.actionType,
                        resultDescription="Target position reached successfully"
                    )
                    self.action_states.append(action_state)
                    
                    self.get_logger().info("Position reported - target reached successfully")
            
            # Reset flags
            self.path_converted_to_vda5050 = False
            self.current_target_node = None
    
    def report_navigation_failed(self):
        """Report navigation failure"""
        self.get_logger().error("Navigation failed")
        
        # Add error to state
        error = {
            "errorType": "NAVIGATION_FAILED",
            "errorLevel": "WARNING",
            "errorDescription": "Failed to reach target position",
            "errorReferences": [
                {
                    "referenceKey": "orderId",
                    "referenceValue": self.current_order.orderId if self.current_order else ""
                }
            ]
        }
        self.errors.append(error)
        
        # Reset flags
        self.path_converted_to_vda5050 = False
        self.target_reached = False
    
    def handle_vda5050_order(self, order: OrderMessage):
        """Handle incoming VDA5050 order"""
        self.get_logger().info(f"Received VDA5050 order: {order.orderId}")
        self.get_logger().info(f"Order has {len(order.nodes)} nodes and {len(order.edges)} edges")
        
        self.current_order = order
        self.current_node_index = 0
        
        # Process the order - convert to ROS2 navigation goals
        self.process_vda5050_order(order)
    
    def process_vda5050_order(self, order: OrderMessage):
        """Convert VDA5050 order to ROS2 navigation goals"""
        if not order.nodes:
            self.get_logger().warning("Received empty order")
            return
        
        # Find the first released node that we haven't reached yet
        target_node = None
        for i, node in enumerate(order.nodes):
            if node.released and node.nodePosition:
                target_node = node
                self.current_node_index = i
                break
        
        if not target_node or not target_node.nodePosition:
            self.get_logger().warning("No valid target node found in order")
            return
        
        # Create ROS2 goal from VDA5050 node
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.map_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.position.x = target_node.nodePosition.x
        goal_msg.pose.position.y = target_node.nodePosition.y
        goal_msg.pose.position.z = 0.0
        
        # Set orientation (if provided)
        if target_node.nodePosition.theta is not None:
            # Convert theta to quaternion
            yaw = target_node.nodePosition.theta
            goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal_msg.pose.orientation.w = 1.0
        
        # Publish goal
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Published navigation goal to node {target_node.nodeId} at ({target_node.nodePosition.x}, {target_node.nodePosition.y})")
        
        # Execute node actions
        self.execute_node_actions(target_node)
    
    def execute_node_actions(self, node: Node):
        """Execute actions associated with a node"""
        for action in node.actions:
            self.get_logger().info(f"Executing action: {action.actionType} (ID: {action.actionId})")
            
            # Add action to state
            action_state = ActionState(
                actionId=action.actionId,
                actionStatus="RUNNING",
                actionType=action.actionType,
                actionDescription=action.actionDescription
            )
            self.action_states.append(action_state)
            
            # Process different action types
            if action.actionType == "wait":
                # Handle wait action
                duration = 1.0  # default
                for param in action.actionParameters:
                    if param.key == "duration":
                        duration = float(param.value)
                
                self.get_logger().info(f"Waiting for {duration} seconds")
                # In a real implementation, this should be non-blocking
                threading.Timer(duration, lambda: self.complete_action(action.actionId)).start()
                
            elif action.actionType == "pick":
                # Handle pick action
                self.get_logger().info("Executing pick action")
                # Simulate pick completion
                threading.Timer(2.0, lambda: self.complete_action(action.actionId)).start()
                
            elif action.actionType == "drop":
                # Handle drop action
                self.get_logger().info("Executing drop action")
                # Simulate drop completion
                threading.Timer(2.0, lambda: self.complete_action(action.actionId)).start()
                
            else:
                self.get_logger().warning(f"Unknown action type: {action.actionType}")
                self.complete_action(action.actionId, "FAILED")
    
    def complete_action(self, action_id: str, status: str = "FINISHED"):
        """Mark an action as completed"""
        for action_state in self.action_states:
            if action_state.actionId == action_id:
                action_state.actionStatus = status
                self.get_logger().info(f"Action {action_id} completed with status: {status}")
                break
    
    def handle_instant_actions(self, actions_data: dict):
        """Handle instant actions from VDA5050"""
        actions = actions_data.get('actions', [])
        self.get_logger().info(f"Received {len(actions)} instant actions")
        
        for action_data in actions:
            action_type = action_data.get('actionType', '')
            action_id = action_data.get('actionId', '')
            
            if action_type == "pause":
                self.get_logger().info("Pausing robot")
                # Stop the robot
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                
            elif action_type == "resume":
                self.get_logger().info("Resuming robot")
                # Resume navigation if we have an active order
                if self.current_order:
                    self.process_vda5050_order(self.current_order)
                    
            elif action_type == "cancelOrder":
                self.get_logger().info("Cancelling current order")
                self.current_order = None
                # Cancel navigation
                cancel_msg = Bool()
                cancel_msg.data = True
                self.cancel_nav_pub.publish(cancel_msg)
    
    def get_current_position(self) -> Optional[AGVPosition]:
        """Get current AGV position from TF"""
        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            
            # Extract position and orientation
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Convert quaternion to yaw
            quat = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            )
            
            return AGVPosition(
                x=x,
                y=y,
                theta=yaw,
                mapId="ros2_map",
                positionInitialized=True,
                localizationScore=0.9  # Assume good localization
            )
            
        except Exception as e:
            self.get_logger().debug(f"Could not get transform: {e}")
            return None
    
    def publish_vda5050_state(self):
        """Publish current AGV state to VDA5050"""
        if not self.mqtt_client.is_connected():
            return
        
        # Get current position
        position = self.get_current_position()
        if not position and self.current_pose:
            # Fallback to AMCL pose
            pose = self.current_pose.pose.pose
            yaw = math.atan2(
                2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z)
            )
            position = AGVPosition(
                x=pose.position.x,
                y=pose.position.y,
                theta=yaw,
                mapId="ros2_map",
                positionInitialized=True
            )
        
        # Create velocity info
        velocity = None
        if self.current_velocity:
            velocity = Velocity(
                vx=self.current_velocity.linear.x,
                vy=self.current_velocity.linear.y,
                omega=self.current_velocity.angular.z
            )
        
        # Create battery state (simulated)
        battery = BatteryState(
            batteryCharge=self.battery_charge,
            charging=False
        )
        
        # Create safety state
        safety = SafetyState(
            eStop="NONE",
            fieldViolation=False
        )
        
        # Create node and edge states from current order
        node_states = []
        edge_states = []
        last_node_id = ""
        last_node_seq_id = 0
        
        if self.current_order:
            for node in self.current_order.nodes:
                node_state = NodeState(
                    nodeId=node.nodeId,
                    sequenceId=node.sequenceId,
                    released=node.released,
                    nodeDescription=node.nodeDescription,
                    nodePosition=node.nodePosition
                )
                node_states.append(node_state)
                
                if node.released:
                    last_node_id = node.nodeId
                    last_node_seq_id = node.sequenceId
            
            for edge in self.current_order.edges:
                edge_state = EdgeState(
                    edgeId=edge.edgeId,
                    sequenceId=edge.sequenceId,
                    released=edge.released,
                    edgeDescription=edge.edgeDescription
                )
                edge_states.append(edge_state)
        
        # Create state message
        state = StateMessage(
            headerId=0,  # Will be set by MQTT client
            timestamp=create_timestamp(),
            version="2.1.0",
            manufacturer=self.manufacturer,
            serialNumber=self.serial_number,
            orderId=self.current_order.orderId if self.current_order else "",
            orderUpdateId=self.current_order.orderUpdateId if self.current_order else 0,
            lastNodeId=last_node_id,
            lastNodeSequenceId=last_node_seq_id,
            driving=self.driving,
            actionStates=self.action_states.copy(),
            batteryState=battery,
            operatingMode="AUTOMATIC",
            errors=self.errors.copy(),
            safetyState=safety,
            nodeStates=node_states,
            edgeStates=edge_states,
            agvPosition=position,
            velocity=velocity
        )
        
        # Publish state
        self.mqtt_client.publish_state(state)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down VDA5050 Bridge")
        if self.mqtt_client:
            self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        bridge = VDA5050Bridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if 'bridge' in locals():
            bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()