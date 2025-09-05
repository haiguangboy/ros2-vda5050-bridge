#!/usr/bin/env python3
"""
VDA5050 MQTT Client
"""

import json
import logging
from typing import Callable, Optional, Dict, Any
import paho.mqtt.client as mqtt
from .vda5050_types import OrderMessage, StateMessage, create_timestamp


class VDA5050MQTTClient:
    """MQTT Client for VDA5050 Protocol"""
    
    def __init__(self, 
                 broker_host: str = "localhost",
                 broker_port: int = 1883,
                 manufacturer: str = "TestManufacturer",
                 serial_number: str = "AGV001",
                 interface_name: str = "uagv",
                 version: str = "v2"):
        
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.manufacturer = manufacturer
        self.serial_number = serial_number
        self.interface_name = interface_name
        self.version = version
        
        # Topic structure: interfaceName/majorVersion/manufacturer/serialNumber/topic
        self.topic_prefix = f"{interface_name}/{version}/{manufacturer}/{serial_number}"
        
        # MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect
        
        # Message handlers
        self.order_handler: Optional[Callable[[OrderMessage], None]] = None
        self.instant_actions_handler: Optional[Callable[[Dict[str, Any]], None]] = None
        
        # Connection state
        self.connected = False
        
        # Header IDs for each topic
        self.header_ids = {
            'state': 0,
            'connection': 0,
            'factsheet': 0,
            'visualization': 0
        }
        
        # Setup logging
        self.logger = logging.getLogger(__name__)
        
    def _on_connect(self, client, userdata, flags, rc):
        """Callback for when the client receives a CONNACK response from the server"""
        if rc == 0:
            self.connected = True
            self.logger.info(f"Connected to MQTT broker at {self.broker_host}:{self.broker_port}")
            
            # Subscribe to incoming topics
            order_topic = f"{self.topic_prefix}/order"
            instant_actions_topic = f"{self.topic_prefix}/instantActions"
            
            client.subscribe(order_topic, qos=0)
            client.subscribe(instant_actions_topic, qos=0)
            
            self.logger.info(f"Subscribed to: {order_topic}")
            self.logger.info(f"Subscribed to: {instant_actions_topic}")
            
            # Send connection message
            self._send_connection_message("ONLINE")
            
        else:
            self.logger.error(f"Failed to connect to MQTT broker, return code {rc}")
            self.connected = False
    
    def _on_message(self, client, userdata, msg):
        """Callback for when a PUBLISH message is received from the server"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            
            self.logger.info(f"Received message on topic: {topic}")
            
            if topic.endswith('/order'):
                if self.order_handler:
                    order = OrderMessage.from_dict(data)
                    self.order_handler(order)
                else:
                    self.logger.warning("Received order but no handler registered")
                    
            elif topic.endswith('/instantActions'):
                if self.instant_actions_handler:
                    self.instant_actions_handler(data)
                else:
                    self.logger.warning("Received instant actions but no handler registered")
                    
        except Exception as e:
            self.logger.error(f"Error processing message: {e}")
    
    def _on_disconnect(self, client, userdata, rc):
        """Callback for when the client disconnects from the server"""
        self.connected = False
        if rc != 0:
            self.logger.warning("Unexpected disconnection from MQTT broker")
        else:
            self.logger.info("Disconnected from MQTT broker")
    
    def connect(self) -> bool:
        """Connect to MQTT broker"""
        try:
            # Set last will message
            connection_topic = f"{self.topic_prefix}/connection"
            last_will_msg = json.dumps({
                "headerId": self._get_next_header_id('connection'),
                "timestamp": create_timestamp(),
                "version": "2.1.0",
                "manufacturer": self.manufacturer,
                "serialNumber": self.serial_number,
                "connectionState": "OFFLINE"
            })
            
            self.client.will_set(connection_topic, last_will_msg, qos=1, retain=False)
            
            # Connect to broker
            self.client.connect(self.broker_host, self.broker_port, 60)
            self.client.loop_start()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect to MQTT broker: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from MQTT broker"""
        if self.connected:
            self._send_connection_message("OFFLINE")
        self.client.loop_stop()
        self.client.disconnect()
    
    def _get_next_header_id(self, topic: str) -> int:
        """Get next header ID for a topic"""
        self.header_ids[topic] += 1
        return self.header_ids[topic]
    
    def _send_connection_message(self, state: str):
        """Send connection state message"""
        topic = f"{self.topic_prefix}/connection"
        message = {
            "headerId": self._get_next_header_id('connection'),
            "timestamp": create_timestamp(),
            "version": "2.1.0",
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,
            "connectionState": state
        }
        
        self.client.publish(topic, json.dumps(message), qos=1)
        self.logger.info(f"Sent connection state: {state}")
    
    def publish_state(self, state_msg: StateMessage):
        """Publish AGV state message"""
        if not self.connected:
            self.logger.warning("Not connected to MQTT broker")
            return False
            
        topic = f"{self.topic_prefix}/state"
        state_msg.headerId = self._get_next_header_id('state')
        
        try:
            json_msg = state_msg.to_json()
            self.client.publish(topic, json_msg, qos=0)
            self.logger.debug(f"Published state message, headerId: {state_msg.headerId}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to publish state: {e}")
            return False
    
    def set_order_handler(self, handler: Callable[[OrderMessage], None]):
        """Set handler for incoming order messages"""
        self.order_handler = handler
        self.logger.info("Order handler registered")
    
    def set_instant_actions_handler(self, handler: Callable[[Dict[str, Any]], None]):
        """Set handler for incoming instant actions messages"""
        self.instant_actions_handler = handler
        self.logger.info("Instant actions handler registered")
    
    def is_connected(self) -> bool:
        """Check if connected to MQTT broker"""
        return self.connected