#!/usr/bin/env python3
"""
Test MQTT Client for VDA5050
"""

import time
import logging
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ros2_vda5050_bridge.mqtt_client import VDA5050MQTTClient
from ros2_vda5050_bridge.vda5050_types import *

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_mqtt_connection():
    """Test basic MQTT connection"""
    print("Testing MQTT connection...")
    
    # Create MQTT client
    client = VDA5050MQTTClient(
        broker_host="localhost",
        broker_port=1883,
        manufacturer="TestManufacturer",
        serial_number="AGV001"
    )
    
    # Set up handlers
    def order_handler(order: OrderMessage):
        print(f"Received order: {order.orderId} with {len(order.nodes)} nodes")
    
    def instant_actions_handler(actions: dict):
        print(f"Received instant actions: {len(actions.get('actions', []))} actions")
    
    client.set_order_handler(order_handler)
    client.set_instant_actions_handler(instant_actions_handler)
    
    # Try to connect
    if client.connect():
        print("✓ Connected to MQTT broker")
        
        # Wait for connection to establish
        time.sleep(2)
        
        if client.is_connected():
            print("✓ Connection confirmed")
            
            # Test publishing state
            battery = BatteryState(batteryCharge=75.0, charging=False)
            safety = SafetyState(eStop="NONE", fieldViolation=False)
            position = AGVPosition(x=0.0, y=0.0, theta=0.0, mapId="test_map", positionInitialized=True)
            
            state = StateMessage(
                headerId=1,
                timestamp=create_timestamp(),
                version="2.1.0",
                manufacturer="TestManufacturer",
                serialNumber="AGV001",
                orderId="",
                orderUpdateId=0,
                lastNodeId="",
                lastNodeSequenceId=0,
                driving=False,
                actionStates=[],
                batteryState=battery,
                operatingMode="AUTOMATIC",
                errors=[],
                safetyState=safety,
                agvPosition=position
            )
            
            if client.publish_state(state):
                print("✓ State message published")
            else:
                print("❌ Failed to publish state")
            
            # Keep connection alive for a bit
            print("Keeping connection alive for 5 seconds...")
            time.sleep(5)
            
        else:
            print("❌ Connection not established")
        
        client.disconnect()
        print("✓ Disconnected from MQTT broker")
        
    else:
        print("❌ Failed to connect to MQTT broker")
        print("Make sure you have an MQTT broker running on localhost:1883")
        print("You can install and run Mosquitto MQTT broker:")
        print("  sudo apt install mosquitto mosquitto-clients")
        print("  sudo systemctl start mosquitto")

def main():
    """Main test function"""
    try:
        test_mqtt_connection()
        print("\n🎉 MQTT client test completed!")
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()