#!/usr/bin/env python3
"""
Test VDA5050 System Integration
"""

import json
import time
import threading
import paho.mqtt.client as mqtt
import sys
import os

# Add the package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'ros2_vda5050_bridge'))
from ros2_vda5050_bridge.vda5050_types import *

class VDA5050TestClient:
    """Test client to simulate VDA5050 master control"""
    
    def __init__(self, broker_host="localhost", broker_port=1883):
        self.broker_host = broker_host
        self.broker_port = broker_port
        
        # MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        
        # Topic configuration
        self.manufacturer = "ROS2Manufacturer"
        self.serial_number = "ROS2_AGV_001"
        self.topic_prefix = f"uagv/v2/{self.manufacturer}/{self.serial_number}"
        
        # State tracking
        self.last_state = None
        self.connected = False
        
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            print(f"✓ Test client connected to MQTT broker")
            
            # Subscribe to AGV state messages
            state_topic = f"{self.topic_prefix}/state"
            client.subscribe(state_topic, qos=0)
            print(f"✓ Subscribed to: {state_topic}")
            
        else:
            print(f"❌ Failed to connect, return code {rc}")
    
    def _on_message(self, client, userdata, msg):
        try:
            if msg.topic.endswith('/state'):
                state_data = json.loads(msg.payload.decode('utf-8'))
                self.last_state = state_data
                print(f"📊 Received state update - Position: ({state_data.get('agvPosition', {}).get('x', 'N/A')}, {state_data.get('agvPosition', {}).get('y', 'N/A')})")
                
        except Exception as e:
            print(f"❌ Error processing message: {e}")
    
    def connect(self):
        """Connect to MQTT broker"""
        try:
            self.client.connect(self.broker_host, self.broker_port, 60)
            self.client.loop_start()
            return True
        except Exception as e:
            print(f"❌ Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from MQTT broker"""
        self.client.loop_stop()
        self.client.disconnect()
    
    def send_test_order(self):
        """Send a test order to the AGV"""
        if not self.connected:
            print("❌ Not connected to MQTT broker")
            return False
        
        # Create test order with multiple nodes
        nodes = [
            Node(
                nodeId="start_node",
                sequenceId=0,
                released=True,
                nodePosition=NodePosition(x=0.0, y=0.0, theta=0.0, mapId="test_map"),
                actions=[]
            ),
            Node(
                nodeId="pickup_node",
                sequenceId=2,
                released=True,
                nodePosition=NodePosition(x=2.0, y=1.0, theta=1.57, mapId="test_map"),
                actions=[
                    Action(
                        actionId="pick_001",
                        actionType="pick",
                        blockingType="HARD",
                        actionParameters=[
                            ActionParameter(key="loadId", value="LOAD_123")
                        ]
                    )
                ]
            ),
            Node(
                nodeId="dropoff_node",
                sequenceId=4,
                released=True,
                nodePosition=NodePosition(x=4.0, y=2.0, theta=3.14, mapId="test_map"),
                actions=[
                    Action(
                        actionId="drop_001",
                        actionType="drop",
                        blockingType="HARD",
                        actionParameters=[
                            ActionParameter(key="loadId", value="LOAD_123")
                        ]
                    )
                ]
            )
        ]
        
        edges = [
            Edge(
                edgeId="edge_start_pickup",
                sequenceId=1,
                released=True,
                startNodeId="start_node",
                endNodeId="pickup_node",
                maxSpeed=1.0
            ),
            Edge(
                edgeId="edge_pickup_dropoff",
                sequenceId=3,
                released=True,
                startNodeId="pickup_node",
                endNodeId="dropoff_node",
                maxSpeed=0.5
            )
        ]
        
        order = OrderMessage(
            headerId=1,
            timestamp=create_timestamp(),
            version="2.1.0",
            manufacturer=self.manufacturer,
            serialNumber=self.serial_number,
            orderId="TEST_ORDER_001",
            orderUpdateId=0,
            nodes=nodes,
            edges=edges
        )
        
        # Publish order
        order_topic = f"{self.topic_prefix}/order"
        order_json = order.to_json()
        
        self.client.publish(order_topic, order_json, qos=0)
        print(f"📤 Sent test order: {order.orderId}")
        print(f"   - {len(nodes)} nodes, {len(edges)} edges")
        
        return True
    
    def send_instant_action(self, action_type: str):
        """Send an instant action"""
        if not self.connected:
            print("❌ Not connected to MQTT broker")
            return False
        
        instant_actions = {
            "headerId": 1,
            "timestamp": create_timestamp(),
            "version": "2.1.0",
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,
            "actions": [
                {
                    "actionId": f"instant_{action_type}_{int(time.time())}",
                    "actionType": action_type,
                    "blockingType": "HARD"
                }
            ]
        }
        
        topic = f"{self.topic_prefix}/instantActions"
        self.client.publish(topic, json.dumps(instant_actions), qos=0)
        print(f"📤 Sent instant action: {action_type}")
        
        return True


def test_vda5050_integration():
    """Test VDA5050 integration"""
    print("🚀 Starting VDA5050 System Integration Test")
    print("=" * 50)
    
    # Create test client
    test_client = VDA5050TestClient()
    
    # Connect to MQTT broker
    if not test_client.connect():
        print("❌ Failed to connect test client to MQTT broker")
        return False
    
    # Wait for connection
    time.sleep(2)
    
    if not test_client.connected:
        print("❌ Test client not connected")
        return False
    
    print("\n📋 Test Plan:")
    print("1. Send test order with pickup and dropoff")
    print("2. Monitor AGV state updates")
    print("3. Send instant actions (pause/resume)")
    print("4. Verify system responses")
    
    # Test 1: Send order
    print("\n🎯 Test 1: Sending test order...")
    if test_client.send_test_order():
        print("✓ Order sent successfully")
    else:
        print("❌ Failed to send order")
        return False
    
    # Wait and monitor state updates
    print("\n⏳ Monitoring state updates for 10 seconds...")
    start_time = time.time()
    state_count = 0
    
    while time.time() - start_time < 10:
        if test_client.last_state:
            state_count += 1
            if state_count % 5 == 0:  # Print every 5th state
                battery = test_client.last_state.get('batteryState', {}).get('batteryCharge', 'N/A')
                driving = test_client.last_state.get('driving', False)
                print(f"   State #{state_count}: Battery={battery}%, Driving={driving}")
        time.sleep(0.5)
    
    if state_count > 0:
        print(f"✓ Received {state_count} state updates")
    else:
        print("❌ No state updates received")
        return False
    
    # Test 2: Send instant actions
    print("\n🎯 Test 2: Testing instant actions...")
    
    # Pause
    if test_client.send_instant_action("pause"):
        print("✓ Pause action sent")
        time.sleep(2)
    
    # Resume
    if test_client.send_instant_action("resume"):
        print("✓ Resume action sent")
        time.sleep(2)
    
    # Cancel order
    if test_client.send_instant_action("cancelOrder"):
        print("✓ Cancel order action sent")
        time.sleep(2)
    
    # Final state check
    print("\n📊 Final System State:")
    if test_client.last_state:
        print(f"   Order ID: {test_client.last_state.get('orderId', 'None')}")
        print(f"   Operating Mode: {test_client.last_state.get('operatingMode', 'Unknown')}")
        print(f"   Safety State: {test_client.last_state.get('safetyState', {}).get('eStop', 'Unknown')}")
        print(f"   Action States: {len(test_client.last_state.get('actionStates', []))}")
    
    # Cleanup
    test_client.disconnect()
    print("\n✅ VDA5050 integration test completed successfully!")
    return True


def main():
    """Main test function"""
    try:
        print("VDA5050 System Integration Test")
        print("Make sure the following are running:")
        print("1. MQTT broker (mosquitto)")
        print("2. ROS2 VDA5050 bridge node")
        print()
        
        input("Press Enter to start the test...")
        
        success = test_vda5050_integration()
        
        if success:
            print("\n🎉 All tests passed!")
        else:
            print("\n❌ Some tests failed!")
            
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()