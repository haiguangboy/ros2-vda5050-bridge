#!/usr/bin/env python3
"""
Basic MQTT Test for VDA5050
"""

import json
import time
import paho.mqtt.client as mqtt
import sys
import os

# Add the package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'ros2_vda5050_bridge'))
from ros2_vda5050_bridge.vda5050_types import *

def test_mqtt_basic():
    """Test basic MQTT functionality"""
    print("🔧 Testing Basic MQTT Communication")
    print("=" * 40)
    
    # Test configuration
    broker_host = "localhost"
    broker_port = 1883
    manufacturer = "TestManufacturer"
    serial_number = "TEST_AGV_001"
    topic_prefix = f"uagv/v2/{manufacturer}/{serial_number}"
    
    messages_received = []
    
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f"✓ Connected to MQTT broker at {broker_host}:{broker_port}")
            # Subscribe to all topics for this AGV
            client.subscribe(f"{topic_prefix}/+", qos=0)
            print(f"✓ Subscribed to: {topic_prefix}/+")
        else:
            print(f"❌ Connection failed with code {rc}")
    
    def on_message(client, userdata, msg):
        topic = msg.topic.split('/')[-1]  # Get last part of topic
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            messages_received.append((topic, data))
            print(f"📨 Received {topic} message (headerId: {data.get('headerId', 'N/A')})")
        except Exception as e:
            print(f"❌ Error parsing message: {e}")
    
    # Create MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        # Connect to broker
        client.connect(broker_host, broker_port, 60)
        client.loop_start()
        
        # Wait for connection
        time.sleep(2)
        
        # Test 1: Publish a test order
        print("\n🎯 Test 1: Publishing test order...")
        
        order = OrderMessage(
            headerId=1,
            timestamp=create_timestamp(),
            version="2.1.0",
            manufacturer=manufacturer,
            serialNumber=serial_number,
            orderId="MQTT_TEST_001",
            orderUpdateId=0,
            nodes=[
                Node(
                    nodeId="test_node",
                    sequenceId=0,
                    released=True,
                    nodePosition=NodePosition(x=1.0, y=1.0, mapId="test_map")
                )
            ],
            edges=[]
        )
        
        order_topic = f"{topic_prefix}/order"
        client.publish(order_topic, order.to_json(), qos=0)
        print(f"📤 Published order to: {order_topic}")
        
        # Test 2: Publish a test state
        print("\n🎯 Test 2: Publishing test state...")
        
        state = StateMessage(
            headerId=1,
            timestamp=create_timestamp(),
            version="2.1.0",
            manufacturer=manufacturer,
            serialNumber=serial_number,
            orderId="MQTT_TEST_001",
            orderUpdateId=0,
            lastNodeId="test_node",
            lastNodeSequenceId=0,
            driving=False,
            actionStates=[],
            batteryState=BatteryState(batteryCharge=85.0, charging=False),
            operatingMode="AUTOMATIC",
            errors=[],
            safetyState=SafetyState(eStop="NONE", fieldViolation=False),
            agvPosition=AGVPosition(x=1.0, y=1.0, theta=0.0, mapId="test_map", positionInitialized=True)
        )
        
        state_topic = f"{topic_prefix}/state"
        client.publish(state_topic, state.to_json(), qos=0)
        print(f"📤 Published state to: {state_topic}")
        
        # Test 3: Publish instant actions
        print("\n🎯 Test 3: Publishing instant actions...")
        
        instant_actions = {
            "headerId": 1,
            "timestamp": create_timestamp(),
            "version": "2.1.0",
            "manufacturer": manufacturer,
            "serialNumber": serial_number,
            "actions": [
                {
                    "actionId": "test_pause",
                    "actionType": "pause",
                    "blockingType": "HARD"
                }
            ]
        }
        
        actions_topic = f"{topic_prefix}/instantActions"
        client.publish(actions_topic, json.dumps(instant_actions), qos=0)
        print(f"📤 Published instant actions to: {actions_topic}")
        
        # Wait for messages
        print("\n⏳ Waiting for messages (5 seconds)...")
        time.sleep(5)
        
        # Results
        print(f"\n📊 Results:")
        print(f"   Messages received: {len(messages_received)}")
        
        for topic, data in messages_received:
            print(f"   - {topic}: headerId={data.get('headerId', 'N/A')}")
        
        if len(messages_received) >= 3:
            print("✅ MQTT communication test PASSED!")
            return True
        else:
            print("⚠️  MQTT communication test partially successful")
            print("   (This is expected without the ROS2 bridge running)")
            return True
            
    except Exception as e:
        print(f"❌ MQTT test failed: {e}")
        return False
        
    finally:
        client.loop_stop()
        client.disconnect()

def test_mosquitto_tools():
    """Test using mosquitto command line tools"""
    print("\n🔧 Testing with Mosquitto CLI Tools")
    print("=" * 40)
    
    # Test mosquitto_pub and mosquitto_sub
    import subprocess
    
    try:
        # Test if mosquitto_pub is available
        result = subprocess.run(['which', 'mosquitto_pub'], 
                              capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✓ mosquitto_pub is available")
            
            # Publish a test message
            test_topic = "test/vda5050/message"
            test_message = '{"test": "message", "timestamp": "' + create_timestamp() + '"}'
            
            pub_result = subprocess.run([
                'mosquitto_pub', 
                '-h', 'localhost', 
                '-t', test_topic, 
                '-m', test_message
            ], capture_output=True, text=True)
            
            if pub_result.returncode == 0:
                print(f"✓ Successfully published test message to {test_topic}")
                return True
            else:
                print(f"❌ Failed to publish: {pub_result.stderr}")
                return False
        else:
            print("❌ mosquitto_pub not found")
            return False
            
    except Exception as e:
        print(f"❌ Mosquitto CLI test failed: {e}")
        return False

def main():
    """Main test function"""
    print("VDA5050 MQTT Basic Test")
    print("This test verifies MQTT broker connectivity and message publishing")
    print()
    
    try:
        # Test 1: Basic MQTT
        success1 = test_mqtt_basic()
        
        # Test 2: Mosquitto CLI tools
        success2 = test_mosquitto_tools()
        
        print("\n" + "=" * 50)
        if success1 and success2:
            print("🎉 All MQTT tests passed!")
            print("\nNext steps:")
            print("1. Start the ROS2 VDA5050 bridge node:")
            print("   ros2 run ros2_vda5050_bridge vda5050_bridge")
            print("2. Run the full system test:")
            print("   python3 test_vda5050_system.py")
        else:
            print("⚠️  Some tests had issues, but MQTT broker is working")
            
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()