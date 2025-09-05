#!/usr/bin/env python3
"""
Test VDA5050 Types Module
"""

import sys
sys.path.append('ros2_vda5050_bridge')

from ros2_vda5050_bridge.vda5050_types import *

def test_basic_types():
    """Test basic VDA5050 data types"""
    print("Testing VDA5050 basic types...")
    
    # Test ActionParameter
    param = ActionParameter(key="duration", value=5.0)
    print(f"ActionParameter: {param}")
    
    # Test Action
    action = Action(
        actionId="test_action_1",
        actionType="wait",
        blockingType="SOFT",
        actionParameters=[param]
    )
    print(f"Action: {action}")
    
    # Test NodePosition
    pos = NodePosition(x=1.0, y=2.0, theta=0.5, mapId="map1")
    print(f"NodePosition: {pos}")
    
    # Test Node
    node = Node(
        nodeId="node1",
        sequenceId=1,
        released=True,
        nodePosition=pos,
        actions=[action]
    )
    print(f"Node: {node}")
    
    print("✓ Basic types test passed!")

def test_order_message():
    """Test OrderMessage creation and JSON conversion"""
    print("\nTesting OrderMessage...")
    
    # Create a simple order
    node1 = Node(nodeId="start", sequenceId=0, released=True)
    node2 = Node(nodeId="end", sequenceId=2, released=True)
    
    edge1 = Edge(
        edgeId="edge1",
        sequenceId=1,
        released=True,
        startNodeId="start",
        endNodeId="end",
        maxSpeed=2.0
    )
    
    order = OrderMessage(
        headerId=1,
        timestamp=create_timestamp(),
        version="2.1.0",
        manufacturer="TestManufacturer",
        serialNumber="AGV001",
        orderId="order_001",
        orderUpdateId=0,
        nodes=[node1, node2],
        edges=[edge1]
    )
    
    print(f"Order created: {order.orderId}")
    
    # Test JSON conversion
    json_str = order.to_json()
    print(f"JSON length: {len(json_str)} characters")
    print("First 200 chars of JSON:")
    print(json_str[:200] + "...")
    
    print("✓ OrderMessage test passed!")

def test_state_message():
    """Test StateMessage creation"""
    print("\nTesting StateMessage...")
    
    battery = BatteryState(batteryCharge=85.0, charging=False)
    safety = SafetyState(eStop="NONE", fieldViolation=False)
    position = AGVPosition(x=1.5, y=2.5, theta=0.0, mapId="map1", positionInitialized=True)
    
    state = StateMessage(
        headerId=1,
        timestamp=create_timestamp(),
        version="2.1.0",
        manufacturer="TestManufacturer",
        serialNumber="AGV001",
        orderId="order_001",
        orderUpdateId=0,
        lastNodeId="start",
        lastNodeSequenceId=0,
        driving=False,
        actionStates=[],
        batteryState=battery,
        operatingMode="AUTOMATIC",
        errors=[],
        safetyState=safety,
        agvPosition=position
    )
    
    print(f"State created for AGV: {state.serialNumber}")
    print(f"Battery: {state.batteryState.batteryCharge}%")
    print(f"Position: ({state.agvPosition.x}, {state.agvPosition.y})")
    
    # Test JSON conversion
    json_str = state.to_json()
    print(f"State JSON length: {len(json_str)} characters")
    
    print("✓ StateMessage test passed!")

if __name__ == "__main__":
    try:
        test_basic_types()
        test_order_message()
        test_state_message()
        print("\n🎉 All VDA5050 types tests passed!")
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()