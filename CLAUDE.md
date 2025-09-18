# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 VDA5050 Bridge that connects ROS2 navigation systems with the VDA5050 protocol for AGV (Automated Guided Vehicle) communication. The system enables bidirectional communication between ROS2-based robots and VDA5050-compliant master control systems.

## Development Environment

### Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- MQTT Broker (Mosquitto)

### Build System
This project uses ROS2 colcon build system:

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build the package
colcon build --packages-select ros2_vda5050_bridge --symlink-install

# Source the workspace
source install/setup.bash
```

### Dependencies
- ROS2 dependencies: rclpy, std_msgs, geometry_msgs, nav_msgs, nav2_msgs, tf2_ros, tf2_geometry_msgs
- Python dependencies: paho-mqtt (install via `pip3 install paho-mqtt`)

## Architecture

### Core Components

1. **VDA5050Bridge** (`ros2_vda5050_bridge/vda5050_bridge.py`): Main ROS2 node that handles:
   - MQTT communication for VDA5050 protocol
   - ROS2 topic subscriptions (/plan, /map, /amcl_pose, /cmd_vel)
   - Data conversion between ROS2 and VDA5050 formats
   - Path conversion from ROS2 Nav2 paths to VDA5050 orders

2. **VDA5050MQTTClient** (`ros2_vda5050_bridge/mqtt_client.py`): MQTT client implementation for:
   - Connection management with MQTT broker
   - Topic subscription (order, instantActions)
   - Message publishing (state, connection, factsheet)
   - VDA5050 protocol message handling

3. **VDA5050 Types** (`ros2_vda5050_bridge/vda5050_types.py`): Complete VDA5050 v2.1.0 protocol implementation:
   - OrderMessage, StateMessage data structures
   - Node, Edge, Action definitions
   - JSON serialization/deserialization

### Data Flow

```
ROS2 Navigation Stack ←→ VDA5050 Bridge ←→ VDA5050 Master Control
        ↓                      ↓                      ↓
     /plan, /map          MQTT Topics            Order/State
     /amcl_pose, /cmd_vel   uagv/v2/...          InstantActions
```

### Key Features

1. **Path Conversion**: Automatically converts ROS2 Nav2 paths to VDA5050 orders
2. **Map Sharing**: Shares occupancy grid maps between ROS2 and VDA5050 systems
3. **Real-time State**: Publishes AGV state at 2Hz (configurable)
4. **Action Handling**: Supports VDA5050 actions (wait, pick, drop, etc.)
5. **Target Detection**: Automatic goal reaching detection with configurable tolerance

## Running the System

### Basic Usage

```bash
# Start MQTT broker (if not running)
sudo systemctl start mosquitto

# Launch bridge with default parameters
ros2 launch ros2_vda5050_bridge bridge_launch.py

# Launch with custom parameters
ros2 launch ros2_vda5050_bridge bridge_launch.py \
    mqtt_host:=localhost \
    mqtt_port:=1883 \
    manufacturer:=MyCompany \
    serial_number:=AGV_001
```

### Testing

```bash
# Run individual tests
python3 test_mqtt_basic.py              # MQTT communication test
python3 test_vda5050_types.py           # VDA5050 data types test
python3 test_path_conversion_simple.py  # Path conversion test
python3 test_ros2_path_integration.py   # Full integration test
python3 test_vda5050_system.py          # System test

# Run demo
python3 demo_vda5050_usage.py            # Interactive demonstration
```

### Simulation Components

```bash
# Map publisher (simulates map sharing)
ros2 run ros2_vda5050_bridge map_publisher

# Path publisher (simulates Nav2 planning)
ros2 run ros2_vda5050_bridge path_publisher
```

## Configuration

### Parameters
- `mqtt_broker_host`: MQTT broker hostname (default: "localhost")
- `mqtt_broker_port`: MQTT broker port (default: 1883)
- `manufacturer`: AGV manufacturer name (default: "ROS2Manufacturer")
- `serial_number`: AGV serial number (default: "ROS2_AGV_001")
- `map_frame`: ROS2 map frame (default: "map")
- `base_frame`: ROS2 base frame (default: "base_link")
- `state_publish_rate`: State publishing frequency in Hz (default: 2.0)

### MQTT Topic Structure
```
uagv/v2/{manufacturer}/{serial_number}/
├── order              # Subscribe to incoming orders
├── instantActions     # Subscribe to instant actions
├── state             # Publish AGV state
├── connection        # Publish connection status
└── factsheet         # Publish AGV information
```

## ROS2 Topics

### Subscribed Topics
- `/plan` (nav_msgs/Path): Nav2 planned path for conversion to VDA5050
- `/map` (nav_msgs/OccupancyGrid): Map data for sharing
- `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped): Position estimates
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/navigation_result` (std_msgs/String): Navigation results

### Published Topics
- `/goal_pose` (geometry_msgs/PoseStamped): Navigation goals
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/cancel_navigation` (std_msgs/Bool): Navigation cancellation

## Development Guidelines

### Code Style
- Follow ROS2 Python coding conventions
- Use type hints consistently
- Implement proper error handling and logging
- Maintain separation between protocol logic and ROS2 integration

### Testing
- All test files are standalone Python scripts
- Tests cover MQTT communication, data conversion, and system integration
- Use the demo script for interactive testing and validation

### Key Integration Points
- Path conversion logic in `convert_ros2_path_to_vda5050_order()`
- State publishing in `publish_vda5050_state()`
- Order processing in `handle_vda5050_order()`
- Action execution in `execute_node_actions()`

## Troubleshooting

### Common Issues
1. **MQTT Connection**: Ensure Mosquitto is running and accessible
2. **TF2 Issues**: Check that map→base_link transform is available
3. **Navigation**: Verify Nav2 is providing paths on `/plan` topic
4. **State Publishing**: Check ROS2 parameter configuration

### Debug Commands
```bash
# Check MQTT connection
mosquitto_pub -h localhost -t test/topic -m "test"

# Check ROS2 topics
ros2 topic list
ros2 topic echo /plan

# Check TF2 transforms
ros2 run tf2_ros tf2_echo map base_link
```