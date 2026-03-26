# 🚀 Distributed MyCobot System - Architecture

## Overview

This system distributes the workload between:
- **Tour (PC)**: Heavy computations (vision, AI, path planning)
- **Raspberry Pi**: Simple task execution (direct robot control)

```
┌─────────────────────────────────────────────────────────────────────┐
│                          TOUR (PC)                                  │
│                    Ubuntu 24.04 / ROS2 Jazzy                        │
│                                                                     │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐  │
│  │ camera_publisher│───▶│ marker_detector │───▶│  bridge_tour    │  │
│  │  (capture)      │    │ (ArUco + coord  │    │  (TCP client)   │  │
│  └─────────────────┘    │  transform)     │    └────────┬────────┘  │
│                         └─────────────────┘             │           │
│                                                         │           │
│  ┌─────────────────┐                                    │           │
│  │ robot_commander │────────────────────────────────────┤           │
│  │ (high-level API)│                                    │           │
│  └─────────────────┘                                    │           │
└─────────────────────────────────────────────────────────┼───────────┘
                                                          │
                                          TCP/IP (10.10.0.x:5005)
                                                          │
┌─────────────────────────────────────────────────────────┼───────────┐
│                      RASPBERRY PI                       │           │
│                  Ubuntu 20.04 / ROS2 Galactic           │           │
│                                                         ▼           │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐  │
│  │   bridge_pi     │◀───│command_executor │◀───│    MyCobot      │  │
│  │  (TCP server)   │    │ (JSON parser +  │    │   (Hardware)    │  │
│  └─────────────────┘    │  robot control) │    └─────────────────┘  │
│                         └─────────────────┘                         │
└─────────────────────────────────────────────────────────────────────┘
```

## Data Flow

### Marker Following Mode
1. **Tour**: Camera captures image → `camera/image_raw`
2. **Tour**: Marker detector processes image (ArUco detection, pose estimation)
3. **Tour**: Coordinate transformation (camera frame → robot frame)
4. **Tour**: Command sent to `/to_robot` as JSON
5. **Bridge**: TCP transmission to Pi
6. **Pi**: Command executor parses JSON, calls `mc.send_coords()`
7. **Pi**: Feedback sent back via `/from_robot`

### Interactive Mode
1. **User**: Enters command via `robot_commander`
2. **Tour**: Command formatted as JSON, published to `/to_robot`
3. **Bridge**: TCP transmission
4. **Pi**: Execution + feedback

## Message Protocol

### Commands (Tour → Pi)

JSON format:
```json
{
  "action": "send_coords",
  "coords": [200.0, 0.0, 250.0, 180.0, 0.0, 0.0],
  "speed": 40,
  "mode": 1
}
```

Supported actions:
- `send_angles` - Send joint angles (degrees)
- `send_coords` - Send Cartesian coordinates
- `send_radians` - Send joint angles (radians)
- `go_home` - Move to home position
- `go_zero` - Move to zero position
- `gripper_open` / `gripper_close`
- `power_on` / `power_off`
- `get_angles` / `get_coords` - Request current state
- `emergency_stop` - Release all servos

### Feedback (Pi → Tour)

```
OK: send_coords [200.0, 0.0, 250.0]
ANGLES: [0.0, 8.0, -127.0, 40.0, 0.0, 0.0]
ERROR: Failed to execute command
🚨 EMERGENCY STOP EXECUTED
```

## Installation

### On Tour (PC)

```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate  # Important!
colcon build --symlink-install
source install/setup.bash
```

### On Raspberry Pi

Copy `command_executor_pi.py` to the Pi's workspace and update the bridge_pi
to forward messages appropriately.

## Usage

### Quick Start

```bash
# Terminal 1: Start bridge + marker following
ros2 launch mycobot_gateway marker_follow.launch.py

# Or just the bridge
ros2 launch mycobot_gateway bridge_only.launch.py
```

### Manual Commands

```bash
# Start bridge
ros2 run mycobot_gateway bridge_tour

# In another terminal - send commands
ros2 run mycobot_gateway robot_commander

# Or directly publish
ros2 topic pub /to_robot std_msgs/msg/String "{data: '{\"action\":\"go_home\"}'}" -1
```

## Configuration

### Network (in bridge_tour.py)
```python
self.pi_ip = '10.10.0.218'  # Pi IP address
self.port = 5005            # TCP port
```

### Camera (via launch parameters)
```bash
ros2 launch mycobot_gateway marker_follow.launch.py camera_index:=0
```

## Nodes Summary

| Node | Location | Purpose |
|------|----------|---------|
| `bridge_tour` | Tour | TCP client, ROS2 ↔ TCP bridge |
| `camera_publisher` | Tour | USB camera capture |
| `marker_detector` | Tour | ArUco detection + coord transform |
| `robot_commander` | Tour | High-level command interface |
| `bridge_pi` | Pi | TCP server, receives commands |
| `command_executor` | Pi | Executes commands on MyCobot |

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/to_robot` | String | Tour → Pi | Commands to robot |
| `/from_robot` | String | Pi → Tour | Robot feedback |
| `/camera/image_raw` | Image | Internal (Tour) | Camera frames |

## Safety Features

1. **Coordinate limits** in marker_detector:
   - X: 130-350 mm
   - Y: -200 to 200 mm
   - Z: 100-400 mm

2. **Command cooldown**: 200ms minimum between commands

3. **Emergency stop**: `emergency_stop` action releases all servos immediately

4. **Reconnection**: Bridge automatically reconnects if TCP connection drops
