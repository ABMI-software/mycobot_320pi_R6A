# 🚀 Deployment Guide - Distributed MyCobot System

## Architecture Overview

Since the **camera is connected to the Raspberry Pi**, we use this architecture:

```
┌─────────────────────────────────────────────────────────────────┐
│                         TOUR (PC)                               │
│                   Ubuntu 24.04 / ROS2 Jazzy                     │
│                                                                 │
│  ┌─────────────────┐         ┌─────────────────┐                │
│  │ robot_commander │────────▶│   bridge_tour   │                │
│  │ (high-level     │         │  (TCP client)   │                │
│  │  commands)      │         └────────┬────────┘                │
│  └─────────────────┘                  │                         │
│                                       │ TCP/IP                  │
└───────────────────────────────────────┼─────────────────────────┘
                                        │
                                        ▼ 10.10.0.218:5005
┌───────────────────────────────────────┼─────────────────────────┐
│                      RASPBERRY PI     │                         │
│                                       ▼                         │
│  ┌─────────────────┐         ┌─────────────────┐                │
│  │  USB Camera     │────────▶│bridge_pi_vision │                │
│  └─────────────────┘         │                 │                │
│                              │ • TCP server    │                │
│  ┌─────────────────┐         │ • ArUco detect  │                │
│  │  MyCobot 320    │◀────────│ • Robot control │                │
│  │  (Hardware)     │         │ • Follow mode   │                │
│  └─────────────────┘         └─────────────────┘                │
└─────────────────────────────────────────────────────────────────┘
```

## Quick Deployment

### Step 1: Deploy on Raspberry Pi

```bash
# Copy the vision-enabled bridge to Pi
scp /home/genji/ros_jazzy/src/mycobot_R6A/mycobot_gateway/scripts/bridge_pi_vision.py \
    pi@10.10.0.218:~/bridge_pi_vision.py

# SSH to Pi and run
ssh pi@10.10.0.218
python3 bridge_pi_vision.py
```

### Step 2: Run on Tour (PC)

```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate  # IMPORTANT!
source install/setup.bash

# Start the bridge and commander
ros2 run mycobot_gateway bridge_tour &
ros2 run mycobot_gateway robot_commander
```

---

## Available Commands

### Robot Commands
| Command | Description |
|---------|-------------|
| `home` | Move to home position |
| `zero` | Move to zero position |
| `open` | Open gripper |
| `close` | Close gripper |
| `angles` | Get current joint angles |
| `coords` | Get current coordinates |
| `stop` | Emergency stop |

### Vision Commands (Camera on Pi)
| Command | Description |
|---------|-------------|
| `follow` | Start marker following mode |
| `nofollow` | Stop marker following |
| `marker` | Get current marker position |
| `status` | Get full robot status |

### JSON Commands (Advanced)
```json
{"action": "follow_on"}
{"action": "follow_off"}
{"action": "get_marker"}
{"action": "send_coords", "coords": [200, 0, 250, 180, 0, 0], "speed": 40}
```

---

## Files Summary

### On Tour (PC)
```
mycobot_gateway/
├── mycobot_gateway/
│   ├── bridge_tour.py       # TCP client to Pi
│   └── robot_commander.py   # Interactive command interface
└── scripts/
    └── bridge_pi_vision.py  # Deploy this to Pi
```

### On Raspberry Pi
```
~/bridge_pi_vision.py   # TCP server + camera + robot control
```

---

## Troubleshooting

### Pi connection drops frequently
The Pi bridge might be crashing. Check:
```bash
ssh pi@10.10.0.218
python3 bridge_pi_vision.py
# Watch for errors
```

### No marker detected
1. Check camera is working: `ls /dev/video*`
2. Ensure ArUco marker (6x6, ID 250 dictionary) is visible
3. Check lighting conditions

### Robot not responding
1. Check serial port: `ls /dev/ttyAMA0`
2. Check robot power
3. Try `status` command to see connection state

---

**Version:** 0.2.0 | **Date:** 26 mars 2026
