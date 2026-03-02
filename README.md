# MSG Gripper ROS2 Driver

ROS2 driver for the MSG gripper using the [Spectral BLDC](https://pypi.org/project/Spectral-BLDC/) library and CAN bus.

**Developed by [UCLA Biomechatronics Lab](https://biomechatronics.ucla.edu/) · and [Ben Forbes](https://ben-forbes.github.io/)**

## Overview

The node accepts `sensor_msgs/JointState` commands with position specified as **encoder counts (0–255)** and publishes feedback with position in encoder counts and effort in milliamps.

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| Command | `/gripper/command_joint_states` | `sensor_msgs/JointState` |
| Feedback | `/gripper/feedback_joint_states` | `sensor_msgs/JointState` |

A `control_msgs/GripperCommand` action server is also available at `/gripper_command`.

## Installation

### Dependencies

```bash
pip install Spectral_BLDC python-can
```

### Build

```bash
cd ~/ros2_ws/src
git clone <repo_url> msg_gripper_ros2
cd ~/ros2_ws
colcon build --packages-select msg_gripper_ros2
source install/setup.bash
```

## Usage

```bash
ros2 launch msg_gripper_ros2 msg_gripper_launch.py
```

With custom config or namespace:

```bash
ros2 launch msg_gripper_ros2 msg_gripper_launch.py \
  config_file:=/path/to/my_config.yaml \
  node_name:=my_gripper \
  namespace:=robot
```

## Command Format

Publish a `sensor_msgs/JointState` to the command topic:

| Field | Description |
|-------|-------------|
| `position[0]` | Target position — encoder counts (0–255) |
| `velocity[0]` | *(optional)* Speed 0–255, default: 20 |
| `effort[0]` | *(optional)* Force in mA, default: 500 |

## Feedback Format

The feedback `JointState` contains:

| Field | Description |
|-------|-------------|
| `position[0]` | Current position in encoder counts |
| `effort[0]` | Motor current in mA |

## Quick Test

```bash
# Command gripper to position 128
ros2 topic pub --once /gripper/command_joint_states sensor_msgs/msg/JointState \
  "{position: [128]}"

# Monitor feedback
ros2 topic echo /gripper/feedback_joint_states
```

## Parameters

All parameters are configurable via `config/default.yaml` or at launch time.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bustype` | `slcan` | CAN bus type |
| `channel` | `/dev/ttyACM0` | CAN interface device |
| `bitrate` | `1000000` | CAN bitrate |
| `node_id` | `0` | Spectral BLDC node ID |
| `default_speed` | `20` | Default speed (0–255) |
| `default_force` | `500` | Default force in mA |
| `position_deadband` | `5` | Minimum change in counts to send a new command |
| `auto_calibrate` | `true` | Run calibration on startup |
| `command_topic` | `/gripper/command_joint_states` | Command subscription topic |
| `feedback_topic` | `/gripper/feedback_joint_states` | Feedback publisher topic |

## Package Structure

```
msg_gripper_ros2/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── LICENSE
├── README.md
├── msg_gripper_ros2/
│   ├── __init__.py
│   └── msg_gripper_node.py
├── launch/
│   └── msg_gripper_launch.py
└── config/
    └── default.yaml
```
