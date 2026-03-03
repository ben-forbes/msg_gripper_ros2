# MSG Gripper ROS2 Driver

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![License: MIT](https://img.shields.io/badge/License-MIT-green)
![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue)

ROS2 driver for the MSG gripper using the [Spectral BLDC](https://pypi.org/project/Spectral-BLDC/) library and CAN bus.

**Developed by [UCLA Biomechatronics Lab](https://biomechatronics.ucla.edu/) · and [Ben Forbes](https://ben-forbes.github.io/)**

## Overview

The node accepts `sensor_msgs/JointState` commands with position specified as **encoder counts (0–255)** and publishes feedback with position in encoder counts and effort in milliamps.

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| Command | `/gripper/command_joint_states` | `sensor_msgs/JointState` |
| Feedback | `/gripper/feedback_joint_states` | `sensor_msgs/JointState` |

A `control_msgs/GripperCommand` action server is also available at `/gripper_command`.

---

## Prerequisites

- **Ubuntu 22.04 LTS** with **ROS2 Humble** installed ([installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))
- Python 3.10+
- A USB-to-CAN adapter that supports the `slcan` protocol (e.g., CANable, PCAN-USB)

> **macOS / Windows:** Not officially tested, but the Python driver code is platform-agnostic. ROS2 and `python-can` must be available on your platform. The only OS-specific setting is the `channel` device path — see [Hardware & CAN Setup](#hardware--can-setup) below.

---

## Hardware & CAN Setup

The gripper communicates over CAN bus via a USB-to-CAN adapter using the `slcan` protocol.

### Finding your device path

After plugging in the USB-to-CAN adapter, find the device path:

**Linux:**
```bash
ls /dev/ttyACM* /dev/ttyUSB*
# Typical result: /dev/ttyACM0  or  /dev/ttyUSB0
```

**macOS:**
```bash
ls /dev/tty.usb*
# Typical result: /dev/tty.usbmodem1101  or  /dev/tty.usbserial-1234
```

**Windows:**
Open Device Manager → Ports (COM & LPT) → note the `COMx` number (e.g., `COM3`).

### Platform `channel` values

The `channel` parameter in `config/default.yaml` (or passed at launch) is the only OS-specific setting:

| Platform | Typical `channel` value |
|----------|------------------------|
| Linux | `/dev/ttyACM0` or `/dev/ttyUSB0` |
| macOS | `/dev/tty.usbmodem*` or `/dev/tty.usbserial*` |
| Windows | `COM3` (adjust number as needed) |

### Linux: Grant serial port access

On Linux you may need to add your user to the `dialout` group to access `/dev/ttyACM*` without `sudo`:

```bash
sudo usermod -aG dialout $USER
# Log out and back in for the change to take effect
```

---

## Installation

### 1. Install pip dependencies

```bash
pip install Spectral_BLDC python-can
```

### 2. Clone and build

```bash
cd ~/ros2_ws/src
git clone https://github.com/<your-org>/msg_gripper_ros2 msg_gripper_ros2
cd ~/ros2_ws
colcon build --packages-select msg_gripper_ros2
source install/setup.bash
```

---

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

Override individual parameters by providing a custom config file:

```bash
# Copy and edit the default config
cp src/msg_gripper_ros2/config/default.yaml ~/my_gripper.yaml
# Edit channel, bustype, etc. in ~/my_gripper.yaml, then:
ros2 launch msg_gripper_ros2 msg_gripper_launch.py \
  config_file:=~/my_gripper.yaml
```

Or run the node directly with `--ros-args` for one-off overrides:

```bash
ros2 run msg_gripper_ros2 msg_gripper_node \
  --ros-args -p channel:=/dev/ttyUSB0
```

---

## Command Format

Publish a `sensor_msgs/JointState` to the command topic:

| Field | Description |
|-------|-------------|
| `position[0]` | Target position — encoder counts (0–255) |
| `velocity[0]` | *(optional)* Speed 0–255, default: 20 |
| `effort[0]` | *(optional)* Force in mA, default: 500 |

Commands are filtered by a deadband: if the new position is within `position_deadband` counts (default: 5) of the last sent position, the command is silently dropped. This prevents redundant CAN traffic.

## Feedback Format

The feedback `JointState` contains:

| Field | Description |
|-------|-------------|
| `position[0]` | Current position in encoder counts |
| `effort[0]` | Motor current in mA |

---

## Testing

All tests are run with pytest from the package directory:

```bash
cd ~/gripper_test/src/msg_gripper_ros2
source /opt/ros/humble/setup.bash
```

### Unit tests only (no hardware required)

```bash
pytest test/ -v -m "not hardware"
```

The unit tests mock the `Spectral_BLDC` library so no CAN adapter or gripper needs to be connected.

### All tests (unit + hardware-in-the-loop)

With the gripper connected (the test suite launches the node automatically):

```bash
pytest test/ -v
```

Custom device path:

```bash
pytest test/ -v --channel /dev/ttyUSB0
```

If the node is already running externally:

```bash
pytest test/ -v --no-launch
```

### Available pytest options

| Option | Default | Description |
|--------|---------|-------------|
| `--channel` | `/dev/ttyACM0` | CAN serial device path |
| `--bustype` | `slcan` | CAN bus type |
| `--bitrate` | `1000000` | CAN bitrate in bps |
| `--no-launch` | `False` | Skip launching the node (assumes it is already running) |
| `--startup-timeout` | `10` | Seconds to wait for node startup |

---

## Quick Test

```bash
# Command gripper to position 128
ros2 topic pub --once /gripper/command_joint_states sensor_msgs/msg/JointState \
  "{position: [128]}"

# Monitor feedback
ros2 topic echo /gripper/feedback_joint_states
```

---

## Action Server

The node exposes a `control_msgs/GripperCommand` action at `/gripper_command`.

`position` maps to encoder counts (0–255); `max_effort` maps to force in mA (0 = use default).

**Send a goal from the command line:**

```bash
ros2 action send_goal /gripper_command control_msgs/action/GripperCommand \
  "{command: {position: 200.0, max_effort: 500.0}}"
```

**Send a goal and stream feedback:**

```bash
ros2 action send_goal --feedback /gripper_command control_msgs/action/GripperCommand \
  "{command: {position: 100.0, max_effort: 0.0}}"
```

> `max_effort: 0.0` uses the node's `default_force` parameter.

---

## Parameters

All parameters are configurable via `config/default.yaml` or at launch time.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bustype` | `slcan` | CAN bus type |
| `channel` | `/dev/ttyACM0` | CAN interface device (OS-specific — see above) |
| `bitrate` | `1000000` | CAN bitrate |
| `node_id` | `0` | Spectral BLDC node ID |
| `default_speed` | `20` | Default speed (0–255) |
| `default_force` | `500` | Default force in mA |
| `position_deadband` | `5` | Minimum change in counts to send a new command |
| `auto_calibrate` | `true` | Run calibration on startup |
| `command_topic` | `/gripper/command_joint_states` | Command subscription topic |
| `feedback_topic` | `/gripper/feedback_joint_states` | Feedback publisher topic |

---

## Troubleshooting

### `Failed to initialize CAN: ...`

- Check that the USB-to-CAN adapter is plugged in and recognized by the OS.
- Verify the `channel` parameter matches the actual device path (e.g., `/dev/ttyACM0`).
- Try a lower `bitrate` if the adapter does not support 1 Mbit/s.

### Permission denied on `/dev/ttyACM*` or `/dev/ttyUSB*` (Linux)

```bash
# Quick fix (requires sudo each session):
sudo chmod a+rw /dev/ttyACM0

# Permanent fix (requires logout):
sudo usermod -aG dialout $USER
```

### Calibration not completing

- The calibration command is sent once ~1 second after startup (`auto_calibrate: true`).
- If the gripper does not move, check CAN communication first (see above).
- You can disable auto-calibration with `auto_calibrate: false` and trigger it manually if needed.
- Ensure the gripper has enough range of motion and is not mechanically obstructed.

### No feedback messages on `/gripper/feedback_joint_states`

- Confirm the node started without errors (`ros2 node list` should show `msg_gripper_node`).
- The motor only responds after being addressed — check that at least one command has been sent.
- Use `ros2 topic hz /gripper/feedback_joint_states` to confirm the publisher is running at ~20 Hz.

---

## Package Structure

```
msg_gripper_ros2/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── pytest.ini
├── LICENSE
├── README.md
├── CONTRIBUTING.md
├── msg_gripper_ros2/
│   ├── __init__.py
│   └── msg_gripper_node.py
├── launch/
│   └── msg_gripper_launch.py
├── config/
│   └── default.yaml
├── resource/
│   └── msg_gripper_ros2
└── test/
    ├── conftest.py       # shared fixtures and CLI options
    ├── test_unit.py      # offline unit tests (no hardware)
    └── test_hardware.py  # hardware-in-the-loop tests
```

---

## Contributing / Issues

Bug reports and pull requests are welcome at:
**https://github.com/\<your-org\>/msg_gripper_ros2/issues**

Please include:
- ROS2 distribution and OS version
- USB-to-CAN adapter model
- Full error output from `ros2 launch msg_gripper_ros2 msg_gripper_launch.py`
