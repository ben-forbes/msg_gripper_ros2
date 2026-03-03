#!/usr/bin/env python3
# Copyright (c) 2026 UCLA Biomechatronics Lab
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
ROS2 driver for the MSG gripper using the Spectral BLDC library.

Accepts JointState commands with position as encoder counts (0-255),
and publishes feedback with position (counts) and effort (motor current in mA).
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

try:
    import Spectral_BLDC as Spectral
except ImportError:
    print("ERROR: Spectral_BLDC library not found!")
    print("Install with: pip install Spectral_BLDC")
    exit(1)


class MSGGripperNode(Node):
    """ROS2 node for controlling an MSG gripper via CAN bus."""

    def __init__(self):
        super().__init__('msg_gripper_node')

        # -- Parameters: CAN bus --
        self.declare_parameter('bustype', 'slcan')
        # Linux default; see README for macOS/Windows
        self.declare_parameter('channel', '/dev/ttyACM0')
        self.declare_parameter('bitrate', 1000000)
        self.declare_parameter('node_id', 0)

        # -- Parameters: control defaults --
        self.declare_parameter('default_speed', 20)
        self.declare_parameter('default_force', 500)

        # -- Parameters: filtering --
        self.declare_parameter('position_deadband', 5)

        # -- Parameters: behavior --
        self.declare_parameter('auto_calibrate', True)

        # -- Parameters: topics --
        self.declare_parameter('command_topic', '/gripper/command_joint_states')
        self.declare_parameter('feedback_topic', '/gripper/feedback_joint_states')

        # -- Read parameters --
        bustype = self.get_parameter('bustype').value
        channel = self.get_parameter('channel').value
        bitrate = self.get_parameter('bitrate').value
        self.node_id = self.get_parameter('node_id').value
        self.default_speed = self.get_parameter('default_speed').value
        self.default_force = self.get_parameter('default_force').value
        self.position_deadband = self.get_parameter('position_deadband').value
        self.auto_calibrate = self.get_parameter('auto_calibrate').value
        command_topic = self.get_parameter('command_topic').value
        feedback_topic = self.get_parameter('feedback_topic').value

        # -- Initialize CAN --
        self.get_logger().info(
            f'Initializing CAN: {bustype} on {channel} @ {bitrate} bps'
        )
        try:
            self.communication = Spectral.CanCommunication(
                bustype=bustype,
                channel=channel,
                bitrate=bitrate,
            )
            self.motor = Spectral.SpectralCAN(
                node_id=self.node_id,
                communication=self.communication,
            )
            self.get_logger().info('CAN communication initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN: {e}')
            raise

        # -- State tracking --
        self.last_commanded_position = None
        self.current_position = 0
        self.current_effort = 0
        self.is_calibrated = False

        # -- Subscriber: JointState commands (position as counts 0-255) --
        self.command_sub = self.create_subscription(
            JointState,
            command_topic,
            self._command_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f'Listening for commands on: {command_topic}')

        # -- Publisher: JointState feedback (position in counts, effort in mA) --
        self.feedback_pub = self.create_publisher(
            JointState,
            feedback_topic,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f'Publishing feedback on: {feedback_topic}')

        # -- Action server: GripperCommand (position as counts 0-255) --
        self.action_server = ActionServer(
            self,
            GripperCommand,
            'gripper_command',
            self._execute_gripper_command,
        )

        # -- Timers --
        self.create_timer(0.05, self._update_state)  # 20 Hz feedback

        if self.auto_calibrate:
            self.get_logger().info('Auto-calibration scheduled...')
            self.create_timer(1.0, self._calibrate_once)

        self.get_logger().info('MSG Gripper node ready')

    # ------------------------------------------------------------------ #
    #  Calibration
    # ------------------------------------------------------------------ #

    def _calibrate_once(self):
        """Send calibration command once at startup."""
        if self.is_calibrated:
            return
        try:
            self.get_logger().info('Sending calibration command...')
            self.motor.Send_gripper_calibrate()
            self.is_calibrated = True
            self.get_logger().info('Calibration complete')
        except Exception as e:
            self.get_logger().error(f'Calibration failed: {e}')

    # ------------------------------------------------------------------ #
    #  Command callback
    # ------------------------------------------------------------------ #

    def _command_callback(self, msg: JointState):
        """
        Handle incoming gripper command.

        Expected JointState fields:
            position[0] — target position as encoder counts (0-255)
            velocity[0] — (optional) speed 0-255, default: default_speed
            effort[0]   — (optional) force in mA, default: default_force
        """
        if len(msg.position) < 1:
            self.get_logger().warn('Command message has no position data')
            return

        target = int(max(0, min(255, msg.position[0])))
        speed = int(msg.velocity[0]) if len(msg.velocity) > 0 else self.default_speed
        force = int(msg.effort[0]) if len(msg.effort) > 0 else self.default_force

        speed = max(0, min(255, speed))
        force = max(0, min(2000, force))

        # Deadband filter
        if self.last_commanded_position is not None:
            if abs(target - self.last_commanded_position) < self.position_deadband:
                return

        try:
            self.get_logger().info(
                f'Command: pos={target} speed={speed} force={force}'
            )
            self.motor.Send_gripper_data_pack(
                target, speed, force,
                1,  # enable
                1,  # control mode
                0, 0,  # reserved
            )
            self.last_commanded_position = target
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')

    # ------------------------------------------------------------------ #
    #  Feedback
    # ------------------------------------------------------------------ #

    def _update_state(self):
        """
        Poll the gripper and publish feedback.

        The Spectral BLDC protocol requires sending a command first; the motor
        only responds after being addressed.  We re-send the last commanded
        position each cycle, then read back the response frames.
        """
        try:
            pos = self.last_commanded_position if self.last_commanded_position is not None else 0
            self.motor.Send_gripper_data_pack(
                pos, self.default_speed, self.default_force,
                1, 1, 0, 0,
            )

            for _ in range(3):
                message, unpacked_id = self.communication.receive_can_messages(
                    timeout=0.001
                )
                if message is None or unpacked_id is None:
                    continue
                self.motor.UnpackData(message, unpacked_id)

            self.current_position = getattr(self.motor, 'gripper_position', 0) or 0
            self.current_effort = getattr(self.motor, 'gripper_current', 0) or 0

            fb = JointState()
            fb.header.stamp = self.get_clock().now().to_msg()
            fb.name = ['gripper_joint']
            fb.position = [float(self.current_position)]
            fb.velocity = []
            fb.effort = [float(self.current_effort)]
            self.feedback_pub.publish(fb)

        except Exception as e:
            if 'timeout' not in str(e).lower():
                self.get_logger().debug(f'CAN read: {e}')

    # ------------------------------------------------------------------ #
    #  GripperCommand action
    # ------------------------------------------------------------------ #

    def _execute_gripper_command(self, goal_handle):
        """Execute GripperCommand action (position as counts 0-255)."""
        goal = goal_handle.request
        target = int(max(0, min(255, goal.command.position)))
        force = (
            int(goal.command.max_effort)
            if goal.command.max_effort > 0
            else self.default_force
        )

        self.get_logger().info(f'Action: pos={target} force={force}')

        try:
            self.motor.Send_gripper_data_pack(
                target, self.default_speed, force,
                1, 1, 0, 0,
            )
            time.sleep(2.0)

            goal_handle.succeed()
            result = GripperCommand.Result()
            result.position = float(target)
            result.reached_goal = True
            result.stalled = False
            return result

        except Exception as e:
            self.get_logger().error(f'Action failed: {e}')
            goal_handle.abort()
            result = GripperCommand.Result()
            result.reached_goal = False
            return result


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MSGGripperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
