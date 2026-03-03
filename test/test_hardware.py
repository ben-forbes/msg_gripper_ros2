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
Hardware-in-the-loop tests for the MSG Gripper ROS2 driver.

Requires a physically connected gripper (node_id=0 on /dev/ttyACM0 by default).

Run:
    pytest test/test_hardware.py -v
    pytest test/test_hardware.py -v --channel /dev/ttyUSB0
    pytest test/test_hardware.py -v --no-launch   # node already running
"""

import time

import pytest
from sensor_msgs.msg import JointState

from conftest import wait_for_message

pytestmark = pytest.mark.hardware

FEEDBACK_TOPIC = "/gripper/feedback_joint_states"
COMMAND_TOPIC = "/gripper/command_joint_states"


# =========================================================================== #
#  TestNodeLifecycle
# =========================================================================== #

class TestNodeLifecycle:
    def test_feedback_topic_exists(self, test_node):
        """A JointState message should arrive within 5 seconds of startup."""
        msg = wait_for_message(test_node, JointState, FEEDBACK_TOPIC, timeout=5.0)
        assert msg is not None, "No feedback received within 5 s"

    def test_feedback_has_required_fields(self, test_node):
        """Feedback must contain position[0], effort[0], and name=['gripper_joint']."""
        msg = wait_for_message(test_node, JointState, FEEDBACK_TOPIC, timeout=5.0)
        assert msg is not None, "No feedback received"
        assert len(msg.position) >= 1, "position array is empty"
        assert len(msg.effort) >= 1, "effort array is empty"
        assert msg.name == ["gripper_joint"], f"Unexpected joint name: {msg.name}"

    def test_feedback_rate(self, test_node):
        """Feedback should arrive at >= 15 Hz averaged over 2 seconds."""
        from rclpy.qos import qos_profile_sensor_data
        import threading

        count = 0
        lock = threading.Lock()

        def _cb(_msg):
            nonlocal count
            with lock:
                count += 1

        sub = test_node.create_subscription(
            JointState, FEEDBACK_TOPIC, _cb, qos_profile_sensor_data
        )
        try:
            time.sleep(2.0)
        finally:
            test_node.destroy_subscription(sub)

        with lock:
            final_count = count

        assert final_count >= 30, (
            f"Expected >= 30 messages in 2 s (15 Hz), got {final_count}"
        )


# =========================================================================== #
#  TestCommandTopic
# =========================================================================== #

class TestCommandTopic:
    def _publish_command(self, test_node, position: float, velocity: float = None,
                         effort: float = None):
        """Publish a single JointState command."""
        from rclpy.qos import qos_profile_sensor_data

        pub = test_node.create_publisher(JointState, COMMAND_TOPIC, qos_profile_sensor_data)
        msg = JointState()
        msg.header.stamp = test_node.get_clock().now().to_msg()
        msg.name = ["gripper_joint"]
        msg.position = [float(position)]
        if velocity is not None:
            msg.velocity = [float(velocity)]
        if effort is not None:
            msg.effort = [float(effort)]
        pub.publish(msg)
        # Give the publisher time to be received
        time.sleep(0.1)
        test_node.destroy_publisher(pub)

    def _wait_for_position(self, test_node, target: float, tolerance: float,
                           timeout: float) -> float:
        """Poll feedback until position is within tolerance of target, or timeout."""
        deadline = time.time() + timeout
        last_pos = None
        while time.time() < deadline:
            msg = wait_for_message(test_node, JointState, FEEDBACK_TOPIC, timeout=1.0)
            if msg is not None and len(msg.position) >= 1:
                last_pos = msg.position[0]
                if abs(last_pos - target) <= tolerance:
                    return last_pos
        return last_pos

    def test_open_gripper(self, test_node):
        """Command position=200; gripper should reach within 25 counts in 6 s."""
        self._publish_command(test_node, position=200)
        pos = self._wait_for_position(test_node, target=200, tolerance=25, timeout=6.0)
        assert pos is not None, "No feedback received"
        assert abs(pos - 200) <= 25, f"Gripper did not open: position={pos}"

    def test_close_gripper(self, test_node):
        """Command position=50; gripper should reach within 25 counts in 6 s."""
        self._publish_command(test_node, position=50)
        pos = self._wait_for_position(test_node, target=50, tolerance=25, timeout=6.0)
        assert pos is not None, "No feedback received"
        assert abs(pos - 50) <= 25, f"Gripper did not close: position={pos}"

    def test_position_upper_clamp(self, test_node):
        """Command 9999 — feedback position must stay <= 255."""
        self._publish_command(test_node, position=9999)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            msg = wait_for_message(test_node, JointState, FEEDBACK_TOPIC, timeout=1.0)
            if msg is not None and len(msg.position) >= 1:
                assert msg.position[0] <= 255, (
                    f"Position exceeded 255: {msg.position[0]}"
                )

    def test_position_lower_clamp(self, test_node):
        """Command -999 — feedback position must stay >= 0."""
        self._publish_command(test_node, position=-999)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            msg = wait_for_message(test_node, JointState, FEEDBACK_TOPIC, timeout=1.0)
            if msg is not None and len(msg.position) >= 1:
                assert msg.position[0] >= 0, (
                    f"Position went below 0: {msg.position[0]}"
                )

    def test_command_empty_position_ignored(self, test_node):
        """Publishing an empty JointState should be silently ignored; node stays alive."""
        from rclpy.qos import qos_profile_sensor_data

        pub = test_node.create_publisher(JointState, COMMAND_TOPIC, qos_profile_sensor_data)
        pub.publish(JointState())
        time.sleep(0.5)
        test_node.destroy_publisher(pub)

        # Node should still be publishing feedback
        msg = wait_for_message(test_node, JointState, FEEDBACK_TOPIC, timeout=3.0)
        assert msg is not None, "Node stopped publishing after empty command"


# =========================================================================== #
#  TestActionServer
# =========================================================================== #

class TestActionServer:
    @pytest.fixture(autouse=True)
    def _action_client(self, test_node):
        from rclpy.action import ActionClient
        from control_msgs.action import GripperCommand

        self._client = ActionClient(test_node, GripperCommand, "gripper_command")
        yield
        self._client.destroy()

    def test_action_server_available(self):
        """Action server should be reachable within 5 seconds."""
        assert self._client.wait_for_server(timeout_sec=5.0), (
            "GripperCommand action server not available"
        )

    def _send_goal(self, position: float, max_effort: float = 0.0, timeout: float = 10.0):
        from control_msgs.action import GripperCommand

        assert self._client.wait_for_server(timeout_sec=5.0)

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        future = self._client.send_goal_async(goal)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        assert future.done(), "Goal send timed out"

        goal_handle = future.result()
        assert goal_handle.accepted, "Goal was rejected"

        result_future = goal_handle.get_result_async()
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.05)
        assert result_future.done(), "Result timed out"
        return result_future.result().result

    def test_action_open(self):
        """Action goal position=200 should succeed with reached_goal=True."""
        result = self._send_goal(position=200)
        assert result.reached_goal, "Action did not reach goal (open)"

    def test_action_close(self):
        """Action goal position=50 should succeed with reached_goal=True."""
        result = self._send_goal(position=50)
        assert result.reached_goal, "Action did not reach goal (close)"

    def test_action_default_effort(self):
        """max_effort=0 should use the node default; action should still succeed."""
        result = self._send_goal(position=128, max_effort=0.0)
        assert result.reached_goal, "Action with default effort did not reach goal"
