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
Offline unit tests for the MSG Gripper ROS2 driver.

No hardware is required.  ``Spectral_BLDC`` is injected into ``sys.modules``
as a MagicMock *before* the node module is imported, so no CAN hardware or
library installation is needed.

Run:
    pytest test/test_unit.py -v
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

# Ensure the local source tree is imported rather than any installed version.
_REPO_ROOT = os.path.join(os.path.dirname(__file__), "..")
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
# Remove any already-cached stale import of the package so the re-insert takes effect.
for _key in list(sys.modules.keys()):
    if _key.startswith("msg_gripper_ros2"):
        del sys.modules[_key]

import pytest  # noqa: E402

# --------------------------------------------------------------------------- #
#  Inject Spectral_BLDC mock before any node import
# --------------------------------------------------------------------------- #

_spectral_mock = MagicMock(name="Spectral_BLDC")

# Configure return values so background timers don't raise
_spectral_mock.CanCommunication.return_value.receive_can_messages.return_value = (
    None, None
)
_can_instance = _spectral_mock.CanCommunication.return_value
_motor_instance = _spectral_mock.SpectralCAN.return_value
_motor_instance.gripper_position = 0
_motor_instance.gripper_current = 0

sys.modules.setdefault("Spectral_BLDC", _spectral_mock)

# Now it is safe to import rclpy and the node
import rclpy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402


# --------------------------------------------------------------------------- #
#  Fixtures
# --------------------------------------------------------------------------- #

@pytest.fixture(scope="module", autouse=True)
def ros_context_unit():
    """Module-scoped rclpy init/shutdown for the unit test file."""
    already_up = rclpy.ok()
    if not already_up:
        rclpy.init()
    yield
    if not already_up:
        rclpy.shutdown()


@pytest.fixture()
def gripper_node():
    """
    Create a fresh MSGGripperNode per test (not spun — timers never fire).

    The Spectral_BLDC mock is already in sys.modules, so __init__ succeeds
    without any hardware.
    """
    # Import inside fixture so the mock is guaranteed to be in sys.modules
    from msg_gripper_ros2.msg_gripper_node import MSGGripperNode

    # Reset mock call history before each test
    _motor_instance.reset_mock()
    _motor_instance.gripper_position = 0
    _motor_instance.gripper_current = 0

    node = MSGGripperNode()
    yield node
    node.destroy_node()


# --------------------------------------------------------------------------- #
#  Helpers
# --------------------------------------------------------------------------- #

def _make_joint_state(position=None, velocity=None, effort=None) -> JointState:
    msg = JointState()
    if position is not None:
        msg.position = [float(position)]
    if velocity is not None:
        msg.velocity = [float(velocity)]
    if effort is not None:
        msg.effort = [float(effort)]
    return msg


def _get_send_args(motor_mock) -> tuple:
    """Return the positional args of the most recent Send_gripper_data_pack call."""
    assert motor_mock.Send_gripper_data_pack.called, (
        "Send_gripper_data_pack was not called"
    )
    return motor_mock.Send_gripper_data_pack.call_args.args


# --------------------------------------------------------------------------- #
#  TestCommandCallback
# --------------------------------------------------------------------------- #

class TestCommandCallback:
    def test_position_clamped_high(self, gripper_node):
        """Position 999 should be clamped to 255."""
        gripper_node._command_callback(_make_joint_state(position=999))
        args = _get_send_args(_motor_instance)
        assert args[0] == 255, f"Expected 255, got {args[0]}"

    def test_position_clamped_low(self, gripper_node):
        """Position -50 should be clamped to 0."""
        gripper_node._command_callback(_make_joint_state(position=-50))
        args = _get_send_args(_motor_instance)
        assert args[0] == 0, f"Expected 0, got {args[0]}"

    def test_default_speed_applied(self, gripper_node):
        """When velocity is absent, default_speed (20) should be used."""
        gripper_node._command_callback(_make_joint_state(position=100))
        args = _get_send_args(_motor_instance)
        assert args[1] == gripper_node.default_speed, (
            f"Expected speed {gripper_node.default_speed}, got {args[1]}"
        )

    def test_default_force_applied(self, gripper_node):
        """When effort is absent, default_force (500) should be used."""
        gripper_node._command_callback(_make_joint_state(position=100))
        args = _get_send_args(_motor_instance)
        assert args[2] == gripper_node.default_force, (
            f"Expected force {gripper_node.default_force}, got {args[2]}"
        )

    def test_explicit_speed_and_force(self, gripper_node):
        """Explicit velocity=50, effort=800 should be forwarded verbatim."""
        gripper_node._command_callback(
            _make_joint_state(position=100, velocity=50, effort=800)
        )
        args = _get_send_args(_motor_instance)
        assert args[1] == 50, f"Expected speed 50, got {args[1]}"
        assert args[2] == 800, f"Expected force 800, got {args[2]}"

    def test_deadband_suppresses_repeat(self, gripper_node):
        """Sending the same position twice should only trigger one CAN call."""
        gripper_node._command_callback(_make_joint_state(position=100))
        gripper_node._command_callback(_make_joint_state(position=100))
        call_count = _motor_instance.Send_gripper_data_pack.call_count
        assert call_count == 1, (
            f"Expected 1 CAN call due to deadband, got {call_count}"
        )

    def test_deadband_allows_large_change(self, gripper_node):
        """A position change larger than the deadband should trigger a second CAN call."""
        gripper_node._command_callback(_make_joint_state(position=100))
        gripper_node._command_callback(_make_joint_state(position=110))
        call_count = _motor_instance.Send_gripper_data_pack.call_count
        assert call_count == 2, (
            f"Expected 2 CAN calls for large position change, got {call_count}"
        )

    def test_empty_position_ignored(self, gripper_node):
        """A JointState with no position array should be silently ignored."""
        gripper_node._command_callback(JointState())
        assert not _motor_instance.Send_gripper_data_pack.called, (
            "Send_gripper_data_pack should not be called for empty JointState"
        )
