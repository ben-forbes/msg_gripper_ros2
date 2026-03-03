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
Shared pytest fixtures and CLI options for the MSG Gripper test suite.

Session-scoped fixtures start the gripper node once and share a single
rclpy context across all tests in a session.
"""

from __future__ import annotations

import subprocess
import threading
import time
import pytest


# --------------------------------------------------------------------------- #
#  CLI options
# --------------------------------------------------------------------------- #

def pytest_addoption(parser):
    parser.addoption(
        "--channel", default="/dev/ttyACM0",
        help="CAN serial channel (default: /dev/ttyACM0)"
    )
    parser.addoption(
        "--bustype", default="slcan",
        help="CAN bus type (default: slcan)"
    )
    parser.addoption(
        "--bitrate", default="1000000",
        help="CAN bitrate in bps (default: 1000000)"
    )
    parser.addoption(
        "--no-launch", action="store_true", default=False,
        help="Skip launching the node (assumes it is already running)"
    )
    parser.addoption(
        "--startup-timeout", default="10", type=str,
        help="Seconds to wait for node startup (default: 10)"
    )


# --------------------------------------------------------------------------- #
#  Derived config fixture
# --------------------------------------------------------------------------- #

@pytest.fixture(scope="session")
def gripper_config(request):
    """Return a dict of CLI-derived configuration values."""
    return {
        "channel": request.config.getoption("--channel"),
        "bustype": request.config.getoption("--bustype"),
        "bitrate": int(request.config.getoption("--bitrate")),
        "no_launch": request.config.getoption("--no-launch"),
        "startup_timeout": float(request.config.getoption("--startup-timeout")),
    }


# --------------------------------------------------------------------------- #
#  ROS context
# --------------------------------------------------------------------------- #

@pytest.fixture(scope="session")
def ros_context():
    """Initialize rclpy once per test session and shut it down on teardown."""
    import rclpy
    rclpy.init()
    yield
    rclpy.shutdown()


# --------------------------------------------------------------------------- #
#  Gripper process
# --------------------------------------------------------------------------- #

@pytest.fixture(scope="session")
def gripper_process(gripper_config):
    """
    Launch the gripper node as a subprocess unless ``--no-launch`` was passed.

    Fails fast if the process exits within the first second (bad CAN device,
    missing library, etc.).
    """
    if gripper_config["no_launch"]:
        yield None
        return

    cmd = [
        "ros2", "run", "msg_gripper_ros2", "msg_gripper_node",
        "--ros-args",
        "-p", f"channel:={gripper_config['channel']}",
        "-p", f"bustype:={gripper_config['bustype']}",
        "-p", f"bitrate:={gripper_config['bitrate']}",
    ]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

    # Give the process a moment to fail fast
    time.sleep(1.0)
    if proc.poll() is not None:
        output = proc.stdout.read().decode(errors="replace")
        pytest.fail(
            f"Gripper node exited immediately (rc={proc.returncode}):\n{output}"
        )

    yield proc

    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


# --------------------------------------------------------------------------- #
#  Test node (rclpy)
# --------------------------------------------------------------------------- #

@pytest.fixture(scope="session")
def test_node(ros_context, gripper_process, gripper_config):
    """
    Create a plain rclpy Node and spin it in a background daemon thread.

    Depends on ``ros_context`` and ``gripper_process`` so that both are
    guaranteed to be alive for the full session.
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    node = rclpy.create_node("pytest_test_node")
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Wait for the gripper node to be reachable
    timeout = gripper_config["startup_timeout"]
    deadline = time.time() + timeout
    while time.time() < deadline:
        node_names = [n for n, _ in node.get_node_names_and_namespaces()]
        if "msg_gripper_node" in node_names:
            break
        time.sleep(0.2)

    yield node

    executor.shutdown()
    node.destroy_node()


# --------------------------------------------------------------------------- #
#  Helper: wait_for_message
# --------------------------------------------------------------------------- #

def wait_for_message(node, msg_type, topic, *, timeout: float = 5.0):
    """
    Subscribe to *topic* and return the first message received within *timeout*.

    Returns ``None`` if no message arrives in time.
    """
    from rclpy.qos import qos_profile_sensor_data

    received: list = []
    event = threading.Event()

    def _cb(msg):
        if not event.is_set():
            received.append(msg)
            event.set()

    sub = node.create_subscription(msg_type, topic, _cb, qos_profile_sensor_data)
    try:
        event.wait(timeout=timeout)
    finally:
        node.destroy_subscription(sub)

    return received[0] if received else None
