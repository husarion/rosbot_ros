# Copyright 2026 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Contract guard for the animation_publisher LED node.

The node must publish a 1x18 ``rgb8`` ``Image`` on ``led_strip`` over a
BEST_EFFORT publisher (the shape + topic consumed by the firmware / MAVLink
bridge). Selection is the ``current_animation`` node parameter: a valid
animation publishes, the reserved ``none`` stops publishing, and an unknown
name is rejected by the on-set validation callback. ``led_strip/enable`` gates
publishing independently of that selection.
"""

import os
import shutil
import struct
import tempfile
import time
import zlib

import launch_pytest
import pytest
import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

NUM_LEDS = 18
NODE_NAME = "animation_publisher"

# Stands in for <config_dir>/rosbot_utils/animations, which is where the snap
# lets an operator drop their own animations.
USER_DIR = os.path.join(tempfile.gettempdir(), "rosbot_test_animations")


@launch_pytest.fixture
def generate_test_description():
    # Start from an empty user dir: a leftover PNG would be picked up by the
    # node's startup scan and cached, so the "not on disk yet" case below would
    # no longer be reachable.
    shutil.rmtree(USER_DIR, ignore_errors=True)
    os.makedirs(USER_DIR, exist_ok=True)
    led_node = Node(
        package="rosbot_utils",
        executable="animation_publisher",
        name=NODE_NAME,
        parameters=[{"current_animation": "rainbow", "user_animations_dir": USER_DIR}],
        output="screen",
    )
    return LaunchDescription([led_node, ReadyToTest()])


def _write_animation(directory, name, frequency):
    """Write a minimal <name>.png (4 frames x NUM_LEDS, rgb8) plus its sidecar."""
    rows = b"".join(b"\x00" + bytes([255, 0, 255]) * NUM_LEDS for _ in range(4))

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", NUM_LEDS, 4, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(rows))
        + chunk(b"IEND", b"")
    )
    with open(os.path.join(directory, f"{name}.png"), "wb") as f:
        f.write(png)
    with open(os.path.join(directory, f"{name}.yaml"), "w") as f:
        f.write(f"frequency: {frequency}\nbrightness: 1.0\n")


def _count_within(node, received, window):
    received.clear()
    deadline = time.time() + window
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    return len(received)


def _set_animation(node, client, value):
    future = client.set_parameters([Parameter("current_animation", Parameter.Type.STRING, value)])
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    assert future.done(), f"set current_animation={value!r} did not respond"
    return future.result().results[0]


@pytest.mark.launch(fixture=generate_test_description)
def test_led_strip_image_contract():
    rclpy.init()
    try:
        node = rclpy.create_node("test_led_strip_sub")
        # Publisher is BEST_EFFORT; a RELIABLE subscriber would be
        # QoS-incompatible and receive nothing.
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        received = []
        node.create_subscription(Image, "led_strip", received.append, qos)

        deadline = time.time() + 15.0
        while time.time() < deadline and not received:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received, "no Image on 'led_strip' within timeout"

        msg = received[0]
        assert msg.height == 1, f"height {msg.height} != 1"
        assert msg.width == NUM_LEDS, f"width {msg.width} != {NUM_LEDS}"
        assert msg.encoding == "rgb8", f"encoding {msg.encoding!r} != 'rgb8'"
        assert not msg.is_bigendian, "is_bigendian should be False"
        assert msg.step == NUM_LEDS * 3, f"step {msg.step} != {NUM_LEDS * 3}"
        assert len(msg.data) == NUM_LEDS * 3, f"data len {len(msg.data)} != {NUM_LEDS * 3}"

        node.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_current_animation_parameter():
    rclpy.init()
    try:
        node = rclpy.create_node("test_current_animation")
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        received = []
        node.create_subscription(Image, "led_strip", received.append, qos)

        client = AsyncParameterClient(node, NODE_NAME)
        assert client.wait_for_services(timeout_sec=15.0), "parameter services unavailable"

        # Publishing under the initial 'rainbow'.
        assert _count_within(node, received, 1.0) > 0, "not publishing initially"

        # 'none' stops publishing (firmware idle takes over).
        result = _set_animation(node, client, "none")
        assert result.successful, f"'none' rejected: {result.reason}"
        assert _count_within(node, received, 1.0) == 0, "still publishing after 'none'"

        # A valid animation resumes publishing.
        result = _set_animation(node, client, "car_wave")
        assert result.successful, f"'car_wave' rejected: {result.reason}"
        assert _count_within(node, received, 1.0) > 0, "did not resume after 'car_wave'"

        # An unknown name is rejected by the on-set validation callback.
        result = _set_animation(node, client, "does-not-exist")
        assert not result.successful, "unknown animation should be rejected"
        assert "does-not-exist" in result.reason

        node.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_animation_loaded_from_user_dir_at_runtime():
    """An animation dropped in after startup is selectable without a restart.

    This is the snap path: `create_config_dir` copies rosbot_utils/animations
    into config_dir, the operator adds a PNG there, and selecting it must not
    require restarting the driver.
    """
    rclpy.init()
    try:
        node = rclpy.create_node("test_animation_user_dir")
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        received = []
        node.create_subscription(Image, "led_strip", received.append, qos)

        client = AsyncParameterClient(node, NODE_NAME)
        assert client.wait_for_services(timeout_sec=15.0), "parameter services unavailable"

        name = "test_runtime_anim"

        # Not on disk yet, so the name must be refused and the reason must point
        # at the directories that were searched.
        result = _set_animation(node, client, name)
        assert not result.successful, "animation that is on no disk should be rejected"
        assert USER_DIR in result.reason, f"reason does not name the user dir: {result.reason}"

        _write_animation(USER_DIR, name, frequency=10.0)
        assert _set_animation(node, client, name).successful, "PNG on disk was not picked up"
        assert _count_within(node, received, 1.0) > 0, "not publishing the new animation"

        # The sidecar is re-read on select, so editing it and reselecting takes
        # effect too — 4 Hz must be clearly distinguishable from 10 Hz.
        _write_animation(USER_DIR, name, frequency=4.0)
        assert _set_animation(node, client, "none").successful
        assert _set_animation(node, client, name).successful
        count = _count_within(node, received, 2.0)
        assert 4 <= count <= 12, f"expected ~8 frames at 4 Hz over 2 s, got {count}"

        node.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_led_strip_enable_service():
    rclpy.init()
    try:
        node = rclpy.create_node("test_led_strip_enable")
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        received = []
        node.create_subscription(Image, "led_strip", received.append, qos)

        client = node.create_client(SetBool, "led_strip/enable")
        assert client.wait_for_service(timeout_sec=15.0), "led_strip/enable unavailable"

        def call(value):
            future = client.call_async(SetBool.Request(data=value))
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            assert future.done(), f"enable={value} did not respond"
            return future.result()

        assert _count_within(node, received, 1.0) > 0, "not publishing initially"

        assert call(False).success
        assert _count_within(node, received, 1.0) == 0, "still publishing after disable"

        # The selection survives the gate: switching while disabled must not
        # resume publishing, and enabling must bring back the new animation.
        param_client = AsyncParameterClient(node, NODE_NAME)
        assert param_client.wait_for_services(timeout_sec=15.0)
        assert _set_animation(node, param_client, "car_wave").successful
        assert _count_within(node, received, 1.0) == 0, "disable overridden by parameter set"

        assert call(True).success
        assert _count_within(node, received, 1.0) > 0, "did not resume after enable"

        node.destroy_node()
    finally:
        rclpy.shutdown()
