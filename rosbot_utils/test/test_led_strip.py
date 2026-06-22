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

"""Contract guard for the C++ led_strip animation nodes.

Both ``led_strip_car_wave`` and ``led_strip_rainbow`` must publish a 1x18 ``rgb8``
``Image`` on ``led_strip`` over a BEST_EFFORT publisher. That shape + topic is the
contract consumed downstream (rosbot_xl.yaml ``exec:`` entry -> LED bridge); the
C++ migration must not drift from it.
"""

import time

import launch_pytest
import pytest
import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

NUM_LEDS = 18
LED_EXECUTABLES = ["led_strip_car_wave", "led_strip_rainbow"]


@launch_pytest.fixture
def generate_test_description(request):
    executable = request.param
    led_node = Node(package="rosbot_utils", executable=executable, output="screen")
    return LaunchDescription([led_node, ReadyToTest()]), executable


@pytest.mark.parametrize("generate_test_description", LED_EXECUTABLES, indirect=True)
@pytest.mark.launch(fixture=generate_test_description)
def test_led_strip_image_contract(generate_test_description):
    _, executable = generate_test_description

    rclpy.init()
    try:
        node = rclpy.create_node("test_led_strip_sub")
        # Publisher is BEST_EFFORT; a RELIABLE subscriber would be QoS-incompatible
        # and receive nothing.
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        received = []
        node.create_subscription(Image, "led_strip", received.append, qos)

        deadline = time.time() + 15.0
        while time.time() < deadline and not received:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received, f"{executable}: no Image on 'led_strip' within timeout"

        msg = received[0]
        assert msg.height == 1, f"{executable}: height {msg.height} != 1"
        assert msg.width == NUM_LEDS, f"{executable}: width {msg.width} != {NUM_LEDS}"
        assert msg.encoding == "rgb8", f"{executable}: encoding {msg.encoding!r} != 'rgb8'"
        assert not msg.is_bigendian, f"{executable}: is_bigendian should be False"
        assert msg.step == NUM_LEDS * 3, f"{executable}: step {msg.step} != {NUM_LEDS * 3}"
        assert (
            len(msg.data) == NUM_LEDS * 3
        ), f"{executable}: data len {len(msg.data)} != {NUM_LEDS * 3}"

        node.destroy_node()
    finally:
        rclpy.shutdown()


def _count_within(node, received, window):
    received.clear()
    deadline = time.time() + window
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    return len(received)


def _call_enable(node, client, enabled):
    request = SetBool.Request()
    request.data = enabled
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    assert future.done(), "led_strip/enable did not respond within timeout"
    assert future.result().success


@pytest.mark.parametrize("generate_test_description", LED_EXECUTABLES, indirect=True)
@pytest.mark.launch(fixture=generate_test_description)
def test_led_strip_enable_service(generate_test_description):
    _, executable = generate_test_description

    rclpy.init()
    try:
        node = rclpy.create_node("test_led_strip_enable")
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        received = []
        node.create_subscription(Image, "led_strip", received.append, qos)

        client = node.create_client(SetBool, "led_strip/enable")
        assert client.wait_for_service(
            timeout_sec=15.0
        ), f"{executable}: led_strip/enable service unavailable"

        assert _count_within(node, received, 1.0) > 0, f"{executable}: not publishing"

        _call_enable(node, client, False)
        assert (
            _count_within(node, received, 1.0) == 0
        ), f"{executable}: still publishing after disable"

        _call_enable(node, client, True)
        assert _count_within(node, received, 1.0) > 0, f"{executable}: did not resume after enable"

        node.destroy_node()
    finally:
        rclpy.shutdown()
