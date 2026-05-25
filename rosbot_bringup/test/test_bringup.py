# Copyright 2024 Husarion sp. z o.o.
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

from itertools import product

import launch_pytest
import pytest
import rclpy
from bringup_helpers import (
    BringupTestNode,
    make_bringup_launch_description,
    readings_data_test,
)


@launch_pytest.fixture
def generate_test_description(request):
    mecanum, namespace, robot_model = request.param
    print(
        f"""
Running test with
    mecanum={mecanum}
    namespace={namespace}
    robot_model={robot_model}
    """
    )

    return (
        make_bringup_launch_description(
            mecanum=mecanum,
            hardware_bridge="False",
            namespace=namespace,
            robot_model=robot_model,
        ),
        mecanum,
        namespace,
        robot_model,
    )


mecanum_options = ["True", "False"]
namespace_options = ["", "test_ns"]
robot_model_options = ["rosbot", "rosbot_xl"]
test_params = list(product(mecanum_options, namespace_options, robot_model_options))


@pytest.mark.parametrize("generate_test_description", test_params, indirect=True)
@pytest.mark.launch(fixture=generate_test_description)
def test_bringup(generate_test_description):
    _, _, namespace, _ = generate_test_description

    rclpy.init()
    try:
        node = BringupTestNode("test_bringup", namespace=namespace)
        node.start_publishing_fake_hardware()
        node.start_node_thread()

        readings_data_test(node)

    finally:
        rclpy.shutdown()
