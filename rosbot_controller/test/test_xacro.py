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

"""xacro -> URDF with per-model controllers.yaml + use_sim plumbing.

Full configuration matrix lives in rosbot_description; here we use two anchors.
"""

import os

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

# basic = no arm, manipulation_pro = arm; widest component set.
ROSBOT_XL_REPRESENTATIVE_CONFIGURATIONS = ["basic", "manipulation_pro"]


def _matrix():
    cases = []
    for mecanum in ("True", "False"):
        for use_sim in ("True", "False"):
            cases.append(("rosbot", mecanum, use_sim, None))
            for cfg in ROSBOT_XL_REPRESENTATIVE_CONFIGURATIONS:
                cases.append(("rosbot_xl", mecanum, use_sim, cfg))
    return cases


@pytest.mark.parametrize("robot_model,mecanum,use_sim,configuration", _matrix())
def test_xacro_with_controller_config(robot_model, mecanum, use_sim, configuration):
    rosbot_controller = get_package_share_directory("rosbot_controller")
    rosbot_description = get_package_share_directory("rosbot_description")
    controller_config = os.path.join(rosbot_controller, "config", robot_model, "controllers.yaml")

    mappings = {
        "controller_config": controller_config,
        "mecanum": mecanum,
        "use_sim": use_sim,
    }
    if robot_model == "rosbot_xl":
        mappings["configuration"] = configuration

    xacro_path = os.path.join(rosbot_description, "urdf", f"{robot_model}.urdf.xacro")
    doc = xacro.process_file(xacro_path, mappings=mappings)
    urdf = doc.toxml()

    assert (
        "<link" in urdf
    ), f"No <link> for ({robot_model}, mecanum={mecanum}, use_sim={use_sim}, {configuration})"
    assert (
        "<joint" in urdf
    ), f"No <joint> for ({robot_model}, mecanum={mecanum}, use_sim={use_sim}, {configuration})"
