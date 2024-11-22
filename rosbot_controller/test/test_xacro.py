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

import itertools
import os

import xacro
from ament_index_python.packages import get_package_share_directory


def test_rosbot_description_parsing():
    mecanum_values = ["true", "false"]
    use_sim_values = ["true", "false"]
    use_multirobot_system_values = ["true", "false"]

    all_combinations = list(
        itertools.product(
            mecanum_values,
            use_sim_values,
            use_multirobot_system_values,
        )
    )

    for combination in all_combinations:
        mecanum, use_sim, use_multirobot_system = combination
        mappings = {
            "mecanum": mecanum,
            "use_sim": use_sim,
            "use_multirobot_system": use_multirobot_system,
        }
        rosbot_description = get_package_share_directory("rosbot_description")
        xacro_path = os.path.join(rosbot_description, "urdf/rosbot.urdf.xacro")
        try:
            xacro.process_file(xacro_path, mappings=mappings)
        except xacro.XacroException as e:
            assert False, (
                f"xacro parsing failed: {str(e)} for mecanum: {mecanum}, use_sim:" f" {use_sim}"
            )
