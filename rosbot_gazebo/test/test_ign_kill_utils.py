# Copyright 2024 Husarion sp. z o.o.
# Copyright 2023 Open Source Robotics Foundation, Inc.
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

import psutil

# The pytest cannot kill properly the Gazebo Ignition's tasks what blocks launching
# several tests in a row.
# https://github.com/ros-controls/gz_ros2_control/blob/master/gz_ros2_control_tests/tests/position_test.py


def kill_ign_linux_processes():
    for proc in psutil.process_iter():
        # check whether the process name matches
        if proc.name() == "ruby":
            proc.kill()


kill_ign_linux_processes()
