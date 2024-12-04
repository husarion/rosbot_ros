# Copyright 2020 ros2_control Development Team
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

from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction


def check_controller_status(context):
    green_color = "\033[92m"
    reset_color = "\033[0m"

    print(f"{green_color}All systems are up and running!{reset_color}")
    with open("/var/tmp/rosbot_status.txt", "w") as status_file:
        status_file.write("healthy")


def generate_launch_description():
    with open("/var/tmp/rosbot_status.txt", "w") as status_file:
        status_file.write("unhealthy")

    check_controller = TimerAction(
        period=15.0,
        actions=[OpaqueFunction(function=check_controller_status)],
    )

    return LaunchDescription([check_controller])
