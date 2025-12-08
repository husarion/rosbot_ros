#!/usr/bin/env python3

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

import subprocess
import sys


def run_command(cmd, timeout_sec):
    try:
        subprocess.run(cmd, shell=True, check=True, timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        print(f"Command timed out: {cmd}")
    except subprocess.CalledProcessError:
        # Mimics '|| true' in bash: ignore non-zero exit codes
        print(f"Command failed but continuing: {cmd}")


def main():
    if len(sys.argv) < 2:
        print("Usage: ros2 run rosbot_controller arm_control [active|inactive]")
        sys.exit(1)

    state = sys.argv[1].lower()

    if state == "inactive":
        print("Manipulator torque will be turned off")
        run_command("ros2 control set_controller_state manipulator_controller inactive", 7)
        run_command("ros2 control set_controller_state gripper_controller inactive", 5)
        run_command("ros2 control set_hardware_component_state OpenManipulatorXSystem inactive", 5)

    elif state == "active":
        run_command("ros2 control set_hardware_component_state OpenManipulatorXSystem active", 7)
        run_command("ros2 control set_controller_state manipulator_controller active", 5)
        run_command("ros2 control set_controller_state gripper_controller active", 5)

    else:
        print(f"Invalid state: '{state}'")
        print("Usage: ros2 run rosbot_controller arm_control [active|inactive]")
        sys.exit(1)


if __name__ == "__main__":
    main()
