# Copyright 2023 Husarion
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

# The pytest cannot kill properly the Gazebo Ignition's tasks what blocks launching
# several tests in a row.


def kill_ign_linux_processes():
    try:
        result = subprocess.run(
            ["pgrep", "-f", "ign gazebo"],
            capture_output=True,
            text=True,
            check=True,
        )

        pids = result.stdout.strip().split("\n")
        for pid in pids:
            subprocess.run(["kill", pid], check=True)
        print("Killed all Ignition Gazebo processes")
    except subprocess.CalledProcessError as e:
        print(e)


kill_ign_linux_processes()
