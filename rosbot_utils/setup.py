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

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rosbot_utils"
common_config = [f for f in glob("config/*") if os.path.isfile(f)]
rosbot_config = [f for f in glob("config/rosbot/*.yaml") if os.path.isfile(f)]
rosbot_xl_config = [f for f in glob("config/rosbot_xl/*.yaml") if os.path.isfile(f)]
rosbot_firmware = [f for f in glob("firmware/rosbot/*.bin") if os.path.isfile(f)]
rosbot_xl_firmware = [f for f in glob("firmware/rosbot_xl/*.bin") if os.path.isfile(f)]
launch_files = glob("launch/*.launch.py")

setup(
    name=package_name,
    version="0.15.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", common_config),
        ("share/" + package_name + "/config/rosbot", rosbot_config),
        ("share/" + package_name + "/config/rosbot_xl", rosbot_xl_config),
        ("share/" + package_name + "/firmware/rosbot", rosbot_firmware),
        ("share/" + package_name + "/firmware/rosbot_xl", rosbot_xl_firmware),
        ("share/" + package_name + "/launch", launch_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Husarion",
    maintainer_email="support@husarion.com",
    description="Utilities for ROSbot Series",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "flash_firmware = rosbot_utils.flash_firmware:main",
            "create_config_dir = rosbot_utils.create_config_dir:main",
        ],
    },
)
