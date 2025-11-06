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

import argparse
import os
import shutil

from ament_index_python.packages import get_package_share_directory


def copy_config_folder(pkg_name, dest_dir):
    """
    Copy the entire <pkg_share>/config directory.

    This includes all files and subfolders into `dest_dir`.
    Prompts the user if the destination already exists.
    """
    try:
        pkg_share = get_package_share_directory(pkg_name)
    except Exception:
        print(
            f"Package '{pkg_name}' not found."
        )  # Hardware/Simulation specific package difference
        return
    src_config_dir = os.path.join(pkg_share, "config")
    dest_config_dir = os.path.join(dest_dir, pkg_name, "config")

    if not os.path.exists(src_config_dir):
        print(f"[WARNING] Package '{pkg_name}' has no 'config' directory at {src_config_dir}")
        return

    # Copy the entire config folder (files + directories)
    shutil.copytree(src_config_dir, dest_config_dir)


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Copy full 'config' folders from multiple ROS 2 packages."
    )
    parser.add_argument(
        "destination", help="Destination base directory to copy config folders into"
    )
    args = parser.parse_args()

    packages_list = [
        "rosbot_bringup",
        "rosbot_controller",
        "rosbot_description",
        "rosbot_gazebo",
        "rosbot_joy",
        "rosbot_localization",
        "rosbot_utils",
    ]

    # Ask if destination exists
    if os.path.exists(args.destination):
        response = (
            input(f"[?] Destination '{args.destination}' exists. Overwrite? [y/N]: ")
            .strip()
            .lower()
        )
        if response != "y":
            print("Aborted.")
            exit()
        shutil.rmtree(args.destination)

    for pkg in packages_list:
        copy_config_folder(pkg, args.destination)

    print(f"✅ All Config folders copied to '{args.destination}'\n")


if __name__ == "__main__":
    main()
