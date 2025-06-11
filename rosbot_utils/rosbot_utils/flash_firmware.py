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
import signal
import sys

import requests
from ament_index_python.packages import get_package_share_directory

from rosbot_utils.flash_firmware_uart import FirmwareFlasherUART
from rosbot_utils.flash_firmware_usb import FirmwareFlasherUSB
from rosbot_utils.utils import find_device_port

# Global variable to hold the subprocess reference
subproc = None


def signal_handler(sig, frame):
    global subproc
    if subproc:
        print("Terminating the flashing process...")
        subproc.terminate()
    sys.exit(0)


def download_firmware(firmware_url, firmware_path):
    response = requests.get(firmware_url, allow_redirects=True)
    if response.status_code == 200:
        with open(firmware_path, "wb") as f:
            f.write(response.content)
        print("Firmware downloaded successfully.")
    else:
        raise Exception(f"Failed to download firmware: HTTP {response.status_code}")


def find_firmware_file(path, robot_model):
    rosbot_link = (
        "https://github.com/husarion/rosbot_ros2_firmware/releases/download/0.11.0/firmware.bin"
    )
    rosbot_xl_link = (
        "https://github.com/husarion/rosbot_firmware/releases/download/v1.4.0/firmware.bin"
    )
    robot_download_link = {"rosbot": rosbot_link, "rosbot_xl": rosbot_xl_link}

    if not path:
        print("Downloading firmware...")
        download_firmware(robot_download_link[robot_model], path)

    return path


def main(args=None):
    global subproc

    # Setting up the signal handler
    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser(description="Flash ROSbot Firmware")
    parser.add_argument(
        "--robot-model",
        required=True,
        default=os.getenv("ROBOT_MODEL_NAME"),
        choices=["rosbot", "rosbot_xl"],
        help="Specify the robot model",
    )
    parser.add_argument(
        "--usb",
        action="store_true",
        help="Flash via USB. Automatically set for ROSbot XL; other ROSbots use UART by default. (You can flash firmware to ROSbot via USB-A from your PC)",
    )
    default_manipulator_serial_port = find_device_port("0403", "6015", "/dev/ttyUSB0")
    parser.add_argument(
        "-p",
        "--port",
        default=default_manipulator_serial_port,
        help="Specify the communication port",
    )
    parser.add_argument("-f", "--file", help="Specify the firmware file")
    args = parser.parse_args(args)

    port = args.port
    robot_model = args.robot_model
    if robot_model == "rosbot_xl":
        args.usb = True

    rosbot_utils = get_package_share_directory("rosbot_utils")
    rosbot_firmware = os.path.join(rosbot_utils, "firmware", "rosbot", "v0.11.0.bin")
    rosbot_xl_firmware = os.path.join(rosbot_utils, "firmware", "rosbot_xl", "v1.4.0.bin")
    firmware_dict = {"rosbot": rosbot_firmware, "rosbot_xl": rosbot_xl_firmware}
    firmware = args.file if args.file else firmware_dict[robot_model]

    try:
        if args.usb:
            FirmwareFlasherUSB(firmware, port)
        else:
            FirmwareFlasherUART(firmware)
        print("Firmware flashing completed successfully!")
    except Exception as e:
        print(f"ERROR: {e}")


if __name__ == "__main__":
    main()
