#!/usr/bin/env python3

import subprocess
import os
import sys
import argparse
import signal
import glob
import requests
import ament_index_python.packages

# Global variable to hold the subprocess reference
subproc = None


def signal_handler(sig, frame):
    global subproc
    if subproc:
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


def find_firmware_file():
    # Find the install directory of 'rosbot_utils' package
    package_install_directory = ament_index_python.packages.get_package_share_directory(
        "rosbot_utils"
    )

    # Construct the path to the firmware directory
    firmware_dir = os.path.join(package_install_directory, "firmware")
    firmware_files = glob.glob(os.path.join(firmware_dir, "firmware*.bin"))

    if not firmware_files:
        firmware_url = (
            "https://github.com/husarion/rosbot_ros2_firmware/releases/download/0.8.0/firmware.bin"
        )
        firmware_path = os.path.join(firmware_dir, "firmware.bin")
        print("Downloading firmware...")
        download_firmware(firmware_url, firmware_path)
        return firmware_path

    return firmware_files[0]  # return the first found firmware file


def main(args=None):
    global subproc

    # Setting up the signal handler
    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser(description="Flash Firmware ROS 2 Node")
    parser.add_argument("--usb", action="store_true", help="Use the USB flashing script")
    parser.add_argument(
        "-p", "--port", default="/dev/ttyUSB0", help="Specify the USB port (default: /dev/ttyUSB0)"
    )
    parser.add_argument("--file", help="Specify the firmware file")

    args = parser.parse_args(args)

    # Determine the firmware file to use
    firmware_file = args.file if args.file else find_firmware_file()

    try:
        if args.usb:
            script_name = "flash-firmware-usb.py"
        else:
            script_name = "flash-firmware.py"

        script_path = os.path.join(os.path.dirname(__file__), script_name)
        additional_args = (
            ["-p", args.port, "--file", firmware_file] if args.usb else ["--file", firmware_file]
        )

        # Print the flashing details
        print(f"Flashing {firmware_file} over {args.port}")

        # # Starting the subprocess
        subproc = subprocess.Popen([sys.executable, script_path] + additional_args)
        return_code = subproc.wait()  # Wait for the subprocess to finish and get the return code

        if return_code != 0:
            print(f"Firmware flashing failed with return code {return_code}")
        else:
            print("Firmware flashing completed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error during firmware flashing: {e}")


if __name__ == "__main__":
    main()
