#!/usr/bin/python3

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


# This file is deprecated but stays here for backward compatibility look here:
# https://github.com/husarion/rosbot-docker/blob/ros2/Dockerfile.hardware#L82

import sh
import time
import sys
import argparse
from gpiozero import OutputDevice


class FirmwareFlasher:
    def __init__(self, sys_arch, binary_file):
        self.binary_file = binary_file
        self.sys_arch = sys_arch

        self.max_approach_no = 3

        print(f"System architecture: {self.sys_arch}")

        if self.sys_arch == "armv7l":
            # Setups ThinkerBoard pins
            print("Device: ThinkerBoard\n")
            self.serial_port = "/dev/ttyS1"
            boot0_pin_no = 164
            reset_pin_no = 184

        elif self.sys_arch == "x86_64":
            # Setups UpBoard pins
            print("Device: UpBoard\n")
            self.serial_port = "/dev/ttyS4"
            boot0_pin_no = 17
            reset_pin_no = 18

        elif self.sys_arch == "aarch64":
            # Setups RPi pins
            print("Device: RPi\n")
            self.serial_port = "/dev/ttyAMA0"
            boot0_pin_no = 17
            reset_pin_no = 18

        else:
            print("Unknown device...")

        self.boot0_pin = OutputDevice(boot0_pin_no)
        self.reset_pin = OutputDevice(reset_pin_no)

    def enter_bootloader_mode(self):
        self.boot0_pin.on()
        self.reset_pin.on()
        time.sleep(0.2)
        self.reset_pin.off()
        time.sleep(0.2)

    def exit_bootloader_mode(self):
        self.boot0_pin.off()
        self.reset_pin.on()
        time.sleep(0.2)
        self.reset_pin.off()
        time.sleep(0.2)

    def try_flash_operation(self, operation_name, flash_command, flash_args):
        for i in range(self.max_approach_no):
            try:
                flash_command(self.serial_port, *flash_args, _out=sys.stdout)
                time.sleep(0.2)
                break
            except Exception as e:
                print(f"{operation_name} error! Trying again.")
                print(f"Error: {e}")
                print("---------------------------------------")
        else:
            print(f"WARNING! {operation_name} went wrong.")

    def flash_firmware(self):
        self.enter_bootloader_mode()

        # Disable the flash write-protection
        self.try_flash_operation("Write-UnProtection", sh.stm32flash, ["-u"])

        # Disable the flash read-protection
        self.try_flash_operation("Read-UnProtection", sh.stm32flash, ["-k"])

        # Flashing the firmware
        flash_args = ["-v", "-w", self.binary_file, "-b", "115200"]
        self.try_flash_operation("Flashing", sh.stm32flash, flash_args)

        self.exit_bootloader_mode()


def main():
    parser = argparse.ArgumentParser(
        description="Flashing the firmware on STM32 microcontroller in ROSbot"
    )

    parser.add_argument(
        "-f",
        "--file",
        nargs="?",
        default="/root/firmware.bin",
        help="Path to a firmware file. Default = /root/firmware.bin",
    )

    binary_file = parser.parse_args().file
    sys_arch = sh.uname("-m").stdout.decode().strip()

    flasher = FirmwareFlasher(sys_arch, binary_file)
    flasher.flash_firmware()
    print("Done!")


if __name__ == "__main__":
    main()
