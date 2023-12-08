#!/usr/bin/python3

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

import sh
import time
import sys
import argparse
from periphery import GPIO


class FirmwareFlasher:
    def __init__(self, sys_arch, binary_file):
        self.binary_file = binary_file
        self.sys_arch = sys_arch

        self.max_approach_no = 3

        print(f"System architecture: {self.sys_arch}")

        if self.sys_arch.stdout == b"armv7l\n":
            # Setups ThinkerBoard pins
            print("Device: ThinkerBoard\n")
            self.port = "/dev/ttyS1"
            boot0_pin_no = 164
            reset_pin_no = 184

        elif self.sys_arch.stdout == b"x86_64\n":
            # Setups UpBoard pins
            print("Device: UpBoard\n")
            self.port = "/dev/ttyS4"
            boot0_pin_no = 17
            reset_pin_no = 18

        elif self.sys_arch.stdout == b"aarch64\n":
            # Setups RPi pins
            print("Device: RPi\n")
            self.port = "/dev/ttyAMA0"
            boot0_pin_no = 17
            reset_pin_no = 18

        else:
            print("Unknown device...")

        self.boot0_pin = GPIO(boot0_pin_no, "out")
        self.reset_pin = GPIO(reset_pin_no, "out")

    def enter_bootloader_mode(self):
        self.boot0_pin.write(True)
        self.reset_pin.write(True)
        time.sleep(0.2)
        self.reset_pin.write(False)
        time.sleep(0.2)

    def exit_bootloader_mode(self):
        self.boot0_pin.write(False)
        self.reset_pin.write(True)
        time.sleep(0.2)
        self.reset_pin.write(False)
        time.sleep(0.2)

    def try_flash_operation(self, operation_name, flash_command, flash_args):
        for i in range(self.max_approach_no):
            try:
                flash_command(self.port, *flash_args, _out=sys.stdout)
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
    sys_arch = sh.uname("-m")

    flasher = FirmwareFlasher(sys_arch, binary_file)
    flasher.flash_firmware()
    print("Done!")


if __name__ == "__main__":
    main()
