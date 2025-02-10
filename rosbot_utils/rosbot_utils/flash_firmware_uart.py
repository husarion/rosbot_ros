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

import sys
import time

import gpiod
import sh


def get_raspberry_pi_model():
    try:
        with open("/proc/cpuinfo", "r") as f:
            for line in f:
                if "Model" in line:
                    model_info = line.split(":")[1].strip()
                    if "Raspberry Pi 4" in model_info:
                        return "Raspberry Pi 4"
                    elif "Raspberry Pi 5" in model_info:
                        return "Raspberry Pi 5"
                    else:
                        return "Unknown Raspberry Pi Model"
    except FileNotFoundError:
        return "Not a Raspberry Pi"


class FirmwareFlasherUART:
    def __init__(self, binary_file):
        self.binary_file = binary_file
        self.acquire_system_info()

        try:
            self.flash_firmware()
        except Exception as e:
            error_msg = e.stderr.decode("utf-8").strip()
            raise RuntimeError(f"{error_msg}") from e

    def acquire_system_info(self):
        sys_arch = str(sh.uname("-m")).strip()
        device = ""
        if sys_arch == "armv7l":
            # Setups ThinkerBoard pins
            device = "ThinkerBoard"
            self.port = "/dev/ttyS1"
            gpio_chip = "/dev/gpiochip0"
            boot0_pin_no = 164
            reset_pin_no = 184

        elif sys_arch == "x86_64":
            # Setups UpBoard pins
            device = "UpBoard"
            self.port = "/dev/ttyS4"
            gpio_chip = "/dev/gpiochip4"
            boot0_pin_no = 17
            reset_pin_no = 18

        elif sys_arch == "aarch64":
            # Setups RPi pins
            device = get_raspberry_pi_model()
            self.port = "/dev/ttyAMA0"
            if device == "Raspberry Pi 4":
                gpio_chip = "/dev/gpiochip0"
            elif device == "Raspberry Pi 5":
                gpio_chip = "/dev/gpiochip4"
            else:
                gpio_chip = "/dev/gpiochip0"  # Default or error handling

            boot0_pin_no = 17
            reset_pin_no = 18
        else:
            raise ("Unknown device. Currently supported: Raspberry Pi 4/5, ThinkerBoard, UpBoard")
        print(
            f"""
UART Flashing:
    Arch   : {sys_arch}
    Device : {device}
    File   : {self.binary_file}
    Port   : {self.port}
"""
        )

        chip = gpiod.Chip(gpio_chip)
        self.boot0_pin = chip.get_line(boot0_pin_no)
        self.reset_pin = chip.get_line(reset_pin_no)

        self.boot0_pin.request("Flash", type=gpiod.LINE_REQ_DIR_OUT, default_val=False)
        self.reset_pin.request("Flash", type=gpiod.LINE_REQ_DIR_OUT, default_val=False)

    def enter_bootloader_mode(self):
        self.boot0_pin.set_value(1)
        self.reset_pin.set_value(1)
        time.sleep(0.1)
        self.reset_pin.set_value(0)
        time.sleep(0.1)

    def exit_bootloader_mode(self):
        self.boot0_pin.set_value(0)
        self.reset_pin.set_value(1)
        time.sleep(0.3)
        self.reset_pin.set_value(0)
        time.sleep(0.1)

    def flashing_operation(self, operation_name):
        print(f"\n{operation_name} operation started")
        time.sleep(0.5)

        if operation_name == "Read-Protection":
            sh.stm32flash("-b", "115200", "-k", self.port)
        elif operation_name == "Write-Protection":
            sh.stm32flash("-b", "115200", "-u", self.port)
        elif operation_name == "Flashing":
            sh.stm32flash("-b", "115200", "-v", "-w", self.binary_file, self.port, _out=sys.stdout)
        else:
            raise ("Unknown operation")

        print("Success")
        time.sleep(0.5)

    def flash_firmware(self):
        self.enter_bootloader_mode()

        # self.flashing_operation("Read-Protection")
        # self.flashing_operation("Write-Protection")
        self.flashing_operation("Flashing")

        self.exit_bootloader_mode()
