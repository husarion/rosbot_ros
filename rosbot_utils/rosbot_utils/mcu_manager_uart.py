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


class McuManagerUART:
    def __init__(self):
        self.acquire_system_info()

    def acquire_system_info(self):
        self.sys_arch = str(sh.uname("-m")).strip()
        self.device = ""
        if self.sys_arch == "armv7l":
            # Setups ThinkerBoard pins
            self.device = "ThinkerBoard"
            self.port = "/dev/ttyS1"
            gpio_chip = "/dev/gpiochip0"
            boot0_pin_no = 164
            reset_pin_no = 184

        elif self.sys_arch == "x86_64":
            # Setups UpBoard pins
            self.device = "UpBoard"
            self.port = "/dev/ttyS4"
            gpio_chip = "/dev/gpiochip4"
            boot0_pin_no = 17
            reset_pin_no = 18

        elif self.sys_arch == "aarch64":
            # Setups RPi pins
            self.device = get_raspberry_pi_model()
            self.port = "/dev/ttyAMA0"
            if self.device == "Raspberry Pi 4":
                gpio_chip = "/dev/gpiochip0"
            elif self.device == "Raspberry Pi 5":
                gpio_chip = "/dev/gpiochip4"
            else:
                gpio_chip = "/dev/gpiochip0"  # Default or error handling

            boot0_pin_no = 17
            reset_pin_no = 18
        else:
            raise ("Unknown device. Currently supported: Raspberry Pi 4/5, ThinkerBoard, UpBoard")

        try:
            chip = gpiod.Chip(gpio_chip)
            self.boot0_pin = chip.get_line(boot0_pin_no)
            self.reset_pin = chip.get_line(reset_pin_no)

            self.boot0_pin.request("Flash", type=gpiod.LINE_REQ_DIR_OUT, default_val=False)
            self.reset_pin.request("Flash", type=gpiod.LINE_REQ_DIR_OUT, default_val=False)
        except Exception as e:
            raise RuntimeError(f"Failed to access GPIO lines: {e}.")

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

    def flashing_operation(self, operation_name, binary_file=None, baudrate=115200):
        print(f"\n{operation_name} operation started")
        time.sleep(0.5)

        if operation_name == "Read-Protection":
            sh.stm32flash("-b", str(baudrate), "-k", self.port)
        elif operation_name == "Write-Protection":
            sh.stm32flash("-b", str(baudrate), "-u", self.port)
        elif operation_name == "Flashing":
            sh.stm32flash("-b", str(baudrate), "-v", "-w", binary_file, self.port, _out=sys.stdout)
        else:
            raise ("Unknown operation")

        print("Success")
        time.sleep(0.5)

    def get_port(self):
        return self.port

    def flash_firmware(self, binary_file):
        print(
            f"""
UART Flashing:
    Arch   : {self.sys_arch}
    Device : {self.device}
    File   : {binary_file}
    Port   : {self.port}
"""
        )
        try:
            self.enter_bootloader_mode()

            # self.flashing_operation("Read-Protection")
            # self.flashing_operation("Write-Protection")
            self.flashing_operation("Flashing", binary_file)

            self.exit_bootloader_mode()
        except Exception as e:
            if hasattr(e, "stderr"):
                error_msg = e.stderr.decode("utf-8").strip()
                raise RuntimeError(f"{error_msg}") from e
            raise e

    def reset_mcu(self):
        self.reset_pin.set_value(1)
        time.sleep(0.1)
        self.reset_pin.set_value(0)
        time.sleep(0.1)
