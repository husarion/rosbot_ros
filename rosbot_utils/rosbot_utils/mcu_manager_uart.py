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
            raise RuntimeError(
                "Unknown device. Currently supported: Raspberry Pi 4/5, ThinkerBoard, UpBoard"
            )

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
            raise ValueError(f"Unknown operation: {operation_name}")

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
            # boot0_pin stays driven high by gpiod across the whole session, so the
            # MCU self-reset triggered by -k/-u always re-enters the bootloader.
            self._disable_write_protection()
            self._flash_with_read_protection_recovery(binary_file)
            self.exit_bootloader_mode()
        except Exception as e:
            if hasattr(e, "stderr"):
                error_msg = e.stderr.decode("utf-8").strip()
                raise RuntimeError(error_msg) from e
            raise

    def _disable_write_protection(self):
        # AN3155: Write Unprotect only clears the WRP option bits and resets
        # the device -- no erase side effect -- so it's cheap and safe to
        # always run. (Readout Unprotect is the opposite: it mass-erases the
        # whole chip unconditionally, so that one stays reactive -- see
        # _flash_with_read_protection_recovery -- instead of running on every
        # flash regardless of whether the chip was ever protected.)
        try:
            self.flashing_operation("Write-Protection")
        except Exception as e:
            print(f"WARNING: Write-Protection step failed, continuing anyway: {e}")

    def _flash_with_read_protection_recovery(self, binary_file):
        try:
            self.flashing_operation("Flashing", binary_file)
        except Exception as e:
            error_msg = (
                e.stderr.decode("utf-8", errors="replace") if hasattr(e, "stderr") else str(e)
            )
            if "0x44" not in error_msg and "erase" not in error_msg.lower():
                raise
            print(
                "WARNING: erase was NACK'd (chip likely read-protected) -- "
                "disabling read-protection and retrying the flash once"
            )
            self.flashing_operation("Read-Protection")
            self.flashing_operation("Flashing", binary_file)

    def reset_mcu(self):
        self.reset_pin.set_value(1)
        time.sleep(0.1)
        self.reset_pin.set_value(0)
        time.sleep(0.1)
