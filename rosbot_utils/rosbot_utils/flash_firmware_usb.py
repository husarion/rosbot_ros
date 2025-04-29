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

import sh
from pyftdi.ftdi import Ftdi

# CBUS0 - BOOT0
# CBUS1 - RST


class FirmwareFlasherUSB:
    def __init__(self, binary_file, port):
        self.device = "ftdi://ftdi:ft-x:/1"
        self.ftdi = Ftdi()

        self.binary_file = binary_file
        self.port = port

        print(
            f"""
USB Flashing:
    File: {binary_file}
    Port: {port}"""
        )
        try:
            self.flash_firmware()
        except Exception as e:
            error_msg = e.stderr.decode("utf-8").strip()
            raise RuntimeError(f"{error_msg}") from e

    def enter_bootloader_mode(self):
        self.ftdi.open_from_url(url=self.device)
        self.ftdi.set_cbus_direction(0b11, 0b11)  # set BOOT0 and RST to output
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b11)  # set BOOT0 to 1 and RST to 1
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b01)  # set BOOT0 to 1 and RST to 0
        time.sleep(0.5)
        self.ftdi.close()
        time.sleep(0.1)
        sh.usbreset("0403:6015")
        time.sleep(1.0)

    def exit_bootloader_mode(self):
        self.ftdi.open_from_url(url=self.device)
        self.ftdi.set_cbus_direction(0b11, 0b11)  # set BOOT0 and RST to output
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b10)  # set BOOT0 to 1 and RST to 1
        time.sleep(0.3)
        self.ftdi.set_cbus_gpio(0b00)  # set BOOT0 to 1 and RST to 0
        time.sleep(0.1)
        self.ftdi.set_cbus_direction(0b11, 0b00)  # set BOOT0 and RST to input
        time.sleep(0.1)
        self.ftdi.close()
        sh.usbreset("0403:6015")
        time.sleep(1.0)

    def flashing_operation(self, operation_name):
        print(f"\n{operation_name} operation started")
        time.sleep(1.0)

        if operation_name == "Read-Protection":
            sh.stm32flash("-b", "115200", "-k", self.port)
        elif operation_name == "Write-Protection":
            sh.stm32flash("-b", "115200", "-u", self.port)
        elif operation_name == "Flashing":
            sh.stm32flash("-b", "115200", "-v", "-w", self.binary_file, self.port, _out=sys.stdout)
        else:
            raise ("Unknown operation")

        print("Success")
        time.sleep(1.0)

    def flash_firmware(self):
        self.enter_bootloader_mode()

        # self.flashing_operation("Read-Protection")
        # self.flashing_operation("Write-Protection")
        self.flashing_operation("Flashing")

        self.exit_bootloader_mode()
