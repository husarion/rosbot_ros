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


class McuManagerFTDI:
    def __init__(self, port: str):
        self.port = port
        self.device = "ftdi://ftdi:ft-x:/1"
        self.ftdi = Ftdi()

    def _open_ftdi_with_retry(self, max_attempts: int = 3, interval: float = 1.5):
        # USB re-enumeration after reset or abrupt process kill can take up to ~2s;
        # device.langids is unavailable during that window causing PyFTDI to fail.
        # On first failure, usbreset forces the kernel to release a stale device state.
        for attempt in range(max_attempts):
            try:
                self.ftdi.open_from_url(url=self.device)
                return
            except Exception:
                if attempt == max_attempts - 1:
                    raise
                if attempt == 0:
                    sh.usbreset("0403:6015")
                time.sleep(interval)
                self.ftdi = Ftdi()

    def enter_bootloader_mode(self):
        self._open_ftdi_with_retry()
        self.ftdi.set_cbus_direction(0b11, 0b11)  # set BOOT0 and RST to output
        self.ftdi.set_cbus_gpio(0b11)  # set BOOT0 to 1 and RST to 1
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b01)  # set BOOT0 to 1 and RST to 0
        time.sleep(0.1)
        self.ftdi.close()
        sh.usbreset("0403:6015")
        time.sleep(0.3)

    def exit_bootloader_mode(self):
        self._open_ftdi_with_retry()
        self.ftdi.set_cbus_direction(0b11, 0b11)  # set BOOT0 and RST to output
        self.ftdi.set_cbus_gpio(0b10)  # set BOOT0 to 1 and RST to 1
        time.sleep(0.3)
        self.ftdi.set_cbus_gpio(0b00)  # set BOOT0 to 1 and RST to 0
        time.sleep(0.1)
        self.ftdi.set_cbus_direction(0b11, 0b00)  # set BOOT0 and RST to input
        self.ftdi.close()
        sh.usbreset("0403:6015")
        time.sleep(0.3)

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

    def flash_firmware(self, binary_file):

        print(
            f"""
USB Flashing:
    File: {binary_file}
    Port: {self.port}"""
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
        self._open_ftdi_with_retry()
        time.sleep(0.1)
        self.ftdi.set_cbus_direction(0b11, 0b11)  # set BOOT0 and RST to output
        self.ftdi.set_cbus_gpio(0b10)  # set BOOT0 to 1 and RST to 1
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b00)  # set BOOT0 to 1 and RST to 0
        time.sleep(0.1)
        self.ftdi.set_cbus_direction(0b11, 0b00)  # set BOOT0 and RST to input
        self.ftdi.close()
        sh.usbreset("0403:6015")
        time.sleep(1.5)
