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

import os
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

    def _open_ftdi(self, timeout: float = 5.0, interval: float = 0.1):
        # USB re-enumeration after a usbreset can take up to ~2s; until the
        # device is back, langids is unavailable and open_from_url raises. Poll
        # for readiness instead of guessing a fixed sleep.
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.ftdi = Ftdi()
                self.ftdi.open_from_url(url=self.device)
                return
            except Exception:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(interval)

    def _wait_for_port(self, timeout: float = 5.0, interval: float = 0.05):
        # usbreset re-binds ftdi_sio and udev recreates the /dev symlink
        # asynchronously; wait for the node before handing it to stm32flash.
        deadline = time.monotonic() + timeout
        while not os.path.exists(self.port):
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Port {self.port} did not reappear after USB reset")
            time.sleep(interval)

    def _pulse_reset(self, boot0_high: bool):
        # Drive the RST rising edge ourselves with BOOT0 actively held, so the
        # MCU latches the intended BOOT0 deterministically. The trailing usbreset
        # only re-binds the kernel serial driver (libusb detached it) — it must
        # not be relied on to produce the reset edge, which is the old flaky path.
        boot0 = 0b01 if boot0_high else 0b00  # CBUS0 = BOOT0
        self._open_ftdi()
        self.ftdi.set_cbus_direction(0b11, 0b11)  # BOOT0 and RST as outputs
        self.ftdi.set_cbus_gpio(0b10 | boot0)  # RST high (idle)
        time.sleep(0.05)
        self.ftdi.set_cbus_gpio(0b00 | boot0)  # RST low (assert reset)
        time.sleep(0.05)
        self.ftdi.set_cbus_gpio(0b10 | boot0)  # RST high while BOOT0 held -> MCU latches BOOT0
        time.sleep(0.05)
        self.ftdi.set_cbus_direction(0b11, 0b00)  # release BOOT0 and RST to inputs
        self.ftdi.close()
        sh.usbreset("0403:6015")
        self._wait_for_port()

    def enter_bootloader_mode(self):
        self._pulse_reset(boot0_high=True)

    def exit_bootloader_mode(self):
        self._pulse_reset(boot0_high=False)

    def flashing_operation(self, operation_name, binary_file=None, baudrate=460800):
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

    def flash_firmware(self, binary_file):

        print(
            f"""
USB Flashing:
    File: {binary_file}
    Port: {self.port}"""
        )
        try:
            self.enter_bootloader_mode()
            self.flashing_operation("Flashing", binary_file)
            self.exit_bootloader_mode()
        except Exception as e:
            if hasattr(e, "stderr"):
                error_msg = e.stderr.decode("utf-8").strip()
                raise RuntimeError(error_msg) from e
            raise

    def reset_mcu(self):
        self._pulse_reset(boot0_high=False)
        time.sleep(1.5)  # let the firmware boot before configure_robot's handshake
