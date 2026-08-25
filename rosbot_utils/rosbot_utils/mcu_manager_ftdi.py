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
import usb.core
import usb.util
from pyftdi.ftdi import Ftdi

# CBUS0 - BOOT0
# CBUS1 - RST


def _error_text(exc):
    if hasattr(exc, "stderr"):
        return exc.stderr.decode("utf-8", errors="replace")
    return str(exc)


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

    def _wait_for_port(self, timeout: float = 5.0, interval: float = 0.05, raise_on_timeout=True):
        # ftdi_sio re-binds and udev recreates the /dev symlink asynchronously;
        # wait for the node before handing it to stm32flash or serial.Serial.
        deadline = time.monotonic() + timeout
        while not os.path.exists(self.port):
            if time.monotonic() >= deadline:
                if raise_on_timeout:
                    raise RuntimeError(f"Port {self.port} did not reappear after USB reset")
                return False
            time.sleep(interval)
        return True

    def _ftdi_on_bus(self):
        return usb.core.find(idVendor=0x0403, idProduct=0x6015) is not None

    def _restore_serial_driver(self):
        # pyftdi detached ftdi_sio to drive CBUS over libusb, so /dev/rosbot
        # vanished. Re-bind the kernel driver directly instead of usbreset: a
        # full USB port reset occasionally fails to re-enumerate and drops the
        # FTDI off the bus until a physical replug. This keeps the device on the
        # bus the whole time. The set_bitmode(RESET) also pulses RST with BOOT0
        # released, so the MCU comes up in firmware -- which is exactly what the
        # reset/exit paths want.
        usb_dev = self.ftdi.usb_dev
        self.ftdi.set_bitmode(0x00, Ftdi.BitMode.RESET)  # leave bitbang -> UART mode
        try:
            usb.util.release_interface(usb_dev, 0)
            if not usb_dev.is_kernel_driver_active(0):
                usb_dev.attach_kernel_driver(0)
        finally:
            usb.util.dispose_resources(usb_dev)

    def _reset_via_rebind(self):
        # Resets the MCU into firmware without a USB port reset, so the FTDI
        # never leaves the bus. Used wherever we only need the MCU running.
        self._open_ftdi_with_retry()
        time.sleep(0.1)
        self.ftdi.set_cbus_direction(0b11, 0b11)  # BOOT0 and RST as outputs
        self.ftdi.set_cbus_gpio(0b10)  # BOOT0 low, RST high
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b00)  # BOOT0 low, RST low (assert reset)
        time.sleep(0.1)
        self.ftdi.set_cbus_direction(0b11, 0b00)  # release -> RST floats high, MCU runs firmware
        self._restore_serial_driver()
        if not self._wait_for_port(raise_on_timeout=False):
            sh.usbreset("0403:6015")  # fallback only if the gentle re-bind didn't restore the tty
            self._wait_for_port()

    def enter_bootloader_mode(self):
        self._open_ftdi_with_retry()
        self.ftdi.set_cbus_direction(0b11, 0b11)  # BOOT0 and RST as outputs
        self.ftdi.set_cbus_gpio(0b11)  # BOOT0 high, RST high
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b01)  # BOOT0 high, RST low (assert reset)
        time.sleep(0.1)
        self.ftdi.close()
        # usbreset's re-enumeration is the RST rising edge that latches BOOT0
        # (held high above) into the bootloader. This is the only entry that
        # works on this board: a self-driven CBUS pulse does not reset this MCU
        # into the bootloader, and the gentle re-bind resets it into firmware.
        # So the bootloader path is the one place usbreset is unavoidable.
        sh.usbreset("0403:6015")
        time.sleep(0.6)  # let the stale /dev node disappear before polling for the new one
        if not self._wait_for_port(raise_on_timeout=False) or not self._ftdi_on_bus():
            raise RuntimeError(
                "FTDI did not re-enumerate after USB reset (dropped off the bus). "
                "Physically replug the robot's USB cable and retry."
            )
        # The tty node can appear slightly before the MCU's ROM bootloader is
        # actually listening/ready for the auto-baud sync byte -- root cause
        # of the occasional "Failed to init device, timeout." from stm32flash
        # (see flash_firmware's connect-retry for the backstop; this is just
        # cutting down how often that retry is needed).
        time.sleep(0.3)

    def exit_bootloader_mode(self):
        self._reset_via_rebind()

    def flashing_operation(self, operation_name, binary_file=None, baudrate=115200):
        print(f"\n{operation_name} operation started")
        # Extra settle time before talking to the bootloader -- see the sleep
        # after usbreset in enter_bootloader_mode() for why.
        time.sleep(1.0)

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

    def flash_firmware(self, binary_file, connect_attempts=3):
        print(
            f"""
USB Flashing:
    File: {binary_file}
    Port: {self.port}"""
        )
        for attempt in range(1, connect_attempts + 1):
            try:
                self.enter_bootloader_mode()
                self._disable_write_protection()
                self._flash_with_read_protection_recovery(binary_file)
                self.exit_bootloader_mode()
                return
            except Exception as e:
                error_msg = _error_text(e)
                # This exact race (port node exists, but the bootloader isn't
                # listening yet right after usbreset re-enumeration) resolved
                # on a bare re-run in HW testing every time it was hit -- so
                # retry the whole entry here instead of making the user do it.
                if attempt < connect_attempts and "Failed to init device" in error_msg:
                    print(
                        f"WARNING: bootloader did not respond after USB reset "
                        f"(attempt {attempt}/{connect_attempts}), retrying entry..."
                    )
                    time.sleep(1.0)
                    continue
                if hasattr(e, "stderr"):
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
            self.enter_bootloader_mode()  # re-latch BOOT0 after the MCU's own reset
        except Exception as e:
            print(f"WARNING: Write-Protection step failed, continuing anyway: {e}")

    def _flash_with_read_protection_recovery(self, binary_file):
        try:
            self.flashing_operation("Flashing", binary_file)
        except Exception as e:
            error_msg = _error_text(e)
            if "0x44" not in error_msg and "erase" not in error_msg.lower():
                raise
            print(
                "WARNING: erase was NACK'd (chip likely read-protected) -- "
                "disabling read-protection and retrying the flash once"
            )
            self.flashing_operation("Read-Protection")
            self.enter_bootloader_mode()  # RDP clear mass-erases + resets; re-latch BOOT0
            self.flashing_operation("Flashing", binary_file)

    def reset_mcu(self):
        self._reset_via_rebind()
        time.sleep(1.5)  # let the firmware boot before configure_robot's handshake
