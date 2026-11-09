# Copyright 2026 Husarion sp. z o.o.
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

"""Device selection for the ROSbot XL FTDI flasher.

Flashing must hit exactly the board the port points at, and an unrelated FTDI
device on the bus must not get in the way.
"""

from types import SimpleNamespace

import pytest
import rosbot_utils.mcu_manager_ftdi as ftdi


def _dev(bus, address):
    return SimpleNamespace(bus=bus, address=address)


def _patch_bus(monkeypatch, devices):
    monkeypatch.setattr(ftdi.usb.core, "find", lambda **_: iter(devices))


def test_single_device_is_used_without_a_port(monkeypatch):
    target = _dev(1, 22)
    _patch_bus(monkeypatch, [target])
    assert ftdi.find_ft_x_device() is target


def test_no_device_is_an_error(monkeypatch):
    _patch_bus(monkeypatch, [])
    with pytest.raises(RuntimeError, match="No FTDI"):
        ftdi.find_ft_x_device("/dev/rosbot")


def test_port_selects_among_several(monkeypatch):
    first, second = _dev(1, 22), _dev(1, 31)
    _patch_bus(monkeypatch, [first, second])
    monkeypatch.setattr(ftdi, "_usb_location_of_tty", lambda _: (1, 31))
    assert ftdi.find_ft_x_device("/dev/ttyUSB3") is second


def test_ambiguous_choice_is_refused(monkeypatch):
    _patch_bus(monkeypatch, [_dev(1, 22), _dev(1, 31)])
    monkeypatch.setattr(ftdi, "_usb_location_of_tty", lambda _: None)
    with pytest.raises(RuntimeError, match="does not identify"):
        ftdi.find_ft_x_device("/dev/rosbot")
    with pytest.raises(RuntimeError, match="does not identify"):
        ftdi.find_ft_x_device()


def test_tty_location_is_read_from_sysfs_parents(tmp_path, monkeypatch):
    usb_dev = tmp_path / "1-5"
    iface = usb_dev / "1-5:1.0"
    iface.mkdir(parents=True)
    (usb_dev / "busnum").write_text("1\n")
    (usb_dev / "devnum").write_text("22\n")
    real = lambda p: str(iface) if str(p).endswith("/device") else str(p)  # noqa: E731
    monkeypatch.setattr(ftdi.os.path, "realpath", real)
    assert ftdi._usb_location_of_tty("/dev/ttyUSB1") == (1, 22)
