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

from typing import Optional

import pyudev


def find_device_port(
    vendor_id: str, product_id: str, default: Optional[str] = None
) -> Optional[str]:
    context = pyudev.Context()

    for device in context.list_devices(subsystem="tty"):
        if device.get("ID_VENDOR_ID") == vendor_id and device.get("ID_MODEL_ID") == product_id:
            return device.device_node

    return default
