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

"""Single source of truth for the MCU firmware version shipped with this driver.

The driver and the firmware are tightly coupled (see CLAUDE.md hard rule #6.1).
Bumping this constant without also swapping the corresponding ``.bin`` files in
``rosbot_utils/firmware/`` will break the firmware sanity check in
``configure_robot`` and is a release-blocking error.
"""

FIRMWARE_VERSION: str = "v2.0.2-jazzy"
