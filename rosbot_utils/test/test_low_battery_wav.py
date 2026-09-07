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

"""Guard the alert asset's format.

battery_alert opens the card through `plughw`, so libasound converts whatever
this file is to the card's native format — but only for formats it can read at
all, and only if the clip stays short enough for the node's play timeout.
"""

import os
import wave

from ament_index_python.packages import get_package_share_directory

PLAY_TIMEOUT_SEC = 5.0


def test_low_battery_wav_is_playable():
    path = os.path.join(get_package_share_directory("rosbot_utils"), "config", "low_battery.wav")
    assert os.path.isfile(path), f"Missing {path}"

    with wave.open(path) as clip:
        assert clip.getsampwidth() == 2, "expected 16-bit PCM"
        assert clip.getnchannels() in (1, 2)
        assert clip.getframerate() > 0
        duration = clip.getnframes() / clip.getframerate()

    assert duration < PLAY_TIMEOUT_SEC, (
        f"{path} is {duration:.2f} s long, which exceeds battery_alert's "
        f"{PLAY_TIMEOUT_SEC} s play timeout"
    )
