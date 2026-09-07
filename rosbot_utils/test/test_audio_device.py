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

"""Card-ranking contract for the low-battery alert's playback device.

The fixtures mirror real /proc/asound dumps: the pcm info "name:" values are the
ones the kernel drivers actually report (snd-usb-audio always says "USB Audio",
which is what makes USB cards recognisable even when the card id does not
mention USB, e.g. id "MicroII").
"""

from rosbot_utils.audio_device import find_playback_device


def _card(root, index, card_id, playback=(), capture=()):
    card_dir = root / f"card{index}"
    card_dir.mkdir()
    (card_dir / "id").write_text(f"{card_id}\n")
    for dev, name in playback:
        pcm = card_dir / f"pcm{dev}p"
        pcm.mkdir()
        (pcm / "info").write_text(f"card: {index}\nname: {name}\n")
    for dev, name in capture:
        pcm = card_dir / f"pcm{dev}c"
        pcm.mkdir()
        (pcm / "info").write_text(f"card: {index}\nname: {name}\n")


def test_prefers_usb_over_analog_and_hdmi(tmp_path):
    _card(tmp_path, 0, "C930e", capture=[(0, "USB Audio")])
    _card(tmp_path, 1, "MicroII", playback=[(0, "USB Audio")])
    _card(tmp_path, 2, "PCH", playback=[(0, "ALC887-VD Analog")])
    _card(tmp_path, 3, "NVidia", playback=[(3, "HDMI 0"), (7, "HDMI 1")])

    assert find_playback_device(str(tmp_path)) == "plughw:CARD=MicroII,DEV=0"


def test_skips_card_without_playback_pcm(tmp_path):
    # The Jetson Orin Nano's internal APE is card 0 and exposes capture only.
    _card(tmp_path, 0, "APE", capture=[(0, "tegra-hda")])
    _card(tmp_path, 1, "Device", playback=[(0, "USB Audio")])

    assert find_playback_device(str(tmp_path)) == "plughw:CARD=Device,DEV=0"


def test_never_picks_hdmi(tmp_path):
    _card(tmp_path, 0, "vc4hdmi0", playback=[(0, "MAI PCM i2s-hifi-0")])
    _card(tmp_path, 1, "NVidia", playback=[(3, "HDMI 0")])

    # vc4hdmi0 is excluded by its card id, NVidia by its pcm name.
    assert find_playback_device(str(tmp_path)) is None


def test_falls_back_to_analog_when_no_usb_card(tmp_path):
    _card(tmp_path, 0, "PCH", playback=[(0, "ALC887-VD Analog")])

    assert find_playback_device(str(tmp_path)) == "plughw:CARD=PCH,DEV=0"


def test_no_playback_device_at_all(tmp_path):
    _card(tmp_path, 0, "C930e", capture=[(0, "USB Audio")])

    assert find_playback_device(str(tmp_path)) is None


def test_empty_and_missing_root(tmp_path):
    assert find_playback_device(str(tmp_path)) is None
    assert find_playback_device(str(tmp_path / "nope")) is None
