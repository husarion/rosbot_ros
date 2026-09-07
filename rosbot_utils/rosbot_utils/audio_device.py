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

"""Pick the ALSA playback device that drives the ROSbot's on-board speaker.

ALSA's ``default`` device is card 0, which is the speaker on none of the
supported SBCs: on the Jetson Orin Nano card 0 is the internal APE (no playback
PCM at all), on the Intel NUC it is the built-in HD-Audio codec and on the
Raspberry Pi 5 it is HDMI. Every ROSbot XL wires the speaker PCB behind a USB
sound card, so the device has to be resolved at runtime.

Discovery reads ``/proc/asound`` rather than parsing ``aplay -L``: ``aplay``'s
output is generated from the ALSA *config* tree, so a broken config (a missing
``/usr/share/alsa`` in the snap, say) makes it report no cards at all instead of
reporting a config problem. ``/proc/asound`` is a kernel ABI and tells the truth
about the hardware regardless of the config state, which keeps the two failure
modes distinguishable in the logs.
"""

import re
from pathlib import Path

PROC_ASOUND = "/proc/asound"

# The speaker PCB always sits behind a USB sound card. HDMI/SPDIF playback
# exists on all three SBCs but is never wired to it, so it must never be picked.
_PREFERRED = ("usb",)
_EXCLUDED = ("hdmi", "spdif", "s/pdif")

_CARD_DIR = re.compile(r"^card(\d+)$")
_PLAYBACK_PCM_DIR = re.compile(r"^pcm(\d+)p$")


def _read(path: Path) -> str:
    try:
        return path.read_text(errors="replace").strip()
    except OSError:
        return ""


def _pcm_name(pcm_dir: Path) -> str:
    for line in _read(pcm_dir / "info").splitlines():
        if line.startswith("name:"):
            return line.split(":", 1)[1].strip()
    return ""


def _candidates(root: Path):
    try:
        card_dirs = sorted(root.iterdir())
    except OSError:
        return

    for card_dir in card_dirs:
        card_match = _CARD_DIR.match(card_dir.name)
        if not card_match:
            continue

        # A card with no pcm<N>p directory has no playback PCM whatsoever. This
        # is what drops the Jetson's APE card, whose 900+ mixer controls once
        # made a naive amixer sweep in robot-tester time out.
        card_id = _read(card_dir / "id")
        if not card_id:
            continue

        try:
            pcm_dirs = sorted(card_dir.iterdir())
        except OSError:
            continue

        for pcm_dir in pcm_dirs:
            pcm_match = _PLAYBACK_PCM_DIR.match(pcm_dir.name)
            if not pcm_match:
                continue
            yield (
                int(card_match.group(1)),
                int(pcm_match.group(1)),
                card_id,
                f"{card_id} {_pcm_name(pcm_dir)}".strip(),
            )


def find_playback_device(root: str = PROC_ASOUND) -> str | None:
    """Return ``plughw:CARD=<id>,DEV=<n>`` for the best playback PCM, or None.

    ``CARD=<id>`` is used over ``plughw:<index>`` because the card index is
    reassigned across boots, while the id is stable. ``plughw`` is used over
    ``hw`` because ``low_battery.wav`` is 44.1 kHz stereo S16_LE and a raw
    device may refuse that format.
    """
    ranked = []
    for card_index, dev, card_id, label in _candidates(Path(root)):
        lowered = label.lower()
        if any(marker in lowered for marker in _EXCLUDED):
            continue
        preferred = 0 if any(marker in lowered for marker in _PREFERRED) else 1
        ranked.append((preferred, card_index, dev, card_id))

    if not ranked:
        return None

    _, _, dev, card_id = min(ranked)
    return f"plughw:CARD={card_id},DEV={dev}"
