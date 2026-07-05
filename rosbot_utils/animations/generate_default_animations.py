#!/usr/bin/env python3
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
"""Render the built-in ROSbot LED animations to PNGs (row = frame, column = LED).

`rainbow` and `car_wave` are ports of the retired procedural nodes
(led_strip_rainbow.cpp / led_strip_car_wave.cpp) so the migration to the
animation_publisher preserves today's default look. Re-run to regenerate:

    python3 generate_default_animations.py
"""
import math
import os

from PIL import Image

NUM_LEDS = 18
HERE = os.path.dirname(os.path.abspath(__file__))


def hsv_to_rgb(hue_opencv):
    # Port of LEDStripPublisher::HsvToRgb (led_strip_rainbow.cpp).
    deg = hue_opencv * 2.0
    sector = deg / 60.0
    x = 1.0 - abs(math.fmod(sector, 2.0) - 1.0)
    s = int(math.floor(sector)) % 6
    r, g, b = [(1, x, 0), (x, 1, 0), (0, 1, x), (0, x, 1), (x, 0, 1), (1, 0, x)][s]
    return (int(r * 255.0), int(g * 255.0), int(b * 255.0))


def rainbow_frames():
    # Port of CreateRainbowGradient + SampleGradient. speed_=1.5,
    # gradient_resolution_=180 -> loops in exactly 180/1.5 = 120 frames.
    resolution = 180
    speed = 1.5
    gradient = [hsv_to_rgb(math.floor(179.0 * (i / resolution))) for i in range(resolution)]
    frames = []
    n_frames = int(resolution / speed)  # 120
    for f in range(n_frames):
        position = math.fmod(f * speed, resolution)
        row = []
        for i in range(NUM_LEDS):
            pos = math.fmod(position + i * resolution / NUM_LEDS, resolution)
            i0 = int(math.floor(pos))
            i1 = (i0 + 1) % resolution
            t = pos - i0
            row.append(tuple(
                int((1.0 - t) * gradient[i0][c] + t * gradient[i1][c]) for c in range(3)))
        frames.append(row)
    return frames


def car_wave_frames():
    # Port of LEDStripWavePublisher::Tick. kMaxPhase=9; render one steady
    # period into 64 frames (phase 0..9) so the PNG loops seamlessly.
    max_phase = 9.0
    wave_width = 2.3
    n_frames = 64
    frames = []
    for f in range(n_frames):
        phase = f * max_phase / n_frames
        row = []
        for i in range(NUM_LEDS):
            if i < 5:
                dist = abs(i - 0.5)
            elif i > 12:
                dist = abs(i - 16.5)
            else:
                dist = abs(i - 8.5)
            white = 5 <= i <= 12
            color = (255, 255, 255) if white else (255, 0, 0)
            offset = phase - wave_width
            intensity = max(0.0, min(1.0, 1.0 - abs(dist - offset) / wave_width))
            if phase < wave_width:
                intensity *= phase / wave_width
            row.append(tuple(int(color[c] * intensity) for c in range(3)))
        frames.append(row)
    return frames


def save(name, frames):
    img = Image.new("RGB", (NUM_LEDS, len(frames)))
    for y, row in enumerate(frames):
        for x, px in enumerate(row):
            img.putpixel((x, y), px)
    path = os.path.join(HERE, f"{name}.png")
    img.save(path)
    print(f"wrote {path} ({NUM_LEDS}x{len(frames)})")


if __name__ == "__main__":
    save("turn-off-lights", [[(0, 0, 0)] * NUM_LEDS])
    save("rainbow", rainbow_frames())
    save("car_wave", car_wave_frames())
