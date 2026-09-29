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

import math

from rosbot_utils.battery_alert_policy import LowBatteryAlertPolicy

SEC = 10**9
INTERVAL = 300 * SEC


def _policy() -> LowBatteryAlertPolicy:
    return LowBatteryAlertPolicy(threshold=0.12, interval_ns=INTERVAL)


def test_alerts_below_threshold_only():
    policy = _policy()
    assert not policy.should_alert(0.12, 0)
    assert not policy.should_alert(0.5, 0)
    assert policy.should_alert(0.11, 0)


def test_ignores_zero_and_nan_from_firmware():
    policy = _policy()
    assert not policy.should_alert(0.0, 0)
    assert not policy.should_alert(math.nan, 0)
    assert not policy.should_alert(-1.0, 0)
    assert policy.should_alert(0.05, 1 * SEC)


def test_jitter_around_threshold_alerts_at_most_once_per_interval():
    policy = _policy()
    alerts = [
        t
        for t in range(0, 1200 * SEC, SEC)
        if policy.should_alert(0.11 if (t // SEC) % 2 else 0.13, t)
    ]
    assert len(alerts) == 4
    assert all(b - a >= INTERVAL for a, b in zip(alerts, alerts[1:]))


def test_zero_glitch_does_not_consume_cooldown():
    policy = _policy()
    policy.should_alert(0.0, 0)
    assert policy.should_alert(0.10, 1 * SEC)
    assert not policy.should_alert(0.10, 299 * SEC)
    assert policy.should_alert(0.10, 301 * SEC)
