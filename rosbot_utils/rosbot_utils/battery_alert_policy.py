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


class LowBatteryAlertPolicy:

    def __init__(self, threshold: float, interval_ns: int):
        self._threshold = threshold
        self._interval_ns = interval_ns
        self._last_alert_ns: int | None = None

    def should_alert(self, percentage: float, now_ns: int) -> bool:
        # Firmware occasionally reports a single 0% sample; a battery really at 0% would
        # have cut the power already. `not >` also rejects NaN (unknown charge).
        if not percentage > 0.0 or percentage >= self._threshold:
            return False

        # The cooldown is deliberately not reset when the charge climbs back above the
        # threshold: the percentage is derived from a noisy voltage, and resetting on every
        # upward jitter re-armed the alert within seconds.
        if self._last_alert_ns is not None and now_ns - self._last_alert_ns < self._interval_ns:
            return False

        self._last_alert_ns = now_ns
        return True
