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

"""controllers.yaml placeholder substitution invariants.

Mirrors the sed step in controller.yaml. Only schema is checked - CLAUDE.md §6 rule #4
forbids touching the tuned numeric values.
"""

import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

PLACEHOLDER_MANIPULATOR_STATE = "<manipulator_state>"
PLACEHOLDER_NAMESPACE_PREFIX = "<namespace>/"  # the sed pattern includes the trailing /


def _twist_mux_values(mecanum):
    """Mirror the `mecanum`-driven let-blocks in controller.yaml."""
    return {
        # The launch substitutes the raw `mecanum` arg, so this exercises the
        # capitalised Python-style value the ROS yaml parser actually receives.
        "<mecanum>": "True" if mecanum else "False",
        "<drive_controller>": (
            "mecanum_drive_controller" if mecanum else "differential_drive_controller"
        ),
    }


def _read(robot_model):
    path = os.path.join(
        get_package_share_directory("rosbot_controller"),
        "config",
        robot_model,
        "controllers.yaml",
    )
    assert os.path.isfile(path), f"Missing {path}"
    with open(path) as f:
        return f.read()


def _substitute(raw, namespace="", manipulator_state="active", mecanum=False):
    # Mirror the sed pattern in rosbot_controller/launch/controller.yaml. Ros2
    # namespaces typically end without a trailing slash; the launch expands the
    # bash var which already includes the slash, so an empty namespace yields
    # the bare topic.
    ns_replacement = f"{namespace}/" if namespace else ""
    out = raw.replace(PLACEHOLDER_NAMESPACE_PREFIX, ns_replacement)
    out = out.replace(PLACEHOLDER_MANIPULATOR_STATE, manipulator_state)
    for placeholder, value in _twist_mux_values(mecanum).items():
        out = out.replace(placeholder, value)
    return out


def test_rosbot_has_no_manipulator_placeholder():
    """rosbot (no arm) must not carry the manipulator initial-state placeholder."""
    raw = _read("rosbot")
    assert (
        PLACEHOLDER_MANIPULATOR_STATE not in raw
    ), "Plain rosbot controllers.yaml unexpectedly references manipulator state"


def test_rosbot_xl_has_manipulator_placeholder():
    raw = _read("rosbot_xl")
    assert PLACEHOLDER_MANIPULATOR_STATE in raw, (
        "rosbot_xl controllers.yaml lost its <manipulator_state> placeholder "
        "— the launch sed will leave the live yaml malformed."
    )


@pytest.mark.parametrize("robot_model", ["rosbot", "rosbot_xl"])
@pytest.mark.parametrize("namespace", ["", "robot1"])
@pytest.mark.parametrize("mecanum", [False, True])
def test_substituted_yaml_parses(robot_model, namespace, mecanum):
    raw = _read(robot_model)
    resolved = _substitute(raw, namespace=namespace, manipulator_state="active", mecanum=mecanum)
    # Verify no placeholders survived the substitution.
    assert "<namespace>" not in resolved, "namespace placeholder leaked through substitution"
    assert "<manipulator_state>" not in resolved, "manipulator_state placeholder leaked"
    assert "<mecanum>" not in resolved, "mecanum placeholder leaked through substitution"
    assert "<drive_controller>" not in resolved, "drive_controller placeholder leaked"
    data = yaml.safe_load(resolved)
    assert isinstance(data, dict)
    assert "/**" in data, "Missing ROS 2 wildcard key '/**'"


@pytest.mark.parametrize("robot_model", ["rosbot", "rosbot_xl"])
def test_twist_mux_controller_declared(robot_model):
    data = yaml.safe_load(_substitute(_read(robot_model)))
    controllers = data["/**"]["controller_manager"]["ros__parameters"]
    assert controllers["twist_mux_controller"]["type"] == "twist_mux_controller/TwistMuxController"


@pytest.mark.parametrize("robot_model", ["rosbot", "rosbot_xl"])
@pytest.mark.parametrize("mecanum", [False, True])
def test_twist_mux_chains_onto_spawned_drive_controller(robot_model, mecanum):
    """twist_mux_controller must target the drive controller the spawner actually
    starts for this `mecanum` value — a mismatch leaves it unable to claim the
    reference interfaces and the robot silently unresponsive. `holonomic` must agree
    too: the controller derives the interface naming convention from it."""
    data = yaml.safe_load(_substitute(_read(robot_model), mecanum=mecanum))
    params = data["/**"]["twist_mux_controller"]["ros__parameters"]
    expected = "mecanum_drive_controller" if mecanum else "differential_drive_controller"

    assert params["holonomic"] is mecanum
    assert params["drive_controller"] == expected
    assert data["/**"]["controller_manager"]["ros__parameters"][expected]["type"]


@pytest.mark.parametrize("robot_model", ["rosbot", "rosbot_xl"])
def test_twist_mux_input_priorities(robot_model):
    """Manual must outrank autonomous, which must outrank the bare cmd_vel input."""
    data = yaml.safe_load(_substitute(_read(robot_model)))
    inputs = data["/**"]["twist_mux_controller"]["ros__parameters"]["cmd_vel_inputs"]

    assert inputs["manual"]["topic"] == "manual/cmd_vel"
    assert inputs["autonomous"]["topic"] == "autonomous/cmd_vel"
    assert inputs["unknown"]["topic"] == "cmd_vel"
    assert (
        inputs["manual"]["priority"]
        > inputs["autonomous"]["priority"]
        > inputs["unknown"]["priority"]
    )


@pytest.mark.parametrize("manipulator_state", ["active", "inactive"])
def test_manipulator_initial_state_routes_to_expected_key(manipulator_state):
    raw = _read("rosbot_xl")
    resolved = _substitute(raw, namespace="", manipulator_state=manipulator_state)
    data = yaml.safe_load(resolved)
    initial_state = data["/**"]["controller_manager"]["ros__parameters"][
        "hardware_components_initial_state"
    ]
    assert manipulator_state in initial_state, (
        f"Substituted manipulator_state='{manipulator_state}' did not land as a key "
        f"under hardware_components_initial_state (got keys: {list(initial_state)})"
    )
    assert "OpenManipulatorXSystem" in initial_state[manipulator_state]
