from __future__ import annotations

import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.backend import (  # noqa: E402
    IsaacWholeRobotBackend,
    slew_velocity_target,
)


def test_velocity_slew_limits_acceleration_and_braking() -> None:
    assert slew_velocity_target(0.0, 2.0, 0.5) == 0.5
    assert slew_velocity_target(2.0, 0.0, 0.5) == 1.5
    assert slew_velocity_target(-2.0, 0.0, 0.5) == -1.5


def test_velocity_slew_does_not_overshoot_target() -> None:
    assert slew_velocity_target(0.8, 1.0, 0.5) == pytest.approx(1.0)
    assert slew_velocity_target(1.2, 1.0, 0.5) == pytest.approx(1.0)


@pytest.mark.parametrize(
    "args",
    [
        (float("nan"), 0.0, 1.0),
        (0.0, float("inf"), 1.0),
        (0.0, 1.0, -0.1),
    ],
)
def test_velocity_slew_rejects_invalid_inputs(args: tuple[float, float, float]) -> None:
    with pytest.raises(ValueError):
        slew_velocity_target(*args)


@pytest.mark.parametrize("active", [False, True])
def test_repeated_safety_state_is_idempotent(active: bool) -> None:
    backend = object.__new__(IsaacWholeRobotBackend)
    backend._safety_stopped = active

    # No other backend fields are populated: a repeated sample must return
    # before it clears the acceleration-limited wheel state.
    backend.set_safety_stop(active)
