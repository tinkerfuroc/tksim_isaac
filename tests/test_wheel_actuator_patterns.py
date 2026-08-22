from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.backend import (  # noqa: E402
    CASTER_JOINT_PATTERNS,
    WHEEL_ACTUATOR_JOINT_PATTERNS,
)

DRIVE_WHEEL_JOINTS = ("front_left_wheel_joint", "front_right_wheel_joint")
CASTER_JOINTS = (
    "rear_left_swivel_joint",
    "rear_right_swivel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)


def _matches(patterns: tuple[str, ...], name: str) -> bool:
    # Isaac Lab resolves joint_names_expr with re.fullmatch per pattern.
    return any(re.fullmatch(pattern, name) for pattern in patterns)


def test_wheel_actuator_group_drives_exactly_the_front_wheels() -> None:
    for name in DRIVE_WHEEL_JOINTS:
        assert _matches(WHEEL_ACTUATOR_JOINT_PATTERNS, name), name
    for name in CASTER_JOINTS:
        assert not _matches(WHEEL_ACTUATOR_JOINT_PATTERNS, name), name


def test_caster_group_frees_swivels_and_caster_wheels() -> None:
    """Casters must be passive: a driven or held caster brakes the base.

    A swivel left to the USD's importer drive is held straight; a caster wheel
    driven at the front wheels' angular velocity is braked by the radius
    mismatch.  Either way the base cannot turn in place.
    """
    for name in CASTER_JOINTS:
        assert _matches(CASTER_JOINT_PATTERNS, name), name
    for name in DRIVE_WHEEL_JOINTS:
        assert not _matches(CASTER_JOINT_PATTERNS, name), name
