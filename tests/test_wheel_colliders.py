from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.backend import (  # noqa: E402
    WHEEL_COLLIDER_LINKS,
    resolve_wheel_collider_mode,
    wheel_sphere_radius,
)


def test_every_wheel_link_gets_the_override() -> None:
    assert WHEEL_COLLIDER_LINKS == (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    )


def test_wheel_collider_defaults_to_sphere() -> None:
    assert resolve_wheel_collider_mode(None) == "sphere"
    assert resolve_wheel_collider_mode("") == "sphere"
    assert resolve_wheel_collider_mode("sphere") == "sphere"


def test_wheel_collider_cylinder_opt_out() -> None:
    assert resolve_wheel_collider_mode("cylinder") == "cylinder"
    assert resolve_wheel_collider_mode(" Cylinder ") == "cylinder"


def test_wheel_collider_rejects_unknown_modes() -> None:
    with pytest.raises(ValueError, match="TINKER_SIM_WHEEL_COLLIDER"):
        resolve_wheel_collider_mode("capsule")


def test_sphere_radius_is_the_authored_wheel_radius() -> None:
    """The sphere touches the floor where the cylinder did, so the base
    height and the odometry wheel radius are unchanged."""
    assert wheel_sphere_radius(0.0525) == 0.0525
    assert wheel_sphere_radius(0.03) == 0.03


@pytest.mark.parametrize("bad", [0.0, -0.01, float("nan"), float("inf"), None, True])
def test_sphere_radius_rejects_bad_input(bad) -> None:
    with pytest.raises((TypeError, ValueError)):
        wheel_sphere_radius(bad)
