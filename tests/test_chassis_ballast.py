from __future__ import annotations

import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.backend import (  # noqa: E402
    CHASSIS_BALLAST_TARGET_DIAGONAL_INERTIA,
    CHASSIS_BALLAST_TARGET_MASS_KG,
    chassis_ballast_target_properties,
)


def test_chassis_ballast_adds_ten_kg_and_scales_inertia() -> None:
    mass, inertia = chassis_ballast_target_properties(20.0)

    assert mass == CHASSIS_BALLAST_TARGET_MASS_KG == 30.0
    assert inertia == CHASSIS_BALLAST_TARGET_DIAGONAL_INERTIA
    assert inertia == pytest.approx((0.33, 0.45, 0.33))


def test_chassis_ballast_override_is_idempotent() -> None:
    assert chassis_ballast_target_properties(30.0) == (
        CHASSIS_BALLAST_TARGET_MASS_KG,
        CHASSIS_BALLAST_TARGET_DIAGONAL_INERTIA,
    )


@pytest.mark.parametrize("mass", [0.0, 19.0, 31.0, float("nan")])
def test_chassis_ballast_rejects_unknown_source_mass(mass: float) -> None:
    with pytest.raises(ValueError, match="20 kg source mass or the 30 kg"):
        chassis_ballast_target_properties(mass)
