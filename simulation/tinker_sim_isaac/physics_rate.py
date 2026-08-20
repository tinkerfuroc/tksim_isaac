"""Resolution of the simulator's physics rate.

Kept in its own module so it can be tested without importing Isaac.
"""

from __future__ import annotations

import math

# Below this, PhysX contact resolution stops being trustworthy for the
# grasp/collision behaviour this simulator is used to validate.
MINIMUM_PHYSICS_HZ = 30.0


def resolve_physics_hz(default_hz: float, override: str | None) -> float:
    """Resolve the physics rate, honouring an explicit opt-in override.

    Every per-step cost is paid ``physics_hz`` times per simulated second, so
    this is the largest single multiplier on wall-clock cost. Lowering it
    trades simulated contact fidelity for speed, which is a decision a run
    makes deliberately -- unset, the validated default is returned unchanged.

    Raising the rate above the default is refused: it would silently change
    the contact behaviour every validated result was produced against.
    """
    if override is None or not str(override).strip():
        return float(default_hz)
    try:
        value = float(str(override).strip())
    except (TypeError, ValueError):
        raise ValueError(
            f"TINKER_SIM_PHYSICS_HZ must be a number, got {override!r}"
        ) from None
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(
            f"TINKER_SIM_PHYSICS_HZ must be finite and positive, got {override!r}"
        )
    if value > float(default_hz):
        raise ValueError(
            f"TINKER_SIM_PHYSICS_HZ={value} exceeds the validated rate "
            f"{default_hz}; raising it would change validated contact behaviour"
        )
    if value < MINIMUM_PHYSICS_HZ:
        raise ValueError(
            f"TINKER_SIM_PHYSICS_HZ={value} is below the {MINIMUM_PHYSICS_HZ} Hz "
            "floor; PhysX contact resolution is not trustworthy below it"
        )
    return value
