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


# Below this the command/clock cadence (joint-target writes, wheel slew,
# /clock, gateway strides) is too coarse for the stack that consumes it.
MINIMUM_CONTROL_HZ = 30.0


def resolve_control_hz(physics_hz: float, override: str | None) -> float:
    """Resolve the Kit/Python step rate, honouring an explicit opt-in override.

    The *control* step is what Isaac Lab, the command-target writes, the
    gateway publish and /clock run on; the *physics* step is PhysX's solver
    step. omni.physx substeps natively, so the control rate may be lower than
    the physics rate by a whole factor: each control step then runs
    ``physics_hz / control_hz`` solver steps of the validated length, and
    every per-step wrapper cost is paid ``control_hz`` times a second instead
    of ``physics_hz``. Unset, the control rate equals the physics rate and
    the simulator behaves exactly as before.
    """
    if override is None or not str(override).strip():
        return float(physics_hz)
    try:
        value = float(str(override).strip())
    except (TypeError, ValueError):
        raise ValueError(
            f"TINKER_SIM_CONTROL_HZ must be a number, got {override!r}"
        ) from None
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(
            f"TINKER_SIM_CONTROL_HZ must be finite and positive, got {override!r}"
        )
    if value > float(physics_hz):
        raise ValueError(
            f"TINKER_SIM_CONTROL_HZ={value} exceeds the physics rate {physics_hz}; "
            "a control step cannot be shorter than the solver step"
        )
    if value < MINIMUM_CONTROL_HZ:
        raise ValueError(
            f"TINKER_SIM_CONTROL_HZ={value} is below the {MINIMUM_CONTROL_HZ} Hz "
            "floor; the command and clock cadence would be too coarse"
        )
    ratio = float(physics_hz) / value
    if abs(ratio - round(ratio)) > 1e-9:
        raise ValueError(
            f"TINKER_SIM_CONTROL_HZ={value} does not divide the physics rate "
            f"{physics_hz} into whole PhysX substeps"
        )
    return value


def physics_substeps(physics_hz: float, control_hz: float) -> int:
    """PhysX solver steps per control step."""
    ratio = float(physics_hz) / float(control_hz)
    substeps = int(round(ratio))
    if substeps < 1 or abs(ratio - substeps) > 1e-9:
        raise ValueError(
            f"physics_hz={physics_hz} is not a whole multiple of control_hz={control_hz}"
        )
    return substeps


# PhysX stores the articulation iteration counts in a byte.
MAXIMUM_SOLVER_ITERATIONS = 255


def resolve_solver_iterations(kind: str, override: str | None) -> int | None:
    """Resolve an opt-in articulation solver iteration count.

    The robot USD authors ``physxArticulation:solverPositionIterationCount``
    (32 for tinker2) and the velocity count (1); every PhysX step pays those
    iterations for the whole articulation, so they multiply the CPU solve
    directly. Unset, ``None`` is returned and the USD value stays in force.
    Lowering the count trades drive/contact convergence for speed, which a
    run must choose deliberately.
    """
    name = f"TINKER_SIM_SOLVER_{kind.upper()}_ITERATIONS"
    if override is None or not str(override).strip():
        return None
    text = str(override).strip()
    try:
        value = int(text)
    except ValueError:
        raise ValueError(f"{name} must be a whole number, got {override!r}") from None
    if value < 1 or value > MAXIMUM_SOLVER_ITERATIONS:
        raise ValueError(
            f"{name}={value} is outside 1..{MAXIMUM_SOLVER_ITERATIONS}"
        )
    return value
