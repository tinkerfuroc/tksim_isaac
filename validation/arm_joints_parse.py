"""Pure helper for gripper_close_probe.py's ``--arm-joints`` diagnostic mode.

Split out of the probe script -- which imports ``isaacsim`` at module load, so
it cannot be imported by a plain unit test -- so the degrees -> radians
parsing can be exercised with no Isaac dependency.
"""
from __future__ import annotations

import math

# xArm7 joint order, matching gripper_close_probe.py's ARM tuple.
ARM_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))


def parse_arm_joints_deg(text: str) -> dict[str, float]:
    """``"d1,d2,d3,d4,d5,d6,d7"`` (xArm7 joint1..joint7, DEGREES) -> a
    ``{joint_name: radians}`` dict in ``ARM_JOINT_NAMES`` order.

    Raises ``ValueError`` (with a message naming the offending input) on a
    wrong item count or a non-numeric entry, so a caller (argparse-style CLI
    validation) can surface it as a normal usage error.
    """
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != len(ARM_JOINT_NAMES):
        raise ValueError(
            f"--arm-joints needs {len(ARM_JOINT_NAMES)} comma-separated degree "
            f"values (joint1..joint{len(ARM_JOINT_NAMES)}), got {len(parts)}: {text!r}"
        )
    try:
        degrees = [float(p) for p in parts]
    except ValueError as error:
        raise ValueError(f"--arm-joints value is not a number: {text!r} ({error})") from None
    return {name: math.radians(d) for name, d in zip(ARM_JOINT_NAMES, degrees)}
