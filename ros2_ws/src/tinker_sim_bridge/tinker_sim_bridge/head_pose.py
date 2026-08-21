"""Startup pose of the simulated pan-tilt head.

The hardware pan-tilt controller drives its own startup pose from
``initial_pan_deg`` / ``initial_tilt_deg`` (tk26_vision
``pan_tilt/config/pan_tilt.yaml``), both **0.0**. At tilt 0 the head camera
looks approximately level: the -45.5 deg pitch in ``camera_mount_joint`` is
already accounted for downstream of the joint, so it is *not* something the
tilt joint has to cancel.

The simulated facade stands in for that controller, so it takes the same two
parameters and the same 0/0 default. Holding the pose matters because nothing
else in a simulation bring-up commands the head, and an uncommanded joint is
free to drift.
"""

from __future__ import annotations

import math

# tilt_joint limits from the simulator's full URDF: -30 deg .. +90 deg about +Y.
TILT_JOINT_LOWER_RAD = -0.523598775598299
TILT_JOINT_UPPER_RAD = 1.570796326794897

# Hardware's pan_tilt.yaml startup pose.
DEFAULT_PAN_DEG = 0.0
DEFAULT_TILT_DEG = 0.0


def _checked(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return value


def resolve_initial_head_pose(
    pan_deg: float | None, tilt_deg: float | None
) -> tuple[float, float]:
    """Return the startup ``(pan_rad, tilt_rad)`` for the head.

    ``None`` means "use the hardware default", which is 0/0 -- roughly level.
    An out-of-range tilt raises rather than clamping: clamping would silently
    leave the camera pointing somewhere the caller did not ask for.
    """
    pan = math.radians(
        _checked(DEFAULT_PAN_DEG if pan_deg is None else pan_deg, "initial_pan_deg")
    )
    tilt = math.radians(
        _checked(DEFAULT_TILT_DEG if tilt_deg is None else tilt_deg, "initial_tilt_deg")
    )
    if not (TILT_JOINT_LOWER_RAD <= tilt <= TILT_JOINT_UPPER_RAD):
        raise ValueError(
            f"initial_tilt_deg={math.degrees(tilt):.3f} is outside the tilt_joint "
            f"limits [{math.degrees(TILT_JOINT_LOWER_RAD):.1f}, "
            f"{math.degrees(TILT_JOINT_UPPER_RAD):.1f}] deg"
        )
    return pan, tilt
