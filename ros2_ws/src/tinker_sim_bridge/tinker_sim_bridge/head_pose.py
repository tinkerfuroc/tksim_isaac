"""Resting pose of the simulated pan-tilt head.

`camera_mount_joint` fixes the head camera to `tilt_link` with a -45.53 deg
pitch. That pitch is real -- it is on the hardware too -- and the pan-tilt
mechanism is what compensates for it, so a head left at tilt 0 points its
optical axis 45 deg down into the robot's own deck rather than at the room.

On hardware the pan-tilt controller drives its own startup pose
(`initial_pan_deg` / `initial_tilt_deg` in tk26_vision's pan_tilt.yaml). This
module gives the simulated facade the same knobs, defaulting to the tilt that
cancels the mount pitch exactly.
"""

from __future__ import annotations

import math

# camera_mount_joint rpy pitch, radians (see the simulator's full URDF).
HEAD_CAMERA_MOUNT_PITCH_RAD = -0.79457

# tilt_joint limits from the same URDF: -30 deg .. +90 deg about +Y.
TILT_JOINT_LOWER_RAD = -0.523598775598299
TILT_JOINT_UPPER_RAD = 1.570796326794897


def level_tilt_rad() -> float:
    """The tilt that puts the optical axis level with the floor."""
    return -HEAD_CAMERA_MOUNT_PITCH_RAD


def _checked(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return value


def resolve_initial_head_pose(
    pan_deg: float | None, tilt_deg: float | None
) -> tuple[float, float]:
    """Return the startup ``(pan_rad, tilt_rad)`` for the head.

    ``None`` means "use the default": pan straight ahead, and the tilt that
    cancels the mount pitch. An out-of-range tilt raises rather than clamping --
    clamping would silently leave the camera pointing somewhere the caller did
    not ask for, which is exactly the failure this module exists to prevent.
    """
    pan = 0.0 if pan_deg is None else math.radians(_checked(pan_deg, "initial_pan_deg"))
    if tilt_deg is None:
        tilt = level_tilt_rad()
    else:
        tilt = math.radians(_checked(tilt_deg, "initial_tilt_deg"))
    _checked(pan, "initial_pan_deg")
    if not (TILT_JOINT_LOWER_RAD <= tilt <= TILT_JOINT_UPPER_RAD):
        raise ValueError(
            f"initial_tilt_deg={math.degrees(tilt):.3f} is outside the tilt_joint "
            f"limits [{math.degrees(TILT_JOINT_LOWER_RAD):.1f}, "
            f"{math.degrees(TILT_JOINT_UPPER_RAD):.1f}] deg"
        )
    return pan, tilt
