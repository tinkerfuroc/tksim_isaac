"""Opt-in, sim-only correction for the head camera's aim.

**This is a workaround, not a fix, and it breaks hardware parity by design.**
Nothing here runs unless ``TINKER_SIM_HEAD_CAMERA_AIM`` is set; the default
remains exactly what the robot description says, because the simulation is
meant to match the robot rather than the reverse.

The robot description aims the head camera above the horizon everywhere it
can reach. From ``tinker_full.full.urdf`` (identically from
``tinker_real.urdf``; ``tk25_basic``'s ``pan_tilt.urdf.xacro`` is worse
still at +67.7 deg), the optical axis sits at:

===========================  ===================
pan / tilt                   optical elevation
===========================  ===================
0 / 0                        **+47.5 deg**
0 / -30 (tilt lower limit)   +17.5 deg
180 / -30 (best reachable)   **+13.6 deg**
===========================  ===================

``tilt_joint`` spans -30..+90 deg, so no reachable pose is level. The
vertical FOV is 58.7 deg, so at the tilt~0 the behaviour tree scans with,
the frame covers +18..+77 deg: wall and sky, straight over a standing
person's head. GPSR run13 drove to the kitchen table and scanned 3930 times
for a person 2 m in front of it. Forced to pan=pi, tilt=-0.5236 -- the one
corner of the workspace that sees anything -- the real, unmodified
generalist answered immediately::

    status=0  source='yolo'  cls='person'  conf=0.93  centroid=(0.08,0.94,2.30)

The simulator is faithful here: the rendered aim, measured from the horizon
in the RTX frame, matched the URDF's forward kinematics to within 2.6 deg.
So the real fix belongs in the robot description, and this module exists
only so GPSR can be exercised end to end until that lands.

The correction is a fixed rotation in the *mount prim's* frame -- the same
place a corrected ``camera_mount_joint`` would act -- so it holds across the
whole pan/tilt range rather than aiming the camera at one pose. Verified
against the URDF chain:

===========  =========  ==========  ==========
pan          tilt       elevation   azimuth
===========  =========  ==========  ==========
0            0          -0.0        -0.0
0            -30        +29.9       -1.1
0            +30        -30.0       +0.2
+45          0          -0.1        -45.0
===========  =========  ==========  ==========

Positive tilt then looks *down*, which is what ``tk26_vision``'s
``pan_tilt.yaml`` assumes with its ``home_tilt_deg: 30.0``.
"""
from __future__ import annotations

from dataclasses import replace
from typing import Iterable, Sequence

#: Set this to ``level-forward`` (or an explicit ``w,x,y,z`` quaternion) to
#: enable the correction. Unset means hardware parity.
HEAD_AIM_ENV = "TINKER_SIM_HEAD_CAMERA_AIM"

_PRESET_NAME = "level-forward"

#: Solved from the URDF chain so that, at pan=0 tilt=0, the optical axis is
#: level and points along the robot's +X. See the module docstring.
LEVEL_FORWARD_CORRECTION_WXYZ = (
    0.008896540071759,
    -0.009014044552286,
    -0.915268373827568,
    0.402645504689423,
)

HEAD_CAMERA_NAME = "head_camera"

#: Forward offset (metres, along the rendered camera's own view axis --
#: see ``camera_rig.CameraStreamSpec.view_axis_forward_offset_m``) that
#: rides along with ``LEVEL_FORWARD_CORRECTION_WXYZ``.
#:
#: Levelling the aim (above) rotates nearby housing geometry that used to
#: sit outside the upward-pointing FOV into it. The femto_bolt mesh on
#: ``head_camera_base_link`` -- the housing the camera sensor itself is
#: mounted in -- is the culprit: at pan=0, tilt=0 with the correction
#: applied, its own bounding box (chained from ``tilt_link`` down through
#: ``head_camera_color_optical_frame`` and the rtx camera's corrected
#: mount rotation) intersects the camera's frustum out to about 9.5mm of
#: forward translation (``UsdGeom.BBoxCache`` + ``Gf.Frustum.Intersects``
#: on the robot artifact, bisected: intersects at 9.4600mm, clear at
#: 9.4604mm). Rendered, that is the black rounded silhouette across the
#: bottom of the frame, worst on the right, that this offset exists to
#: clear. 3cm gives roughly 3x margin over the measured threshold; it is
#: a dolly of the render origin along the view axis, not a lens change, so
#: it has no effect on anything already a metre or more away (the arena
#: scenes GPSR scans).
LEVEL_FORWARD_VIEW_OFFSET_M = 0.03


def _normalise(quaternion: Sequence[float]) -> tuple[float, float, float, float]:
    norm = sum(float(value) * float(value) for value in quaternion) ** 0.5
    if norm <= 1e-9:
        raise ValueError(
            f"{HEAD_AIM_ENV}: quaternion has zero length: {tuple(quaternion)}"
        )
    return tuple(float(value) / norm for value in quaternion)  # type: ignore[return-value]


def _resolve_correction(
    env_name: str,
    preset_name: str,
    preset_value: tuple[float, float, float, float],
    value: str | None,
) -> tuple[float, float, float, float] | None:
    """Shared parser for the aim-override env vars; ``None`` keeps parity.

    Accepts the named preset or an explicit ``w,x,y,z`` quaternion.
    Anything else raises: a silently ignored typo here looks exactly like
    the override working, and the whole point of these flags is that a
    parity break is never accidental.
    """
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    if text.lower() == preset_name:
        return preset_value

    parts = [item.strip() for item in text.split(",")]
    if len(parts) != 4:
        raise ValueError(
            f"{env_name}: expected {preset_name!r} or four comma-separated "
            f"quaternion components (w,x,y,z), got {value!r}"
        )
    try:
        numbers = [float(item) for item in parts]
    except ValueError as error:
        raise ValueError(
            f"{env_name}: quaternion components must be numbers: {value!r}"
        ) from error
    return _normalise(numbers)


def resolve_head_aim_correction(
    value: str | None,
) -> tuple[float, float, float, float] | None:
    """Parse ``TINKER_SIM_HEAD_CAMERA_AIM``; ``None`` means leave parity alone."""
    return _resolve_correction(
        HEAD_AIM_ENV, _PRESET_NAME, LEVEL_FORWARD_CORRECTION_WXYZ, value
    )


#: Opt-in, sim-only correction for the WRIST camera's aim -- the same
#: defect class as the head camera, in the same hand-authored inertial-less
#: camera stub frames. Measured 2026-08-31 from the artifact's own
#: ``robot.urdf`` forward kinematics: the description mounts the wrist
#: camera with its optical axis exactly 90 deg away from the tool approach
#: axis. At joint zeros the gripper points straight down (-90 deg) while
#: the camera looks dead level; at the orchestrator's table-scan pose
#: (joints [0, -0.942, -0.017, 0.611, 0, 0.820, -0.017]) the TCP aims
#: forward-down at -48 deg while the camera looks UP at +42 deg -- the
#: rendered ceiling frame that ended every live grasp in referee fallback.
#: A wrist-mounted RealSense physically looks along the approach axis (the
#: real robot's grasp pipeline survives the bad frames because hand-eye
#: calibration, not URDF TF, supplies its extrinsics). Rotating the camera
#: +90 deg about the optical frame's own +X maps the render axis exactly
#: onto the TCP forward: at the scan pose, -Y of the optical frame equals
#: the TCP +Z to three decimals. As with the head: this deliberately
#: breaks hardware parity, defaults off, and the real fix belongs in the
#: robot description.
WRIST_AIM_ENV = "TINKER_SIM_WRIST_CAMERA_AIM"

_WRIST_PRESET_NAME = "tool-forward"

#: +90 deg about the mount (optical) frame's +X: (cos 45, sin 45, 0, 0).
TOOL_FORWARD_CORRECTION_WXYZ = (
    0.7071067811865476,
    0.7071067811865476,
    0.0,
    0.0,
)

WRIST_CAMERA_NAME = "wrist_camera"


def resolve_wrist_aim_correction(
    value: str | None,
) -> tuple[float, float, float, float] | None:
    """Parse ``TINKER_SIM_WRIST_CAMERA_AIM``; ``None`` means leave parity alone."""
    return _resolve_correction(
        WRIST_AIM_ENV, _WRIST_PRESET_NAME, TOOL_FORWARD_CORRECTION_WXYZ, value
    )


def _multiply(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def apply_head_aim_correction(
    specs: Iterable, correction: Sequence[float], *, camera_name: str = HEAD_CAMERA_NAME
) -> tuple:
    """Return *specs* with *correction* composed onto the head camera's mount.

    The rendered orientation is ``R_mount_prim . R_spec``, so the correction
    multiplies on the **left** of the spec's rotation: that places it in the
    mount's frame, where a corrected ``camera_mount_joint`` would act. On the
    right it would rotate the camera in its own optical frame instead, and
    the aim would only be right at the pose it was solved for.

    When *correction* is exactly ``LEVEL_FORWARD_CORRECTION_WXYZ``, this also
    sets ``view_axis_forward_offset_m`` to ``LEVEL_FORWARD_VIEW_OFFSET_M`` --
    see that constant's docstring for why the level-forward aim needs it. An
    explicit, non-preset correction gets no offset: the measured clearance
    is specific to the preset's geometry, not a general property of "some
    rotation was applied".
    """
    specs = tuple(specs)
    if not any(spec.name == camera_name for spec in specs):
        raise ValueError(f"no {camera_name!r} among the camera specs to correct")
    correction = tuple(correction)
    forward_offset_m = (
        LEVEL_FORWARD_VIEW_OFFSET_M
        if correction == LEVEL_FORWARD_CORRECTION_WXYZ
        else 0.0
    )
    corrected = []
    for spec in specs:
        if spec.name != camera_name:
            corrected.append(spec)
            continue
        corrected.append(
            replace(
                spec,
                mount_rotation_wxyz=_normalise(
                    _multiply(correction, spec.mount_rotation_wxyz)
                ),
                view_axis_forward_offset_m=forward_offset_m,
            )
        )
    return tuple(corrected)
