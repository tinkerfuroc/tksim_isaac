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

import math
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
    presets: dict[str, tuple[float, float, float, float]],
    value: str | None,
) -> tuple[float, float, float, float] | None:
    """Shared parser for the aim-override env vars; ``None`` keeps parity.

    Accepts one of the named *presets* or an explicit ``w,x,y,z``
    quaternion. Anything else raises: a silently ignored typo here looks
    exactly like the override working, and the whole point of these flags
    is that a parity break is never accidental.
    """
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    preset = presets.get(text.lower())
    if preset is not None:
        return preset

    parts = [item.strip() for item in text.split(",")]
    if len(parts) != 4:
        names = " / ".join(repr(name) for name in presets)
        raise ValueError(
            f"{env_name}: expected {names} or four comma-separated "
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
        HEAD_AIM_ENV, {_PRESET_NAME: LEVEL_FORWARD_CORRECTION_WXYZ}, value
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
#: A wrist-mounted RealSense physically looks toward the tool (the real
#: robot's grasp pipeline survives the bad frames because hand-eye
#: calibration, not URDF TF, supplies its extrinsics). The correction
#: rotates the camera about the optical frame's own +X toward the tool
#: axis -- but NOT the full 90 deg onto it: the description places the
#: camera almost co-axial above the gripper, so a perfectly tool-aligned
#: view stares straight into the hand (measured live 2026-08-31: at
#: 90/80 deg the frame is all gripper, median depth 76 mm; the depth
#: near-clip makes it read as an all-black close-up). A tilt sweep at the
#: scan pose picked 60 deg: the scene fills the frame with the gripper
#: riding the bottom edge (real wrist-camera framing), and the view
#: centres a desk-height surface about a metre ahead. As with the head:
#: this deliberately breaks hardware parity, defaults off, and the real
#: fix belongs in the robot description.
WRIST_AIM_ENV = "TINKER_SIM_WRIST_CAMERA_AIM"

_WRIST_PRESET_NAME = "tool-forward"

#: +60 deg about the mount (optical) frame's +X: (cos 30, sin 30, 0, 0).
#: 30 deg shy of the tool axis, clearing the co-axial gripper.
TOOL_FORWARD_CORRECTION_WXYZ = (
    0.8660254037844387,
    0.5,
    0.0,
    0.0,
)

WRIST_CAMERA_NAME = "wrist_camera"

#: ``cam-stand``: render the wrist camera from where the real one IS.
#:
#: ``tool-forward`` fixed the aim but rotates the camera IN PLACE, and the
#: place is wrong. The sim description (tk26_sim ``tinker_full.urdf.xacro``)
#: layers Intel's ``sensor_d435`` macro onto ``link_eef`` with a placeholder
#: ``<origin xyz="0 0 0" rpy="0 0 0"/>``, so the colour optical frame sits at
#: link_eef + (0.0106, 0.0325, 0.0125): 12.5 mm above the flange, 33 mm off
#: the tool axis -- on the surface of ``xarm_gripper_base_link``'s housing,
#: looking 90 deg off the tool. Tilt that view 60 deg toward the tool and
#: the housing's far wall sits just past the rig's 0.05 m near clip at the
#: top of the frustum: the black band across the top of every wrist frame
#: (grasp bench, 2026-09-04: 9.4% of the 848x480 image, depth 50-60 mm,
#: worst on the right). An offline USD frustum model of ``robot.usd``
#: reproduces that band to within a row and names the gripper base link as
#: the only occluder (developer log, 2026-09-04).
#:
#: The real robot (``tinker_real.urdf`` / xArm's
#: ``realsense_d435i.urdf.xacro``, "vendor factory-nominal" extrinsics)
#: mounts ``xarm_camera_link`` on the D435 cam-stand bracket at
#: ``CAM_STAND_CAMERA_LINK_XYZ`` / ``_RPY`` below: 67 mm out along +X_eef,
#: 24 mm up, looking straight along the tool axis with image-up pointing
#: radially outward. From there the same frustum model shows zero housing
#: pixels -- only the fingertips at the bottom edge, which is what a wrist
#: camera sees. ``cam-stand`` places the render origin at exactly that
#: pose: ``CAM_STAND_MOUNT_OFFSET_XYZ`` is the bracket translation and
#: ``CAM_STAND_CORRECTION_WXYZ`` the rotation, both expressed in the
#: artifact's (placeholder) colour optical frame, i.e.
#: ``inv(T_artifact_optical) . T_vendor_optical`` (tests re-derive both from
#: the two URDF chains). Still opt-in and sim-only; the durable fix is the
#: description's origin, after which this preset becomes a no-op.
#:
#: TF must move with the render: ``cam_stand_robot_description`` rewrites
#: the artifact URDF's ``xarm_camera_joint`` so ``robot_state_publisher``
#: puts ``xarm_camera_color_optical_frame`` at the same vendor pose the
#: camera renders from (``tinker_sim_deploy.runtime.sim_robot_description``
#: applies it in every bridge launch under the same env value).
_CAM_STAND_PRESET_NAME = "cam-stand"

#: xArm cam-stand bracket: ``link_eef -> xarm_camera_link`` (URDF origin).
CAM_STAND_CAMERA_LINK_XYZ = (0.06746, -0.0175, 0.0237)
CAM_STAND_CAMERA_LINK_RPY = (3.141592653589793, -1.5707963267948966, 0.0)

#: Intel ``sensor_d435`` macro: ``bottom_screw_frame -> camera_link`` (URDF
#: origin, no rotation) -- the artifact's chain above the placeholder joint.
INTEL_D435_CAMERA_LINK_XYZ = (0.0106, 0.0175, 0.0125)

#: Bracket translation in the artifact's colour optical frame (x right,
#: y down, z forward): R_opt^T . (t_vendor - t_artifact) with
#: t_vendor = (0.06746, -0.0325, 0.0237), t_artifact = (0.0106, 0.0325,
#: 0.0125) in link_eef, and the optical axes x = -Y_eef, y = -Z_eef,
#: z = +X_eef.
CAM_STAND_MOUNT_OFFSET_XYZ = (0.065, -0.0112, 0.05686)

#: Bracket rotation in the same frame: 180 deg about (0, -1, 1)/sqrt(2) --
#: maps the optical view axis (+z) onto -y_opt = +Z_eef (the tool axis)
#: and image-up (-y) onto +z_opt = +X_eef (radially outward).
CAM_STAND_CORRECTION_WXYZ = (
    0.0,
    0.0,
    -0.7071067811865476,
    0.7071067811865476,
)

_WRIST_PRESETS = {
    _WRIST_PRESET_NAME: TOOL_FORWARD_CORRECTION_WXYZ,
    _CAM_STAND_PRESET_NAME: CAM_STAND_CORRECTION_WXYZ,
}

#: Mount-frame translations that ride along with a preset rotation (the
#: head's ``view_axis_forward_offset_m`` is the same idea on the other side
#: of the orient op). Keyed on the exact preset quaternion, like the head
#: dolly: an explicit non-preset correction gets no offset.
_PRESET_MOUNT_OFFSETS = {
    CAM_STAND_CORRECTION_WXYZ: CAM_STAND_MOUNT_OFFSET_XYZ,
}


def resolve_wrist_aim_correction(
    value: str | None,
) -> tuple[float, float, float, float] | None:
    """Parse ``TINKER_SIM_WRIST_CAMERA_AIM``; ``None`` means leave parity alone."""
    return _resolve_correction(WRIST_AIM_ENV, _WRIST_PRESETS, value)


def _rpy_matrix(
    roll: float, pitch: float, yaw: float
) -> tuple[tuple[float, float, float], ...]:
    """URDF ``rpy`` -> rotation matrix (Rz(yaw) . Ry(pitch) . Rx(roll))."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def cam_stand_camera_joint_origin() -> tuple[
    tuple[float, float, float], tuple[float, float, float]
]:
    """``(xyz, rpy)`` for the artifact's ``xarm_camera_joint`` under cam-stand.

    That joint is ``link_eef -> xarm_camera_bottom_screw_frame`` (the
    placeholder identity). Re-authoring it as
    ``T_vendor_camera_link . inv(T_intel_bottom_screw_to_camera_link)``
    leaves the Intel macro chain above it untouched and lands
    ``xarm_camera_link`` -- hence the colour optical frame -- exactly on the
    vendor cam-stand pose.
    """
    rotation = _rpy_matrix(*CAM_STAND_CAMERA_LINK_RPY)
    back = tuple(-v for v in INTEL_D435_CAMERA_LINK_XYZ)
    xyz = tuple(
        CAM_STAND_CAMERA_LINK_XYZ[i] + sum(rotation[i][k] * back[k] for k in range(3))
        for i in range(3)
    )
    return xyz, CAM_STAND_CAMERA_LINK_RPY  # type: ignore[return-value]


CAM_STAND_CAMERA_JOINT = "xarm_camera_joint"


def cam_stand_robot_description(urdf: str) -> str:
    """Return *urdf* with ``xarm_camera_joint``'s origin moved to the cam-stand.

    For ``robot_state_publisher``: with ``cam-stand`` active the rendered
    wrist camera sits on the vendor bracket, so the TF the images advertise
    (``xarm_camera_color_optical_frame``) has to say the same or every
    deprojection lands 6 cm off and 90 deg rotated. Raises if the joint is
    missing: a description without the placeholder joint is not the
    artifact this preset was measured against, and silently publishing the
    old pose is exactly the mismatch this exists to prevent.
    """
    import xml.etree.ElementTree as ET

    root = ET.fromstring(urdf)
    joints = [
        joint for joint in root.findall("joint")
        if joint.get("name") == CAM_STAND_CAMERA_JOINT
    ]
    if len(joints) != 1:
        raise ValueError(
            f"cam-stand: expected exactly one {CAM_STAND_CAMERA_JOINT!r} in the "
            f"robot description, found {len(joints)}"
        )
    origin = joints[0].find("origin")
    if origin is None:
        origin = ET.SubElement(joints[0], "origin")
    xyz, rpy = cam_stand_camera_joint_origin()
    origin.set("xyz", " ".join(repr(float(v)) for v in xyz))
    origin.set("rpy", " ".join(repr(float(v)) for v in rpy))
    return ET.tostring(root, encoding="unicode")


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
    see that constant's docstring for why the level-forward aim needs it.
    Likewise ``CAM_STAND_CORRECTION_WXYZ`` carries
    ``CAM_STAND_MOUNT_OFFSET_XYZ`` into ``mount_frame_offset_xyz``. An
    explicit, non-preset correction gets no offset of either kind: the
    measured geometry is specific to the preset, not a general property of
    "some rotation was applied".
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
    mount_offset_xyz = _PRESET_MOUNT_OFFSETS.get(correction, (0.0, 0.0, 0.0))
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
                mount_frame_offset_xyz=mount_offset_xyz,
            )
        )
    return tuple(corrected)
