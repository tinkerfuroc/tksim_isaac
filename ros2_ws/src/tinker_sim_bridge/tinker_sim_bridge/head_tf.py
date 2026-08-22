"""The head's two moving transforms, which nothing else publishes.

``robot_state_publisher`` emits a joint's transform only for joints it sees
in ``/joint_states``. The head pan and tilt joints are not ros2_control
joints -- ``pan_tilt_facade`` drives them over a topic -- so
``joint_state_broadcaster``, the single permitted ``/joint_states``
publisher, never names them, and RSP never publishes ``base_link ->
pan_link`` or ``pan_link -> tilt_link``.

Everything below the tilt joint is fixed, so RSP does publish it on
``/tf_static``: ``camera_link`` and every ``head_camera_*`` frame exists --
as an island floating free of ``base_link``. TF says exactly that::

    Could not find a connection between 'map' and 'camera_color_optical_frame'
    because they are not part of the same tree. Tf has two or more
    unconnected trees.

That silently discarded GPSR run16's detections: the scan found the person
(``cls_name 'person', conf 1.0``) and returned ``n_objects: 0``, because the
request asked for ``target_frame: "map"`` and no transform reached it. The
behaviour tree only ever saw ``no matches for "person"``.

A second ``/joint_states`` publisher would be the obvious fix and is not
available: ``pick_and_place`` requires exactly one, and adding another
produced 156 "controller manager observation refresh failed" errors. So the
facade, which already owns the head's state, broadcasts these two transforms
itself. It is the only publisher of them, so nothing is duplicated.

Constants are the robot description's (``tinker_full.full.urdf``), not
values chosen here.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class RevoluteJoint:
    """One URDF revolute joint: a fixed origin plus a rotation axis."""

    name: str
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]


PAN_JOINT = RevoluteJoint(
    name="pan_joint",
    parent="base_link",
    child="pan_link",
    xyz=(-0.310913, 0.00283274, 1.35846),
    rpy=(0.019631607, 0.033441848, 0.052035773),
    axis=(0.0, 0.0, -1.0),
)

TILT_JOINT = RevoluteJoint(
    name="tilt_joint",
    parent="pan_link",
    child="tilt_link",
    xyz=(0.0, 0.0, 0.135),
    rpy=(0.0, 0.0, 0.0),
    axis=(0.0, 1.0, 0.0),
)


@dataclass(frozen=True)
class HeadTransform:
    parent: str
    child: str
    xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _axis_angle_to_quaternion(axis: Sequence[float], angle: float):
    norm = math.sqrt(sum(component * component for component in axis))
    if norm <= 0.0:
        raise ValueError(f"joint axis has zero length: {tuple(axis)}")
    half = angle / 2.0
    scale = math.sin(half) / norm
    return (axis[0] * scale, axis[1] * scale, axis[2] * scale, math.cos(half))


def _multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _normalise(q):
    norm = math.sqrt(sum(component * component for component in q))
    return tuple(component / norm for component in q)


def _transform(joint: RevoluteJoint, angle: float) -> HeadTransform:
    """The joint's fixed origin composed with its rotation about the axis."""
    origin = _rpy_to_quaternion(*joint.rpy)
    rotation = _axis_angle_to_quaternion(joint.axis, angle)
    return HeadTransform(
        parent=joint.parent,
        child=joint.child,
        xyz=joint.xyz,
        quaternion_xyzw=_normalise(_multiply(origin, rotation)),
    )


def head_transforms(pan_rad: float, tilt_rad: float) -> tuple[HeadTransform, HeadTransform]:
    """The two transforms that connect the head camera to ``base_link``."""
    return (_transform(PAN_JOINT, pan_rad), _transform(TILT_JOINT, tilt_rad))
