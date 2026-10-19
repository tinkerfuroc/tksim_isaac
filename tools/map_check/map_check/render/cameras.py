"""Render cameras: specs, pinhole projection, the head camera from forward kinematics.

Frames follow USD cameras: the camera looks down -Z with +Y up. Matrices are
4x4 and act on column vectors. Poses are anything with x, y and yaw (radians),
e.g. map_check.poses.Pose, which this module does not import (it needs yaml).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Sequence

import numpy as np

Vec3 = tuple[float, float, float]

APERTURE_MM = 36.0  # horizontal aperture of every camera; vertical = 36 * h / w

HEAD_FRAME = "head_camera_color_optical_frame"
HEAD_WIDTH, HEAD_HEIGHT = 1280, 720
HEAD_HFOV_DEG = 90.0
# Optical frame (+Z forward, +Y down) to USD camera (-Z forward, +Y up): 180 deg
# about X, wxyz (0, 1, 0, 0) in simulation/sensors/hardware-parity.json.
OPTICAL_TO_USD_WXYZ = (0.0, 1.0, 0.0, 0.0)
# The sim's level-forward head aim, as scripts/gpsr-stack runs it, so pan and
# tilt mean what robot commands mean. Copied from
# simulation/tinker_sim_isaac/head_camera_aim.py (a test checks they match):
# the correction multiplies the mount rotation on the left, and the render
# origin moves this far along the camera's own view axis.
LEVEL_FORWARD_CORRECTION_WXYZ = (
    0.008896540071759,
    -0.009014044552286,
    -0.915268373827568,
    0.402645504689423,
)
LEVEL_FORWARD_VIEW_OFFSET_M = 0.03

# +1 if raising tilt_joint turns the head camera down, -1 if up. With the
# level-forward aim the camera at pan 0 looks along +x, and tilt_joint turns
# about +Y, so +tilt lowers the view (test_tilt_down_sign checks this via FK).
TILT_DOWN_SIGN = 1


@dataclass(frozen=True)
class CameraSpec:
    name: str
    eye: Vec3
    target: Vec3
    up: Vec3
    focal_mm: float
    width: int
    height: int
    # Set for the head camera, where eye/target/up only describe it.
    world_from_cam: np.ndarray | None = field(default=None, compare=False)


def _quat_mul(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def quat_matrix(wxyz: Sequence[float]) -> np.ndarray:
    """3x3 rotation of a (normalized here) w, x, y, z quaternion."""
    w, x, y, z = np.asarray(wxyz, float) / np.linalg.norm(wxyz)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def focal_from_fov(hfov_deg: float, aperture_mm: float = APERTURE_MM) -> float:
    """USD focal length (mm) for a horizontal FOV, as camera_rig.focal_from_fov."""
    return aperture_mm / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))


def look_at(eye: Sequence[float], target: Sequence[float], up: Sequence[float]) -> np.ndarray:
    """world_from_cam of a camera at eye looking at target, +Y towards up."""
    eye = np.asarray(eye, float)
    fwd = np.asarray(target, float) - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, np.asarray(up, float))
    n = np.linalg.norm(right)
    if n < 1e-9:
        raise ValueError("look_at: up is parallel to the view direction")
    right /= n
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = right, np.cross(right, fwd), -fwd, eye
    return m


def cam_to_world(cam: CameraSpec) -> np.ndarray:
    if cam.world_from_cam is not None:
        return cam.world_from_cam
    return look_at(cam.eye, cam.target, cam.up)


def project(cam: CameraSpec, xyz: Sequence[float]) -> tuple[float, float]:
    """Pixel (u right, v down) of a world point; meaningless behind the camera."""
    m = cam_to_world(cam)
    r = m[:3, :3]
    pc = r.T @ (np.asarray(xyz, float) - m[:3, 3])
    fpx = cam.focal_mm / APERTURE_MM * cam.width
    return (float(cam.width / 2 + fpx * pc[0] / -pc[2]),
            float(cam.height / 2 - fpx * pc[1] / -pc[2]))


def overview_top(bounds: Sequence[float], w: int = 2400, h: int = 1700) -> CameraSpec:
    """Straight down on the arena (xmin, xmax, ymin, ymax), +y image-up."""
    x0, x1, y0, y1 = bounds
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    focal = 35.0
    height = 1.08 * max(((x1 - x0) / 2) / (APERTURE_MM / 2 / focal),
                        ((y1 - y0) / 2) / (APERTURE_MM * h / w / 2 / focal)) + 1.7
    return CameraSpec("overview", (cx, cy, height), (cx, cy, 0.0), (0.0, 1.0, 0.0), focal, w, h)


def behind_left(pose, w: int = 1280, h: int = 800) -> CameraSpec:
    """Behind and left of the robot, looking past it at what it faces."""
    a = pose.yaw + math.pi - 0.7
    eye = (pose.x + 2.2 * math.cos(a), pose.y + 2.2 * math.sin(a), 2.0)
    target = (pose.x + 0.5 * math.cos(pose.yaw), pose.y + 0.5 * math.sin(pose.yaw), 0.5)
    return CameraSpec("behind", eye, target, (0.0, 0.0, 1.0), 20.0, w, h)


def top_down(pose, w: int = 1280, h: int = 800) -> CameraSpec:
    """Straight down on the robot, its heading image-up."""
    return CameraSpec("top", (pose.x, pose.y, 4.0), (pose.x, pose.y, 0.0),
                      (math.cos(pose.yaw), math.sin(pose.yaw), 0.0), 24.0, w, h)


def head_cam(pose, fk: Mapping[str, np.ndarray], z0: float) -> CameraSpec:
    """Tinker's head camera with the level-forward aim; fk from robot.link_transforms."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    base = np.array([[c, -s, 0.0, pose.x], [s, c, 0.0, pose.y], [0.0, 0.0, 1.0, z0],
                     [0.0, 0.0, 0.0, 1.0]])
    m = base @ fk[HEAD_FRAME]
    m[:3, :3] = m[:3, :3] @ quat_matrix(_quat_mul(LEVEL_FORWARD_CORRECTION_WXYZ, OPTICAL_TO_USD_WXYZ))
    m[:3, 3] -= LEVEL_FORWARD_VIEW_OFFSET_M * m[:3, 2]  # dolly along the view axis (-Z)
    eye = m[:3, 3]
    return CameraSpec("head", tuple(float(v) for v in eye), tuple(float(v) for v in eye - m[:3, 2]),
                      tuple(float(v) for v in m[:3, 1]), focal_from_fov(HEAD_HFOV_DEG),
                      HEAD_WIDTH, HEAD_HEIGHT, world_from_cam=m)
