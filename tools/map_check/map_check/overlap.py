"""Overlap check: robot surface points at a pose against the arena boxes."""
from __future__ import annotations

import functools
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Sequence

import numpy as np

from map_check import robot
from map_check.arena_build import Box
from map_check.poses import Pose

_CHUNK = 2_000_000  # points x boxes evaluated per numpy batch


@dataclass(frozen=True)
class Flag:
    kind: Literal["overlap", "near", "outside"]
    part: str
    target: str
    distance_m: float  # negative = penetration depth
    text: str


def part_name(link: str) -> str:
    """Friendly name of the robot part a URDF link belongs to."""
    if link in ("pan_link", "tilt_link", "camera_link") or link.startswith("head_camera"):
        return "head"
    if re.fullmatch(r"link\d+", link) or link == "link_base":
        return f"arm ({link})"
    if (link.startswith("xarm_") or link in ("link_eef", "link_tcp")
            or any(s in link for s in ("gripper", "finger", "knuckle"))):
        return "gripper"
    if (link in ("base_link", "ballast", "livox_frame")
            or any(s in link for s in ("wheel", "swivel", "caster"))):
        return "base"
    return link


def arena_bounds(boxes: Sequence[Box]) -> tuple[float, float, float, float]:
    """(xmin, xmax, ymin, ymax) of the union of the boxes."""
    if not boxes:
        raise ValueError("no boxes")
    return (min(b.x0 for b in boxes), max(b.x1 for b in boxes),
            min(b.y0 for b in boxes), max(b.y1 for b in boxes))


def _target(b: Box) -> str:
    return "wall" if b.kind == "wall" else f"shape {b.shape_id}"


def _cm(d: float) -> int:
    return max(1, round(abs(d) * 100))


def _signed_distance(p: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Smallest signed distance of points p (N x 3) to each box (K x 3 corners) -> (K,)."""
    out = np.empty(len(lo))
    step = max(1, _CHUNK // max(1, len(p)))
    for s in range(0, len(lo), step):
        q = np.maximum(lo[None, s:s + step] - p[:, None], p[:, None] - hi[None, s:s + step])
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=2)
        inside = np.minimum(q.max(axis=2), 0.0)
        out[s:s + step] = (outside + inside).min(axis=0)
    return out


def check_pose(pose: Pose, boxes: Sequence[Box], arena_bounds: tuple, robot_points: dict,
               fk: dict, z0: float, margin: float = 0.02) -> list[Flag]:
    """Flags for the robot at pose; fk holds the link transforms for its arm/pan/tilt."""
    xmin, xmax, ymin, ymax = arena_bounds
    if not (xmin <= pose.x <= xmax and ymin <= pose.y <= ymax):
        off = math.hypot(max(xmin - pose.x, 0.0, pose.x - xmax),
                         max(ymin - pose.y, 0.0, pose.y - ymax))
        return [Flag("outside", "base", "arena", off, "outside the arena")]
    if not boxes:
        return []
    lo = np.array([(b.x0, b.y0, b.z0) for b in boxes], float)
    hi = np.array([(b.x1, b.y1, b.z1) for b in boxes], float)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    world = np.array([[c, -s, 0.0, pose.x], [s, c, 0.0, pose.y],
                      [0.0, 0.0, 1.0, z0], [0.0, 0.0, 0.0, 1.0]])
    worst = {}  # part -> (distance, target)
    for link, pts in robot_points.items():
        if not len(pts):
            continue
        t = world @ fk[link]
        # Cull with the link's world AABB (from its link-frame AABB corners).
        a, b = pts.min(axis=0), pts.max(axis=0)
        corners = np.array([[x, y, z] for x in (a[0], b[0]) for y in (a[1], b[1])
                            for z in (a[2], b[2])]) @ t[:3, :3].T + t[:3, 3]
        wlo, whi = corners.min(axis=0) - margin, corners.max(axis=0) + margin
        near = np.flatnonzero(((lo < whi) & (hi > wlo)).all(axis=1))
        if not len(near):
            continue
        d = _signed_distance(pts @ t[:3, :3].T + t[:3, 3], lo[near], hi[near])
        i = int(d.argmin())
        if d[i] >= margin:
            continue
        part = part_name(link)
        if part not in worst or d[i] < worst[part][0]:
            worst[part] = (float(d[i]), _target(boxes[near[i]]))
    flags = []
    for part, (d, target) in worst.items():
        if d < 0:
            flags.append(Flag("overlap", part, target, d, f"{part} overlaps {target} by {_cm(d)} cm"))
        else:
            flags.append(Flag("near", part, target, d,
                              f"{part} {_cm(d)} cm from {target} (under {_cm(margin)} cm)"))
    return sorted(flags, key=lambda f: (f.distance_m, f.part))


class RobotModel:
    """Robot points, grounded height and link transforms, cached for repeated checks."""

    def __init__(self, urdf: Path, usd: Path, cache_dir: Path, repo_root: Path | None = None,
                 margin: float = 0.02):
        self.urdf, self.usd, self.cache_dir = Path(urdf), Path(usd), Path(cache_dir)
        self.repo_root, self.margin = repo_root, margin
        self._fk = functools.lru_cache(maxsize=64)(self._link_transforms)

    @functools.cached_property
    def points(self) -> dict[str, np.ndarray]:
        return robot.link_points(self.usd, self.cache_dir)

    @functools.cached_property
    def z0(self) -> float:
        return robot.grounded_z(self.urdf, self.repo_root)

    def _link_transforms(self, arm_deg, pan_deg, tilt_deg):
        return robot.link_transforms(self.urdf, robot.joint_values(arm_deg, pan_deg, tilt_deg))

    def fk(self, arm_deg: Sequence[float], pan_deg: float, tilt_deg: float) -> dict[str, np.ndarray]:
        return self._fk(tuple(float(a) for a in arm_deg), float(pan_deg), float(tilt_deg))

    def flags(self, pose: Pose, arm_deg: Sequence[float], pan_deg: float, tilt_deg: float,
              boxes: Sequence[Box], bounds: tuple) -> list[Flag]:
        return check_pose(pose, boxes, bounds, self.points, self.fk(arm_deg, pan_deg, tilt_deg),
                          self.z0, self.margin)
