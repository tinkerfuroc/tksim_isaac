"""Poses: read Map Studio files, parse pasted PoseStamped code, export, find clashes.

Pasted text is only matched with regexes, never evaluated.
"""
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

import yaml

from .names import check_map_name, check_pose_name

POSES_SUFFIX = ".poses.yaml"

# Same text as Map Studio's poses.CODE_IMPORTS.
CODE_IMPORTS = ("from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion\n"
                "from std_msgs.msg import Header\n"
                "import rclpy\n")
FRAME_ID = "map"


@dataclass(frozen=True)
class Pose:
    name: str
    x: float
    y: float
    yaw: float  # radians, in (-pi, pi]


@dataclass(frozen=True)
class PasteError:
    index: int  # 1-based block number
    text: str  # first 60 chars of the block
    reason: str


def normalize_yaw(yaw):
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return math.pi if yaw == -math.pi else yaw


def key(name: str) -> str:
    return (name[4:] if name.startswith("POS_") else name).lower()


def read_map_studio_poses(maps_dir: Path, map_name: str) -> list[Pose]:
    path = Path(maps_dir) / f"{check_map_name(map_name)}{POSES_SUFFIX}"
    if not path.exists():
        return []
    try:
        doc = yaml.safe_load(path.read_text())
    except (OSError, UnicodeError, yaml.YAMLError) as e:
        raise ValueError(f"{path.name}: cannot read: {e}") from e
    if doc is None:
        return []
    raw = doc.get("poses") if isinstance(doc, dict) else None
    if raw is None:
        return []
    if not isinstance(raw, dict):
        raise ValueError(f"{path.name}: poses must be a mapping")
    out = []
    for name, v in raw.items():
        try:
            check_pose_name(str(name))
            out.append(Pose(str(name), float(v["x"]), float(v["y"]), normalize_yaw(float(v["yaw"]))))
        except (ValueError, TypeError, KeyError) as e:
            raise ValueError(f"{path.name}: pose {name!r}: {e}") from e
    return out


_START = re.compile(r"\bPOS_[A-Za-z0-9_]+\s*=\s*PoseStamped\s*\(")
_NAME = re.compile(r"POS_[A-Za-z0-9_]+")
_NUM = r"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)"
_POINT = re.compile(r"Point\s*\(\s*x\s*=\s*" + _NUM + r"\s*,\s*y\s*=\s*" + _NUM)
_QUAT = re.compile(r"Quaternion\s*\(([^)]*)\)")


def _quat_field(args, field):
    m = re.search(r"\b" + field + r"\s*=\s*" + _NUM, args)
    return float(m.group(1)) if m else None


def _parse_block(text):
    name = _NAME.match(text).group(0)
    try:
        check_pose_name(name)
    except ValueError as e:
        raise ValueError(str(e)) from e
    pt = _POINT.search(text)
    if not pt:
        raise ValueError("no Point(x=..., y=...)")
    q = _QUAT.search(text)
    if not q:
        raise ValueError("no Quaternion(...)")
    qz, qw = _quat_field(q.group(1), "z"), _quat_field(q.group(1), "w")
    if qz is None or qw is None:
        raise ValueError("Quaternion needs z= and w=")
    if qz == 0 and qw == 0:
        raise ValueError("Quaternion z and w are both 0")
    return Pose(name, float(pt.group(1)), float(pt.group(2)),
                normalize_yaw(2 * math.atan2(qz, qw)))


def parse_pasted(text: str) -> tuple[list[Pose], list[PasteError]]:
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    starts = [m.start() for m in _START.finditer(text)]
    poses, errors = [], []
    for i, s in enumerate(starts):
        chunk = text[s:starts[i + 1] if i + 1 < len(starts) else len(text)]
        try:
            poses.append(_parse_block(chunk))
        except ValueError as e:
            errors.append(PasteError(i + 1, chunk[:60], str(e)))
    return poses, errors


def clashes(saved: list[Pose], pasted: list[Pose]) -> dict[str, float]:
    by_key = {key(p.name): p for p in saved}
    return {p.name: math.hypot(p.x - s.x, p.y - s.y)
            for p in pasted if (s := by_key.get(key(p.name)))}


def export_code(poses) -> str:
    """Byte-identical to Map Studio's CODE_IMPORTS + '\\n' + '\\n'.join(pose_code(...))."""
    blocks = []
    for p in poses:
        qz, qw = math.sin(p.yaw / 2.0), math.cos(p.yaw / 2.0)
        var = f"POS_{key(p.name).upper()}"
        pad = " " * len(f"{var} = PoseStamped(")
        blocks.append(
            f"{var} = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), "
            f"frame_id='{FRAME_ID}'),\n"
            f"{pad}pose=Pose(position=Point(x={p.x!r}, y={p.y!r}, z=0.0),\n"
            f"{pad}          orientation=Quaternion(x=0.0, y=0.0, z={qz!r}, w={qw!r})))\n")
    return CODE_IMPORTS + "\n" + "\n".join(blocks)


def yaml_name(name: str) -> str:
    """The pose's name in an exported .poses.yaml.

    Map Studio keeps saved names exactly as written ("TestSpawn" stays "TestSpawn"),
    so only pasted code names (POS_FOO) are converted, to "foo".
    """
    return name[4:].lower() if name.startswith("POS_") else name


def _yaw_text(yaw):
    return f"{min(max(round(yaw, 4), -3.1415), 3.1415):.4f}"


def export_yaml(map_name: str, poses) -> str:
    lines = [f"map: {json.dumps(map_name)}", "frame_id: map"]
    if poses:
        lines.append("poses:")
        lines += [f"  {json.dumps(yaml_name(p.name))}: {{x: {p.x:.4f}, y: {p.y:.4f}, "
                  f"yaw: {_yaw_text(p.yaw)}}}" for p in poses]
    else:
        lines.append("poses: {}")
    return "\n".join(lines) + "\n"
