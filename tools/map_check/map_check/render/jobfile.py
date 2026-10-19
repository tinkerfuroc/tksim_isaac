"""The render job file the server writes for the Isaac script, and the script's progress lines.

The Isaac script prints one JSON object per progress line on stdout:
{"event": "view", "key", "view", "file"}, {"event": "done"} or
{"event": "error", "message"}; every other line is Isaac's own output.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

VIEWS = ("behind", "top", "head")  # close-ups per pose
OVERVIEW = "overview"  # view name of the arena overview in progress lines
EVENTS = ("view", "done", "error")


@dataclass
class JobPose:
    key: str
    name: str
    x: float
    y: float
    yaw: float  # radians
    views: list[str] = field(default_factory=list)
    pan_deg: float = 0.0
    tilt_deg: float = 0.0


@dataclass
class JobFile:
    job_id: str
    map: str
    arena_usd: str
    robot_usd: str
    robot_urdf: str
    z0: float
    gpu: int
    out_dir: str
    arm_deg: list[float]
    poses: list[JobPose]
    overview: bool

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Any) -> "JobFile":
        if not isinstance(d, dict):
            raise ValueError("job file must be a JSON object")
        try:
            poses = [_pose(p) for p in d["poses"]]
            return cls(job_id=str(d["job_id"]), map=str(d["map"]), arena_usd=str(d["arena_usd"]),
                       robot_usd=str(d["robot_usd"]), robot_urdf=str(d["robot_urdf"]),
                       z0=float(d["z0"]), gpu=int(d["gpu"]), out_dir=str(d["out_dir"]),
                       arm_deg=[float(a) for a in d["arm_deg"]], poses=poses,
                       overview=bool(d["overview"]))
        except KeyError as e:
            raise ValueError(f"job file: missing {e.args[0]}") from e
        except (TypeError, ValueError) as e:
            raise ValueError(f"job file: {e}") from e

    def write(self, path: Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2) + "\n")

    @classmethod
    def read(cls, path: Path) -> "JobFile":
        try:
            doc = json.loads(Path(path).read_text())
        except (OSError, ValueError) as e:
            raise ValueError(f"{path}: cannot read: {e}") from e
        return cls.from_dict(doc)


def _pose(p: Any) -> JobPose:
    views = [str(v) for v in p["views"]]
    bad = [v for v in views if v not in VIEWS]
    if bad:
        raise ValueError(f"pose {p.get('key')!r}: unknown views {bad}")
    return JobPose(key=str(p["key"]), name=str(p["name"]), x=float(p["x"]), y=float(p["y"]),
                   yaw=float(p["yaw"]), views=views, pan_deg=float(p.get("pan_deg", 0.0)),
                   tilt_deg=float(p.get("tilt_deg", 0.0)))


def view_event(key: str, view: str, file: str) -> str:
    return json.dumps({"event": "view", "key": key, "view": view, "file": file})


def done_event() -> str:
    return json.dumps({"event": "done"})


def error_event(message: str) -> str:
    return json.dumps({"event": "error", "message": message})


def parse_event(line: str) -> dict | None:
    """The progress event on a stdout line, or None for any other output."""
    line = line.strip()
    if not line.startswith("{"):
        return None
    try:
        doc = json.loads(line)
    except ValueError:
        return None
    if not isinstance(doc, dict) or doc.get("event") not in EVENTS:
        return None
    return doc
