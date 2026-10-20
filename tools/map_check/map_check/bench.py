"""Shared per-map bench state: which poses are checked, with which views, plus the last render.

One JSON file per map under <state_dir>/bench/. Every change bumps `version`
so the UI can tell when another tab changed something.
"""
import copy
import json
import math
import os
import tempfile
import threading
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .names import check_map_name
from .overlap import Flag
from .poses import PasteError, Pose, clashes, key, parse_pasted, read_map_studio_poses

DEFAULT_ARM_POSE = "arm_pos_navigating"
JOB_STATUSES = ("queued", "running", "done", "failed", "interrupted", "timeout")
PAN_RANGE = (-180.0, 180.0)
TILT_RANGE = (-30.0, 90.0)


@dataclass
class RowState:
    included: bool = False
    behind: bool = True
    top: bool = False
    head: bool = False
    pan_deg: float = 0
    tilt_deg: float = 20


@dataclass
class JobStatus:
    id: str
    map: str
    status: str
    queued_at: float
    started_at: float | None = None
    finished_at: float | None = None
    done_views: int = 0
    total_views: int = 0
    position: int | None = None
    message: str = ""

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "JobStatus":
        return cls(**d)


@dataclass
class RenderRecord:
    job_id: str
    finished_at: float
    arm_pose: str
    # each item: {key, name, flags: [Flag], views: [{view, url, caption}]}
    items: list[dict] = field(default_factory=list)
    overview_url: str | None = None

    def to_dict(self) -> dict:
        d = asdict(self)  # Flag dataclasses become dicts
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "RenderRecord":
        items = []
        for it in d.get("items", []):
            it = dict(it)
            it["flags"] = [f if isinstance(f, Flag) else Flag(**f) for f in it.get("flags", [])]
            items.append(it)
        return cls(job_id=d["job_id"], finished_at=d["finished_at"], arm_pose=d["arm_pose"],
                   items=items, overview_url=d.get("overview_url"))


@dataclass
class BenchState:
    map: str
    version: int = 0
    arm_pose: str = DEFAULT_ARM_POSE
    pasted: list[Pose] = field(default_factory=list)
    removed_saved: list[str] = field(default_factory=list)
    rows: dict[str, RowState] = field(default_factory=dict)
    last_render: RenderRecord | None = None
    job: JobStatus | None = None

    def to_dict(self) -> dict:
        return {
            "map": self.map, "version": self.version, "arm_pose": self.arm_pose,
            "pasted": [asdict(p) for p in self.pasted],
            "removed_saved": list(self.removed_saved),
            "rows": {k: asdict(r) for k, r in self.rows.items()},
            "last_render": self.last_render.to_dict() if self.last_render else None,
            "job": self.job.to_dict() if self.job else None,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "BenchState":
        return cls(
            map=d["map"], version=int(d.get("version", 0)),
            arm_pose=d.get("arm_pose", DEFAULT_ARM_POSE),
            pasted=[Pose(**p) for p in d.get("pasted", [])],
            removed_saved=list(d.get("removed_saved", [])),
            rows={k: RowState(**r) for k, r in d.get("rows", {}).items()},
            last_render=RenderRecord.from_dict(d["last_render"]) if d.get("last_render") else None,
            job=JobStatus.from_dict(d["job"]) if d.get("job") else None,
        )


@dataclass
class BenchRow:
    key: str
    group: str  # "saved" | "pasted"
    pose: Pose
    state: RowState
    clash_m: float | None = None


@dataclass
class BenchView:
    map: str
    rows: list[BenchRow]
    removed_count: int


_ROW_TYPES = {"included": bool, "behind": bool, "top": bool, "head": bool,
              "pan_deg": (int, float), "tilt_deg": (int, float)}


def _clean_fields(fields_: Any) -> dict:
    if not isinstance(fields_, dict):
        raise ValueError("fields must be an object")
    out = {}
    for name, v in fields_.items():
        want = _ROW_TYPES.get(name)
        if want is None:
            raise ValueError(f"unknown row field {name!r}")
        if not isinstance(v, want) or (want != bool and isinstance(v, bool)):
            raise ValueError(f"row field {name!r}: wrong type")
        if want != bool and not math.isfinite(v):
            raise ValueError(f"row field {name!r}: must be a finite number")
        if name == "pan_deg":
            v = min(max(float(v), PAN_RANGE[0]), PAN_RANGE[1])
        elif name == "tilt_deg":
            v = min(max(float(v), TILT_RANGE[0]), TILT_RANGE[1])
        out[name] = v
    return out


def _check_row_key(k: Any) -> str:
    if not isinstance(k, str) or k.split(":", 1)[0] not in ("saved", "pasted") or ":" not in k:
        raise ValueError(f"invalid row key {k!r}")
    return k


class BenchStore:
    def __init__(self, state_dir):
        self._dir = Path(state_dir) / "bench"
        self._lock = threading.Lock()
        self._cache: dict[str, BenchState] = {}

    # -- persistence (call with lock held) --
    def _path(self, map_name: str) -> Path:
        return self._dir / f"{check_map_name(map_name)}.json"

    def _load(self, map_name: str) -> BenchState:
        if map_name in self._cache:
            return self._cache[map_name]
        path = self._path(map_name)
        if path.exists():
            st = BenchState.from_dict(json.loads(path.read_text()))
            if st.job and st.job.status in ("running", "queued"):
                st.job.status = "interrupted"
        else:
            st = BenchState(map=map_name)
        self._cache[map_name] = st
        return st

    def _save(self, st: BenchState) -> None:
        path = self._path(st.map)
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name + ".", suffix=".tmp")
        try:
            with os.fdopen(fd, "w") as f:
                json.dump(st.to_dict(), f, indent=1)
            os.replace(tmp, path)
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
        self._cache[st.map] = st

    # -- public --
    def get(self, map_name: str) -> BenchState:
        with self._lock:
            return copy.deepcopy(self._load(map_name))

    def apply(self, map_name: str, ops: list[dict]) -> tuple[BenchState, list[PasteError]]:
        with self._lock:
            st = copy.deepcopy(self._load(map_name))
            errors: list[PasteError] = []
            for op in ops:
                self._apply_one(st, op, errors)
            st.version += 1
            self._save(st)
            return copy.deepcopy(st), errors

    def _apply_one(self, st: BenchState, op: Any, errors: list) -> None:
        name = op.get("op") if isinstance(op, dict) else None
        if name == "set_row":
            self._set(st, [_check_row_key(op.get("row"))], _clean_fields(op.get("fields")))
        elif name == "set_rows":
            rows = op.get("rows")
            if not isinstance(rows, list):
                raise ValueError("rows must be a list")
            self._set(st, [_check_row_key(k) for k in rows], _clean_fields(op.get("fields")))
        elif name == "add_pasted":
            text = op.get("text")
            if not isinstance(text, str):
                raise ValueError("text must be a string")
            poses, errs = parse_pasted(text)
            errors.extend(errs)
            for p in poses:
                for i, old in enumerate(st.pasted):
                    if key(old.name) == key(p.name):
                        st.pasted[i] = p
                        if old.name != p.name:
                            st.rows.pop(f"pasted:{old.name}", None)
                        break
                else:
                    st.pasted.append(p)
        elif name == "remove":
            k = _check_row_key(op.get("row"))
            group, pname = k.split(":", 1)
            st.rows.pop(k, None)
            if group == "pasted":
                st.pasted = [p for p in st.pasted if p.name != pname]
            elif pname not in st.removed_saved:
                st.removed_saved.append(pname)
        elif name == "restore_saved":
            st.removed_saved = []
        elif name == "clear_pasted":
            for p in st.pasted:
                st.rows.pop(f"pasted:{p.name}", None)
            st.pasted = []
        elif name == "set_arm":
            arm = op.get("name")
            if not isinstance(arm, str) or not arm:
                raise ValueError("arm pose name must be a non-empty string")
            st.arm_pose = arm
        else:
            raise ValueError(f"unknown op {name!r}")

    @staticmethod
    def _set(st: BenchState, keys: list[str], values: dict) -> None:
        for k in keys:
            cur = st.rows.get(k, RowState())
            for f, v in values.items():
                setattr(cur, f, v)
            st.rows[k] = cur

    def set_job(self, map_name: str, status: JobStatus) -> None:
        with self._lock:
            st = copy.deepcopy(self._load(map_name))
            st.job = copy.deepcopy(status)
            st.version += 1
            self._save(st)

    def set_last_render(self, map_name: str, record: RenderRecord) -> None:
        with self._lock:
            st = copy.deepcopy(self._load(map_name))
            st.last_render = copy.deepcopy(record)
            st.version += 1
            self._save(st)

    def view(self, map_name: str, maps_dir) -> BenchView:
        st = self.get(map_name)
        saved_all = read_map_studio_poses(Path(maps_dir), map_name)
        removed = set(st.removed_saved)
        saved = [p for p in saved_all if p.name not in removed]
        clash = clashes(saved, st.pasted)
        rows = [BenchRow(f"saved:{p.name}", "saved", p,
                         copy.deepcopy(st.rows.get(f"saved:{p.name}", RowState()))) for p in saved]
        rows += [BenchRow(f"pasted:{p.name}", "pasted", p,
                          copy.deepcopy(st.rows.get(f"pasted:{p.name}", RowState())),
                          clash.get(p.name)) for p in st.pasted]
        return BenchView(map=map_name, rows=rows,
                         removed_count=sum(1 for p in saved_all if p.name in removed))
