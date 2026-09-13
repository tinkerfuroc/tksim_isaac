# tinker_designs M1 — design source + import pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `./.venv/bin/python tools/design_import.py --design designs/<name>` turns a GUI-authored URDF plus a `design.yaml` role sidecar into a content-addressed robot artifact under `artifacts/robot/<name>/<hash>/` (URDF, USD, generated `robot-profile.yaml`, manifest, source lock, `current.json`), and `designs/tinker2_ref` reproduces today's tinker2 artifact URDF byte-for-byte.

**Architecture:** A new pure-stdlib package `tinker_designs/` (schema → model → clean → contract → derive → heuristics → lock), a CLI `tools/design_import.py` that orchestrates those stages behind an injectable `ConverterHooks` protocol (exactly like `tools/arena_import.py`), and a small refactor of `tools/tinker_sim_deploy/workspace.py` that extracts the tinker2 publisher into `publish_robot_artifact(robot=...)` and turns the arm-mount literal into a parameter. Only `tools/design_convert.py` touches Isaac Sim.

**Tech Stack:** Python 3.12 (`.venv`, uv), stdlib `xml.etree` (no lxml in the venv), PyYAML 6.0.3, `unittest.TestCase` tests run via `scripts/pytest-clean`, Isaac Sim 6.0.1 `isaacsim.asset.importer.urdf` (`URDFParseAndImportFile` kit command), `/opt/ros/humble/bin/xacro` (subprocess, only for `.xacro` sources).

**Spec:** `docs/superpowers/specs/2026-09-13-tinker-designs-chassis-exploration-design.md` (rev 2 + §5/§3B amendments in this commit). M2 (runtime selection, metrics) is a separate plan.

## Global Constraints

- Tests are `unittest.TestCase` classes; each test module starts with `ROOT = Path(__file__).resolve().parents[1]` and `sys.path.insert(0, str(ROOT / "tools"))` (and `str(ROOT)` for `tinker_designs`). There is no conftest.
- Run tests with `scripts/pytest-clean tests/<file>.py -q` (it unsets the ROS overlay and uses the main checkout's `.venv` from a worktree). Never `pytest` bare with ROS sourced.
- `tinker_designs/` imports only the stdlib and `yaml`. No `lxml`, no `numpy`, no Isaac imports outside `tools/design_convert.py`.
- Artifact layout, `PUBLICATION_SCHEMA = 4`, `SOURCE_LOCK_SCHEMA = 3`, the atomic stage+rename publisher and `artifact_identity()` are reused from `workspace.py`, never duplicated.
- The published `robot.urdf` keeps `package://` URIs; the `file://`-resolved URDF is a temp file for the importer only.
- Every movable joint must be claimed by exactly one role; unclaimed joints are contract violations (spec §5).
- Existing tinker2 tests (`tests/test_artifact_export.py`, `tests/test_workspace.py`, `tests/test_provenance.py`, `tests/test_current_artifact.py`) must stay green after the Task 7 refactor.
- GPU rule: Task 12 boots Kit. Run `nvidia-smi` immediately before, announce start/end, never overlap another running stack (memory: `feedback-wait-for-user-before-gpu-run`).
- No `git stash`. Commit after every task on the current worktree branch.
- Commit messages end with the attribution lines from the session's system reminder.

---

## File map

| Path | Responsibility |
|---|---|
| `tinker_designs/__init__.py` | package marker, `__all__` |
| `tinker_designs/schema.py` | `Design` dataclasses, `load_design(design_dir)`, `DesignError` |
| `tinker_designs/model.py` | stdlib URDF reading: `parse_urdf`, `Joint`, `Link`, `Origin`, `tree`, `movable_joints` |
| `tinker_designs/clean.py` | `expand_xacro`, `strip_gazebo`, `resolve_mesh_uris`, `package_share_dirs`, `canonical_bytes` |
| `tinker_designs/contract.py` | `check_contract(root, design) -> list[str]` |
| `tinker_designs/derive.py` | `derive_profile(root, design) -> dict` (wheels, footprint, mass, CoG, arm reach) |
| `tinker_designs/heuristics.py` | `draft_design(root, name) -> dict` for `--init` |
| `tinker_designs/lock.py` | `design_source_lock(design_dir, repo_root, extra_files) -> bytes` |
| `tools/tinker_sim_deploy/workspace.py` | refactor: `publish_robot_artifact(...)`, `canonicalize_urdf(data, *, mount_origin=...)`, `_normalized_source_lock(records, robot=...)` |
| `tools/design_import.py` | `ConverterHooks`, `run_import`, `main`, exit codes |
| `tools/design_convert.py` | `IsaacHooks.import_urdf` (Kit only) |
| `designs/tinker2_ref/robot.urdf`, `design.yaml` | parity design |
| `tests/design_fixtures.py` | `two_arm_urdf()`, `two_arm_design()` shared fixtures |
| `tests/test_design_*.py` | one test module per package module + CLI + parity |

---

### Task 1: `tinker_designs.schema` — design.yaml model

**Files:**
- Create: `tinker_designs/__init__.py`, `tinker_designs/schema.py`
- Test: `tests/test_design_schema.py`

**Interfaces:**
- Produces:
  ```python
  class DesignError(ValueError): ...
  @dataclass(frozen=True) class Gripper: drive: str; mimics: tuple[str, ...]
  @dataclass(frozen=True) class Arm: name: str; mount: str; joints: tuple[str, ...]; gripper: Gripper | None; stiffness: float; damping: float; estimated: bool
  @dataclass(frozen=True) class Wheels: driven: tuple[str, ...]; caster_swivel: tuple[str, ...]; caster_wheel: tuple[str, ...]
  @dataclass(frozen=True) class Sensor: type: str; frame: str
  @dataclass(frozen=True) class Design: name: str; kinematics: str; base_frame: str; wheels: Wheels; arms: tuple[Arm, ...]; pan_tilt: tuple[str, ...]; sensors: tuple[Sensor, ...]; footprint: tuple[tuple[float, float], ...] | None; source: str  # "robot.urdf" or "robot.urdf.xacro"
  def load_design(design_dir: Path) -> Design
  def design_from_mapping(raw: Mapping, *, source: str) -> Design
  ```

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_design_schema.py
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tinker_designs.schema import Design, DesignError, design_from_mapping, load_design

MINIMAL = {
    "name": "two_arm_fixture",
    "kinematics": "diff_drive",
    "base_frame": "base_link",
    "wheels": {
        "driven": ["front_left_wheel_joint", "front_right_wheel_joint"],
        "caster_swivel": ["rear_left_swivel_joint", "rear_right_swivel_joint"],
        "caster_wheel": ["rear_left_wheel_joint", "rear_right_wheel_joint"],
    },
    "arms": [
        {"name": "left", "mount": "left_arm_base", "joints": ["l_j1", "l_j2", "l_j3"],
         "gripper": {"drive": "l_grip", "mimics": ["l_finger"]},
         "drive": {"stiffness": 400.0, "damping": 40.0}},
        {"name": "right", "mount": "right_arm_base", "joints": ["r_j1", "r_j2", "r_j3"],
         "gripper": None, "drive": {"stiffness": 400.0, "damping": 40.0}},
    ],
    "pan_tilt": {"joints": ["pan_joint", "tilt_joint"]},
    "sensors": [{"type": "livox_mid360", "frame": "livox_frame"}],
}


class DesignSchemaTest(unittest.TestCase):
    def test_minimal_mapping_loads(self) -> None:
        design = design_from_mapping(MINIMAL, source="robot.urdf")
        self.assertIsInstance(design, Design)
        self.assertEqual(design.name, "two_arm_fixture")
        self.assertEqual(design.wheels.driven, ("front_left_wheel_joint", "front_right_wheel_joint"))
        self.assertEqual(design.arms[0].gripper.drive, "l_grip")
        self.assertIsNone(design.arms[1].gripper)
        self.assertEqual(design.arms[0].stiffness, 400.0)
        self.assertFalse(design.arms[0].estimated)
        self.assertEqual(design.pan_tilt, ("pan_joint", "tilt_joint"))
        self.assertIsNone(design.footprint)

    def test_kinematics_other_than_diff_drive_is_rejected(self) -> None:
        raw = dict(MINIMAL, kinematics="mecanum")
        with self.assertRaisesRegex(DesignError, "kinematics"):
            design_from_mapping(raw, source="robot.urdf")

    def test_duplicate_joint_claims_are_rejected(self) -> None:
        raw = dict(MINIMAL)
        raw["pan_tilt"] = {"joints": ["pan_joint", "l_j1"]}
        with self.assertRaisesRegex(DesignError, "l_j1"):
            design_from_mapping(raw, source="robot.urdf")

    def test_diff_drive_requires_exactly_two_driven_wheels(self) -> None:
        raw = dict(MINIMAL, wheels=dict(MINIMAL["wheels"], driven=["front_left_wheel_joint"]))
        with self.assertRaisesRegex(DesignError, "driven"):
            design_from_mapping(raw, source="robot.urdf")

    def test_footprint_override_is_parsed(self) -> None:
        raw = dict(MINIMAL, footprint=[[0.15, 0.25], [0.15, -0.25], [-0.35, -0.25], [-0.35, 0.25]])
        design = design_from_mapping(raw, source="robot.urdf")
        self.assertEqual(design.footprint[2], (-0.35, -0.25))

    def test_load_design_reads_yaml_and_finds_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            design_dir = Path(temporary) / "two_arm_fixture"
            design_dir.mkdir()
            (design_dir / "design.yaml").write_text(yaml.safe_dump(MINIMAL), encoding="utf-8")
            (design_dir / "robot.urdf").write_text("<robot name='x'/>", encoding="utf-8")
            design = load_design(design_dir)
            self.assertEqual(design.source, "robot.urdf")

    def test_load_design_requires_exactly_one_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            design_dir = Path(temporary) / "d"
            design_dir.mkdir()
            (design_dir / "design.yaml").write_text(yaml.safe_dump(MINIMAL), encoding="utf-8")
            with self.assertRaisesRegex(DesignError, "robot.urdf"):
                load_design(design_dir)
            (design_dir / "robot.urdf").write_text("<robot/>", encoding="utf-8")
            (design_dir / "robot.urdf.xacro").write_text("<robot/>", encoding="utf-8")
            with self.assertRaisesRegex(DesignError, "exactly one"):
                load_design(design_dir)

    def test_name_must_match_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            design_dir = Path(temporary) / "other"
            design_dir.mkdir()
            (design_dir / "design.yaml").write_text(yaml.safe_dump(MINIMAL), encoding="utf-8")
            (design_dir / "robot.urdf").write_text("<robot/>", encoding="utf-8")
            with self.assertRaisesRegex(DesignError, "directory"):
                load_design(design_dir)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_design_schema.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'tinker_designs'`

- [ ] **Step 3: Implement**

```python
# tinker_designs/__init__.py
"""Candidate robot designs: GUI-authored URDF + design.yaml roles -> robot artifact."""

__all__ = ["schema", "model", "clean", "contract", "derive", "heuristics", "lock"]
```

```python
# tinker_designs/schema.py
"""design.yaml: the roles a candidate URDF's joints and links play."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import yaml

KINEMATICS = ("diff_drive",)
SOURCE_NAMES = ("robot.urdf", "robot.urdf.xacro")


class DesignError(ValueError):
    """design.yaml is malformed or inconsistent."""


@dataclass(frozen=True)
class Gripper:
    drive: str
    mimics: tuple[str, ...]


@dataclass(frozen=True)
class Arm:
    name: str
    mount: str
    joints: tuple[str, ...]
    gripper: Gripper | None
    stiffness: float
    damping: float
    estimated: bool

    def claimed_joints(self) -> tuple[str, ...]:
        if self.gripper is None:
            return self.joints
        return self.joints + (self.gripper.drive,) + self.gripper.mimics


@dataclass(frozen=True)
class Wheels:
    driven: tuple[str, ...]
    caster_swivel: tuple[str, ...]
    caster_wheel: tuple[str, ...]


@dataclass(frozen=True)
class Sensor:
    type: str
    frame: str


@dataclass(frozen=True)
class Design:
    name: str
    kinematics: str
    base_frame: str
    wheels: Wheels
    arms: tuple[Arm, ...]
    pan_tilt: tuple[str, ...]
    sensors: tuple[Sensor, ...]
    footprint: tuple[tuple[float, float], ...] | None
    source: str

    def claimed_joints(self) -> dict[str, str]:
        """joint name -> role label; raises on double claims."""
        claims: dict[str, str] = {}

        def claim(names: Sequence[str], role: str) -> None:
            for name in names:
                if name in claims:
                    raise DesignError(f"joint {name!r} is claimed by both {claims[name]} and {role}")
                claims[name] = role

        claim(self.wheels.driven, "wheels.driven")
        claim(self.wheels.caster_swivel, "wheels.caster_swivel")
        claim(self.wheels.caster_wheel, "wheels.caster_wheel")
        for arm in self.arms:
            claim(arm.joints, f"arms.{arm.name}.joints")
            if arm.gripper is not None:
                claim((arm.gripper.drive,), f"arms.{arm.name}.gripper.drive")
                claim(arm.gripper.mimics, f"arms.{arm.name}.gripper.mimics")
        claim(self.pan_tilt, "pan_tilt")
        return claims


def _names(raw: object, label: str) -> tuple[str, ...]:
    if raw is None:
        return ()
    if not isinstance(raw, Sequence) or isinstance(raw, str) or not all(isinstance(item, str) for item in raw):
        raise DesignError(f"{label} must be a list of joint names")
    return tuple(raw)


def _arm(raw: object, index: int) -> Arm:
    if not isinstance(raw, Mapping):
        raise DesignError(f"arms[{index}] must be a mapping")
    name = raw.get("name")
    mount = raw.get("mount")
    if not isinstance(name, str) or not isinstance(mount, str):
        raise DesignError(f"arms[{index}] needs string name and mount")
    joints = _names(raw.get("joints"), f"arms.{name}.joints")
    if not joints:
        raise DesignError(f"arms.{name}.joints must not be empty")
    gripper_raw = raw.get("gripper")
    gripper = None
    if gripper_raw is not None:
        if not isinstance(gripper_raw, Mapping) or not isinstance(gripper_raw.get("drive"), str):
            raise DesignError(f"arms.{name}.gripper needs a drive joint")
        gripper = Gripper(gripper_raw["drive"], _names(gripper_raw.get("mimics"), f"arms.{name}.gripper.mimics"))
    drive = raw.get("drive") or {}
    if not isinstance(drive, Mapping):
        raise DesignError(f"arms.{name}.drive must be a mapping")
    return Arm(
        name=name, mount=mount, joints=joints, gripper=gripper,
        stiffness=float(drive.get("stiffness", 400.0)), damping=float(drive.get("damping", 40.0)),
        estimated=bool(raw.get("estimated", False)),
    )


def design_from_mapping(raw: Mapping, *, source: str) -> Design:
    if not isinstance(raw, Mapping):
        raise DesignError("design.yaml must be a mapping")
    name = raw.get("name")
    if not isinstance(name, str) or not name:
        raise DesignError("name is required")
    kinematics = raw.get("kinematics")
    if kinematics not in KINEMATICS:
        raise DesignError(f"kinematics must be one of {KINEMATICS}, got {kinematics!r}")
    base_frame = raw.get("base_frame", "base_link")
    if not isinstance(base_frame, str):
        raise DesignError("base_frame must be a string")
    wheels_raw = raw.get("wheels")
    if not isinstance(wheels_raw, Mapping):
        raise DesignError("wheels is required")
    wheels = Wheels(
        _names(wheels_raw.get("driven"), "wheels.driven"),
        _names(wheels_raw.get("caster_swivel"), "wheels.caster_swivel"),
        _names(wheels_raw.get("caster_wheel"), "wheels.caster_wheel"),
    )
    if kinematics == "diff_drive" and len(wheels.driven) != 2:
        raise DesignError("diff_drive needs exactly two wheels.driven joints")
    if len(wheels.caster_swivel) != len(wheels.caster_wheel):
        raise DesignError("wheels.caster_swivel and wheels.caster_wheel must pair up")
    arms_raw = raw.get("arms") or []
    if not isinstance(arms_raw, Sequence) or isinstance(arms_raw, str):
        raise DesignError("arms must be a list")
    arms = tuple(_arm(item, index) for index, item in enumerate(arms_raw))
    if len({arm.name for arm in arms}) != len(arms):
        raise DesignError("arm names must be unique")
    pan_tilt_raw = raw.get("pan_tilt")
    pan_tilt = () if pan_tilt_raw is None else _names(pan_tilt_raw.get("joints") if isinstance(pan_tilt_raw, Mapping) else None, "pan_tilt.joints")
    sensors_raw = raw.get("sensors") or []
    sensors = []
    for item in sensors_raw:
        if not isinstance(item, Mapping) or not isinstance(item.get("type"), str) or not isinstance(item.get("frame"), str):
            raise DesignError("each sensor needs string type and frame")
        sensors.append(Sensor(item["type"], item["frame"]))
    footprint_raw = raw.get("footprint")
    footprint = None
    if footprint_raw is not None:
        try:
            footprint = tuple((float(x), float(y)) for x, y in footprint_raw)
        except (TypeError, ValueError) as error:
            raise DesignError("footprint must be a list of [x, y] pairs") from error
        if len(footprint) < 3:
            raise DesignError("footprint needs at least three vertices")
    design = Design(name, kinematics, base_frame, wheels, arms, pan_tilt, tuple(sensors), footprint, source)
    design.claimed_joints()
    return design


def load_design(design_dir: Path) -> Design:
    design_dir = Path(design_dir)
    yaml_path = design_dir / "design.yaml"
    if not yaml_path.is_file():
        raise DesignError(f"missing {yaml_path}")
    present = [name for name in SOURCE_NAMES if (design_dir / name).is_file()]
    if not present:
        raise DesignError(f"{design_dir} needs robot.urdf or robot.urdf.xacro")
    if len(present) != 1:
        raise DesignError(f"{design_dir} must contain exactly one of {SOURCE_NAMES}")
    raw = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    design = design_from_mapping(raw, source=present[0])
    if design.name != design_dir.name:
        raise DesignError(f"design name {design.name!r} must equal its directory name {design_dir.name!r}")
    return design
```

- [ ] **Step 4: Run to verify pass**

Run: `scripts/pytest-clean tests/test_design_schema.py -q`
Expected: 8 passed

- [ ] **Step 5: Commit**

```bash
git add tinker_designs/__init__.py tinker_designs/schema.py tests/test_design_schema.py
git commit -m "feat(designs): design.yaml schema for candidate robots"
```

---

### Task 2: `tinker_designs.model` — stdlib URDF reading + shared fixture

**Files:**
- Create: `tinker_designs/model.py`, `tests/design_fixtures.py`
- Test: `tests/test_design_model.py`

**Interfaces:**
- Produces:
  ```python
  Vec3 = tuple[float, float, float]
  @dataclass(frozen=True) class Origin: xyz: Vec3; rpy: Vec3
  @dataclass(frozen=True) class Geometry: kind: str  # "box"|"cylinder"|"sphere"|"mesh"|"none"
                                          size: Vec3 | None; radius: float | None; length: float | None; filename: str | None
  @dataclass(frozen=True) class Inertial: mass: float; origin: Origin; ixx: float; iyy: float; izz: float; ixy: float; ixz: float; iyz: float
  @dataclass(frozen=True) class Link: name: str; inertial: Inertial | None; collisions: tuple[tuple[Origin, Geometry], ...]
  @dataclass(frozen=True) class Joint: name: str; type: str; parent: str; child: str; origin: Origin; axis: Vec3; lower: float | None; upper: float | None; mimic: str | None
  def parse_urdf(data: bytes) -> ET.Element
  def links(root) -> dict[str, Link]
  def joints(root) -> dict[str, Joint]
  def root_links(root) -> list[str]             # links with no parent joint
  def movable_joints(root) -> list[str]         # type != "fixed"
  def parent_joint(root) -> dict[str, Joint]    # child link -> joint
  def fixed_subtree(root, link: str) -> set[str]  # link + everything reachable via fixed joints only
  def rotation(rpy: Vec3) -> tuple[Vec3, Vec3, Vec3]   # row-major R
  def apply(origin: Origin, point: Vec3) -> Vec3       # R·p + t
  def compose(a: Origin, b: Origin) -> Origin          # a then b (matrix product; returned rpy re-extracted)
  def rotate_axis_angle(axis: Vec3, angle: float) -> tuple[Vec3, Vec3, Vec3]
  ```
- `tests/design_fixtures.py`: `two_arm_urdf() -> bytes`, `two_arm_design() -> dict` (the MINIMAL mapping from Task 1 as a function).

- [ ] **Step 1: Write the fixture module**

```python
# tests/design_fixtures.py
"""A two-arm, diff-drive primitive robot used by every tinker_designs test."""
from __future__ import annotations

import xml.etree.ElementTree as ET


def _link(root: ET.Element, name: str, mass: float, geometry: ET.Element | None = None,
          origin: tuple[str, str] = ("0 0 0", "0 0 0"), inertia: tuple[str, ...] = ("0.01",) * 3) -> None:
    link = ET.SubElement(root, "link", {"name": name})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": str(mass)})
    ET.SubElement(inertial, "inertia", {"ixx": inertia[0], "iyy": inertia[1], "izz": inertia[2], "ixy": "0", "ixz": "0", "iyz": "0"})
    if geometry is not None:
        for tag in ("visual", "collision"):
            element = ET.SubElement(link, tag)
            ET.SubElement(element, "origin", {"xyz": origin[0], "rpy": origin[1]})
            ET.SubElement(element, "geometry").append(ET.fromstring(ET.tostring(geometry)))


def _joint(root: ET.Element, name: str, type_: str, parent: str, child: str, xyz: str,
           axis: str | None = None, limits: tuple[str, str] | None = None, mimic: str | None = None) -> None:
    joint = ET.SubElement(root, "joint", {"name": name, "type": type_})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": "0 0 0"})
    if axis is not None:
        ET.SubElement(joint, "axis", {"xyz": axis})
    if type_ == "continuous":
        ET.SubElement(joint, "limit", {"effort": "10", "velocity": "20"})
    if limits is not None:
        ET.SubElement(joint, "limit", {"lower": limits[0], "upper": limits[1], "effort": "10", "velocity": "2"})
    if mimic is not None:
        ET.SubElement(joint, "mimic", {"joint": mimic, "multiplier": "1", "offset": "0"})


def two_arm_urdf() -> bytes:
    root = ET.Element("robot", {"name": "two_arm_fixture"})
    box = ET.Element("box", {"size": "0.5 0.4 0.2"})
    front = ET.Element("cylinder", {"radius": "0.06", "length": "0.05"})
    rear = ET.Element("cylinder", {"radius": "0.04", "length": "0.03"})
    _link(root, "base_link", 20.0, box)
    for side, y in (("left", "0.2"), ("right", "-0.2")):
        _link(root, f"front_{side}_wheel", 1.0, front, ("0 0 0", "1.5708 0 0"))
        _joint(root, f"front_{side}_wheel_joint", "continuous", "base_link", f"front_{side}_wheel", f"0 {y} -0.05", "0 1 0")
        _link(root, f"rear_{side}_swivel", 0.1)
        _joint(root, f"rear_{side}_swivel_joint", "continuous", "base_link", f"rear_{side}_swivel", f"-0.3 {y} -0.07", "0 0 1")
        _link(root, f"rear_{side}_wheel", 1.0, rear, ("0 0 0", "1.5708 0 0"))
        _joint(root, f"rear_{side}_wheel_joint", "continuous", f"rear_{side}_swivel", f"rear_{side}_wheel", "-0.02 0 0", "0 1 0")
    for prefix, y in (("l", "0.15"), ("r", "-0.15")):
        base = f"{'left' if prefix == 'l' else 'right'}_arm_base"
        _link(root, base, 0.5)
        _joint(root, f"{base}_joint", "fixed", "base_link", base, f"0.1 {y} 0.1")
        _link(root, f"{prefix}_link1", 1.0)
        _joint(root, f"{prefix}_j1", "revolute", base, f"{prefix}_link1", "0 0 0.05", "0 0 1", ("-3.14", "3.14"))
        _link(root, f"{prefix}_link2", 1.0)
        _joint(root, f"{prefix}_j2", "revolute", f"{prefix}_link1", f"{prefix}_link2", "0 0 0.1", "0 1 0", ("-1.57", "1.57"))
        _link(root, f"{prefix}_link3", 1.0)
        _joint(root, f"{prefix}_j3", "revolute", f"{prefix}_link2", f"{prefix}_link3", "0.3 0 0", "0 1 0", ("-1.57", "1.57"))
    _link(root, "l_gripper", 0.1)
    _joint(root, "l_grip", "revolute", "l_link3", "l_gripper", "0.3 0 0", "0 0 1", ("0", "0.8"))
    _link(root, "l_finger_link", 0.1)
    _joint(root, "l_finger", "revolute", "l_gripper", "l_finger_link", "0.02 0 0", "0 0 1", ("0", "0.8"), mimic="l_grip")
    _link(root, "pan_link", 0.2)
    _joint(root, "pan_joint", "revolute", "base_link", "pan_link", "0 0 0.3", "0 0 1", ("-1.5", "1.5"))
    _link(root, "tilt_link", 0.2)
    _joint(root, "tilt_joint", "revolute", "pan_link", "tilt_link", "0 0 0.05", "0 1 0", ("-0.5", "0.5"))
    _link(root, "livox_frame", 0.25, ET.Element("box", {"size": "0.06 0.1 0.06"}))
    _joint(root, "livox_joint", "fixed", "base_link", "livox_frame", "0.2 0 0.15")
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def two_arm_design() -> dict:
    return {
        "name": "two_arm_fixture",
        "kinematics": "diff_drive",
        "base_frame": "base_link",
        "wheels": {
            "driven": ["front_left_wheel_joint", "front_right_wheel_joint"],
            "caster_swivel": ["rear_left_swivel_joint", "rear_right_swivel_joint"],
            "caster_wheel": ["rear_left_wheel_joint", "rear_right_wheel_joint"],
        },
        "arms": [
            {"name": "left", "mount": "left_arm_base", "joints": ["l_j1", "l_j2", "l_j3"],
             "gripper": {"drive": "l_grip", "mimics": ["l_finger"]}, "drive": {"stiffness": 400.0, "damping": 40.0}},
            {"name": "right", "mount": "right_arm_base", "joints": ["r_j1", "r_j2", "r_j3"],
             "gripper": None, "drive": {"stiffness": 400.0, "damping": 40.0}},
        ],
        "pan_tilt": {"joints": ["pan_joint", "tilt_joint"]},
        "sensors": [{"type": "livox_mid360", "frame": "livox_frame"}],
    }
```

Total fixture mass: base 20 + 4 wheels 4.0 + 2 swivels 0.2 + 2 arm bases 1.0 + 6 arm links 6.0 + gripper 0.1 + finger 0.1 + pan 0.2 + tilt 0.2 + livox 0.25 = **32.05 kg**. Wheel bottoms: front −0.05−0.06 = −0.11; rear −0.07−0.04 = −0.11 (same ground plane).

- [ ] **Step 2: Write the failing tests**

```python
# tests/test_design_model.py
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from design_fixtures import two_arm_urdf
from tinker_designs.model import (
    Origin, apply, compose, fixed_subtree, joints, links, movable_joints, parse_urdf, root_links, rotate_axis_angle,
)


class ModelTest(unittest.TestCase):
    def setUp(self) -> None:
        self.root = parse_urdf(two_arm_urdf())

    def test_links_and_joints_are_indexed_by_name(self) -> None:
        link_index = links(self.root)
        joint_index = joints(self.root)
        self.assertEqual(link_index["base_link"].inertial.mass, 20.0)
        self.assertEqual(link_index["front_left_wheel"].collisions[0][1].kind, "cylinder")
        self.assertEqual(link_index["front_left_wheel"].collisions[0][1].radius, 0.06)
        self.assertEqual(link_index["rear_left_swivel"].collisions, ())
        wheel = joint_index["front_left_wheel_joint"]
        self.assertEqual((wheel.type, wheel.parent, wheel.child), ("continuous", "base_link", "front_left_wheel"))
        self.assertEqual(wheel.origin.xyz, (0.0, 0.2, -0.05))
        self.assertEqual(wheel.axis, (0.0, 1.0, 0.0))
        self.assertEqual(joint_index["l_finger"].mimic, "l_grip")
        self.assertEqual(joint_index["l_j2"].lower, -1.57)
        self.assertIsNone(wheel.lower)

    def test_roots_and_movable_joints(self) -> None:
        self.assertEqual(root_links(self.root), ["base_link"])
        movable = movable_joints(self.root)
        self.assertIn("front_left_wheel_joint", movable)
        self.assertIn("l_finger", movable)
        self.assertNotIn("livox_joint", movable)
        self.assertEqual(len(movable), 8 + 6 + 2 + 2)  # wheels/casters, arm joints, gripper+mimic, pan/tilt

    def test_fixed_subtree_stops_at_movable_joints(self) -> None:
        subtree = fixed_subtree(self.root, "base_link")
        self.assertEqual(subtree, {"base_link", "left_arm_base", "right_arm_base", "livox_frame"})

    def test_transforms(self) -> None:
        origin = Origin((1.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2))
        x, y, z = apply(origin, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(z, 0.0)
        composed = compose(origin, Origin((1.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
        self.assertAlmostEqual(composed.xyz[0], 1.0)
        self.assertAlmostEqual(composed.xyz[1], 1.0)
        rotation = rotate_axis_angle((0.0, 1.0, 0.0), -math.pi / 2)
        vx, vy, vz = (sum(rotation[row][col] * (0.3, 0.0, 0.0)[col] for col in range(3)) for row in range(3))
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(vz, 0.3)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 3: Run to verify failure**

Run: `scripts/pytest-clean tests/test_design_model.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'tinker_designs.model'`

- [ ] **Step 4: Implement**

```python
# tinker_designs/model.py
"""Read a URDF with the stdlib and expose links, joints and rigid transforms."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]
ZERO: Vec3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Origin:
    xyz: Vec3 = ZERO
    rpy: Vec3 = ZERO


@dataclass(frozen=True)
class Geometry:
    kind: str
    size: Vec3 | None = None
    radius: float | None = None
    length: float | None = None
    filename: str | None = None


@dataclass(frozen=True)
class Inertial:
    mass: float
    origin: Origin
    ixx: float
    iyy: float
    izz: float
    ixy: float
    ixz: float
    iyz: float


@dataclass(frozen=True)
class Link:
    name: str
    inertial: Inertial | None
    collisions: tuple[tuple[Origin, Geometry], ...]


@dataclass(frozen=True)
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: Origin
    axis: Vec3
    lower: float | None
    upper: float | None
    mimic: str | None


def parse_urdf(data: bytes) -> ET.Element:
    root = ET.fromstring(data)
    if root.tag != "robot":
        raise ValueError("URDF root element must be <robot>")
    return root


def _vec3(text: str | None, default: Vec3 = ZERO) -> Vec3:
    if text is None:
        return default
    parts = [float(item) for item in text.split()]
    if len(parts) != 3:
        raise ValueError(f"expected three numbers, got {text!r}")
    return (parts[0], parts[1], parts[2])


def _origin(element: ET.Element | None) -> Origin:
    if element is None:
        return Origin()
    return Origin(_vec3(element.get("xyz")), _vec3(element.get("rpy")))


def _geometry(element: ET.Element | None) -> Geometry:
    if element is None:
        return Geometry("none")
    box = element.find("box")
    if box is not None:
        return Geometry("box", size=_vec3(box.get("size")))
    cylinder = element.find("cylinder")
    if cylinder is not None:
        return Geometry("cylinder", radius=float(cylinder.get("radius", "0")), length=float(cylinder.get("length", "0")))
    sphere = element.find("sphere")
    if sphere is not None:
        return Geometry("sphere", radius=float(sphere.get("radius", "0")))
    mesh = element.find("mesh")
    if mesh is not None:
        return Geometry("mesh", filename=mesh.get("filename"))
    return Geometry("none")


def links(root: ET.Element) -> dict[str, Link]:
    result: dict[str, Link] = {}
    for element in root.findall("link"):
        name = element.get("name") or ""
        inertial_element = element.find("inertial")
        inertial = None
        if inertial_element is not None:
            mass_element = inertial_element.find("mass")
            inertia = inertial_element.find("inertia")
            get = (lambda key: float(inertia.get(key, "0"))) if inertia is not None else (lambda key: 0.0)
            inertial = Inertial(
                mass=float(mass_element.get("value", "0")) if mass_element is not None else 0.0,
                origin=_origin(inertial_element.find("origin")),
                ixx=get("ixx"), iyy=get("iyy"), izz=get("izz"), ixy=get("ixy"), ixz=get("ixz"), iyz=get("iyz"),
            )
        collisions = tuple(
            (_origin(collision.find("origin")), _geometry(collision.find("geometry")))
            for collision in element.findall("collision")
        )
        result[name] = Link(name, inertial, collisions)
    return result


def joints(root: ET.Element) -> dict[str, Joint]:
    result: dict[str, Joint] = {}
    for element in root.findall("joint"):
        parent = element.find("parent")
        child = element.find("child")
        limit = element.find("limit")
        axis = element.find("axis")
        mimic = element.find("mimic")
        result[element.get("name") or ""] = Joint(
            name=element.get("name") or "",
            type=element.get("type") or "",
            parent=parent.get("link") if parent is not None else "",
            child=child.get("link") if child is not None else "",
            origin=_origin(element.find("origin")),
            axis=_vec3(axis.get("xyz") if axis is not None else None, (1.0, 0.0, 0.0)),
            lower=float(limit.get("lower")) if limit is not None and limit.get("lower") is not None else None,
            upper=float(limit.get("upper")) if limit is not None and limit.get("upper") is not None else None,
            mimic=mimic.get("joint") if mimic is not None else None,
        )
    return result


def parent_joint(root: ET.Element) -> dict[str, Joint]:
    return {joint.child: joint for joint in joints(root).values()}


def root_links(root: ET.Element) -> list[str]:
    children = {joint.child for joint in joints(root).values()}
    return [name for name in links(root) if name not in children]


def movable_joints(root: ET.Element) -> list[str]:
    return [name for name, joint in joints(root).items() if joint.type != "fixed"]


def fixed_subtree(root: ET.Element, link: str) -> set[str]:
    by_parent: dict[str, list[Joint]] = {}
    for joint in joints(root).values():
        by_parent.setdefault(joint.parent, []).append(joint)
    seen = {link}
    stack = [link]
    while stack:
        current = stack.pop()
        for joint in by_parent.get(current, []):
            if joint.type == "fixed" and joint.child not in seen:
                seen.add(joint.child)
                stack.append(joint.child)
    return seen


def rotation(rpy: Vec3) -> Mat3:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # URDF: R = Rz(yaw) · Ry(pitch) · Rx(roll)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def rotate_axis_angle(axis: Vec3, angle: float) -> Mat3:
    norm = math.sqrt(sum(component * component for component in axis)) or 1.0
    x, y, z = (component / norm for component in axis)
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return (
        (t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    )


def _matmul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)) for row in range(3))  # type: ignore[return-value]


def _matvec(m: Mat3, v: Vec3) -> Vec3:
    return tuple(sum(m[row][col] * v[col] for col in range(3)) for row in range(3))  # type: ignore[return-value]


def _rpy_from_matrix(m: Mat3) -> Vec3:
    pitch = math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0]))
    if abs(math.cos(pitch)) < 1e-9:
        return (math.atan2(-m[1][2], m[1][1]), pitch, 0.0)
    return (math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0]))


def apply(origin: Origin, point: Vec3) -> Vec3:
    rotated = _matvec(rotation(origin.rpy), point)
    return (rotated[0] + origin.xyz[0], rotated[1] + origin.xyz[1], rotated[2] + origin.xyz[2])


def compose(a: Origin, b: Origin) -> Origin:
    """Transform that applies b first, then a (T_a · T_b)."""
    return Origin(apply(a, b.xyz), _rpy_from_matrix(_matmul(rotation(a.rpy), rotation(b.rpy))))


def compose_rotation(a: Origin, r: Mat3) -> Origin:
    """T_a followed by a pure rotation r about a's origin (used for joint angles)."""
    return Origin(a.xyz, _rpy_from_matrix(_matmul(rotation(a.rpy), r)))
```

- [ ] **Step 5: Run to verify pass**

Run: `scripts/pytest-clean tests/test_design_model.py -q`
Expected: 4 passed

- [ ] **Step 6: Commit**

```bash
git add tinker_designs/model.py tests/design_fixtures.py tests/test_design_model.py
git commit -m "feat(designs): stdlib URDF model and two-arm test fixture"
```

---

### Task 3: `tinker_designs.clean` — xacro, gazebo strip, mesh URI resolution, canonical bytes

**Files:**
- Create: `tinker_designs/clean.py`
- Test: `tests/test_design_clean.py`

**Interfaces:**
- Produces:
  ```python
  class CleanError(RuntimeError): ...
  def expand_xacro(xacro_path: Path, *, runner=subprocess.run) -> bytes     # shells out to `xacro`
  def strip_gazebo(root: ET.Element) -> int                                 # returns count removed
  def package_share_dirs(env: Mapping[str, str] = os.environ) -> dict[str, Path]  # pkg -> share dir, from AMENT_PREFIX_PATH
  def resolve_mesh_uris(root: ET.Element, *, design_dir: Path, packages: Mapping[str, Path]) -> list[Path]
      # rewrites mesh filename package://pkg/rel -> file:///abs and bare relative -> file://<design_dir>/rel;
      # returns every resolved file path; raises CleanError listing every unresolved URI
  def canonical_bytes(root: ET.Element) -> bytes                            # ET.canonicalize, trailing newline
  ```

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_design_clean.py
from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tinker_designs.clean import CleanError, canonical_bytes, expand_xacro, package_share_dirs, resolve_mesh_uris, strip_gazebo

URDF = b"""<?xml version="1.0"?>
<robot name="c">
  <gazebo reference="base_link"><material>x</material></gazebo>
  <link name="base_link">
    <visual><geometry><mesh filename="package://demo_pkg/meshes/body.stl"/></geometry></visual>
    <collision><geometry><mesh filename="meshes/local.stl"/></geometry></collision>
    <gazebo><nested/></gazebo>
  </link>
</robot>
"""


class CleanTest(unittest.TestCase):
    def test_strip_gazebo_removes_nested_blocks(self) -> None:
        root = ET.fromstring(URDF)
        self.assertEqual(strip_gazebo(root), 2)
        self.assertEqual(root.findall(".//gazebo"), [])

    def test_package_share_dirs_scans_ament_prefix_path(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            prefix = Path(temporary) / "install"
            share = prefix / "share" / "demo_pkg"
            share.mkdir(parents=True)
            (share / "package.xml").write_text("<package/>", encoding="utf-8")
            (prefix / "share" / "not_a_pkg").mkdir()
            found = package_share_dirs({"AMENT_PREFIX_PATH": f"{prefix}:/nonexistent"})
            self.assertEqual(found, {"demo_pkg": share})

    def test_resolve_rewrites_package_and_relative_uris(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            share = Path(temporary) / "share" / "demo_pkg"
            (share / "meshes").mkdir(parents=True)
            (share / "meshes" / "body.stl").write_bytes(b"solid")
            design_dir = Path(temporary) / "design"
            (design_dir / "meshes").mkdir(parents=True)
            (design_dir / "meshes" / "local.stl").write_bytes(b"solid")
            root = ET.fromstring(URDF)
            resolved = resolve_mesh_uris(root, design_dir=design_dir, packages={"demo_pkg": share})
            names = [mesh.get("filename") for mesh in root.iter("mesh")]
            self.assertEqual(names, [f"file://{share / 'meshes' / 'body.stl'}", f"file://{design_dir / 'meshes' / 'local.stl'}"])
            self.assertEqual(sorted(resolved), sorted([share / "meshes" / "body.stl", design_dir / "meshes" / "local.stl"]))

    def test_resolve_reports_every_unresolved_uri(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = ET.fromstring(URDF)
            with self.assertRaises(CleanError) as caught:
                resolve_mesh_uris(root, design_dir=Path(temporary), packages={})
            message = str(caught.exception)
            self.assertIn("demo_pkg", message)
            self.assertIn("meshes/local.stl", message)

    def test_canonical_bytes_is_idempotent_and_drops_comments(self) -> None:
        root = ET.fromstring(b"<robot name='c'><!-- note --><link name='a'/></robot>")
        first = canonical_bytes(root)
        self.assertNotIn(b"note", first)
        self.assertTrue(first.endswith(b"\n"))
        self.assertEqual(canonical_bytes(ET.fromstring(first)), first)

    def test_expand_xacro_shells_out_and_surfaces_stderr(self) -> None:
        calls = []

        def runner(command, **kwargs):
            calls.append(command)
            return subprocess.CompletedProcess(command, 0, stdout=b"<robot name='x'/>", stderr=b"")

        self.assertEqual(expand_xacro(Path("/tmp/r.urdf.xacro"), runner=runner), b"<robot name='x'/>")
        self.assertEqual(calls[0][:2], ["xacro", "/tmp/r.urdf.xacro"])

        def failing(command, **kwargs):
            return subprocess.CompletedProcess(command, 1, stdout=b"", stderr=b"boom")

        with self.assertRaisesRegex(CleanError, "boom"):
            expand_xacro(Path("/tmp/r.urdf.xacro"), runner=failing)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_design_clean.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'tinker_designs.clean'`

- [ ] **Step 3: Implement**

```python
# tinker_designs/clean.py
"""Turn a design's URDF/xacro into what Isaac's importer accepts.

Ported from tk26_sim's render_for_isaac.sh: xacro expansion, <gazebo> strip,
package:// -> file:// resolution. Uses the stdlib only (lxml is not in the venv).
"""
from __future__ import annotations

import os
import subprocess
import xml.etree.ElementTree as ET
from collections.abc import Callable, Mapping
from pathlib import Path


class CleanError(RuntimeError):
    """A render/clean stage failed; the message lists every problem found."""


def expand_xacro(xacro_path: Path, *, runner: Callable[..., subprocess.CompletedProcess] = subprocess.run) -> bytes:
    command = ["xacro", str(xacro_path)]
    completed = runner(command, capture_output=True, check=False)
    if completed.returncode != 0:
        raise CleanError(f"xacro failed ({completed.returncode}):\n{completed.stderr.decode('utf-8', 'replace')}")
    return completed.stdout


def strip_gazebo(root: ET.Element) -> int:
    removed = 0
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "gazebo":
                parent.remove(child)
                removed += 1
    return removed


def package_share_dirs(env: Mapping[str, str] = os.environ) -> dict[str, Path]:
    found: dict[str, Path] = {}
    for prefix in (item for item in env.get("AMENT_PREFIX_PATH", "").split(":") if item):
        share = Path(prefix) / "share"
        if not share.is_dir():
            continue
        for candidate in share.iterdir():
            if candidate.is_dir() and (candidate / "package.xml").is_file():
                found.setdefault(candidate.name, candidate)
    return found


def resolve_mesh_uris(root: ET.Element, *, design_dir: Path, packages: Mapping[str, Path]) -> list[Path]:
    resolved: list[Path] = []
    problems: list[str] = []
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename", "")
        if uri.startswith("file://"):
            target = Path(uri[len("file://"):])
        elif uri.startswith("package://"):
            package, _, relative = uri[len("package://"):].partition("/")
            share = packages.get(package)
            if share is None:
                problems.append(f"unknown package {package!r} in {uri!r}")
                continue
            target = Path(share) / relative
        else:
            target = Path(design_dir) / uri
        if not target.is_file():
            problems.append(f"missing mesh file {target} (from {uri!r})")
            continue
        mesh.set("filename", f"file://{target}")
        resolved.append(target)
    if problems:
        raise CleanError("unresolved mesh URIs:\n  " + "\n  ".join(problems))
    return resolved


def canonical_bytes(root: ET.Element) -> bytes:
    xml = ET.tostring(root, encoding="unicode")
    canonical = ET.canonicalize(xml_data=xml, with_comments=False, strip_text=False)
    return (canonical.rstrip("\n") + "\n").encode("utf-8")
```

- [ ] **Step 4: Run to verify pass**

Run: `scripts/pytest-clean tests/test_design_clean.py -q`
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add tinker_designs/clean.py tests/test_design_clean.py
git commit -m "feat(designs): clean stage — xacro, gazebo strip, mesh URI resolution"
```

---

### Task 4: `tinker_designs.contract` — role-driven structural check

**Files:**
- Create: `tinker_designs/contract.py`
- Test: `tests/test_design_contract.py`

**Interfaces:**
- Consumes: `schema.Design`, `model.*`
- Produces:
  ```python
  class ContractError(ValueError): violations: list[str]
  def check_contract(root: ET.Element, design: Design) -> list[str]   # [] when clean; never raises
  def require_contract(root, design) -> None                          # raises ContractError with all violations
  ```
  Rules (spec §5 + inertia sanity): single root == `base_frame` (or a `world` link with a zero fixed joint to it); every movable joint claimed exactly once and every claimed joint exists; driven wheels continuous, parent in `fixed_subtree(base_frame)`, axis ±Y (|ay| > 0.99); caster swivel continuous axis ±Z with parent in the fixed subtree; caster wheel continuous axis ±Y with parent == its swivel's child; each arm: `mount` in fixed subtree, `joints` a serial chain starting at `mount`; gripper drive's parent chain reaches the arm's last link; mimics carry `<mimic joint=drive>`; every link with an `<inertial>` has mass > 0 and diagonal inertia > 0 with |off-diagonal| ≤ sqrt(ixx·iyy) style PD check on the 3×3 (Sylvester's criterion); wheel ground contact z (joint z − radius, through swivel offsets) equal within 1 mm across driven+caster wheels; sensor frames exist.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_design_contract.py
from __future__ import annotations

import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from design_fixtures import two_arm_design, two_arm_urdf
from tinker_designs.contract import ContractError, check_contract, require_contract
from tinker_designs.model import parse_urdf
from tinker_designs.schema import design_from_mapping


def _design(**overrides):
    raw = two_arm_design()
    raw.update(overrides)
    return design_from_mapping(raw, source="robot.urdf")


def _set_attr(root: ET.Element, joint: str, tag: str, attribute: str, value: str) -> None:
    for element in root.findall("joint"):
        if element.get("name") == joint:
            element.find(tag).set(attribute, value)


class ContractTest(unittest.TestCase):
    def test_fixture_passes(self) -> None:
        self.assertEqual(check_contract(parse_urdf(two_arm_urdf()), _design()), [])

    def test_unclaimed_movable_joint_is_a_violation(self) -> None:
        raw = two_arm_design()
        raw["pan_tilt"] = None
        design = design_from_mapping(raw, source="robot.urdf")
        violations = check_contract(parse_urdf(two_arm_urdf()), design)
        self.assertTrue(any("pan_joint" in item and "unclaimed" in item for item in violations), violations)
        self.assertTrue(any("tilt_joint" in item for item in violations))

    def test_claimed_joint_missing_from_urdf(self) -> None:
        raw = two_arm_design()
        raw["arms"][1]["joints"].append("r_j4")
        violations = check_contract(parse_urdf(two_arm_urdf()), design_from_mapping(raw, source="robot.urdf"))
        self.assertTrue(any("r_j4" in item and "not in the URDF" in item for item in violations), violations)

    def test_driven_wheel_axis_must_be_y(self) -> None:
        root = parse_urdf(two_arm_urdf())
        _set_attr(root, "front_left_wheel_joint", "axis", "xyz", "1 0 0")
        violations = check_contract(root, _design())
        self.assertTrue(any("front_left_wheel_joint" in item and "axis" in item for item in violations), violations)

    def test_arm_mount_must_be_fixed_to_base(self) -> None:
        root = parse_urdf(two_arm_urdf())
        for element in root.findall("joint"):
            if element.get("name") == "left_arm_base_joint":
                element.set("type", "revolute")
                ET.SubElement(element, "axis", {"xyz": "0 0 1"})
                ET.SubElement(element, "limit", {"lower": "-1", "upper": "1", "effort": "1", "velocity": "1"})
        violations = check_contract(root, _design())
        self.assertTrue(any("left_arm_base" in item and "fixed" in item for item in violations), violations)

    def test_arm_chain_must_be_serial_from_mount(self) -> None:
        raw = two_arm_design()
        raw["arms"][0]["joints"] = ["l_j1", "l_j3", "l_j2"]
        violations = check_contract(parse_urdf(two_arm_urdf()), design_from_mapping(raw, source="robot.urdf"))
        self.assertTrue(any("left" in item and "serial" in item for item in violations), violations)

    def test_mimic_must_point_at_drive(self) -> None:
        root = parse_urdf(two_arm_urdf())
        for element in root.findall("joint"):
            if element.get("name") == "l_finger":
                element.find("mimic").set("joint", "l_j1")
        violations = check_contract(root, _design())
        self.assertTrue(any("l_finger" in item and "mimic" in item for item in violations), violations)

    def test_wheels_must_share_a_ground_plane(self) -> None:
        root = parse_urdf(two_arm_urdf())
        _set_attr(root, "rear_left_swivel_joint", "origin", "xyz", "-0.3 0.2 -0.09")
        violations = check_contract(root, _design())
        self.assertTrue(any("ground" in item for item in violations), violations)

    def test_inertia_must_be_positive_definite(self) -> None:
        root = parse_urdf(two_arm_urdf())
        for link in root.findall("link"):
            if link.get("name") == "l_link2":
                link.find("inertial/inertia").set("ixx", "-0.01")
        violations = check_contract(root, _design())
        self.assertTrue(any("l_link2" in item and "inertia" in item for item in violations), violations)

    def test_zero_mass_is_a_violation(self) -> None:
        root = parse_urdf(two_arm_urdf())
        for link in root.findall("link"):
            if link.get("name") == "livox_frame":
                link.find("inertial/mass").set("value", "0")
        violations = check_contract(root, _design())
        self.assertTrue(any("livox_frame" in item and "mass" in item for item in violations), violations)

    def test_world_root_with_zero_fixed_joint_is_accepted(self) -> None:
        root = parse_urdf(two_arm_urdf())
        root.insert(0, ET.Element("link", {"name": "world"}))
        joint = ET.SubElement(root, "joint", {"name": "world_joint", "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": "world"})
        ET.SubElement(joint, "child", {"link": "base_link"})
        ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        self.assertEqual(check_contract(root, _design()), [])

    def test_require_contract_raises_with_all_violations(self) -> None:
        raw = two_arm_design()
        raw["pan_tilt"] = None
        with self.assertRaises(ContractError) as caught:
            require_contract(parse_urdf(two_arm_urdf()), design_from_mapping(raw, source="robot.urdf"))
        self.assertEqual(len(caught.exception.violations), 2)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_design_contract.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'tinker_designs.contract'`

- [ ] **Step 3: Implement**

```python
# tinker_designs/contract.py
"""Spec §5: what a candidate URDF must satisfy given its design.yaml roles."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from .model import Inertial, Joint, Link, apply, fixed_subtree, joints, links, movable_joints, parent_joint, root_links
from .schema import Design

GROUND_TOLERANCE_M = 0.001


class ContractError(ValueError):
    def __init__(self, violations: list[str]) -> None:
        super().__init__("contract violations:\n  " + "\n  ".join(violations))
        self.violations = violations


def _axis_is(joint: Joint, axis_index: int) -> bool:
    norm = math.sqrt(sum(c * c for c in joint.axis)) or 1.0
    return abs(joint.axis[axis_index] / norm) > 0.99


def _positive_definite(inertial: Inertial) -> bool:
    a, b, c = inertial.ixx, inertial.iyy, inertial.izz
    d, e, f = inertial.ixy, inertial.ixz, inertial.iyz
    minor2 = a * b - d * d
    det = a * (b * c - f * f) - d * (d * c - f * e) + e * (d * f - b * e)
    return a > 0 and minor2 > 0 and det > 0


def _link_origin_in_base(root_map: dict[str, Joint], link: str, base: str, angle_zero: bool = True) -> tuple[float, float, float] | None:
    """Position of `link`'s frame origin in `base`'s frame at zero joint angles."""
    point = (0.0, 0.0, 0.0)
    current = link
    guard = 0
    while current != base:
        joint = root_map.get(current)
        if joint is None or guard > 256:
            return None
        point = apply(joint.origin, point)
        current = joint.parent
        guard += 1
    return point


def _wheel_radius(link: Link) -> float | None:
    for _, geometry in link.collisions:
        if geometry.kind == "cylinder" and geometry.radius:
            return geometry.radius
        if geometry.kind == "sphere" and geometry.radius:
            return geometry.radius
    return None


def check_contract(root: ET.Element, design: Design) -> list[str]:
    violations: list[str] = []
    link_index = links(root)
    joint_index = joints(root)
    by_child = parent_joint(root)

    # Root: base_frame, or world -> base_frame via a zero fixed joint.
    roots = root_links(root)
    if roots == ["world"]:
        world_joint = by_child.get(design.base_frame)
        if world_joint is None or world_joint.parent != "world" or world_joint.type != "fixed" or any(abs(v) > 1e-9 for v in world_joint.origin.xyz + world_joint.origin.rpy):
            violations.append(f"world must connect to {design.base_frame} by a zero fixed joint")
    elif roots != [design.base_frame]:
        violations.append(f"URDF root must be {design.base_frame!r}, found {roots}")
    if design.base_frame not in link_index:
        violations.append(f"base_frame {design.base_frame!r} is not a link")
        return violations

    # Every movable joint claimed exactly once; every claim exists.
    claims = design.claimed_joints()
    for name in movable_joints(root):
        if name not in claims:
            violations.append(f"movable joint {name!r} is unclaimed by design.yaml")
    for name, role in claims.items():
        if name not in joint_index:
            violations.append(f"{role} joint {name!r} is not in the URDF")
        elif joint_index[name].type == "fixed":
            violations.append(f"{role} joint {name!r} is fixed")
    if violations:
        return violations

    fixed_from_base = fixed_subtree(root, design.base_frame)

    # Wheels.
    contact_heights: list[tuple[str, float]] = []
    for name in design.wheels.driven:
        joint = joint_index[name]
        if joint.type != "continuous":
            violations.append(f"driven wheel {name!r} must be continuous")
        if not _axis_is(joint, 1):
            violations.append(f"driven wheel {name!r} axis must be ±Y")
        if joint.parent not in fixed_from_base:
            violations.append(f"driven wheel {name!r} must hang off the fixed chassis subtree")
        radius = _wheel_radius(link_index[joint.child])
        centre = _link_origin_in_base(by_child, joint.child, design.base_frame)
        if radius is None or centre is None:
            violations.append(f"driven wheel {name!r} needs a cylinder/sphere collision on {joint.child!r}")
        else:
            contact_heights.append((name, centre[2] - radius))
    for swivel_name, wheel_name in zip(design.wheels.caster_swivel, design.wheels.caster_wheel):
        swivel = joint_index[swivel_name]
        wheel = joint_index[wheel_name]
        if swivel.type != "continuous" or not _axis_is(swivel, 2):
            violations.append(f"caster swivel {swivel_name!r} must be continuous about ±Z")
        if swivel.parent not in fixed_from_base:
            violations.append(f"caster swivel {swivel_name!r} must hang off the fixed chassis subtree")
        if wheel.type != "continuous" or not _axis_is(wheel, 1):
            violations.append(f"caster wheel {wheel_name!r} must be continuous about ±Y")
        if wheel.parent != swivel.child:
            violations.append(f"caster wheel {wheel_name!r} must be the child of swivel {swivel_name!r}")
        radius = _wheel_radius(link_index[wheel.child])
        centre = _link_origin_in_base(by_child, wheel.child, design.base_frame)
        if radius is None or centre is None:
            violations.append(f"caster wheel {wheel_name!r} needs a cylinder/sphere collision on {wheel.child!r}")
        else:
            contact_heights.append((wheel_name, centre[2] - radius))
    if contact_heights:
        lowest = min(height for _, height in contact_heights)
        for name, height in contact_heights:
            if abs(height - lowest) > GROUND_TOLERANCE_M:
                violations.append(f"wheel {name!r} ground contact z={height:.4f} differs from {lowest:.4f}; wheels must share a ground plane")

    # Arms.
    for arm in design.arms:
        if arm.mount not in link_index:
            violations.append(f"arm {arm.name!r} mount {arm.mount!r} is not a link")
            continue
        if arm.mount not in fixed_from_base:
            violations.append(f"arm {arm.name!r} mount {arm.mount!r} must be fixed to the chassis (fixed joints only from {design.base_frame})")
        expected_parent = arm.mount
        for name in arm.joints:
            joint = joint_index[name]
            if joint.parent != expected_parent:
                violations.append(f"arm {arm.name!r} joints must form a serial chain from {arm.mount!r}; {name!r} hangs off {joint.parent!r}, expected {expected_parent!r}")
                break
            expected_parent = joint.child
        last_link = joint_index[arm.joints[-1]].child
        if arm.gripper is not None:
            drive = joint_index[arm.gripper.drive]
            chain_link = drive.parent
            guard = 0
            while chain_link not in (last_link, design.base_frame) and chain_link in by_child and guard < 64:
                chain_link = by_child[chain_link].parent
                guard += 1
            if chain_link != last_link:
                violations.append(f"arm {arm.name!r} gripper drive {arm.gripper.drive!r} must descend from {last_link!r}")
            for mimic_name in arm.gripper.mimics:
                if joint_index[mimic_name].mimic != arm.gripper.drive:
                    violations.append(f"gripper mimic {mimic_name!r} must carry <mimic joint={arm.gripper.drive!r}>")

    # Sensors.
    for sensor in design.sensors:
        if sensor.frame not in link_index:
            violations.append(f"sensor {sensor.type!r} frame {sensor.frame!r} is not a link")

    # Inertia sanity.
    for link in link_index.values():
        if link.inertial is None:
            continue
        if link.inertial.mass <= 0:
            violations.append(f"link {link.name!r} mass must be > 0")
        elif not _positive_definite(link.inertial):
            violations.append(f"link {link.name!r} inertia tensor is not positive definite")
    return violations


def require_contract(root: ET.Element, design: Design) -> None:
    violations = check_contract(root, design)
    if violations:
        raise ContractError(violations)
```

- [ ] **Step 4: Run to verify pass**

Run: `scripts/pytest-clean tests/test_design_contract.py -q`
Expected: 12 passed

- [ ] **Step 5: Commit**

```bash
git add tinker_designs/contract.py tests/test_design_contract.py
git commit -m "feat(designs): role-driven structural contract check"
```

---

### Task 5: `tinker_designs.derive` — profile from URDF + roles

**Files:**
- Create: `tinker_designs/derive.py`
- Test: `tests/test_design_derive.py`

**Interfaces:**
- Consumes: `model.*`, `schema.Design`, `contract` (assumes the contract already passed)
- Produces:
  ```python
  class DeriveError(ValueError): ...
  def derive_profile(root: ET.Element, design: Design) -> dict
  # {
  #   "robot", "kinematics", "base_frame",
  #   "wheels": {"driven": [...], "caster_swivel": [...], "caster_wheel": [...], "radius_m", "track_m"},
  #   "arms": [{"name","mount","joints","gripper":{...}|None,"drive":{"stiffness","damping"},"reach_m","estimated"}],
  #   "pan_tilt": {"joints":[...]} | None,
  #   "sensors": [{"type","frame"}],
  #   "footprint": [[x,y],...], "footprint_source": "derived"|"override",
  #   "mass_kg", "cog_base_link": [x,y,z], "cog_arms_extended": [x,y,z],
  # }
  def convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]   # monotone chain, CCW
  ```

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_design_derive.py
from __future__ import annotations

import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from design_fixtures import two_arm_design, two_arm_urdf
from tinker_designs.derive import DeriveError, convex_hull, derive_profile
from tinker_designs.model import parse_urdf
from tinker_designs.schema import design_from_mapping


class DeriveTest(unittest.TestCase):
    def setUp(self) -> None:
        self.root = parse_urdf(two_arm_urdf())
        self.design = design_from_mapping(two_arm_design(), source="robot.urdf")

    def test_wheel_geometry(self) -> None:
        profile = derive_profile(self.root, self.design)
        self.assertAlmostEqual(profile["wheels"]["radius_m"], 0.06)
        self.assertAlmostEqual(profile["wheels"]["track_m"], 0.4)
        self.assertEqual(profile["wheels"]["driven"], ["front_left_wheel_joint", "front_right_wheel_joint"])

    def test_mass_and_cog(self) -> None:
        profile = derive_profile(self.root, self.design)
        self.assertAlmostEqual(profile["mass_kg"], 32.05)
        x, y, z = profile["cog_base_link"]
        self.assertAlmostEqual(y, 0.0, places=6)          # symmetric fixture (gripper 0.2 kg breaks symmetry slightly)
        self.assertGreater(x, -0.05)
        self.assertLess(x, 0.15)
        self.assertGreater(z, 0.0)

    def test_footprint_is_hull_of_primitives_and_wheels(self) -> None:
        profile = derive_profile(self.root, self.design)
        self.assertEqual(profile["footprint_source"], "derived")
        xs = [x for x, _ in profile["footprint"]]
        ys = [y for _, y in profile["footprint"]]
        self.assertAlmostEqual(max(xs), 0.25)            # chassis box front
        self.assertAlmostEqual(min(xs), -0.36)           # rear caster wheel: -0.3 - 0.02 - 0.04
        self.assertAlmostEqual(max(ys), 0.225)           # front wheel: 0.2 + 0.05/2
        self.assertAlmostEqual(min(ys), -0.225)

    def test_footprint_override_wins_when_present(self) -> None:
        raw = two_arm_design()
        raw["footprint"] = [[0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1]]
        profile = derive_profile(self.root, design_from_mapping(raw, source="robot.urdf"))
        self.assertEqual(profile["footprint_source"], "override")
        self.assertEqual(profile["footprint"], [[0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1]])

    def test_mesh_chassis_without_override_is_an_error(self) -> None:
        for link in self.root.findall("link"):
            if link.get("name") == "base_link":
                for collision in link.findall("collision"):
                    geometry = collision.find("geometry")
                    geometry.remove(geometry.find("box"))
                    ET.SubElement(geometry, "mesh", {"filename": "package://x/body.stl"})
        with self.assertRaisesRegex(DeriveError, "footprint"):
            derive_profile(self.root, self.design)

    def test_arm_reach_and_extended_cog(self) -> None:
        profile = derive_profile(self.root, self.design)
        left = profile["arms"][0]
        self.assertEqual(left["name"], "left")
        self.assertAlmostEqual(left["reach_m"], 0.45, places=3)   # j2 = -1.57 points link2 (0.3 m) straight up
        self.assertEqual(left["gripper"], {"drive": "l_grip", "mimics": ["l_finger"]})
        self.assertIsNone(profile["arms"][1]["gripper"])
        self.assertGreater(profile["cog_arms_extended"][2], profile["cog_base_link"][2])

    def test_convex_hull_is_ccw_and_drops_interior(self) -> None:
        hull = convex_hull([(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5), (0, 0)])
        self.assertEqual(hull, [(0, 0), (1, 0), (1, 1), (0, 1)])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_design_derive.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'tinker_designs.derive'`

- [ ] **Step 3: Implement**

```python
# tinker_designs/derive.py
"""Derive the robot profile (kinematics, footprint, mass, reach) from a URDF + roles."""
from __future__ import annotations

import itertools
import math
import xml.etree.ElementTree as ET

from .model import Joint, Link, Origin, Vec3, apply, compose, compose_rotation, fixed_subtree, joints, links, parent_joint, rotate_axis_angle
from .schema import Design


class DeriveError(ValueError):
    """The profile cannot be derived from this URDF + design.yaml."""


def convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    unique = sorted(set(points))
    if len(unique) <= 2:
        return unique

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _frame_in_base(by_child: dict[str, Joint], link: str, base: str, angles: dict[str, float] | None = None) -> Origin:
    """Pose of `link` in `base` with the given joint angles (zero for unlisted joints)."""
    chain: list[Joint] = []
    current = link
    while current != base:
        joint = by_child.get(current)
        if joint is None:
            raise DeriveError(f"link {link!r} is not connected to {base!r}")
        chain.append(joint)
        current = joint.parent
    pose = Origin()
    for joint in reversed(chain):
        pose = compose(pose, joint.origin)
        angle = (angles or {}).get(joint.name, 0.0)
        if angle:
            pose = compose_rotation(pose, rotate_axis_angle(joint.axis, angle))
    return pose


def _radius(link: Link) -> float:
    for _, geometry in link.collisions:
        if geometry.kind in ("cylinder", "sphere") and geometry.radius:
            return geometry.radius
    raise DeriveError(f"wheel link {link.name!r} has no cylinder/sphere collision")


def _primitive_corners(origin: Origin, geometry) -> list[Vec3]:
    if geometry.kind == "box" and geometry.size:
        hx, hy, hz = (component / 2 for component in geometry.size)
        return [apply(origin, (sx * hx, sy * hy, sz * hz)) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    if geometry.kind == "cylinder" and geometry.radius is not None and geometry.length is not None:
        corners = []
        for angle in (k * math.pi / 8 for k in range(16)):
            for sz in (-1, 1):
                corners.append(apply(origin, (geometry.radius * math.cos(angle), geometry.radius * math.sin(angle), sz * geometry.length / 2)))
        return corners
    if geometry.kind == "sphere" and geometry.radius is not None:
        r = geometry.radius
        return [apply(origin, (sx * r, sy * r, 0.0)) for sx in (-1, 1) for sy in (-1, 1)]
    return []


def _footprint(root: ET.Element, design: Design, link_index: dict[str, Link], by_child: dict[str, Joint]) -> tuple[list[list[float]], str]:
    if design.footprint is not None:
        return [[x, y] for x, y in design.footprint], "override"
    points: list[tuple[float, float]] = []
    chassis = fixed_subtree(root, design.base_frame)
    for name in chassis:
        pose = _frame_in_base(by_child, name, design.base_frame)
        for origin, geometry in link_index[name].collisions:
            for corner in _primitive_corners(compose(pose, origin), geometry):
                points.append((corner[0], corner[1]))
    joint_index = joints(root)
    for name in design.wheels.driven + design.wheels.caster_wheel:
        child = joint_index[name].child
        pose = _frame_in_base(by_child, child, design.base_frame)
        for origin, geometry in link_index[child].collisions:
            for corner in _primitive_corners(compose(pose, origin), geometry):
                points.append((corner[0], corner[1]))
    if not points:
        raise DeriveError("no primitive collision geometry to derive a footprint from; add `footprint:` to design.yaml")
    hull = convex_hull(points)
    if len(hull) < 3:
        raise DeriveError("derived footprint is degenerate; add `footprint:` to design.yaml")
    return [[round(x, 6), round(y, 6)] for x, y in hull], "derived"


def _mass_and_cog(link_index: dict[str, Link], by_child: dict[str, Joint], base: str, angles: dict[str, float]) -> tuple[float, Vec3]:
    total = 0.0
    weighted = [0.0, 0.0, 0.0]
    for link in link_index.values():
        if link.inertial is None or link.inertial.mass <= 0:
            continue
        if link.name != base and link.name not in by_child:
            continue  # e.g. the `world` link
        pose = _frame_in_base(by_child, link.name, base, angles)
        centre = apply(pose, link.inertial.origin.xyz)
        total += link.inertial.mass
        for axis in range(3):
            weighted[axis] += link.inertial.mass * centre[axis]
    if total <= 0:
        raise DeriveError("robot has no mass")
    return total, (weighted[0] / total, weighted[1] / total, weighted[2] / total)


def _reach(arm, joint_index: dict[str, Joint], by_child: dict[str, Joint], base: str) -> tuple[float, dict[str, float]]:
    mount = _frame_in_base(by_child, arm.mount, base).xyz
    last_link = joint_index[arm.joints[-1]].child
    corners = []
    for name in arm.joints:
        joint = joint_index[name]
        lower = joint.lower if joint.lower is not None else -math.pi
        upper = joint.upper if joint.upper is not None else math.pi
        corners.append((lower, 0.0, upper))
    best, best_angles = 0.0, {}
    for combination in itertools.product(*corners):
        angles = dict(zip(arm.joints, combination))
        tip = _frame_in_base(by_child, last_link, base, angles).xyz
        distance = math.dist(tip, mount)
        if distance > best:
            best, best_angles = distance, angles
    return best, best_angles


def derive_profile(root: ET.Element, design: Design) -> dict:
    link_index = links(root)
    joint_index = joints(root)
    by_child = parent_joint(root)
    left, right = (joint_index[name] for name in design.wheels.driven)
    radii = {_radius(link_index[joint.child]) for joint in (left, right)}
    if len(radii) != 1:
        raise DeriveError(f"driven wheels have different radii: {sorted(radii)}")
    centres = [_frame_in_base(by_child, joint.child, design.base_frame).xyz for joint in (left, right)]
    track = abs(centres[0][1] - centres[1][1])
    footprint, footprint_source = _footprint(root, design, link_index, by_child)
    mass, cog = _mass_and_cog(link_index, by_child, design.base_frame, {})
    arms = []
    extended_angles: dict[str, float] = {}
    for arm in design.arms:
        reach, angles = _reach(arm, joint_index, by_child, design.base_frame)
        extended_angles.update(angles)
        arms.append({
            "name": arm.name, "mount": arm.mount, "joints": list(arm.joints),
            "gripper": None if arm.gripper is None else {"drive": arm.gripper.drive, "mimics": list(arm.gripper.mimics)},
            "drive": {"stiffness": arm.stiffness, "damping": arm.damping},
            "reach_m": round(reach, 6), "estimated": arm.estimated,
        })
    _, cog_extended = _mass_and_cog(link_index, by_child, design.base_frame, extended_angles)
    return {
        "robot": design.name,
        "kinematics": design.kinematics,
        "base_frame": design.base_frame,
        "wheels": {
            "driven": list(design.wheels.driven),
            "caster_swivel": list(design.wheels.caster_swivel),
            "caster_wheel": list(design.wheels.caster_wheel),
            "radius_m": round(radii.pop(), 6),
            "track_m": round(track, 6),
        },
        "arms": arms,
        "pan_tilt": {"joints": list(design.pan_tilt)} if design.pan_tilt else None,
        "sensors": [{"type": sensor.type, "frame": sensor.frame} for sensor in design.sensors],
        "footprint": footprint,
        "footprint_source": footprint_source,
        "mass_kg": round(mass, 6),
        "cog_base_link": [round(component, 6) for component in cog],
        "cog_arms_extended": [round(component, 6) for component in cog_extended],
    }
```

- [ ] **Step 4: Run to verify pass**

Run: `scripts/pytest-clean tests/test_design_derive.py -q`
Expected: 7 passed. If `test_mass_and_cog`'s x bound fails, print the CoG and adjust the *bound* only if the arithmetic (base at 0, arms at +0.1, casters at −0.3) justifies it — do not weaken to `assertTrue(True)`.

- [ ] **Step 5: Commit**

```bash
git add tinker_designs/derive.py tests/test_design_derive.py
git commit -m "feat(designs): derive wheel geometry, footprint, mass, CoG and arm reach"
```

---

### Task 6: `tinker_designs.heuristics` — draft design.yaml for `--init`

**Files:**
- Create: `tinker_designs/heuristics.py`
- Test: `tests/test_design_heuristics.py`

**Interfaces:**
- Produces: `def draft_design(root: ET.Element, name: str) -> dict` — a mapping that `design_from_mapping` accepts; heuristics: base_frame = the single root link (or the child of `world`); driven wheels = continuous ±Y joints whose parent is in the base fixed subtree, preferring names containing `front`; caster swivels = continuous ±Z joints off the base subtree, caster wheels = their continuous ±Y children; arms = for every fixed link off the base subtree that has a revolute child, follow the serial chain of revolute joints while each link has exactly one revolute child (chain ≥ 3 → arm; name from the mount link with `_arm_base`/`_base`/`link_base` stripped, `arm` if empty); gripper = first revolute joint below the chain's last link plus every joint that mimics it; pan_tilt = the pair of revolute joints named `pan_joint`/`tilt_joint` if present; sensors = links whose names contain `livox` (`livox_mid360`) or `head_camera_link` (`head_camera`). Unclassified movable joints are left unclaimed on purpose so the contract reports them.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_design_heuristics.py
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from design_fixtures import two_arm_design, two_arm_urdf
from tinker_designs.contract import check_contract
from tinker_designs.heuristics import draft_design
from tinker_designs.model import parse_urdf
from tinker_designs.schema import design_from_mapping


class HeuristicsTest(unittest.TestCase):
    def test_draft_reproduces_fixture_roles(self) -> None:
        root = parse_urdf(two_arm_urdf())
        draft = draft_design(root, "two_arm_fixture")
        expected = two_arm_design()
        self.assertEqual(draft["wheels"], expected["wheels"])
        self.assertEqual([arm["name"] for arm in draft["arms"]], ["left", "right"])
        self.assertEqual(draft["arms"][0]["joints"], ["l_j1", "l_j2", "l_j3"])
        self.assertEqual(draft["arms"][0]["gripper"], {"drive": "l_grip", "mimics": ["l_finger"]})
        self.assertIsNone(draft["arms"][1]["gripper"])
        self.assertEqual(draft["pan_tilt"], {"joints": ["pan_joint", "tilt_joint"]})
        self.assertEqual(draft["sensors"], [{"type": "livox_mid360", "frame": "livox_frame"}])
        design = design_from_mapping(draft, source="robot.urdf")
        self.assertEqual(check_contract(root, design), [])

    def test_draft_leaves_unknown_joints_unclaimed(self) -> None:
        root = parse_urdf(two_arm_urdf())
        import xml.etree.ElementTree as ET
        link = ET.SubElement(root, "link", {"name": "lift"})
        ET.SubElement(ET.SubElement(link, "inertial"), "mass", {"value": "1"})
        joint = ET.SubElement(root, "joint", {"name": "lift_joint", "type": "prismatic"})
        ET.SubElement(joint, "parent", {"link": "base_link"})
        ET.SubElement(joint, "child", {"link": "lift"})
        ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
        ET.SubElement(joint, "limit", {"lower": "0", "upper": "0.3", "effort": "1", "velocity": "1"})
        draft = draft_design(root, "two_arm_fixture")
        design = design_from_mapping(draft, source="robot.urdf")
        violations = check_contract(root, design)
        self.assertTrue(any("lift_joint" in item and "unclaimed" in item for item in violations), violations)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_design_heuristics.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'tinker_designs.heuristics'`

- [ ] **Step 3: Implement**

```python
# tinker_designs/heuristics.py
"""Draft a design.yaml from naming/topology conventions (design_import.py --init)."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from .model import Joint, fixed_subtree, joints, links, root_links


def _axis_is(joint: Joint, index: int) -> bool:
    norm = math.sqrt(sum(c * c for c in joint.axis)) or 1.0
    return abs(joint.axis[index] / norm) > 0.99


def _arm_name(mount: str) -> str:
    for suffix in ("_arm_base_link", "_arm_base", "_base_link", "_base", "link_base"):
        if mount.endswith(suffix) and mount != suffix:
            return mount[: -len(suffix)]
    return "arm" if mount in ("link_base", "arm_base") else mount


def draft_design(root: ET.Element, name: str) -> dict:
    link_index = links(root)
    joint_index = joints(root)
    roots = root_links(root)
    base_frame = roots[0] if roots else "base_link"
    if base_frame == "world":
        base_frame = next((joint.child for joint in joint_index.values() if joint.parent == "world"), "base_link")
    chassis = fixed_subtree(root, base_frame)
    by_parent: dict[str, list[Joint]] = {}
    for joint in joint_index.values():
        by_parent.setdefault(joint.parent, []).append(joint)

    driven = sorted(
        (j.name for j in joint_index.values() if j.type == "continuous" and _axis_is(j, 1) and j.parent in chassis),
        key=lambda n: (0 if "front" in n else 1, n),
    )[:2]
    swivels = sorted(j.name for j in joint_index.values() if j.type == "continuous" and _axis_is(j, 2) and j.parent in chassis)
    caster_wheels = []
    for swivel_name in swivels:
        child = joint_index[swivel_name].child
        wheel = next((j.name for j in by_parent.get(child, []) if j.type == "continuous" and _axis_is(j, 1)), None)
        caster_wheels.append(wheel or "")

    arms = []
    for mount in sorted(chassis):
        if mount == base_frame:
            continue
        revolute_children = [j for j in by_parent.get(mount, []) if j.type == "revolute"]
        if len(revolute_children) != 1:
            continue
        chain = [revolute_children[0]]
        while True:
            nxt = [j for j in by_parent.get(chain[-1].child, []) if j.type == "revolute" and j.mimic is None]
            if len(nxt) != 1:
                break
            chain.append(nxt[0])
        if len(chain) < 3:
            continue
        # Split the gripper off: the last revolute whose child has a mimic sibling, or the last joint if ≥4 and named like a gripper.
        gripper = None
        arm_joints = chain
        drive_candidates = [j for j in chain[1:] if any(m.mimic == j.name for m in joint_index.values())]
        if drive_candidates:
            drive = drive_candidates[0]
            arm_joints = chain[: chain.index(drive)]
            gripper = {"drive": drive.name, "mimics": sorted(m.name for m in joint_index.values() if m.mimic == drive.name)}
        elif any(token in chain[-1].name for token in ("grip", "finger", "hand")):
            gripper = {"drive": chain[-1].name, "mimics": []}
            arm_joints = chain[:-1]
        arms.append({
            "name": _arm_name(mount), "mount": mount, "joints": [j.name for j in arm_joints],
            "gripper": gripper, "drive": {"stiffness": 400.0, "damping": 40.0},
        })

    pan_tilt = {"joints": ["pan_joint", "tilt_joint"]} if {"pan_joint", "tilt_joint"} <= set(joint_index) else None
    sensors = []
    for link_name in sorted(link_index):
        if "livox" in link_name:
            sensors.append({"type": "livox_mid360", "frame": link_name})
        elif link_name == "head_camera_link":
            sensors.append({"type": "head_camera", "frame": link_name})
    return {
        "name": name,
        "kinematics": "diff_drive",
        "base_frame": base_frame,
        "wheels": {"driven": driven, "caster_swivel": swivels, "caster_wheel": caster_wheels},
        "arms": arms,
        "pan_tilt": pan_tilt,
        "sensors": sensors,
    }
```

- [ ] **Step 4: Run to verify pass**

Run: `scripts/pytest-clean tests/test_design_heuristics.py -q`
Expected: 2 passed

- [ ] **Step 5: Commit**

```bash
git add tinker_designs/heuristics.py tests/test_design_heuristics.py
git commit -m "feat(designs): draft design.yaml roles from URDF conventions"
```

---

### Task 7: `workspace.py` refactor — `publish_robot_artifact`, `mount_origin`, `robot` in source lock

**Files:**
- Modify: `tools/tinker_sim_deploy/workspace.py` (`_normalized_source_lock` ~233, `canonicalize_urdf` 569, `_ensure_mount_topology` 466-479, `_validate_canonical_root` 539-544, `_export_tinker2_locked` 661-790)
- Test: `tests/test_design_publish.py` (new); `tests/test_artifact_export.py` and `tests/test_workspace.py` must still pass.

**Interfaces:**
- Produces (in `workspace.py`):
  ```python
  def _normalized_source_lock(records, *, robot: str = "tinker2") -> bytes
  def canonicalize_urdf(data: bytes, *, mount_origin: tuple[float, float, float] = _ARM_MOUNT_ORIGIN) -> bytes
  def publish_robot_artifact(
      artifacts: Path, *, robot: str, file_bytes: dict[str, bytes], canonical_urdf: bytes,
      source_lock_bytes: bytes, canonicalizer: str, manifest_extra: dict[str, object],
      source_path: str, source_sha256: str,
  ) -> ExportResult
  ```
  `publish_robot_artifact` contains everything `_export_tinker2_locked` did from `payload_hashes = ...` to `return ExportResult(...)`, with `"tinker2"` replaced by `robot` and the `canonicalization`/`provenance`/`kinematics` manifest blocks supplied by the caller through `manifest_extra` (the tinker2 caller passes exactly the dict it builds today, so the tinker2 manifest bytes are unchanged). Keys with `/` in `file_bytes` (e.g. `meshes/arm.stl`) are written under sub-directories (`target.parent.mkdir(parents=True, exist_ok=True)`), and `_path_parts_are_safe` is applied to each key. Requires `robot` to match `^[a-z0-9][a-z0-9_-]{0,63}$`.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_design_publish.py
from __future__ import annotations

import hashlib
import json
import re
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.workspace import (
    ARTIFACT_FILES, PUBLICATION_SCHEMA, ArtifactPublicationError, UnsafePathError, _normalized_source_lock,
    artifact_identity, canonicalize_urdf, publish_robot_artifact,
)


def _publish(artifacts: Path, robot: str = "demo", urdf: bytes = b"<robot name='d'/>\n") -> tuple[Path, dict]:
    lock = _normalized_source_lock([{"path": "designs/demo/robot.urdf", "size": len(urdf), "sha256": hashlib.sha256(urdf).hexdigest()}], robot=robot)
    result = publish_robot_artifact(
        artifacts, robot=robot,
        file_bytes={"robot.urdf": urdf, "robot.usd": b"usd", "robot-profile.yaml": b"robot: demo\n", "meshes/a.stl": b"solid"},
        canonical_urdf=urdf, source_lock_bytes=lock, canonicalizer="tinker-designs-canonical-v1",
        manifest_extra={"kinematics": {"wheel_radius_m": 0.1}}, source_path="designs/demo/robot.urdf",
        source_sha256=hashlib.sha256(urdf).hexdigest(),
    )
    return result.artifact_dir, result.manifest


class PublishRobotArtifactTest(unittest.TestCase):
    def test_publishes_under_robot_family_with_pointer(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifacts = Path(temporary) / "artifacts"
            artifact_dir, manifest = _publish(artifacts)
            self.assertTrue(re.fullmatch(r"[0-9a-f]{64}", artifact_dir.name))
            self.assertEqual(artifact_dir.parent, artifacts / "robot" / "demo")
            self.assertEqual(manifest["robot"], "demo")
            self.assertEqual(manifest["schema_version"], PUBLICATION_SCHEMA)
            self.assertEqual(manifest["kinematics"], {"wheel_radius_m": 0.1})
            self.assertEqual(manifest["canonicalization"]["algorithm"], "tinker-designs-canonical-v1")
            self.assertEqual({item["path"] for item in manifest["files"]}, {f"artifacts/robot/demo/{artifact_dir.name}/{n}" for n in ("robot.urdf", "robot.usd", "robot-profile.yaml", "meshes/a.stl")})
            self.assertTrue((artifact_dir / "meshes" / "a.stl").is_file())
            current = json.loads((artifacts / "robot" / "demo" / "current.json").read_text())
            self.assertEqual(current["robot"], "demo")
            self.assertEqual(current["artifact_dir"], f"artifacts/robot/demo/{artifact_dir.name}")
            self.assertEqual(current["robot_urdf_sha256"], hashlib.sha256(b"<robot name='d'/>\n").hexdigest())

    def test_identity_is_content_addressed_and_republish_is_idempotent(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifacts = Path(temporary) / "artifacts"
            first, _ = _publish(artifacts)
            second, _ = _publish(artifacts)
            self.assertEqual(first, second)
            third, _ = _publish(artifacts, urdf=b"<robot name='e'/>\n")
            self.assertNotEqual(first, third)

    def test_robot_name_and_file_keys_are_validated(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifacts = Path(temporary) / "artifacts"
            with self.assertRaises(ArtifactPublicationError):
                _publish(artifacts, robot="Bad Name")
            lock = _normalized_source_lock([], robot="demo")
            with self.assertRaises(UnsafePathError):
                publish_robot_artifact(
                    artifacts, robot="demo", file_bytes={"../escape": b"x", "robot.urdf": b"<robot/>"},
                    canonical_urdf=b"<robot/>", source_lock_bytes=lock, canonicalizer="c", manifest_extra={},
                    source_path="p", source_sha256="0" * 64,
                )

    def test_source_lock_carries_robot(self) -> None:
        lock = json.loads(_normalized_source_lock([], robot="demo"))
        self.assertEqual(lock["robot"], "demo")
        self.assertEqual(json.loads(_normalized_source_lock([]))["robot"], "tinker2")


class CanonicalizeMountOriginTest(unittest.TestCase):
    def test_mount_origin_parameter_is_honoured(self) -> None:
        sys.path.insert(0, str(ROOT / "tests"))
        from test_artifact_export import _fixture_urdf
        moved = _fixture_urdf().replace(b'xyz="-0.03 0 0.527"', b'xyz="0.1 0 0.6"')
        with self.assertRaises(Exception):
            canonicalize_urdf(moved)
        canonical = canonicalize_urdf(moved, mount_origin=(0.1, 0.0, 0.6))
        self.assertIn(b'xyz="0.1 0 0.6"', canonical)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_design_publish.py -q`
Expected: FAIL — `ImportError: cannot import name 'publish_robot_artifact'`

- [ ] **Step 3: Implement the refactor**

In `workspace.py`:

1. `_normalized_source_lock(records: list[dict[str, object]], *, robot: str = "tinker2") -> bytes` — replace the `"robot": "tinker2"` literal with `robot`.
2. Thread `mount_origin` through the canonicalizer:
   ```python
   def _ensure_mount_topology(root: ET.Element, mount_origin: tuple[float, float, float] = _ARM_MOUNT_ORIGIN) -> None:
       ...  # every `_ARM_MOUNT_ORIGIN` inside becomes `mount_origin` (lines 482, 486, 492, 494)

   def _validate_canonical_root(root: ET.Element, mount_origin: tuple[float, float, float] = _ARM_MOUNT_ORIGIN) -> None:
       ...  # line 542's `_ARM_MOUNT_ORIGIN` becomes `mount_origin`

   def canonicalize_urdf(data: bytes, *, mount_origin: tuple[float, float, float] = _ARM_MOUNT_ORIGIN) -> bytes:
       root = _parse_urdf(data)
       ...
       _ensure_mount_topology(root, mount_origin)
       _ensure_drive_control(root)
       _validate_canonical_root(root, mount_origin)
       ...
   ```
   Also update `_set_origin`'s caller at line 326 (`"-0.03 0 0.527" if xyz == _ARM_MOUNT_ORIGIN else "0 0 0"`) to format any origin: `" ".join(_format_number(v) for v in xyz)` where `_format_number` renders `0.0 -> "0"`, `-0.03 -> "-0.03"`, `0.527 -> "0.527"` (`repr` trimmed of trailing `.0`). Verify `tests/test_artifact_export.py::CanonicalUrdfTest::test_determinism_idempotence_and_semantic_preservation` still passes — it pins today's bytes.
3. Extract the publisher:
   ```python
   _ROBOT_NAME = re.compile(r"[a-z0-9][a-z0-9_-]{0,63}")

   def publish_robot_artifact(
       artifacts: Path, *, robot: str, file_bytes: dict[str, bytes], canonical_urdf: bytes,
       source_lock_bytes: bytes, canonicalizer: str, manifest_extra: dict[str, object],
       source_path: str, source_sha256: str,
   ) -> ExportResult:
       if not _ROBOT_NAME.fullmatch(robot):
           raise ArtifactPublicationError(f"invalid robot name: {robot!r}")
       for name in file_bytes:
           _path_parts_are_safe(Path(name), "artifact payload name")
       artifacts = _safe_dir(artifacts, "artifacts root", create=True)
       payload_hashes = {name: hashlib.sha256(data).hexdigest() for name, data in file_bytes.items()}
       digest = artifact_identity(payload_hashes, canonical_urdf, source_lock_bytes, canonicalizer)
       artifact_root = artifacts / "robot" / robot
       _safe_dir(artifact_root, "artifact root", create=True)
       with _publication_lock(artifact_root):
           _recover_staging(artifact_root)
           destination = artifact_root / digest
           if destination.exists() and (destination.is_symlink() or not destination.is_dir()):
               raise ArtifactPublicationError(f"content-addressed artifact path is unsafe: {destination}")
           manifest: dict[str, object] = {
               "schema_version": PUBLICATION_SCHEMA,
               "robot": robot,
               "artifact_id": digest,
               "source_lock": f"artifacts/robot/{robot}/{digest}/source-lock.json",
               "qualification": manifest_extra.pop("qualification", "blocked_calibration_missing"),
               "files": [{"path": f"artifacts/robot/{robot}/{digest}/{name}", "sha256": payload_hashes[name]} for name in sorted(file_bytes)],
               "canonicalization": {
                   "algorithm": canonicalizer,
                   "source_path": source_path,
                   "source_sha256": source_sha256,
                   "output_sha256": payload_hashes["robot.urdf"],
               },
           }
           manifest.update(manifest_extra)
           # ... the existing body from `manifest_bytes = ...` through `_atomic_write(current, current_bytes)`,
           # with every f"artifacts/robot/tinker2/..." replaced by f"artifacts/robot/{robot}/..." and
           # `target.parent.mkdir(parents=True, exist_ok=True)` before each `target.write_bytes(data)`.
           return ExportResult(destination, manifest)
   ```
   Then `_export_tinker2_locked` ends with:
   ```python
       extra = {
           "canonicalization": {  # merged: keep tinker2's extra keys
               "algorithm": CANONICALIZER_ALGORITHM,
               "source_path": source_paths["robot.urdf"],
               "source_sha256": hashlib.sha256(source_data["robot.urdf"]).hexdigest(),
               "source_lock_record": next(record for record in current_records if record["path"] == source_paths["robot.urdf"]),
               "output_sha256": hashlib.sha256(canonical_urdf).hexdigest(),
           },
           "provenance": { ...unchanged... },
           "kinematics": { ...unchanged literals... },
       }
       return publish_robot_artifact(
           artifacts, robot="tinker2", file_bytes=file_bytes, canonical_urdf=canonical_urdf,
           source_lock_bytes=source_lock_bytes, canonicalizer=CANONICALIZER_ALGORITHM, manifest_extra=extra,
           source_path=source_paths["robot.urdf"], source_sha256=hashlib.sha256(source_data["robot.urdf"]).hexdigest(),
       )
   ```
   `export_tinker2` keeps its outer `_publication_lock`; make `publish_robot_artifact` accept an already-held lock by giving it a keyword `locked: bool = False` that skips the inner `with _publication_lock(...)` when `True`, and pass `locked=True` from `_export_tinker2_locked`. Keep the tinker2 `"files"` ordering identical to today (it iterates `ARTIFACT_FILES`); to preserve manifest bytes, order `files` by `file_bytes` insertion order rather than `sorted(...)` and have the tinker2 caller keep building `file_bytes` in `ARTIFACT_FILES` order.
4. `import re` at the top if missing.

- [ ] **Step 4: Run the new and the existing tinker2 tests**

Run: `scripts/pytest-clean tests/test_design_publish.py tests/test_artifact_export.py tests/test_workspace.py tests/test_provenance.py tests/test_current_artifact.py -q`
Expected: all pass. `test_full_sha_identity_binds_payload_lock_schema_and_version` and `test_determinism_idempotence_and_semantic_preservation` are the two that catch a changed tinker2 manifest/URDF byte stream — if either fails, the refactor changed tinker2 output; fix the refactor, not the test.

- [ ] **Step 5: Commit**

```bash
git add tools/tinker_sim_deploy/workspace.py tests/test_design_publish.py
git commit -m "refactor(deploy): publish_robot_artifact(robot=...) and canonicalize_urdf(mount_origin=...)"
```

---

### Task 8: `tinker_designs.lock` — design source lock

**Files:**
- Create: `tinker_designs/lock.py`
- Test: `tests/test_design_lock.py`

**Interfaces:**
- Consumes: `workspace._normalized_source_lock(records, robot=)`, `workspace.SOURCE_LOCK_SCHEMA`
- Produces:
  ```python
  def design_records(design_dir: Path, repo_root: Path, extra_files: Sequence[Path]) -> list[dict[str, object]]
      # every regular file under design_dir (sorted, repo-relative path when inside repo_root, else absolute)
      # plus extra_files (the resolved upstream meshes/xacros), each {"path","size","sha256"}; symlinks rejected
  def design_source_lock(design_dir: Path, repo_root: Path, extra_files: Sequence[Path]) -> bytes
      # _normalized_source_lock(records, robot=design_dir.name)
  ```

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_design_lock.py
from __future__ import annotations

import hashlib
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from tinker_designs.lock import design_records, design_source_lock


class DesignLockTest(unittest.TestCase):
    def test_records_cover_design_files_and_extras_in_path_order(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            design_dir = repo / "designs" / "demo"
            (design_dir / "meshes").mkdir(parents=True)
            (design_dir / "robot.urdf").write_bytes(b"<robot/>")
            (design_dir / "design.yaml").write_bytes(b"name: demo\n")
            (design_dir / "meshes" / "a.stl").write_bytes(b"solid")
            upstream = repo.parent / "upstream_mesh.stl"
            upstream.write_bytes(b"external")
            records = design_records(design_dir, repo, [upstream])
            self.assertEqual([r["path"] for r in records], sorted([
                "designs/demo/design.yaml", "designs/demo/meshes/a.stl", "designs/demo/robot.urdf", str(upstream),
            ]))
            self.assertEqual(records[0]["sha256"], hashlib.sha256(b"name: demo\n").hexdigest())
            lock = json.loads(design_source_lock(design_dir, repo, [upstream]))
            self.assertEqual(lock["robot"], "demo")
            self.assertEqual(lock["schema_version"], 3)
            self.assertEqual(len(lock["files"]), 4)

    def test_symlinks_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            design_dir = repo / "designs" / "demo"
            design_dir.mkdir(parents=True)
            (design_dir / "robot.urdf").write_bytes(b"<robot/>")
            os.symlink(design_dir / "robot.urdf", design_dir / "link.urdf")
            with self.assertRaises(Exception):
                design_records(design_dir, repo, [])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_design_lock.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'tinker_designs.lock'`

- [ ] **Step 3: Implement**

```python
# tinker_designs/lock.py
"""Source lock for a design: every file under designs/<name>/ plus resolved upstream inputs."""
from __future__ import annotations

import hashlib
from collections.abc import Sequence
from pathlib import Path

from tinker_sim_deploy.workspace import UnsafePathError, _normalized_source_lock


def _record(path: Path, label: str) -> dict[str, object]:
    if path.is_symlink():
        raise UnsafePathError(f"design source is a symlink: {path}")
    data = path.read_bytes()
    return {"path": label, "size": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def _label(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def design_records(design_dir: Path, repo_root: Path, extra_files: Sequence[Path]) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    for path in sorted(Path(design_dir).rglob("*")):
        if path.is_symlink():
            raise UnsafePathError(f"design source is a symlink: {path}")
        if path.is_file():
            records.append(_record(path, _label(path, repo_root)))
    for path in extra_files:
        records.append(_record(Path(path), _label(Path(path), repo_root)))
    records.sort(key=lambda item: str(item["path"]))
    return records


def design_source_lock(design_dir: Path, repo_root: Path, extra_files: Sequence[Path]) -> bytes:
    return _normalized_source_lock(design_records(design_dir, repo_root, extra_files), robot=Path(design_dir).name)
```

(`tinker_designs.lock` is the one module that imports `tinker_sim_deploy`; callers put `tools/` on `sys.path`, as `tools/design_import.py` will.)

- [ ] **Step 4: Run to verify pass**

Run: `scripts/pytest-clean tests/test_design_lock.py -q`
Expected: 2 passed

- [ ] **Step 5: Commit**

```bash
git add tinker_designs/lock.py tests/test_design_lock.py
git commit -m "feat(designs): design source lock"
```

---

### Task 9: `tools/design_import.py` — orchestration, hooks, CLI

**Files:**
- Create: `tools/design_import.py`
- Test: `tests/test_design_import_cli.py`

**Interfaces:**
- Consumes: everything above.
- Produces:
  ```python
  EXIT_RENDER, EXIT_CONTRACT, EXIT_IMPORT, EXIT_PUBLISH = 2, 3, 4, 5
  DESIGN_CANONICALIZER = "tinker-designs-canonical-v1"

  class ConverterHooks(Protocol):
      def import_urdf(self, urdf_path: Path, usd_path: Path) -> None: ...

  @dataclass(frozen=True)
  class ImportResult: artifact_dir: Path | None; manifest: dict | None; profile: dict; canonical_urdf: bytes

  def render(design_dir: Path, design: Design, *, packages: Mapping[str, Path], work_dir: Path, xacro_runner=subprocess.run) -> tuple[ET.Element, Path, list[Path]]
      # returns (clean root with package:// intact, path of the file://-resolved URDF written to work_dir/"robot.isaac.urdf", resolved upstream files)
  def run_import(design_dir: Path, repo_root: Path, hooks: ConverterHooks | None, *, no_import: bool = False,
                 packages: Mapping[str, Path] | None = None, work_dir: Path | None = None) -> ImportResult
  def write_init(design_dir: Path, name: str) -> Path      # --init: writes design.yaml from heuristics (refuses to overwrite)
  def main(argv: list[str] | None = None) -> int
  ```
  Stages inside `run_import`: `load_design` → `render` (`CleanError`/`DesignError` → exit 2) → `require_contract` (→ 3) → `derive_profile` (→ 3) → if `no_import`: return with `artifact_dir=None` → `hooks.import_urdf(isaac_urdf, work_dir/"robot.usd")` (any exception → 4; missing/empty USD → 4) → `design_source_lock` + `publish_robot_artifact(robot=design.name, file_bytes={"robot.urdf": canonical, "robot.usd": ..., "robot-profile.yaml": yaml.safe_dump(profile, sort_keys=True).encode(), plus "meshes/<rel>" for every file under design_dir/meshes}, canonicalizer=DESIGN_CANONICALIZER, manifest_extra={"qualification": "design_candidate", "kinematics": {"front_left_joint": driven[0], "front_right_joint": driven[1], "wheel_radius_m", "wheel_track_m", "footprint"}, "profile": profile, "provenance": {"source_lock_sha256", "source_identity", "source_files", "design_dir": "designs/<name>"}})` (→ 5).
  `main`: `--design PATH` (required), `--init`, `--no-import`, `--stub-converter`, `--artifacts PATH` (default `<repo>/artifacts`), `--package-root PKG=PATH` (repeatable, merged over `package_share_dirs()`). Without `--no-import`/`--stub-converter`/`--init` it boots `SimulationApp({"headless": True})` exactly like `arena_import.main` and uses `design_convert.IsaacHooks()` (Task 10). Prints the artifact dir on success.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_design_import_cli.py
from __future__ import annotations

import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests"))

from design_fixtures import two_arm_design, two_arm_urdf
import design_import
from design_import import EXIT_CONTRACT, EXIT_IMPORT, EXIT_RENDER, ImportResult, main, run_import, write_init


class StubHooks:
    def __init__(self, fail: bool = False) -> None:
        self.calls: list[tuple[Path, Path]] = []
        self.fail = fail

    def import_urdf(self, urdf_path: Path, usd_path: Path) -> None:
        self.calls.append((urdf_path, usd_path))
        if self.fail:
            raise RuntimeError("kit exploded")
        usd_path.write_bytes(b"#usda 1.0\n" + urdf_path.read_bytes()[:32])


def _make_design(repo: Path, name: str = "two_arm_fixture", urdf: bytes | None = None, design: dict | None = None) -> Path:
    design_dir = repo / "designs" / name
    design_dir.mkdir(parents=True)
    (design_dir / "robot.urdf").write_bytes(urdf or two_arm_urdf())
    raw = design or two_arm_design()
    raw["name"] = name
    (design_dir / "design.yaml").write_text(yaml.safe_dump(raw), encoding="utf-8")
    return design_dir


class RunImportTest(unittest.TestCase):
    def test_full_pipeline_with_stub_hooks_publishes_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            design_dir = _make_design(repo)
            hooks = StubHooks()
            result = run_import(design_dir, repo, hooks, packages={})
            self.assertIsInstance(result, ImportResult)
            self.assertEqual(len(hooks.calls), 1)
            self.assertEqual(hooks.calls[0][0].name, "robot.isaac.urdf")
            artifact_dir = result.artifact_dir
            self.assertEqual(artifact_dir.parent, repo / "artifacts" / "robot" / "two_arm_fixture")
            for name in ("robot.urdf", "robot.usd", "robot-profile.yaml", "manifest.json", "source-lock.json"):
                self.assertTrue((artifact_dir / name).is_file(), name)
            profile = yaml.safe_load((artifact_dir / "robot-profile.yaml").read_text())
            self.assertEqual(profile["robot"], "two_arm_fixture")
            self.assertAlmostEqual(profile["wheels"]["track_m"], 0.4)
            self.assertEqual(len(profile["arms"]), 2)
            manifest = json.loads((artifact_dir / "manifest.json").read_text())
            self.assertEqual(manifest["qualification"], "design_candidate")
            self.assertEqual(manifest["kinematics"]["front_left_joint"], "front_left_wheel_joint")
            self.assertEqual(manifest["kinematics"]["wheel_track_m"], profile["wheels"]["track_m"])
            self.assertEqual(manifest["profile"]["mass_kg"], profile["mass_kg"])
            self.assertEqual(manifest["provenance"]["design_dir"], "designs/two_arm_fixture")
            current = json.loads((repo / "artifacts" / "robot" / "two_arm_fixture" / "current.json").read_text())
            self.assertEqual(current["artifact_id"], artifact_dir.name)
            self.assertEqual((artifact_dir / "robot.urdf").read_bytes(), result.canonical_urdf)

    def test_no_import_stops_before_hooks(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            hooks = StubHooks()
            result = run_import(_make_design(repo), repo, hooks, no_import=True, packages={})
            self.assertIsNone(result.artifact_dir)
            self.assertEqual(hooks.calls, [])
            self.assertAlmostEqual(result.profile["mass_kg"], 32.05)

    def test_contract_failure_exits_3_listing_all_violations(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            raw = two_arm_design()
            raw["pan_tilt"] = None
            design_dir = _make_design(repo, design=raw)
            err = io.StringIO()
            with redirect_stdout(err):
                code = main(["--design", str(design_dir), "--stub-converter", "--artifacts", str(repo / "artifacts")])
            self.assertEqual(code, EXIT_CONTRACT)
            self.assertIn("pan_joint", err.getvalue())
            self.assertIn("tilt_joint", err.getvalue())
            self.assertFalse((repo / "artifacts").exists())

    def test_unresolved_mesh_exits_2(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            urdf = two_arm_urdf().replace(b'<box size="0.06 0.1 0.06" />', b'<mesh filename="package://nope/x.stl" />')
            design_dir = _make_design(repo, urdf=urdf)
            out = io.StringIO()
            with redirect_stdout(out):
                code = main(["--design", str(design_dir), "--stub-converter", "--artifacts", str(repo / "artifacts")])
            self.assertEqual(code, EXIT_RENDER)
            self.assertIn("nope", out.getvalue())

    def test_import_failure_exits_4_and_publishes_nothing(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            design_dir = _make_design(repo)
            with self.assertRaises(design_import.ImportStageError):
                run_import(design_dir, repo, StubHooks(fail=True), packages={})
            self.assertFalse((repo / "artifacts" / "robot").exists())

    def test_main_stub_converter_prints_artifact_dir(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            design_dir = _make_design(repo)
            out = io.StringIO()
            with redirect_stdout(out):
                code = main(["--design", str(design_dir), "--stub-converter", "--artifacts", str(repo / "artifacts")])
            self.assertEqual(code, 0)
            printed = Path(out.getvalue().strip().splitlines()[-1])
            self.assertTrue((printed / "manifest.json").is_file())

    def test_init_writes_design_yaml_and_refuses_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            design_dir = Path(temporary) / "two_arm_fixture"
            design_dir.mkdir()
            (design_dir / "robot.urdf").write_bytes(two_arm_urdf())
            path = write_init(design_dir, "two_arm_fixture")
            raw = yaml.safe_load(path.read_text())
            self.assertEqual(raw["wheels"]["driven"], ["front_left_wheel_joint", "front_right_wheel_joint"])
            with self.assertRaises(FileExistsError):
                write_init(design_dir, "two_arm_fixture")
            code = main(["--design", str(design_dir), "--init"])
            self.assertNotEqual(code, 0)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_design_import_cli.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'design_import'`

- [ ] **Step 3: Implement**

```python
#!/usr/bin/env python3
# tools/design_import.py
"""Candidate robot design importer CLI.

designs/<name>/{robot.urdf|robot.urdf.xacro, design.yaml, meshes/} ->
artifacts/robot/<name>/<hash>/{robot.urdf, robot.usd, robot-profile.yaml,
manifest.json, source-lock.json} + current.json.

Every Isaac call sits behind ``ConverterHooks`` so ``run_import``/``main``
run under system Python with ``--stub-converter`` (see tests).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import traceback
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(REPO_ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "tools"))

from tinker_designs.clean import CleanError, canonical_bytes, expand_xacro, package_share_dirs, resolve_mesh_uris, strip_gazebo
from tinker_designs.contract import ContractError, require_contract
from tinker_designs.derive import DeriveError, derive_profile
from tinker_designs.heuristics import draft_design
from tinker_designs.lock import design_records, design_source_lock
from tinker_designs.model import parse_urdf
from tinker_designs.schema import Design, DesignError, load_design
from tinker_sim_deploy.workspace import ArtifactExportError, publish_robot_artifact

EXIT_RENDER, EXIT_CONTRACT, EXIT_IMPORT, EXIT_PUBLISH = 2, 3, 4, 5
DESIGN_CANONICALIZER = "tinker-designs-canonical-v1"


class ImportStageError(RuntimeError):
    """The Isaac import hook failed or produced no USD."""


class ConverterHooks(Protocol):
    def import_urdf(self, urdf_path: Path, usd_path: Path) -> None: ...


class StubHooks:
    """--stub-converter: writes a placeholder USD so the pipeline runs without Kit."""

    def import_urdf(self, urdf_path: Path, usd_path: Path) -> None:
        usd_path.write_bytes(b"#usda 1.0\n# stub conversion of " + urdf_path.name.encode() + b"\n")


@dataclass(frozen=True)
class ImportResult:
    artifact_dir: Path | None
    manifest: dict | None
    profile: dict
    canonical_urdf: bytes


def render(design_dir: Path, design: Design, *, packages: Mapping[str, Path], work_dir: Path,
           resolve: bool = True, xacro_runner=subprocess.run) -> tuple[ET.Element, Path | None, list[Path]]:
    """Clean the source into (canonical root with package:// intact, Isaac input path, resolved upstream files).

    With ``resolve=False`` (the --no-import loop) no mesh URI is touched, so the
    fast loop never needs a sourced ROS environment; the Isaac path is None.
    """
    source = design_dir / design.source
    data = expand_xacro(source, runner=xacro_runner) if source.suffix == ".xacro" else source.read_bytes()
    root = parse_urdf(data)
    strip_gazebo(root)
    canonical_root = ET.fromstring(canonical_bytes(root))
    if not resolve:
        return canonical_root, None, []
    isaac_root = ET.fromstring(canonical_bytes(root))
    resolved = resolve_mesh_uris(isaac_root, design_dir=design_dir, packages=packages)
    isaac_path = work_dir / "robot.isaac.urdf"
    isaac_path.write_bytes(ET.tostring(isaac_root, encoding="utf-8", xml_declaration=True))
    return canonical_root, isaac_path, resolved


def run_import(design_dir: Path, repo_root: Path, hooks: ConverterHooks | None, *, no_import: bool = False,
               packages: Mapping[str, Path] | None = None, work_dir: Path | None = None,
               artifacts: Path | None = None) -> ImportResult:
    design_dir = Path(design_dir).resolve()
    repo_root = Path(repo_root).resolve()
    artifacts = repo_root / "artifacts" if artifacts is None else Path(artifacts)
    design = load_design(design_dir)
    packages = package_share_dirs() if packages is None else packages
    owned_work = work_dir is None
    work_dir = Path(tempfile.mkdtemp(prefix="design-import-")) if work_dir is None else Path(work_dir)
    try:
        root, isaac_path, resolved = render(design_dir, design, packages=packages, work_dir=work_dir, resolve=not no_import)
        require_contract(root, design)
        profile = derive_profile(root, design)
        canonical = canonical_bytes(root)
        if no_import:
            return ImportResult(None, None, profile, canonical)
        if hooks is None:
            raise ImportStageError("no converter hooks (pass --stub-converter or run under Isaac)")
        usd_path = work_dir / "robot.usd"
        try:
            hooks.import_urdf(isaac_path, usd_path)
        except Exception as error:
            raise ImportStageError(f"URDF import failed: {error}") from error
        if not usd_path.is_file() or usd_path.stat().st_size == 0:
            raise ImportStageError(f"importer wrote no USD at {usd_path}")
        file_bytes: dict[str, bytes] = {
            "robot.urdf": canonical,
            "robot.usd": usd_path.read_bytes(),
            "robot-profile.yaml": yaml.safe_dump(profile, sort_keys=True).encode("utf-8"),
        }
        meshes = design_dir / "meshes"
        if meshes.is_dir():
            for path in sorted(meshes.rglob("*")):
                if path.is_file():
                    file_bytes[f"meshes/{path.relative_to(meshes).as_posix()}"] = path.read_bytes()
        source_lock = design_source_lock(design_dir, repo_root, resolved)
        lock = json.loads(source_lock)
        source_bytes = (design_dir / design.source).read_bytes()
        driven = profile["wheels"]["driven"]
        extra = {
            "qualification": "design_candidate",
            "kinematics": {
                "front_left_joint": driven[0], "front_right_joint": driven[1],
                "wheel_radius_m": profile["wheels"]["radius_m"], "wheel_track_m": profile["wheels"]["track_m"],
                "footprint": profile["footprint"],
            },
            "profile": profile,
            "provenance": {
                "source_lock_sha256": hashlib.sha256(source_lock).hexdigest(),
                "source_identity": lock["source_identity"],
                "source_files": lock["files"],
                "design_dir": _label(design_dir, repo_root),
            },
        }
        result = publish_robot_artifact(
            artifacts,
            robot=design.name, file_bytes=file_bytes, canonical_urdf=canonical, source_lock_bytes=source_lock,
            canonicalizer=DESIGN_CANONICALIZER, manifest_extra=extra,
            source_path=_label(design_dir / design.source, repo_root), source_sha256=hashlib.sha256(source_bytes).hexdigest(),
        )
        return ImportResult(result.artifact_dir, result.manifest, profile, canonical)
    finally:
        if owned_work:
            shutil.rmtree(work_dir, ignore_errors=True)
```

```python
def _label(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def write_init(design_dir: Path, name: str) -> Path:
    design_dir = Path(design_dir)
    target = design_dir / "design.yaml"
    if target.exists():
        raise FileExistsError(f"{target} already exists; edit it or delete it first")
    source = next((design_dir / candidate for candidate in ("robot.urdf", "robot.urdf.xacro") if (design_dir / candidate).is_file()), None)
    if source is None:
        raise DesignError(f"{design_dir} has no robot.urdf or robot.urdf.xacro")
    data = expand_xacro(source) if source.suffix == ".xacro" else source.read_bytes()
    draft = draft_design(parse_urdf(data), name)
    target.write_text(yaml.safe_dump(draft, sort_keys=False), encoding="utf-8")
    return target


def _parse_package_roots(items: list[str]) -> dict[str, Path]:
    result: dict[str, Path] = {}
    for item in items:
        package, _, path = item.partition("=")
        if not package or not path:
            raise argparse.ArgumentTypeError(f"--package-root expects PKG=PATH, got {item!r}")
        result[package] = Path(path)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--design", type=Path, required=True, help="designs/<name> directory")
    parser.add_argument("--init", action="store_true", help="write a draft design.yaml from the URDF and exit")
    parser.add_argument("--no-import", action="store_true", help="stop after render + contract + derive")
    parser.add_argument("--stub-converter", action="store_true", help="skip Isaac; write a placeholder USD")
    parser.add_argument("--artifacts", type=Path, default=REPO_ROOT / "artifacts")
    parser.add_argument("--package-root", action="append", default=[], metavar="PKG=PATH")
    args = parser.parse_args(argv)

    design_dir = args.design.resolve()
    if args.init:
        try:
            print(write_init(design_dir, design_dir.name))
            return 0
        except (FileExistsError, DesignError, CleanError) as error:
            print(f"design_import: {error}")
            return EXIT_RENDER

    packages = dict(package_share_dirs())
    packages.update(_parse_package_roots(args.package_root))

    def _run(hooks) -> int:
        try:
            result = run_import(design_dir, REPO_ROOT, hooks, no_import=args.no_import, packages=packages, artifacts=args.artifacts)
        except (DesignError, CleanError) as error:
            print(f"design_import: render failed\n{error}")
            return EXIT_RENDER
        except (ContractError, DeriveError) as error:
            print(f"design_import: contract failed\n{error}")
            return EXIT_CONTRACT
        except ImportStageError as error:
            print(f"design_import: import failed\n{error}")
            return EXIT_IMPORT
        except ArtifactExportError as error:
            print(f"design_import: publish failed\n{error}")
            return EXIT_PUBLISH
        if result.artifact_dir is None:
            print(f"design_import: {design_dir.name} passes render + contract (mass {result.profile['mass_kg']} kg, track {result.profile['wheels']['track_m']} m)")
        else:
            print(result.artifact_dir)
        return 0

    if args.no_import:
        return _run(None)
    if args.stub_converter:
        return _run(StubHooks())

    from isaacsim import SimulationApp  # noqa: E402  (GPU/Kit only from here)

    real_argv, sys.argv = sys.argv, sys.argv[:1]
    try:
        app = SimulationApp({"headless": True})
    finally:
        sys.argv = real_argv
    try:
        from design_convert import IsaacHooks

        code = _run(IsaacHooks())
    except BaseException:
        traceback.print_exc()
        app.close()
        return EXIT_IMPORT
    app.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run to verify pass**

Run: `scripts/pytest-clean tests/test_design_import_cli.py -q`
Expected: 7 passed

- [ ] **Step 5: Commit**

```bash
git add tools/design_import.py tests/test_design_import_cli.py
git commit -m "feat(designs): design_import.py CLI — render, contract, derive, import, publish"
```

---

### Task 10: `tools/design_convert.py` — the Isaac hook

**Files:**
- Create: `tools/design_convert.py`
- Test: none possible without Kit; exercised live in Task 12. A smoke import test asserts the module imports lazily (no Isaac import at module import time).

**Interfaces:**
- Produces: `class IsaacHooks: def import_urdf(self, urdf_path: Path, usd_path: Path) -> None`

- [ ] **Step 1: Write the failing test (append to `tests/test_design_import_cli.py`)**

```python
class DesignConvertImportTest(unittest.TestCase):
    def test_module_imports_without_isaac(self) -> None:
        before = {name for name in sys.modules if name.startswith(("isaacsim", "omni", "pxr"))}
        import design_convert
        after = {name for name in sys.modules if name.startswith(("isaacsim", "omni", "pxr"))}
        self.assertTrue(hasattr(design_convert, "IsaacHooks"))
        self.assertEqual(after, before, "design_convert must import Isaac only inside IsaacHooks.import_urdf")
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_design_import_cli.py -q -k DesignConvert`
Expected: FAIL — `ModuleNotFoundError: No module named 'design_convert'`

- [ ] **Step 3: Implement**

```python
# tools/design_convert.py
"""Isaac Sim side of design_import: URDF -> USD through isaacsim.asset.importer.urdf.

Ported from tk26_sim/src/isaac_bringup/scripts/verify_in_isaac.py. Import
config matches the tinker2 artifact: no merged fixed joints, free base,
URDF inertia honoured, mimics parsed. Convex decomposition is off for
primitives (the box/cylinder chassis) — meshes get the importer's default
convex hull, which is what tinker2's arm links use today.
"""
from __future__ import annotations

from pathlib import Path


class IsaacHooks:
    def import_urdf(self, urdf_path: Path, usd_path: Path) -> None:
        import omni.kit.commands
        import omni.usd
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.asset.importer.urdf")
        from isaacsim.asset.importer.urdf import _urdf

        config = _urdf.ImportConfig()
        config.merge_fixed_joints = False
        config.convex_decomp = False
        config.replace_cylinders_with_capsules = False
        config.import_inertia_tensor = True
        config.fix_base = False
        config.distance_scale = 1.0
        config.density = 0.0
        status, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile", urdf_path=str(urdf_path), import_config=config,
        )
        if not status or not prim_path:
            raise RuntimeError(f"URDFParseAndImportFile returned status={status!r} prim_path={prim_path!r}")
        stage = omni.usd.get_context().get_stage()
        joints = sum(1 for prim in stage.Traverse() if str(prim.GetPath()).startswith(prim_path) and "Joint" in (prim.GetTypeName() or ""))
        if joints == 0:
            raise RuntimeError(f"imported {prim_path} has no joints")
        usd_path.parent.mkdir(parents=True, exist_ok=True)
        if not stage.Export(str(usd_path)):
            raise RuntimeError(f"stage.Export({usd_path}) returned False")
        print(f"design_convert: imported {prim_path} ({joints} joints) -> {usd_path}", flush=True)
```

- [ ] **Step 4: Run to verify pass**

Run: `scripts/pytest-clean tests/test_design_import_cli.py -q`
Expected: 8 passed

- [ ] **Step 5: Commit**

```bash
git add tools/design_convert.py tests/test_design_import_cli.py
git commit -m "feat(designs): Isaac URDF->USD hook for design_import"
```

---

### Task 11: `designs/tinker2_ref` parity design

**Files:**
- Create: `designs/tinker2_ref/robot.urdf` (copy of the current tinker2 artifact's canonical `robot.urdf`), `designs/tinker2_ref/design.yaml`
- Test: `tests/test_design_tinker2_ref.py`

**Interfaces:**
- Consumes: `run_import(..., no_import=True)`, `check_contract`, `derive_profile`.
- The reference URDF comes from the main checkout: `artifacts/robot/tinker2/$(jq -r .artifact_id artifacts/robot/tinker2/current.json)/robot.urdf` (today `347aef74…`). The test does not depend on `artifacts/` existing — it pins the copied file's canonical idempotence and the derived constants that `workspace.py` types today.

- [ ] **Step 1: Copy the reference URDF and write `design.yaml`**

```bash
MAIN=/home/tinker/tinker-sim/6.0.1
ID=$(python3 -c "import json;print(json.load(open('$MAIN/artifacts/robot/tinker2/current.json'))['artifact_id'])")
mkdir -p designs/tinker2_ref
cp "$MAIN/artifacts/robot/tinker2/$ID/robot.urdf" designs/tinker2_ref/robot.urdf
```

```yaml
# designs/tinker2_ref/design.yaml
# The current Tinker 2 as a design: the parity gate for the design pipeline.
# robot.urdf is the canonical tinker2 artifact URDF (artifact 347aef74…, 2026-09-13).
name: tinker2_ref
kinematics: diff_drive
base_frame: base_link
wheels:
  driven: [front_left_wheel_joint, front_right_wheel_joint]
  caster_swivel: [rear_left_swivel_joint, rear_right_swivel_joint]
  caster_wheel: [rear_left_wheel_joint, rear_right_wheel_joint]
arms:
  - name: xarm
    mount: link_base
    joints: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    gripper:
      drive: drive_joint
      mimics: [left_finger_joint, left_inner_knuckle_joint, right_outer_knuckle_joint, right_finger_joint, right_inner_knuckle_joint]
    drive: {stiffness: 400.0, damping: 40.0}
pan_tilt: {joints: [pan_joint, tilt_joint]}
sensors:
  - {type: livox_mid360, frame: livox_frame}
  - {type: head_camera, frame: head_camera_link}
# base_link's collision is the tracer_mini STL, so the footprint cannot be
# derived from primitives; this is the value workspace.py types today.
footprint: [[0.15, 0.25], [0.15, -0.25], [-0.35, -0.25], [-0.35, 0.25]]
```

- [ ] **Step 2: Write the failing test**

```python
# tests/test_design_tinker2_ref.py
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from design_import import run_import
from tinker_designs.clean import canonical_bytes
from tinker_designs.model import parse_urdf

DESIGN = ROOT / "designs" / "tinker2_ref"


class Tinker2RefParityTest(unittest.TestCase):
    def test_reference_urdf_is_canonical_and_pipeline_preserves_it(self) -> None:
        source = (DESIGN / "robot.urdf").read_bytes()
        self.assertEqual(canonical_bytes(parse_urdf(source)), source, "reference URDF must already be in canonical form")
        with tempfile.TemporaryDirectory() as temporary:
            result = run_import(DESIGN, ROOT, None, no_import=True, packages={}, work_dir=Path(temporary))
        self.assertEqual(result.canonical_urdf, source)
        self.assertIn(b"package://", result.canonical_urdf)

    def test_derived_profile_matches_workspace_constants(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            result = run_import(DESIGN, ROOT, None, no_import=True, packages={}, work_dir=Path(temporary))
        profile = result.profile
        self.assertAlmostEqual(profile["wheels"]["radius_m"], 0.0525)
        self.assertAlmostEqual(profile["wheels"]["track_m"], 0.25)
        self.assertEqual(profile["wheels"]["driven"], ["front_left_wheel_joint", "front_right_wheel_joint"])
        self.assertEqual(profile["footprint"], [[0.15, 0.25], [0.15, -0.25], [-0.35, -0.25], [-0.35, 0.25]])
        self.assertEqual(profile["footprint_source"], "override")
        self.assertEqual(profile["arms"][0]["joints"], [f"joint{i}" for i in range(1, 8)])
        self.assertEqual(profile["arms"][0]["gripper"]["drive"], "drive_joint")
        self.assertGreater(profile["mass_kg"], 60.0)
        self.assertGreater(profile["arms"][0]["reach_m"], 0.6)


if __name__ == "__main__":
    unittest.main()
```

`packages={}` is fine here: `run_import(..., no_import=True)` calls `render(..., resolve=False)` (Task 9), so no `package://` URI is resolved and the `--no-import` loop never needs a sourced ROS environment.

- [ ] **Step 3: Run to verify failure**

Run: `scripts/pytest-clean tests/test_design_tinker2_ref.py -q`
Expected: FAIL until `designs/tinker2_ref/` exists. Once it does, if the contract reports violations for tinker2's URDF, read them — the plausible ones are a joint `design.yaml` missed (add it to `design.yaml`, never to an ignore list) or the `world` root (already accepted by `check_contract`).

- [ ] **Step 4: Run to verify pass**

Run: `scripts/pytest-clean tests/test_design_tinker2_ref.py tests/test_design_import_cli.py -q`
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add designs/tinker2_ref tests/test_design_tinker2_ref.py
git commit -m "feat(designs): tinker2_ref parity design and gate"
```

---

### Task 12: Live import of `tinker2_ref` and the two-arm fixture, docs

**Files:**
- Create: `designs/two_arm_demo/` (the fixture URDF written to disk via `tests/design_fixtures.py`, plus its design.yaml) — the first non-xArm candidate
- Modify: `README.md` (new "Candidate robot designs" section after the arena/YCB import commands near line 281), `docs/developer-log.md` (entry)

- [ ] **Step 1: Materialise the demo design**

```bash
mkdir -p designs/two_arm_demo
./.venv/bin/python - <<'EOF'
import sys, yaml
sys.path.insert(0, "tests")
from design_fixtures import two_arm_urdf, two_arm_design
open("designs/two_arm_demo/robot.urdf", "wb").write(two_arm_urdf())
raw = two_arm_design(); raw["name"] = "two_arm_demo"
open("designs/two_arm_demo/design.yaml", "w").write(yaml.safe_dump(raw, sort_keys=False))
EOF
scripts/pytest-clean tests/test_design_import_cli.py -q
./.venv/bin/python tools/design_import.py --design designs/two_arm_demo --no-import
```
Expected: last command prints `design_import: two_arm_demo passes render + contract (mass 32.05 kg, track 0.4 m)`, exit 0.

- [ ] **Step 2: GPU pre-flight (mandatory)**

```bash
nvidia-smi --query-gpu=index,memory.used,memory.total --format=csv
pgrep -af "isaac|run_sim|gpsr-stack" || echo "no sim running"
```
Announce "starting design_import live run" in the session before Step 3; do not proceed if another stack is running.

- [ ] **Step 3: Live import, tinker2_ref first (needs ROS for `package://`)**

```bash
source /opt/ros/humble/setup.bash && source /home/tinker/tk25_ws/install/setup.bash
./.venv/bin/python tools/design_import.py --design designs/tinker2_ref 2>&1 | tee /home/tinker/.claude/jobs/b44d5a8f/tmp/design-import-tinker2_ref.log
./.venv/bin/python tools/design_import.py --design designs/two_arm_demo 2>&1 | tee /home/tinker/.claude/jobs/b44d5a8f/tmp/design-import-two_arm_demo.log
```
Expected for each: `design_convert: imported /tinker2_ref (N joints) -> …/robot.usd`, then the artifact path printed; `artifacts/robot/tinker2_ref/current.json` and `artifacts/robot/two_arm_demo/current.json` exist; `robot.usd` is non-empty. Record joint counts (tinker2_ref: 47 URDF joints; the importer reports physics joints only, expect 24 movable + fixed handling per `merge_fixed_joints=False`). If the importer fails on `package://` after ROS is sourced, pass `--package-root tinker_urdf=/home/tinker/tk25_ws/install/tinker_urdf/share/tinker_urdf` etc. and record that `package_share_dirs` missed it in the developer log as a defect, then fix `package_share_dirs`.

Note the worktree has no `artifacts/` — run these from the worktree with `--artifacts /home/tinker/.claude/jobs/b44d5a8f/tmp/artifacts` so nothing lands in the shared store; the main checkout's `artifacts/` is untouched by M1.

- [ ] **Step 4: Docs**

Add to `README.md` after the arena/YCB import lines (~281):

````markdown
### Candidate robot designs

Design a robot in URDF Studio (or any tool), export it to `designs/<name>/robot.urdf`,
draft the role sidecar, iterate without Isaac, then import:

```bash
./.venv/bin/python tools/design_import.py --design designs/<name> --init        # writes design.yaml
./.venv/bin/python tools/design_import.py --design designs/<name> --no-import   # render + contract + derive (seconds)
source /opt/ros/humble/setup.bash && source $TINKER_WS/install/setup.bash        # only if the URDF uses package://
./.venv/bin/python tools/design_import.py --design designs/<name>               # Isaac URDF->USD, publishes artifacts/robot/<name>/
```

`designs/tinker2_ref` is the current robot as a design and must reproduce the tinker2 artifact URDF
(`tests/test_design_tinker2_ref.py`). Spec: `docs/superpowers/specs/2026-09-13-tinker-designs-chassis-exploration-design.md`.
````

Append to `docs/developer-log.md` (follow the file's existing entry format) a dated entry: what M1 adds, the two live import results (joint counts, USD sizes), any importer gotcha hit, and that runtime selection (`TINKER_SIM_ROBOT`) is M2.

- [ ] **Step 5: Full suite and commit**

Run: `scripts/pytest-clean tests/test_design_schema.py tests/test_design_model.py tests/test_design_clean.py tests/test_design_contract.py tests/test_design_derive.py tests/test_design_heuristics.py tests/test_design_publish.py tests/test_design_lock.py tests/test_design_import_cli.py tests/test_design_tinker2_ref.py tests/test_artifact_export.py tests/test_workspace.py tests/test_provenance.py tests/test_current_artifact.py -q`
Expected: all pass.

```bash
git add designs/two_arm_demo README.md docs/developer-log.md
git commit -m "docs(designs): candidate design workflow; two_arm_demo design; M1 live import log"
git push -u origin worktree-tinker-designs-spec
```

---

## Self-review

**Spec coverage (M1 = §3A + §3B, §5, §6, §7 unit/parity):** §3A layout → Tasks 1, 11, 12; `--init` heuristics → Task 6/9; `estimated` flag → Task 1/5; §3B stage 1 → Task 3/9, stage 2 → Task 4, stage 3 → Task 10, stage 4 → Task 5, stage 5 → Tasks 7/8/9 (meshes/ copied, `map.*` not produced); `package://` kept in published URDF → Task 9 `render` (canonical root vs isaac root) + Task 11 assertion; §5 rules → Task 4 (root/world, claims, wheel axes, casters, serial arm chains, gripper descent, mimic target, sensors, inertia PD, ground plane), `mount_origin` parameter → Task 7; §6 exit codes 2/3/4/5, all violations listed, no partial artifact (publisher's stage+rename is reused) → Tasks 4/7/9; §7 unit tests per module, contract fixtures, derive hand-values, stub converter, parity → Tasks 1–11; live import → Task 12. Not in M1 by design: `TINKER_SIM_ROBOT`, runtime profile consumers, metrics recorder (M2 plan); parameter sweeps (later).

**Placeholder scan:** no TBD/TODO/"similar to Task N"; every code step carries its code. The only judgement call left to the implementer is Task 5 Step 4's CoG bound, and the instruction says what evidence justifies changing it.

**Type consistency:** `Design.claimed_joints()` (Task 1) is used by Task 4; `parse_urdf/links/joints/parent_joint/root_links/movable_joints/fixed_subtree/apply/compose/compose_rotation/rotate_axis_angle` (Task 2) are the names imported in Tasks 4–6; `canonical_bytes/strip_gazebo/resolve_mesh_uris/expand_xacro/package_share_dirs/CleanError` (Task 3) are what Task 9 imports; `check_contract/require_contract/ContractError` (Task 4), `derive_profile/DeriveError` (Task 5), `draft_design` (Task 6), `publish_robot_artifact/_normalized_source_lock(robot=)/canonicalize_urdf(mount_origin=)` (Task 7), `design_source_lock/design_records` (Task 8), `render(..., resolve=)`, `run_import(design_dir, repo_root, hooks, *, no_import, packages, work_dir, artifacts)`, `ImportResult`, `ImportStageError`, `StubHooks`, `write_init`, `EXIT_*` (Task 9), `IsaacHooks` (Task 10) all match their call sites in Tasks 11–12.
