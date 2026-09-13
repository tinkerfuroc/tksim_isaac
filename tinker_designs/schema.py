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
