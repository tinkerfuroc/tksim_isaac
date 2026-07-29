from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Mapping


class CalibrationStatus(str, Enum):
    MISSING = "missing"
    CALIBRATED = "calibrated"


@dataclass(frozen=True)
class BaseCalibration:
    """Versioned base model used by the ROS hardware-parity facade.

    A missing profile is intentionally usable for functional development, but
    it is never qualification-worthy. Odometry is still integrated from wheel
    observations; simulator root pose is not an input to this model.
    """

    status: CalibrationStatus
    profile_id: str
    wheel_radius_m: float
    wheel_track_m: float
    command_timeout_s: float
    max_linear_mps: float
    max_angular_rps: float
    odom_rate_hz: float
    command_latency_s: float = 0.0
    linear_scale: float = 1.0
    angular_scale: float = 1.0
    linear_bias_mps: float = 0.0
    angular_bias_rps: float = 0.0
    pose_variance_xy: float = 0.0
    pose_variance_yaw: float = 0.0
    twist_variance_linear: float = 0.0
    twist_variance_angular: float = 0.0
    dataset_sha256: str | None = None

    @classmethod
    def development_default(cls) -> "BaseCalibration":
        return cls(
            status=CalibrationStatus.MISSING,
            profile_id="tinker2-development-uncalibrated",
            wheel_radius_m=0.0525,
            wheel_track_m=0.25,
            command_timeout_s=0.25,
            max_linear_mps=0.60,
            max_angular_rps=1.0,
            odom_rate_hz=50.0,
        )

    @classmethod
    def load(cls, path: Path | None) -> "BaseCalibration":
        if path is None or not path.is_file():
            return cls.development_default()
        raw = json.loads(path.read_text(encoding="utf-8"))
        status = CalibrationStatus(str(raw.get("status", "missing")))
        model = raw.get("base", raw)
        if not isinstance(model, Mapping):
            raise ValueError("calibration base model must be an object")
        result = cls(
            status=status,
            profile_id=str(raw.get("profile_id", path.stem)),
            wheel_radius_m=float(model["wheel_radius_m"]),
            wheel_track_m=float(model["wheel_track_m"]),
            command_timeout_s=float(model.get("command_timeout_s", 0.25)),
            max_linear_mps=float(model.get("max_linear_mps", 0.60)),
            max_angular_rps=float(model.get("max_angular_rps", 1.0)),
            odom_rate_hz=float(model.get("odom_rate_hz", 50.0)),
            command_latency_s=float(model.get("command_latency_s", 0.0)),
            linear_scale=float(model.get("linear_scale", 1.0)),
            angular_scale=float(model.get("angular_scale", 1.0)),
            linear_bias_mps=float(model.get("linear_bias_mps", 0.0)),
            angular_bias_rps=float(model.get("angular_bias_rps", 0.0)),
            pose_variance_xy=float(model.get("pose_variance_xy", 0.0)),
            pose_variance_yaw=float(model.get("pose_variance_yaw", 0.0)),
            twist_variance_linear=float(model.get("twist_variance_linear", 0.0)),
            twist_variance_angular=float(model.get("twist_variance_angular", 0.0)),
            dataset_sha256=(str(raw["dataset_sha256"]) if raw.get("dataset_sha256") else None),
        )
        result.validate()
        return result

    def validate(self) -> None:
        positive = {
            "wheel_radius_m": self.wheel_radius_m,
            "wheel_track_m": self.wheel_track_m,
            "command_timeout_s": self.command_timeout_s,
            "max_linear_mps": self.max_linear_mps,
            "max_angular_rps": self.max_angular_rps,
            "odom_rate_hz": self.odom_rate_hz,
        }
        bad = [name for name, value in positive.items() if value <= 0.0]
        if bad:
            raise ValueError("calibration values must be positive: " + ", ".join(bad))
        if self.command_latency_s < 0.0:
            raise ValueError("command_latency_s must be non-negative")
        if self.status is CalibrationStatus.CALIBRATED and not self.dataset_sha256:
            raise ValueError("a calibrated profile requires dataset_sha256")

    def qualification_error(self) -> str | None:
        if self.status is not CalibrationStatus.CALIBRATED:
            return "Tinker 2 navigation calibration is missing"
        if not self.dataset_sha256:
            return "calibration has no source dataset hash"
        return None

    def as_dict(self) -> dict[str, Any]:
        return {
            "status": self.status.value,
            "profile_id": self.profile_id,
            "dataset_sha256": self.dataset_sha256,
            "base": {
                key: value
                for key, value in self.__dict__.items()
                if key not in {"status", "profile_id", "dataset_sha256"}
            },
        }
