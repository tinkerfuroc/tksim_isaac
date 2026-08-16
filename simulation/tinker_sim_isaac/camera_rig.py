from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

#: Contract camera order: head first, wrist second.
CAMERA_NAMES = ("head_camera", "wrist_camera")
COLOR_ANNOTATOR = "rgb"
DEPTH_ANNOTATOR = "distance_to_image_plane"
#: Local rotation from a ROS optical frame (+Z forward, +Y down) to a USD
#: camera (-Z forward, +Y up): 180 degrees about X, as wxyz.
OPTICAL_TO_USD_CAMERA_WXYZ = (0.0, 1.0, 0.0, 0.0)
#: Default USD camera aperture the focal length is solved against (mm).
HORIZONTAL_APERTURE_MM = 20.955


@dataclass(frozen=True)
class CameraStreamSpec:
    name: str
    color_topic: str
    depth_topic: str
    camera_info_topics: tuple[str, ...]
    frame_id: str
    mount_prim: str
    width: int
    height: int
    horizontal_fov_deg: float
    tick_rate_hz: float


def _string(mapping: Mapping[str, Any], key: str, owner: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{owner}.{key} must be a non-empty string")
    return value


def _positive_number(mapping: Mapping[str, Any], key: str, owner: str) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise ValueError(f"{owner}.{key} must be positive")
    return float(value)


def load_camera_specs(path: Path) -> tuple[CameraStreamSpec, ...]:
    """Load the camera contract; malformed declarations raise, never guess."""
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if raw.get("schema_version") != 2:
        raise ValueError("hardware-parity sensor contract must be schema_version 2")
    qos = raw.get("camera_qos")
    if not isinstance(qos, Mapping) or qos.get("reliability") != "reliable":
        raise ValueError("camera_qos.reliability must be reliable")
    if qos.get("durability") != "volatile" or qos.get("history") != "keep_last":
        raise ValueError("camera_qos must be reliable + volatile + keep_last")
    specs = []
    for name in CAMERA_NAMES:
        camera = raw.get(name)
        if not isinstance(camera, Mapping):
            raise ValueError(f"sensor contract is missing {name}")
        if camera.get("color_encoding") != "rgb8":
            raise ValueError(f"{name}.color_encoding must be rgb8")
        if camera.get("depth_encoding") != "16UC1":
            raise ValueError(f"{name}.depth_encoding must be 16UC1")
        if camera.get("depth_unit") != "millimeter":
            raise ValueError(f"{name}.depth_unit must be millimeter")
        info_topics = camera.get("camera_info_topics")
        if (
            not isinstance(info_topics, list)
            or not info_topics
            or any(not isinstance(topic, str) or not topic for topic in info_topics)
        ):
            raise ValueError(f"{name}.camera_info_topics must be non-empty strings")
        specs.append(
            CameraStreamSpec(
                name=name,
                color_topic=_string(camera, "color_topic", name),
                depth_topic=_string(camera, "depth_topic", name),
                camera_info_topics=tuple(info_topics),
                frame_id=_string(camera, "frame_id", name),
                mount_prim=_string(camera, "mount_prim", name),
                width=int(_positive_number(camera, "width", name)),
                height=int(_positive_number(camera, "height", name)),
                horizontal_fov_deg=_positive_number(camera, "horizontal_fov_deg", name),
                tick_rate_hz=_positive_number(camera, "tick_rate_hz", name),
            )
        )
    return tuple(specs)
