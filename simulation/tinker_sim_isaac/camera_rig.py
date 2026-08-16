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


def focal_from_fov(
    width_px: int,
    horizontal_fov_deg: float,
    horizontal_aperture_mm: float = HORIZONTAL_APERTURE_MM,
) -> float:
    """USD focal length (mm) so the render matches the declared horizontal FOV."""
    if width_px <= 0 or not 0.0 < horizontal_fov_deg < 180.0:
        raise ValueError("focal_from_fov requires positive width and FOV in (0, 180)")
    return horizontal_aperture_mm / (
        2.0 * math.tan(math.radians(horizontal_fov_deg) / 2.0)
    )


def camera_info_fields(spec: CameraStreamSpec) -> dict[str, Any]:
    """Pinhole CameraInfo values consistent with the rendered FOV by construction."""
    fx = spec.width / (2.0 * math.tan(math.radians(spec.horizontal_fov_deg) / 2.0))
    cx = spec.width / 2.0
    cy = spec.height / 2.0
    return {
        "height": spec.height,
        "width": spec.width,
        "distortion_model": "plumb_bob",
        "d": [0.0] * 5,
        "k": [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    }


def to_numpy(value: Any):
    """Normalize warp/torch/proxy buffers to a host numpy array."""
    import numpy as np

    candidate = value
    if hasattr(candidate, "cpu"):
        candidate = candidate.cpu()
    if hasattr(candidate, "numpy"):
        candidate = candidate.numpy()
    return np.asarray(candidate)


def rgb8_array(value: Any, height: int, width: int):
    """Normalize an RTX rgb annotator buffer to contiguous uint8 (H, W, 3)."""
    import numpy as np

    array = to_numpy(value)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3 or array.shape[:2] != (height, width):
        raise ValueError(f"unexpected RGB frame shape: {array.shape}")
    if array.shape[2] < 3:
        raise ValueError(f"RGB frame has fewer than three channels: {array.shape}")
    array = array[:, :, :3]
    if array.dtype.kind == "f":
        if not np.isfinite(array).all():
            raise ValueError("RGB frame contains non-finite values")
        scale = 255.0 if float(array.max(initial=0.0)) <= 1.0 else 1.0
        array = np.clip(array * scale, 0.0, 255.0).astype(np.uint8)
    else:
        array = np.clip(array, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(array)


def depth_to_16uc1_mm(value: Any):
    """Metres -> 16UC1 millimetres; NaN/Inf/non-positive -> 0; clamp 65535."""
    import numpy as np

    depth = to_numpy(value)
    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]
    if depth.ndim != 2:
        raise ValueError(f"depth frame must be HxW: {depth.shape}")
    if depth.dtype.kind != "f":
        depth = depth.astype(np.float64)
    millimetres = np.where(
        np.isfinite(depth) & (depth > 0.0), depth * 1000.0, 0.0
    )
    return np.clip(np.rint(millimetres), 0.0, 65535.0).astype(np.uint16)


def pack_registered_cloud(
    depth_value: Any, *, fx: float, fy: float, cx: float, cy: float
) -> bytes:
    """Organized float32 xyz+pad cloud (point_step=16) from registered depth.

    Matches the real Orbbec driver layout: dense HxW rows, NaN xyz for invalid
    depth, optical-frame axes.  This deliberately reproduces the driver's
    4-float stride that ``door_detection``'s 5-float parser cannot read.
    """
    import numpy as np

    depth = to_numpy(depth_value)
    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]
    if depth.ndim != 2:
        raise ValueError(f"depth frame must be HxW: {depth.shape}")
    depth = depth.astype(np.float32, copy=False)
    height, width = depth.shape
    z = np.where(np.isfinite(depth) & (depth > 0.0), depth, np.float32(np.nan))
    columns = np.arange(width, dtype=np.float32)[None, :]
    rows = np.arange(height, dtype=np.float32)[:, None]
    cloud = np.zeros((height, width, 4), dtype=np.float32)
    cloud[:, :, 0] = (columns - cx) / fx * z
    cloud[:, :, 1] = (rows - cy) / fy * z
    cloud[:, :, 2] = z
    return cloud.tobytes()


class CameraRig:
    """Robot-mounted RTX cameras serving the hardware-parity contract.

    Cameras are created as children of the robot's optical-frame prims so they
    track pan/tilt and arm motion through physics.  Initialization is
    fail-closed: a contract whose mounts or sensors cannot be realized must
    not produce a silently camera-less simulator.
    """

    def __init__(self, specs: tuple[CameraStreamSpec, ...]) -> None:
        if not specs:
            raise ValueError("camera rig requires at least one camera spec")
        self.specs = tuple(specs)
        self._sensors: dict[str, Any] = {}

    def initialize(self, app: Any) -> None:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.sensors.experimental.rtx")
        for _ in range(4):
            app.update()

        import omni.usd
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
        from pxr import Gf, Usd, UsdGeom

        stage = omni.usd.get_context().get_stage()
        robot = stage.GetPrimAtPath("/World/Tinker")
        if not robot.IsValid():
            raise RuntimeError("camera rig requires the spawned /World/Tinker robot")
        for spec in self.specs:
            mounts = [
                prim
                for prim in Usd.PrimRange(robot)
                if prim.GetName() == spec.mount_prim
            ]
            if len(mounts) != 1:
                raise RuntimeError(
                    f"expected exactly one {spec.mount_prim!r} prim under "
                    f"/World/Tinker, found {len(mounts)}"
                )
            camera_path = f"{mounts[0].GetPath().pathString}/rtx_camera"
            camera = RtxCamera(camera_path, tick_rate=float(spec.tick_rate_hz))
            prim = stage.GetPrimAtPath(camera_path)
            usd_camera = UsdGeom.Camera(prim)
            focal = focal_from_fov(spec.width, spec.horizontal_fov_deg)
            usd_camera.GetFocalLengthAttr().Set(focal)
            usd_camera.GetHorizontalApertureAttr().Set(HORIZONTAL_APERTURE_MM)
            usd_camera.GetVerticalApertureAttr().Set(
                HORIZONTAL_APERTURE_MM * spec.height / spec.width
            )
            usd_camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 200.0))
            # Identity translation; rotate the USD camera into the ROS optical
            # convention so the render looks along the frame's +Z axis.
            # RtxCamera authors xformOp:orient as double precision (quatd);
            # match it or AddOrientOp raises a precision-mismatch Tf error.
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Quatd(*OPTICAL_TO_USD_CAMERA_WXYZ)
            )
            self._sensors[spec.name] = CameraSensor(
                camera,
                resolution=(spec.height, spec.width),
                annotators=[COLOR_ANNOTATOR, DEPTH_ANNOTATOR],
            )
        for _ in range(4):
            app.update()

    def capture(self) -> dict[str, tuple[Any, Any]]:
        frames: dict[str, tuple[Any, Any]] = {}
        for name, sensor in self._sensors.items():
            rgb, _info = sensor.get_data(COLOR_ANNOTATOR)
            depth, _info = sensor.get_data(DEPTH_ANNOTATOR)
            frames[name] = (rgb, depth)
        return frames
