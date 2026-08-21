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
    mount_rotation_wxyz: tuple[float, float, float, float]
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


def _quaternion_wxyz(
    mapping: Mapping[str, Any], key: str, owner: str
) -> tuple[float, float, float, float]:
    value = mapping.get(key)
    if (
        not isinstance(value, list)
        or len(value) != 4
        or any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value)
    ):
        raise ValueError(f"{owner}.{key} must be a list of four numbers")
    components = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in components):
        raise ValueError(f"{owner}.{key} must contain finite numbers")
    norm = math.sqrt(sum(item * item for item in components))
    if norm <= 0.0:
        raise ValueError(f"{owner}.{key} must have non-zero norm")
    return components


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
                mount_rotation_wxyz=_quaternion_wxyz(
                    camera, "mount_rotation_wxyz", name
                ),
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


def _rgb8_array_reference(value: Any, height: int, width: int):
    """Reference semantics for ``rgb8_array``; kept verbatim for equivalence tests.

    Normalize an RTX rgb annotator buffer to contiguous uint8 (H, W, 3).
    """
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


def _depth_to_16uc1_mm_reference(value: Any):
    """Reference semantics for ``depth_to_16uc1_mm``; kept verbatim for tests.

    Metres -> 16UC1 millimetres; NaN/Inf/non-positive -> 0; clamp 65535.
    """
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


#: Per-(bucket, shape) scratch buffers reused across frames so the steady
#: state of ``rgb8_array``/``depth_to_16uc1_mm`` allocates nothing. Safe only
#: because the camera publish loop is single-threaded and converts each
#: returned buffer to bytes before the next call for that bucket/shape lands
#: (see ``RosStandardGateway.publish_cameras``); nothing else retains a
#: reference to the arrays these functions return.
_SCRATCH: dict[str, dict[Any, Any]] = {}


def _scratch(bucket: str, key: Any, shape: tuple[int, ...], dtype: Any) -> Any:
    """Return the cached ndarray for ``(bucket, key)``, (re)allocating on mismatch."""
    import numpy as np

    store = _SCRATCH.setdefault(bucket, {})
    array = store.get(key)
    if array is None or array.shape != shape or array.dtype != dtype:
        array = np.empty(shape, dtype=dtype)
        store[key] = array
    return array


def rgb8_array(value: Any, height: int, width: int):
    """Normalize an RTX rgb annotator buffer to contiguous uint8 (H, W, 3).

    Byte-identical to ``_rgb8_array_reference`` (see
    ``tests/test_camera_publish_equivalence.py``); avoids per-frame
    allocation by writing into a cached (height, width, 3) uint8 buffer and
    fusing the trailing clip+cast into that single write.
    """
    import numpy as np

    array = to_numpy(value)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3 or array.shape[:2] != (height, width):
        raise ValueError(f"unexpected RGB frame shape: {array.shape}")
    if array.shape[2] < 3:
        raise ValueError(f"RGB frame has fewer than three channels: {array.shape}")
    channels = array[:, :, :3]
    out_shape = (height, width, 3)
    out = _scratch("rgb_out", out_shape, out_shape, np.uint8)
    if channels.dtype.kind == "f":
        if not np.isfinite(channels).all():
            raise ValueError("RGB frame contains non-finite values")
        scale = 255.0 if float(channels.max(initial=0.0)) <= 1.0 else 1.0
        scaled = _scratch("rgb_scaled", out_shape, out_shape, channels.dtype)
        np.multiply(channels, scale, out=scaled)
        np.clip(scaled, 0.0, 255.0, out=out, casting="unsafe")
    else:
        np.clip(channels, 0, 255, out=out, casting="unsafe")
    return out


def depth_to_16uc1_mm(value: Any):
    """Metres -> 16UC1 millimetres; NaN/Inf/non-positive -> 0; clamp 65535.

    Byte-identical to ``_depth_to_16uc1_mm_reference`` (see
    ``tests/test_camera_publish_equivalence.py``); replaces the per-frame
    ``isfinite``/``&``/``where``/``rint``/``clip``/``astype`` temporaries with
    cached scratch buffers and in-place ``out=`` writes. The scale step uses
    the ``multiply`` ufunc's ``where=`` mask directly against a pre-zeroed
    buffer instead of computing a full ``scaled`` temporary and selecting
    into it with ``copyto`` — one fused masked pass instead of two full
    passes, which measurably wins at 1280x720 despite adding a per-call
    dispatch fixed cost that a plain ``np.where`` does not pay (see the
    micro-benchmark referenced in the commit message).
    """
    import numpy as np

    depth = to_numpy(value)
    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]
    if depth.ndim != 2:
        raise ValueError(f"depth frame must be HxW: {depth.shape}")
    shape = depth.shape
    if depth.dtype.kind != "f":
        upcast = _scratch("depth_upcast", shape, shape, np.float64)
        np.multiply(depth, 1.0, out=upcast, casting="unsafe")
        depth = upcast
    work_dtype = depth.dtype
    finite_mask = _scratch("depth_finite", shape, shape, np.bool_)
    positive_mask = _scratch("depth_positive", shape, shape, np.bool_)
    millimetres = _scratch("depth_mm", shape, shape, work_dtype)
    out = _scratch("depth_out", shape, shape, np.uint16)

    np.isfinite(depth, out=finite_mask)
    np.greater(depth, 0.0, out=positive_mask)
    np.logical_and(finite_mask, positive_mask, out=finite_mask)
    millimetres.fill(0.0)
    np.multiply(depth, 1000.0, out=millimetres, where=finite_mask)
    np.rint(millimetres, out=millimetres)
    np.clip(millimetres, 0.0, 65535.0, out=out, casting="unsafe")
    return out


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
            # Identity translation; rotate the USD camera into the mount's
            # optical convention so the render looks along the frame's +Z
            # axis. This is per-camera contract data, not a single constant:
            # a proper REP-103 optical mount (z forward, y down) takes
            # OPTICAL_TO_USD_CAMERA_WXYZ, (0, 1, 0, 0) - 180 deg about X. This
            # artifact's xarm wrist optical frame is instead authored y-up,
            # so it takes the y-flip variant (0, 0, 1, 0) - 180 deg about Y -
            # to avoid rendering upside down.
            # RtxCamera authors xformOp:orient as double precision (quatd);
            # match it or AddOrientOp raises a precision-mismatch Tf error.
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Quatd(*spec.mount_rotation_wxyz)
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
