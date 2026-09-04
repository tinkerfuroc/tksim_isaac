from __future__ import annotations

import json
import math
import os
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

#: ``/rtx/post/aa/op`` value for DLSS "DLAA" (Deep Learning Anti-Aliasing):
#: DLSS's neural denoiser/AA running at *native* resolution, i.e. with no
#: internal-resolution downscale (contrast the default op, 3 = "DLSS" Super
#: Resolution, which renders at a lower internal resolution and upscales).
#: See ``CameraRig.initialize(..., stable_aa=True)`` for why this matters.
AA_OP_DLAA = 4

#: Per-render-product token equivalent of ``AA_OP_DLAA``, for the
#: ``omni:rtx:post:aa:op`` USD attribute on an individual render product's
#: ``UsdRender.Product`` prim -- unlike the global ``/rtx/post/aa/op`` carb
#: setting (an int enum), this attribute is USD-schema-typed as a token
#: with a *different* label for the same DLAA op (confirmed against
#: ``omni.usd.schema.render_settings.rtx``'s own test, which asserts
#: ``settings.get("/rtx/post/aa/op") == 4`` maps to this exact token).
AA_OP_DLAA_TOKEN = "rtxaa"


#: Last-seen annotator buffer pointer per (camera, kind), used to log pool
#: churn events for the whole run while ``TINKER_SIM_CAMERA_DEBUG`` is on.
_DEBUG_LAST_PTRS: dict[tuple[str, str], set[int]] = {}


def _debug_annotator_array(camera: str, kind: str, arr: Any, *, full: bool) -> None:
    """Trace one annotator array's raw metadata (``TINKER_SIM_CAMERA_DEBUG``).

    Printed BEFORE ``capture()`` copies from or launches kernels on the
    array, so a first-cycle CUDA-700 repro shows exactly what the RTX
    annotator handed the rig (see ``.superpowers/arena-cam-debug/``).
    ``full`` dumps every call (first cycles); afterwards only NEW buffer
    pointers -- annotator pool churn -- are logged, which is the event that
    immediately preceded the reproduced crash.
    """
    if arr is None:
        if full:
            print(f"[camera-debug] {camera} {kind}: None", flush=True)
        return
    ptr = getattr(arr, "ptr", None)
    seen = _DEBUG_LAST_PTRS.setdefault((camera, kind), set())
    fresh = ptr is not None and ptr not in seen
    if ptr is not None:
        seen.add(ptr)
    if not full and not fresh:
        return
    tag = " NEW-BUFFER" if (fresh and not full) else ""
    print(
        f"[camera-debug]{tag} {camera} {kind}: "
        f"device={getattr(arr, 'device', '?')} "
        f"dtype={getattr(arr, 'dtype', '?')} "
        f"shape={getattr(arr, 'shape', '?')} "
        f"strides={getattr(arr, 'strides', '?')} "
        f"ptr={hex(ptr) if ptr else '?'}",
        flush=True,
    )


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
    #: World-fixed mount translation (metres, stage frame); ``None`` (the
    #: default) means "robot-mounted" -- unchanged behaviour, resolved by
    #: searching for ``mount_prim`` under ``/World/Tinker`` as today. A
    #: non-``None`` value means ``mount_prim`` is an absolute stage path
    #: (e.g. ``/World/ArenaCamera``) the rig creates directly, translated
    #: here and oriented by ``mount_rotation_wxyz``.
    mount_translation: tuple[float, float, float] | None = None
    #: Extra translation (metres) along the *rendered* camera's own view
    #: axis (local -Z, applied after ``mount_rotation_wxyz`` -- see
    #: ``CameraRig.initialize``), on top of whatever ``mount_prim`` and
    #: ``mount_rotation_wxyz`` already place it at. ``0.0`` (the default) is
    #: unchanged behaviour: identity translation, camera origin == mount
    #: origin. Non-zero is a dolly forward (positive) or backward (negative)
    #: along that axis; see ``tinker_sim_isaac.head_camera_aim`` for the one
    #: user of a non-zero value today (clearing the head camera's own
    #: housing mesh out of the corrected, level-forward view).
    view_axis_forward_offset_m: float = 0.0
    #: Translation (metres) in the *mount prim's own* frame, applied before
    #: ``mount_rotation_wxyz`` (see ``camera_xform_ops``): where the camera
    #: sits relative to the URDF link it is mounted on, when the description
    #: puts that link somewhere the camera is not. ``(0, 0, 0)`` (the
    #: default) is unchanged behaviour. The one user today is the wrist
    #: camera's ``cam-stand`` preset (``tinker_sim_isaac.head_camera_aim``),
    #: which moves the render origin from the description's placeholder
    #: flange mount onto xArm's D435 cam-stand bracket.
    mount_frame_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)


def camera_xform_ops(
    spec: CameraStreamSpec,
) -> tuple[tuple[str, tuple[float, ...]], ...]:
    """The rtx_camera prim's xformOpOrder for *spec*, outermost first.

    USD applies the LAST listed op to the geometry first, so the first op in
    this tuple is the one expressed in the mount prim's frame and the last
    is the one expressed in the camera's own (already oriented) frame:

    * ``mount_frame_offset_xyz`` -> ``translate`` listed BEFORE ``orient``:
      a bracket offset in the mount link's axes. Listing it after would
      rotate it with the aim (a wrist bracket would swing back onto the
      gripper housing the moment the aim tilts).
    * ``mount_rotation_wxyz`` -> ``orient``.
    * ``view_axis_forward_offset_m`` -> ``translate`` listed AFTER
      ``orient``: a dolly along the camera's own local -Z (its view axis).
      Verified empirically against ``UsdGeom.XformCache`` on the robot
      artifact -- swapping this order silently changes which axis the
      offset lands on.

    Pure data so the order is unit-testable without Kit;
    ``CameraRig.initialize`` authors exactly this sequence.
    """
    ops: list[tuple[str, tuple[float, ...]]] = []
    if any(spec.mount_frame_offset_xyz):
        ops.append(("translate", tuple(float(v) for v in spec.mount_frame_offset_xyz)))
    ops.append(("orient", tuple(float(v) for v in spec.mount_rotation_wxyz)))
    if spec.view_axis_forward_offset_m:
        ops.append(("translate", (0.0, 0.0, -float(spec.view_axis_forward_offset_m))))
    return tuple(ops)


def is_color_only(spec: CameraStreamSpec) -> bool:
    """True when ``spec`` publishes color only (no depth annotator/stream)."""
    return spec.depth_topic == ""


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


def load_spectator_spec(path: Path) -> CameraStreamSpec:
    """Load an optional world-fixed spectator camera (sim-only, not parity).

    Kept out of the hardware-parity contract on purpose: this camera has no
    real-robot counterpart, so it lives in its own opt-in spec file
    (``TINKER_SIM_SPECTATOR_CAMERA``). The mount must be an absolute stage
    path with an explicit world translation. Always color-only — a depth
    annotator on a pure observer stream would tax RTF for nothing.
    """
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    name = "spectator_camera"
    camera = raw.get(name)
    if not isinstance(camera, Mapping):
        raise ValueError(f"spectator spec is missing {name}")
    mount_prim = _string(camera, "mount_prim", name)
    if not mount_prim.startswith("/"):
        raise ValueError(f"{name}.mount_prim must be an absolute stage path")
    translation = camera.get("mount_translation_xyz")
    if (
        not isinstance(translation, list)
        or len(translation) != 3
        or any(
            isinstance(item, bool) or not isinstance(item, (int, float))
            for item in translation
        )
    ):
        raise ValueError(f"{name}.mount_translation_xyz must be three numbers")
    info_topics = camera.get("camera_info_topics")
    if (
        not isinstance(info_topics, list)
        or not info_topics
        or any(not isinstance(topic, str) or not topic for topic in info_topics)
    ):
        raise ValueError(f"{name}.camera_info_topics must be non-empty strings")
    return CameraStreamSpec(
        name=name,
        color_topic=_string(camera, "color_topic", name),
        depth_topic="",
        camera_info_topics=tuple(info_topics),
        frame_id=_string(camera, "frame_id", name),
        mount_prim=mount_prim,
        mount_rotation_wxyz=_quaternion_wxyz(camera, "mount_rotation_wxyz", name),
        width=int(_positive_number(camera, "width", name)),
        height=int(_positive_number(camera, "height", name)),
        horizontal_fov_deg=_positive_number(camera, "horizontal_fov_deg", name),
        tick_rate_hz=_positive_number(camera, "tick_rate_hz", name),
        mount_translation=tuple(float(item) for item in translation),
    )


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

    ``CameraRig.capture()`` now runs this exact metres->mm conversion on the
    GPU (a Warp kernel; see ``CameraRig._DEPTH_KERNEL``) and hands back an
    already-converted uint16 (H, W) buffer. A uint16 (H, W) input is
    therefore treated as pre-converted and passed through contiguous with no
    recomputation -- detected by dtype alone, so any uint16 (H, W) array
    (not just one produced by the GPU kernel) takes this path. Float (and
    other non-uint16) input is converted exactly as before.
    """
    import numpy as np

    depth = to_numpy(value)
    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]
    if depth.ndim != 2:
        raise ValueError(f"depth frame must be HxW: {depth.shape}")
    if depth.dtype == np.uint16:
        return np.ascontiguousarray(depth)
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


#: Lazily-built Warp kernel for the metres->16UC1-mm depth conversion,
#: cached at module scope so every ``CameraRig`` shares one compiled kernel.
#: Built lazily (not with a module-level ``@wp.kernel``) so importing this
#: module never requires ``warp`` to be installed -- only ``CameraRig``
#: (Isaac-only) touches it; ``depth_to_16uc1_mm``/tests run under plain
#: system Python with no ``warp`` available.
#:
#: Semantics mirror ``_depth_to_16uc1_mm_reference`` exactly for a single
#: scalar: NaN/Inf/non-positive -> 0, else round-half-to-even (``wp.rint``,
#: matching ``numpy.rint``; ``wp.round`` is round-half-away-from-zero and
#: disagrees at exact .5mm ties -- see outputs/bench/probe_gpu_depth_rgba.py)
#: then clamp to [0, 65535] before the uint16 cast. Proven bit-identical to
#: the reference on 120 real annotator frames plus a hand-picked edge set
#: (NaN, +-Inf, -1, 0, 65.5345, 65.5355, 0.0005, 0.0015, 1e-45, 70000) in
#: that probe; re-verified here in
#: ``tests/test_camera_publish_equivalence.py`` by running this exact
#: kernel (not a numpy re-implementation) on Warp's CPU device.
_DEPTH_KERNEL: Any = None


def _depth_to_mm_u16_kernel() -> Any:
    global _DEPTH_KERNEL
    if _DEPTH_KERNEL is None:
        import warp as wp

        @wp.kernel
        def depth_to_mm_u16(
            depth: wp.array(dtype=wp.float32), out_arr: wp.array(dtype=wp.uint16)
        ):
            i = wp.tid()
            x = depth[i]
            mm = wp.float32(0.0)
            if wp.isfinite(x) and x > wp.float32(0.0):
                mm = x * wp.float32(1000.0)
                mm = wp.rint(mm)
                mm = wp.clamp(mm, wp.float32(0.0), wp.float32(65535.0))
            out_arr[i] = wp.uint16(mm)

        _DEPTH_KERNEL = depth_to_mm_u16
    return _DEPTH_KERNEL


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
        #: Per-camera (rgb, depth) pinned host wp.array buffers, sized to the
        #: exact post-conversion shape (depth is post metres->16UC1-mm, done
        #: on the GPU; see ``capture()``). Reused every ``capture()`` call so
        #: the per-frame D2H copy lands in existing pinned memory instead of
        #: a fresh pageable allocation.
        self._pinned: dict[str, tuple[Any, Any]] = {}
        #: Per-camera device-side uint16 scratch the depth kernel writes
        #: into, sized (height, width); copied to the pinned host uint16
        #: buffer above. Kept device-resident between frames like the pinned
        #: host buffers are.
        self._depth_gpu_out: dict[str, Any] = {}
        #: Names of specs with no depth annotator/stream (``is_color_only``);
        #: ``capture()`` skips the depth branch entirely for these.
        self._color_only: set[str] = set()
        #: Cameras ticking slower than the rig's fastest camera. Their
        #: annotator buffers go STALE between renders, and re-reading a
        #: stale buffer while the RTX annotator pool reallocates under
        #: load is the reproduced CUDA-700 crash (arena camera, 4 Hz tick
        #: vs 12 Hz capture poll; see ``capture()`` and
        #: ``.superpowers/arena-cam-debug/findings-phase1.md`` E1-E6).
        #: ``capture()`` consumes these cameras' arrays only when freshly
        #: rendered (buffer pointer changed) and otherwise serves the
        #: pinned copy it already made.
        fastest = max(spec.tick_rate_hz for spec in self.specs)
        self._subrate: set[str] = {
            spec.name for spec in self.specs if spec.tick_rate_hz < fastest
        }
        #: Last annotator buffer pointer consumed per (camera, kind) for
        #: sub-rate cameras; equality means "same buffer we already
        #: copied" (the RTX pool alternates >=2 buffers per stream, so a
        #: fresh render always lands at a different pointer than the one
        #: consumed last).
        self._consumed_ptrs: dict[tuple[str, str], int] = {}
        #: Debug aid (``TINKER_SIM_CAMERA_DEBUG=1``): number of remaining
        #: ``capture()`` cycles that trace each camera's raw annotator
        #: array metadata (device/shape/strides/ptr) BEFORE any copy or
        #: kernel launch touches it. Used to pin down the CUDA-700
        #: first-cycle race (see ``.superpowers/arena-cam-debug/``).
        self._debug_cycles_left: int = (
            10 if os.environ.get("TINKER_SIM_CAMERA_DEBUG") == "1" else 0
        )

    def initialize(
        self,
        app: Any,
        *,
        stable_aa: bool = False,
        stable_aa_cameras: frozenset[str] | None = None,
    ) -> None:
        """Create this rig's RTX cameras.

        ``stable_aa``: force DLSS to its native-resolution ``DLAA`` op
        (``AA_OP_DLAA``) instead of its default ``DLSS`` Super Resolution op
        *before* any render product in ``self.specs`` is created.

        ``stable_aa_cameras``: when given (and ``stable_aa`` is set), scope
        the DLAA pin to only the named cameras' render products, authoring
        ``omni:rtx:post:aa:op`` directly on each one's ``UsdRender.Product``
        prim instead of touching the global ``/rtx/post/aa/op`` setting --
        confirmed per-render-product-overridable by probing a live render
        product's authored attributes (see the developer log, Task 4a).
        ``None`` (the default) keeps the old global-setting behaviour, which
        pins every render product regardless of name. An empty
        ``stable_aa_cameras`` set is not the same as ``None``: with
        ``stable_aa=True`` and an empty set, no camera's name is ever a
        member, so the pin is authored on nothing -- by design, not a bug.
        Callers that want the global pin must pass ``None``, not ``frozenset()``.

        Original hypothesis (2026-08-26): DLSS's default op auto-picks an
        internal render resolution below the render product's declared
        output resolution, then live-resizes the render target up if that
        pick falls under DLSS's ~300 px minimum input size (observed as
        "DLSS increasing input dimensions" for the 848x480 wrist camera,
        whose default-picked internal size is 424x240 -- both under 300).
        With 3+ concurrent RTX camera render products alive (hardware-parity
        head+wrist plus the world-fixed arena observer camera), that live
        resize has raced this rig's Warp D2H copy/synchronize in
        ``capture()``: ~15 s after the resize, `wp_cuda_stream_synchronize`
        reports CUDA error 700 ("illegal memory access"), followed by an
        RT-pipeline/semaphore cascade -- reproduced 3/3 in
        ``gpsr_stack_logs/2026082[56]T*`` runs with the arena camera on, 0/2
        with only the two hardware-parity cameras. DLAA never downscales
        (input resolution == output resolution, always), so the resize --
        and the race it enables -- cannot happen. Scoped to ``stable_aa``
        (set only when the arena camera pushes the render-product count to
        3+; see ``run_sim.py``) so hardware-parity-only runs keep their
        previously-verified-stable default AA behaviour unchanged.

        Further scoped to just the arena camera's render product via
        ``stable_aa_cameras`` (see ``run_sim.py``): the wrist camera's
        live-resize documented above still happens under this narrower
        scoping -- head and wrist keep the default DLSS op, resize
        included -- because the CUDA-700 race was root-caused separately to
        stale sub-rate annotator reads and fixed independently in a9fa951
        (``capture()`` now consumes an annotator's buffer only once freshly
        rendered, not on every poll). With that race closed, the wrist
        resize is believed harmless on its own; that belief rests on a
        clean 3/3 x 5 min full-stack crash recipe run 2026-08-29 with the
        pin scoped this way (see the developer log, Task 4a) and would need
        re-testing if error 700 returns. The pin stays on the arena
        camera's render product as defence in depth for the one product
        added after the original crash -- at negligible cost, since Task 3 /
        Phase 0 measured the global pin taxing the 12 Hz parity renders by
        ~50 ms per Kit pump (``scripts/arena-rtf-spike`` variant D) for no
        benefit to cameras that were never implicated in the race.
        """
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.sensors.experimental.rtx")
        for _ in range(4):
            app.update()

        if stable_aa and stable_aa_cameras is None:
            import carb.settings

            carb.settings.get_settings().set_int("/rtx/post/aa/op", AA_OP_DLAA)

        import omni.usd
        import warp as wp
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
        from pxr import Gf, Usd, UsdGeom

        if os.environ.get("TINKER_SIM_WARP_VERIFY") == "1":
            # Debug aid: have Warp check ``cudaGetLastError`` after every
            # launch/copy so an async illegal access (CUDA 700) is
            # attributed to the exact Warp call instead of the next
            # ``synchronize_stream`` (see ``.superpowers/arena-cam-debug/``).
            wp.config.verify_cuda = True

        stage = omni.usd.get_context().get_stage()
        robot = stage.GetPrimAtPath("/World/Tinker")
        if not robot.IsValid():
            raise RuntimeError("camera rig requires the spawned /World/Tinker robot")
        for spec in self.specs:
            if spec.mount_prim.startswith("/"):
                # World-fixed mount: an absolute stage path this rig owns
                # outright (e.g. ``/World/ArenaCamera``), not a named prim
                # searched for under the robot. Create it directly and place
                # it with ``mount_translation``; the RtxCamera child below
                # gets the usual orient treatment.
                if spec.mount_translation is None:
                    raise RuntimeError(
                        f"{spec.mount_prim!r} is an absolute mount path but "
                        "has no mount_translation"
                    )
                mount_prim = UsdGeom.Xform.Define(stage, spec.mount_prim).GetPrim()
                UsdGeom.Xformable(mount_prim).AddTranslateOp(
                    UsdGeom.XformOp.PrecisionDouble
                ).Set(Gf.Vec3d(*spec.mount_translation))
                mount_path = mount_prim.GetPath().pathString
            else:
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
                mount_path = mounts[0].GetPath().pathString
            camera_path = f"{mount_path}/rtx_camera"
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
            # Op order matters and is documented (and unit-tested) on
            # camera_xform_ops: a mount-frame bracket offset goes BEFORE
            # orient, the view-axis dolly AFTER it. Two translate ops need
            # distinct suffixes or the second AddTranslateOp raises.
            for index, (kind, value) in enumerate(camera_xform_ops(spec)):
                if kind == "orient":
                    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
                        Gf.Quatd(*value)
                    )
                else:
                    xform.AddTranslateOp(
                        UsdGeom.XformOp.PrecisionDouble, f"op{index}"
                    ).Set(Gf.Vec3d(*value))
            color_only = is_color_only(spec)
            if color_only:
                self._color_only.add(spec.name)
            annotators = (
                [COLOR_ANNOTATOR] if color_only else [COLOR_ANNOTATOR, DEPTH_ANNOTATOR]
            )
            self._sensors[spec.name] = CameraSensor(
                camera,
                resolution=(spec.height, spec.width),
                annotators=annotators,
            )
            if (
                stable_aa
                and stable_aa_cameras is not None
                and spec.name in stable_aa_cameras
            ):
                # Per-product DLAA pin: author the same op the global
                # setting above would otherwise set everywhere, but only on
                # this camera's own render product prim, so the parity
                # cameras (not in ``stable_aa_cameras``) keep their default,
                # cheaper AA op untouched. The attribute is already present
                # (pre-authored by the render-product schema, confirmed by
                # probing a live prim -- see the developer log, Task 4a), so
                # ``GetAttribute`` finds it rather than needing to create
                # it -- but it is USD-token-typed, not int-typed like the
                # global carb setting, hence ``AA_OP_DLAA_TOKEN`` and not
                # ``AA_OP_DLAA`` here (a first attempt with the int raised
                # ``Type mismatch ... expected 'TfToken', got 'int'``; see
                # the developer log, Task 4a).
                self._sensors[spec.name].render_product.GetPrim().GetAttribute(
                    "omni:rtx:post:aa:op"
                ).Set(AA_OP_DLAA_TOKEN)
            # Pinned (page-locked) host buffers, sized to what rgb8_array/
            # depth_to_16uc1_mm expect (post RGBA->RGB slice for rgb; already
            # metres->16UC1-mm converted, on the GPU, for depth -- see
            # capture()). Pinned memory lets the D2H copy in capture() run as
            # a real async DMA instead of the synchronous-by-necessity copy
            # a pageable destination forces. Color-only streams (``depth_topic
            # == ""``) get no depth annotator, so no depth pinned buffer or
            # GPU scratch is allocated for them either.
            self._pinned[spec.name] = (
                wp.empty((spec.height, spec.width, 3), dtype=wp.uint8, device="cpu", pinned=True),
                None
                if color_only
                else wp.empty(
                    (spec.height, spec.width), dtype=wp.uint16, device="cpu", pinned=True
                ),
            )
            if not color_only:
                self._depth_gpu_out[spec.name] = wp.zeros(
                    (spec.height, spec.width), dtype=wp.uint16, device="cuda"
                )
        # JIT-compile the depth kernel now, on a throwaway array, so the
        # first real frame doesn't pay Warp's first-launch compile cost.
        _warm_kernel = _depth_to_mm_u16_kernel()
        _warm_in = wp.zeros(1, dtype=wp.float32, device="cuda")
        _warm_out = wp.zeros(1, dtype=wp.uint16, device="cuda")
        wp.launch(_warm_kernel, dim=1, inputs=[_warm_in, _warm_out])
        wp.synchronize_device(_warm_in.device)
        for _ in range(4):
            app.update()

    def capture(self) -> dict[str, tuple[Any, Any]]:
        """Fetch each camera's RGB+depth into reused pinned host buffers.

        ``sensor.get_data`` returns a GPU-side wp.array (a strided RGBA->RGB
        view for rgb, a contiguous buffer for depth); rgb is copied with
        ``wp.copy`` straight into this rig's pinned host scratch for that
        camera instead of letting ``to_numpy()`` do its usual
        ``warp.clone(device="cpu")`` (a fresh pageable allocation every
        frame). depth instead runs through ``_depth_to_mm_u16_kernel()`` on
        the GPU first (metres -> 16UC1 mm, byte-identical to
        ``depth_to_16uc1_mm``/``_depth_to_16uc1_mm_reference`` -- see that
        kernel's docstring and ``tests/test_camera_publish_equivalence.py``)
        into this camera's device uint16 scratch, and only the uint16
        result -- half the bytes of the float32 source -- is D2H-copied into
        the pinned host buffer. Because the pinned destination makes the D2H
        copy a real async DMA, all four copies/launches (two cameras x
        rgb/depth) are enqueued before a single ``wp.synchronize_stream``
        blocks for all of them, instead of each ``to_numpy()`` call
        synchronizing on its own.

        Deliberately ``synchronize_stream``, not ``synchronize_device``:
        ``wp.copy`` with a non-CUDA destination runs on the *source*
        array's current stream (see ``warp.copy``), the same stream the RTX
        annotator pipeline enqueues its own GPU work on; a kernel launched
        with no explicit ``stream=``/``device=`` also runs on that array's
        device's current stream, so it lands on the same stream as the
        annotator's own work and the pinned copies that follow it. Waiting
        for just that stream to drain already makes the copied data valid.
        ``wp.synchronize_device`` instead blocks on the whole CUDA context
        -- every other stream Kit has outstanding work on too, including
        the renderer's passes for frames we don't even want yet -- which a
        probe measured (outputs/bench/probe_camera.py) as 4-7x slower than
        ``synchronize_stream`` for the exact same copies, i.e. worse than
        the ``to_numpy()`` baseline this is meant to beat.

        The returned rgb array is byte-identical to what ``to_numpy()``
        would have produced from the un-pinned GPU array (same underlying
        gather + copy); only the destination allocation and synchronization
        scope change. The returned depth array is byte-identical to what
        ``depth_to_16uc1_mm(to_numpy(<un-pinned GPU depth array>))`` would
        have produced; ``depth_to_16uc1_mm`` treats a uint16 (H, W) array as
        already converted and passes it through unchanged, so callers keep
        calling it exactly as before.
        """
        import warp as wp

        debug_enabled = self._debug_cycles_left != 0 or bool(_DEBUG_LAST_PTRS)
        debug_full = self._debug_cycles_left > 0
        if debug_full:
            self._debug_cycles_left -= 1
        debug = debug_enabled
        # Debug aid (TINKER_SIM_CAPTURE_SKIP=<name>[,<name>..]): leave these
        # cameras' render products alive but never consume their annotator
        # arrays -- discriminates "our Warp reads trigger the CUDA 700" from
        # "the renderer faults on its own" (.superpowers/arena-cam-debug/).
        skip = {
            s for s in os.environ.get("TINKER_SIM_CAPTURE_SKIP", "").split(",") if s
        }
        frames: dict[str, tuple[Any, Any]] = {}
        sync_device = None
        for name, sensor in self._sensors.items():
            if name in skip:
                frames[name] = (None, None)
                continue
            rgb, _info = sensor.get_data(COLOR_ANNOTATOR)
            if debug:
                _debug_annotator_array(name, "rgb", rgb, full=debug_full)
            rgb_pinned, depth_pinned = self._pinned[name]
            if rgb is not None:
                if name in self._subrate and self._consumed_ptrs.get(
                    (name, "rgb")
                ) == rgb.ptr:
                    # Same buffer as last consume: the sub-rate camera has
                    # not rendered since. Serve the pinned copy already
                    # made instead of re-reading a device buffer the
                    # annotator pool may reclaim mid-copy (the reproduced
                    # CUDA-700; full-rate cameras never hit this branch --
                    # their buffers are rewritten every cycle).
                    rgb = rgb_pinned
                else:
                    if name in self._subrate:
                        self._consumed_ptrs[(name, "rgb")] = rgb.ptr
                    wp.copy(rgb_pinned, rgb)
                    sync_device = rgb.device
                    rgb = rgb_pinned
            if name in self._color_only:
                # No depth annotator was created for this camera (see
                # ``initialize()``); nothing to fetch or convert.
                frames[name] = (rgb, None)
                continue
            depth, _info = sensor.get_data(DEPTH_ANNOTATOR)
            if debug:
                _debug_annotator_array(name, "depth", depth, full=debug_full)
            if (
                depth is not None
                and name in self._subrate
                and self._consumed_ptrs.get((name, "depth")) == depth.ptr
            ):
                frames[name] = (rgb, depth_pinned)
                continue
            if depth is not None:
                if name in self._subrate:
                    self._consumed_ptrs[(name, "depth")] = depth.ptr
                depth_gpu_out = self._depth_gpu_out[name]
                n = depth_gpu_out.size
                wp.launch(
                    _depth_to_mm_u16_kernel(),
                    dim=n,
                    inputs=[depth.reshape(n), depth_gpu_out.reshape(n)],
                )
                wp.copy(depth_pinned, depth_gpu_out)
                sync_device = depth.device
                depth = depth_pinned
            frames[name] = (rgb, depth)
        if sync_device is not None:
            wp.synchronize_stream(sync_device)
        return frames
