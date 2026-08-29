"""World-fixed "arena observer" camera: pose math, spec, and env gate.

Recorded GPSR battery runs need a bird's-eye recording of the arena that
survives regardless of where the robot or its head/wrist cameras are
looking. This module is pure (no ROS, no Isaac, no GPU) so it is unit
testable under plain system Python: it derives the observer's pose from
the arena's occupancy grid, converts that pose to the RTX-camera-ready
``CameraStreamSpec`` (Task 2's world-fixed, color-only extension), and
resolves whether/how fast the stream should run from the environment.
"""

from __future__ import annotations

import math
from typing import Mapping

from tinker_sim_isaac.camera_rig import CameraStreamSpec

#: ``look_at_wxyz`` authors a y-up view frame (like the xarm wrist optical
#: frame -- see ``CameraRig.initialize``'s orient-op comment), so mapping it
#: onto the USD camera (-Z forward, +Y up) takes the y-flip variant, 180
#: degrees about Y. The x-flip (``OPTICAL_TO_USD_CAMERA_WXYZ``) also faces
#: the camera correctly but renders the arena rolled 180 degrees (verified
#: live 2026-08-27: people rendered head-down).
YUP_VIEW_TO_USD_CAMERA_WXYZ = (0.0, 0.0, 1.0, 0.0)

#: Opt-in gate: unset/falsy disables the arena camera entirely (its
#: mount is never created, no topics are advertised).
ARENA_CAMERA_ENV = "TINKER_SIM_ARENA_CAMERA"
#: Optional override for the publish rate (Hz); may only lower the
#: default, never raise it -- a low, coarse rate is the entire point of a
#: fixed overview camera that only needs to prove where things ended up.
#: Measured 2026-08-29 (Task 3 table, top entry of docs/developer-log.md):
#: with the DLAA fix in place, 2 Hz vs 0.5 Hz and 960 vs 640 px moved RTF
#: by less than ~2%, so these defaults are a free cut, not a real tradeoff.
ARENA_CAMERA_HZ_ENV = "TINKER_SIM_ARENA_CAMERA_HZ"
ARENA_CAMERA_DEFAULT_HZ = 2.0

#: Optional override for the render size, ``WIDTHxHEIGHT``; may only lower
#: either dimension (the bird's-eye view cannot resolve 10 cm objects at
#: the default size anyway, so smaller is never a fidelity loss that
#: matters -- and every arena pixel is paid for on the sim's GPU). 640x360
#: still shows a recognisable person and table in the bird's-eye frame; see
#: the HZ comment above for the 2026-08-29 measurement this default relies on.
ARENA_CAMERA_SIZE_ENV = "TINKER_SIM_ARENA_CAMERA_SIZE"
ARENA_CAMERA_DEFAULT_SIZE = (640, 360)

_TRUTHY = {"1", "true", "yes"}


def arena_camera_pose(
    occupancy: object,
) -> tuple[list[float], list[float], list[float]]:
    """Eye/target/bounds for a bird's-eye view of the whole arena.

    Moved verbatim from ``validation/run_sim.py``'s ``_arena_camera_pose``;
    see that module for the thin wrapper kept for callers.
    """
    width = int(getattr(occupancy, "width"))
    height = int(getattr(occupancy, "height"))
    resolution = float(getattr(occupancy, "resolution"))
    origin_x = float(getattr(occupancy, "origin_x"))
    origin_y = float(getattr(occupancy, "origin_y"))
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError("arena occupancy dimensions and resolution must be positive")
    size_x = width * resolution
    size_y = height * resolution
    span = max(size_x, size_y)
    center = [origin_x + size_x / 2.0, origin_y + size_y / 2.0, 0.5]
    eye = [center[0] - 0.75 * span, center[1] - 0.75 * span, 0.90 * span]
    bounds = [origin_x, origin_y, origin_x + size_x, origin_y + size_y]
    return eye, center, bounds


def look_at_wxyz(
    eye: tuple[float, float, float], target: tuple[float, float, float]
) -> tuple[float, float, float, float]:
    """Orientation (wxyz) whose optical +Z points from ``eye`` at ``target``.

    Builds an orthonormal basis with optical +Z = normalized(target - eye),
    +X = normalize(cross(world_up, Z)) (world_up = (0, 0, 1), falling back
    to (0, 1, 0) when nearly parallel to Z), +Y = cross(Z, X); the 3x3
    matrix with columns (X, Y, Z) is converted to a quaternion via the
    standard trace method.
    """
    zx, zy, zz = (target[0] - eye[0], target[1] - eye[1], target[2] - eye[2])
    z_norm = math.sqrt(zx * zx + zy * zy + zz * zz)
    if z_norm <= 0.0:
        raise ValueError("look_at_wxyz requires eye and target to differ")
    zx, zy, zz = (zx / z_norm, zy / z_norm, zz / z_norm)

    up = (0.0, 0.0, 1.0)
    # cross(up, Z)
    xx = up[1] * zz - up[2] * zy
    xy = up[2] * zx - up[0] * zz
    xz = up[0] * zy - up[1] * zx
    x_norm = math.sqrt(xx * xx + xy * xy + xz * xz)
    if x_norm < 1e-6:
        up = (0.0, 1.0, 0.0)
        xx = up[1] * zz - up[2] * zy
        xy = up[2] * zx - up[0] * zz
        xz = up[0] * zy - up[1] * zx
        x_norm = math.sqrt(xx * xx + xy * xy + xz * xz)
        if x_norm < 1e-6:
            raise ValueError("look_at_wxyz: direction too close to both up candidates")
    xx, xy, xz = (xx / x_norm, xy / x_norm, xz / x_norm)

    # Y = cross(Z, X)
    yx = zy * xz - zz * xy
    yy = zz * xx - zx * xz
    yz = zx * xy - zy * xx

    # Columns (X, Y, Z) -> rotation matrix, row-major.
    m00, m01, m02 = xx, yx, zx
    m10, m11, m12 = xy, yy, zy
    m20, m21, m22 = xz, yz, zz

    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return (w, x, y, z)


def resolve_arena_camera(env: Mapping[str, str]) -> float | None:
    """Arena camera publish rate (Hz) from ``env``, or ``None`` if disabled.

    Enabled iff ``env[ARENA_CAMERA_ENV]`` is a truthy literal ("1", "true",
    "yes", case-insensitive). When enabled, the rate is
    ``min(ARENA_CAMERA_DEFAULT_HZ, override)`` if
    ``env[ARENA_CAMERA_HZ_ENV]`` is set (a non-numeric override raises
    ``ValueError``), else ``ARENA_CAMERA_DEFAULT_HZ``.
    """
    raw = env.get(ARENA_CAMERA_ENV)
    if raw is None or raw.strip().lower() not in _TRUTHY:
        return None
    override = env.get(ARENA_CAMERA_HZ_ENV)
    if override is None:
        return ARENA_CAMERA_DEFAULT_HZ
    try:
        override_hz = float(override)
    except ValueError as exc:
        raise ValueError(
            f"{ARENA_CAMERA_HZ_ENV} must be numeric, got {override!r}"
        ) from exc
    return min(ARENA_CAMERA_DEFAULT_HZ, override_hz)


def resolve_arena_camera_size(env: Mapping[str, str]) -> tuple[int, int]:
    """Arena render size ``(width, height)`` from ``env``.

    ``env[ARENA_CAMERA_SIZE_ENV]`` is ``WIDTHxHEIGHT`` (positive integers);
    each dimension is clamped to ``ARENA_CAMERA_DEFAULT_SIZE`` (the
    override may only lower). Unset -> the default. Malformed -> ValueError.
    """
    raw = env.get(ARENA_CAMERA_SIZE_ENV)
    if raw is None:
        return ARENA_CAMERA_DEFAULT_SIZE
    parts = raw.lower().split("x")
    if len(parts) != 2 or not all(p.isdigit() for p in parts):
        raise ValueError(
            f"{ARENA_CAMERA_SIZE_ENV} must be WIDTHxHEIGHT, got {raw!r}"
        )
    width, height = (int(p) for p in parts)
    if width <= 0 or height <= 0:
        raise ValueError(f"{ARENA_CAMERA_SIZE_ENV} dimensions must be positive")
    return (
        min(ARENA_CAMERA_DEFAULT_SIZE[0], width),
        min(ARENA_CAMERA_DEFAULT_SIZE[1], height),
    )


def quat_mul_wxyz(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    """Hamilton product ``a * b`` for wxyz quaternions (apply ``b`` first)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def arena_camera_spec(
    occupancy: object, *, hz: float, size: tuple[int, int] = ARENA_CAMERA_DEFAULT_SIZE
) -> CameraStreamSpec:
    """The world-fixed, color-only ``CameraStreamSpec`` for the arena camera."""
    eye, target, _bounds = arena_camera_pose(occupancy)
    width, height = size
    return CameraStreamSpec(
        name="arena_camera",
        color_topic="/sim/arena_camera/image_raw",
        depth_topic="",
        camera_info_topics=("/sim/arena_camera/camera_info",),
        frame_id="arena_camera_optical_frame",
        mount_prim="/World/ArenaCamera",
        # ``look_at_wxyz`` orients the OPTICAL frame (+Z at the target),
        # but the rig authors this quaternion directly onto the USD camera
        # prim, which renders along its local -Z (see
        # ``CameraRig.initialize``'s orient-op comment: mount_rotation is
        # per-camera contract data that already includes the optical->USD
        # correction for the robot cameras). Without the composed flip the
        # arena camera faces exactly backward and renders only the dome
        # light -- uniform grey frames, first observed on the first boot
        # that survived arena capture (2026-08-27).
        mount_rotation_wxyz=quat_mul_wxyz(
            look_at_wxyz(eye, target), YUP_VIEW_TO_USD_CAMERA_WXYZ
        ),
        width=width,
        height=height,
        horizontal_fov_deg=70.0,
        tick_rate_hz=hz,
        mount_translation=tuple(eye),
    )
