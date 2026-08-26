import inspect
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.camera_rig import (  # noqa: E402
    AA_OP_DLAA,
    CameraRig,
    CameraStreamSpec,
    load_camera_specs,
)


def _spec(**kw):
    base = dict(name="arena_camera", color_topic="/sim/arena_camera/image_raw",
                depth_topic="", camera_info_topics=("/sim/arena_camera/camera_info",),
                frame_id="arena_camera_optical_frame", mount_prim="/World/ArenaCamera",
                mount_rotation_wxyz=(1.0, 0.0, 0.0, 0.0), width=960, height=540,
                horizontal_fov_deg=70.0, tick_rate_hz=4.0,
                mount_translation=(1.0, 2.0, 6.0))
    base.update(kw)
    return CameraStreamSpec(**base)


def test_parity_specs_unchanged():
    specs = load_camera_specs(ROOT / "simulation/sensors/hardware-parity.json")
    assert all(s.mount_translation is None for s in specs)
    assert all(s.depth_topic for s in specs)


def test_world_fixed_flag():
    s = _spec()
    assert s.mount_translation == (1.0, 2.0, 6.0)
    assert s.depth_topic == ""


def test_color_only_helper():
    from tinker_sim_isaac.camera_rig import is_color_only
    assert is_color_only(_spec())
    assert not is_color_only(_spec(depth_topic="/x"))


def test_aa_op_dlaa_is_the_native_resolution_op():
    # /rtx/post/aa/op: 3 = "DLSS" (Super Resolution, internal-res downscale
    # + live resize), 4 = "DLAA" (native resolution, no downscale/resize).
    # See CameraRig.initialize's docstring for why this distinction matters.
    assert AA_OP_DLAA == 4


def test_initialize_accepts_stable_aa_keyword():
    # Pure signature check (no GPU/Isaac needed): confirms the wiring point
    # run_sim.py's sensor-rich block depends on (camera_rig.initialize(app,
    # stable_aa=arena_camera_enabled)) actually exists on CameraRig.
    params = inspect.signature(CameraRig.initialize).parameters
    assert "stable_aa" in params
    assert params["stable_aa"].default is False
    assert params["stable_aa"].kind is inspect.Parameter.KEYWORD_ONLY
