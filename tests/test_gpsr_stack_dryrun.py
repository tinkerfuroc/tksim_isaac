"""Task 6: gpsr-stack dry-run / pure stage_commands() tests.

These tests never launch a process: they only exercise the pure
``stage_commands(cfg)`` function and StackConfig, plus the bench scenario's
JSON shape. ``up``/``down``/``status`` (which do spawn subprocesses) are
exercised only manually via ``--dry-run``, never here.
"""
import importlib.util
import sys
from importlib.machinery import SourceFileLoader
from pathlib import Path

# scripts/gpsr-stack has no .py suffix, so spec_from_file_location needs an
# explicit loader to recognize it as source (without one it returns None).
_SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "gpsr-stack"
spec = importlib.util.spec_from_file_location(
    "gpsr_stack", _SCRIPT_PATH, loader=SourceFileLoader("gpsr_stack", str(_SCRIPT_PATH)))
mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mod  # dataclasses needs the module registered to resolve types
spec.loader.exec_module(mod)


def _cfg(**kw):
    base = dict(scenario="gpsr-rcw2026-bench", seed=0, manipulation="mock",
                manip_gpu=None, sim_gpu=0, log_dir="gpsr_stack_logs")
    base.update(kw)
    return mod.StackConfig(**base)


def test_mock_skips_stage4():
    names = [s["name"] for s in mod.stage_commands(_cfg())]
    assert names == ["sim", "bridge", "vision", "nav_extras"]


def test_live_includes_stage4_with_gpu():
    stages = mod.stage_commands(_cfg(manipulation="live", manip_gpu=1))
    manip = [s for s in stages if s["name"] == "manipulation"][0]
    assert manip["env"]["CUDA_VISIBLE_DEVICES"] == "1"


def test_sim_stage_env():
    sim = mod.stage_commands(_cfg())[0]
    env = sim["env"]
    assert env["ROS_DOMAIN_ID"] == "42"
    assert env["TINKER_SIM_ARENA_CAMERA"] == "1"
    assert "--scenario" in sim["cmd"] and "gpsr-rcw2026-bench" in sim["cmd"]


# --- Fix round 3: two-render-product cap (wrist off in hybrid, arena off in
# live) --------------------------------------------------------------------

def test_mock_mode_sim_env_disables_wrist_keeps_arena():
    sim = mod.stage_commands(_cfg(manipulation="mock"))[0]
    env = sim["env"]
    assert env["TINKER_SIM_ARENA_CAMERA"] == "1"
    assert env["TINKER_SIM_DISABLE_WRIST_CAMERA"] == "1"


def test_live_mode_sim_env_disables_arena_keeps_wrist():
    sim = mod.stage_commands(_cfg(manipulation="live", manip_gpu=1))[0]
    env = sim["env"]
    assert env["TINKER_SIM_ARENA_CAMERA"] == "0"
    assert "TINKER_SIM_DISABLE_WRIST_CAMERA" not in env


def test_mock_mode_sim_gate_excludes_wrist_topics():
    stacks = mod._gate_census_stacks(_cfg(manipulation="mock"))
    assert stacks["sim"] == ("sim cameras",)
    assert "sim cameras wrist" not in stacks["sim"]


def test_live_mode_sim_gate_requires_wrist_topics():
    stacks = mod._gate_census_stacks(_cfg(manipulation="live", manip_gpu=1))
    assert "sim cameras wrist" in stacks["sim"]
    assert "sim cameras" in stacks["sim"]


def test_scenario_json_has_two_actors():
    import json
    data = json.loads((Path(__file__).resolve().parents[1] /
                       "simulation/scenarios/gpsr-rcw2026-bench.json").read_text())
    assert len(data["actors"]) == 2


def test_sim_stage_default_cuda_visible_devices():
    """Default sim_gpu (0) → CUDA_VISIBLE_DEVICES == '0'."""
    sim = mod.stage_commands(_cfg())[0]
    assert sim["env"]["CUDA_VISIBLE_DEVICES"] == "0"


def test_sim_stage_custom_cuda_visible_devices():
    """Custom sim_gpu (1) → CUDA_VISIBLE_DEVICES == '1'."""
    sim = mod.stage_commands(_cfg(sim_gpu=1))[0]
    assert sim["env"]["CUDA_VISIBLE_DEVICES"] == "1"


def test_only_sim_stage_has_sim_gpu_cuda_visible_devices():
    """Only sim stage should have CUDA_VISIBLE_DEVICES from sim_gpu.

    Other stages (vision, bridge, nav_extras, manipulation) should not gain
    CUDA_VISIBLE_DEVICES from the sim_gpu change. Manipulation's existing
    manip_gpu behavior should remain unchanged.
    """
    stages = mod.stage_commands(_cfg(manipulation="live", manip_gpu=1, sim_gpu=1))
    sim_stage = [s for s in stages if s["name"] == "sim"][0]
    bridge_stage = [s for s in stages if s["name"] == "bridge"][0]
    vision_stage = [s for s in stages if s["name"] == "vision"][0]
    manip_stage = [s for s in stages if s["name"] == "manipulation"][0]
    nav_stage = [s for s in stages if s["name"] == "nav_extras"][0]

    # Sim stage should have CUDA_VISIBLE_DEVICES from sim_gpu
    assert sim_stage["env"]["CUDA_VISIBLE_DEVICES"] == "1"

    # Bridge stage should NOT have CUDA_VISIBLE_DEVICES
    assert "CUDA_VISIBLE_DEVICES" not in bridge_stage["env"]

    # Vision stage should NOT have CUDA_VISIBLE_DEVICES
    assert "CUDA_VISIBLE_DEVICES" not in vision_stage["env"]

    # Manipulation stage should still have CUDA_VISIBLE_DEVICES from manip_gpu
    assert manip_stage["env"]["CUDA_VISIBLE_DEVICES"] == "1"

    # Nav stage should NOT have CUDA_VISIBLE_DEVICES
    assert "CUDA_VISIBLE_DEVICES" not in nav_stage["env"]


# --- extra coverage beyond the brief's minimum, still pure-function only ---

def test_live_requires_manip_gpu():
    import pytest
    with pytest.raises(ValueError):
        mod.stage_commands(_cfg(manipulation="live", manip_gpu=None))


def test_stage_order_live():
    names = [s["name"] for s in mod.stage_commands(_cfg(manipulation="live", manip_gpu=1))]
    assert names == ["sim", "bridge", "vision", "manipulation", "nav_extras"]


def test_gates_present_and_known():
    valid = {"sim", "bridge", "vision", "manipulation", "nav"}
    stages = mod.stage_commands(_cfg(manipulation="live", manip_gpu=1))
    for s in stages:
        assert s["gate"] in valid
    by_name = {s["name"]: s["gate"] for s in stages}
    assert by_name["sim"] == "sim"
    assert by_name["bridge"] == "bridge"
    assert by_name["vision"] == "vision"
    assert by_name["manipulation"] == "manipulation"
    assert by_name["nav_extras"] == "nav"


def test_vision_stage_has_three_popens_including_named_camera_server_and_pan_tilt():
    stages = mod.stage_commands(_cfg())
    vision = [s for s in stages if s["name"] == "vision"][0]
    cmd = vision["cmd"]
    assert isinstance(cmd[0], list), "vision stage must be multiple Popens"
    flat = [" ".join(c) for c in cmd]
    assert any("vision_bringup" in c for c in flat)
    assert any("__node:=head_camera_server" in c for c in flat)
    assert any("pan_tilt" in c and "state_publisher" in c for c in flat)


def test_every_stage_has_required_keys():
    for s in mod.stage_commands(_cfg(manipulation="live", manip_gpu=1)):
        assert set(("name", "cmd", "env", "cwd", "gate")) <= set(s.keys())


def test_stage_commands_pure_no_side_effects(tmp_path, monkeypatch):
    # Calling stage_commands must not touch the filesystem or subprocess.
    import subprocess
    def _boom(*a, **kw):
        raise AssertionError("stage_commands must not spawn subprocesses")
    monkeypatch.setattr(subprocess, "Popen", _boom)
    monkeypatch.setattr(subprocess, "run", _boom)
    mod.stage_commands(_cfg(manipulation="live", manip_gpu=1))


def test_dry_run_cli_smoke(capsys):
    # up --dry-run must not spawn anything and must exit 0.
    rc = mod.main(["up", "--scenario", "gpsr-rcw2026-bench", "--seed", "0",
                   "--manipulation", "mock", "--dry-run",
                   "--log-dir", "gpsr_stack_logs"])
    assert rc == 0
    out = capsys.readouterr().out
    assert "sim" in out and "bridge" in out and "vision" in out and "nav_extras" in out


# --- Task 6 review fix round 1: _run_up teardown-on-failure, census ROS env -

class _FakePopenProc:
    """Stands in for a Popen result: only ``.pid`` is read by _run_up."""

    def __init__(self, pid: int):
        self.pid = pid


def test_run_up_tears_down_already_started_stages_on_gate_failure(tmp_path, monkeypatch):
    import os
    import pytest

    monkeypatch.setattr(mod.subprocess, "Popen",
                         lambda argv, **kw: _FakePopenProc(os.getpid()))

    gate_calls = []

    def fake_wait_for_gate(gate):
        gate_calls.append(gate)
        if gate == "bridge":
            raise RuntimeError("gate 'bridge' not ready after 180s")

    monkeypatch.setattr(mod, "_wait_for_gate", fake_wait_for_gate)

    teardown_calls = []
    monkeypatch.setattr(mod, "_teardown_run_dir", lambda run_dir: teardown_calls.append(run_dir))

    cfg = _cfg(log_dir=str(tmp_path))
    with pytest.raises(RuntimeError):
        mod._run_up(cfg, dry_run=False)

    # stage 2 ("bridge") is the one whose gate raised; stage 1 ("sim") must
    # have been given a chance to become ready first.
    assert gate_calls == ["sim", "bridge"]

    # Teardown must have been invoked exactly once, for the run dir that was
    # actually used.
    assert len(teardown_calls) == 1
    run_dir = teardown_calls[0]
    assert run_dir.is_dir()

    # Stage 1's process was really "started" (its PGID was recorded) before
    # the stage-2 failure -- this is what teardown must clean up.
    assert list(run_dir.glob("01-sim*.pgid"))
    # Stage 2's own process was also spawned (Popen succeeded) before its
    # gate timed out, so it too needs tearing down.
    assert list(run_dir.glob("02-bridge*.pgid"))


def test_run_up_success_path_never_tears_down(tmp_path, monkeypatch):
    import os

    monkeypatch.setattr(mod.subprocess, "Popen",
                         lambda argv, **kw: _FakePopenProc(os.getpid()))
    monkeypatch.setattr(mod, "_wait_for_gate", lambda gate: None)
    teardown_calls = []
    monkeypatch.setattr(mod, "_teardown_run_dir", lambda run_dir: teardown_calls.append(run_dir))

    cfg = _cfg(log_dir=str(tmp_path))
    rc = mod._run_up(cfg, dry_run=False)

    assert rc == 0
    assert teardown_calls == []


def test_main_up_converts_gate_failure_to_nonzero_exit_without_traceback(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(mod, "_run_up",
                         lambda cfg, dry_run: (_ for _ in ()).throw(RuntimeError("gate 'bridge' not ready")))

    rc = mod.main(["up", "--scenario", "gpsr-rcw2026-bench", "--seed", "0",
                   "--manipulation", "mock", "--log-dir", str(tmp_path)])

    assert rc == 1
    err = capsys.readouterr().err
    assert "gate 'bridge' not ready" in err


def test_census_argv_uses_the_same_ros_sourcing_wrapper_as_stages():
    argv = mod._census_argv()
    assert argv[0] == "bash" and argv[1] == "-lc"
    script = argv[2]
    assert "source /opt/ros/humble/setup.bash" in script
    assert f"source {mod.TINKER_WS}/install/setup.bash" in script
    assert "gpsr_interface_census.py" in script


def test_run_census_runs_sourced_argv_with_full_env(monkeypatch):
    captured = {}

    class _Result:
        returncode = 0
        stdout = "ok\n"
        stderr = ""

    def fake_run(argv, **kwargs):
        captured["argv"] = argv
        captured["env"] = kwargs.get("env")
        captured["cwd"] = kwargs.get("cwd")
        return _Result()

    monkeypatch.setattr(mod.subprocess, "run", fake_run)

    rc, output = mod._run_census()

    assert rc == 0
    assert output == "ok\n"
    assert captured["argv"] == mod._census_argv()
    assert captured["cwd"] == mod.REPO_ROOT
    # Same sourcing preamble every stage's _ros_argv-wrapped command gets,
    # so gates resolve correctly even invoked from a plain, un-sourced shell.
    assert captured["argv"][:2] == ["bash", "-lc"]
    assert "source /opt/ros/humble/setup.bash" in captured["argv"][2]
    # Same ROS_DOMAIN_ID every stage's env carries, via _preamble_env/_full_env.
    assert captured["env"]["ROS_DOMAIN_ID"] == "42"


# --- Fix round 1: gpsr-stack must run from its own repo root (worktree-safe)

def test_repo_root_is_the_script_s_own_repo_not_a_hardcoded_checkout():
    # mod.REPO_ROOT must be derived from the script's own location, so a
    # worktree checkout (this one) resolves to itself, not to whatever
    # REPO_ROOT used to be hardcoded to.
    assert Path(mod.REPO_ROOT) == _SCRIPT_PATH.resolve().parents[1]


def test_sim_stage_cwd_is_repo_root():
    sim = mod.stage_commands(_cfg())[0]
    assert sim["name"] == "sim"
    assert Path(sim["cwd"]) == Path(mod.REPO_ROOT)


def test_all_stage_cwds_are_repo_root():
    stages = mod.stage_commands(_cfg(manipulation="live", manip_gpu=1))
    for s in stages:
        assert Path(s["cwd"]) == Path(mod.REPO_ROOT), s["name"]


def test_bridge_stage_project_root_arg_is_repo_root():
    bridge = [s for s in mod.stage_commands(_cfg()) if s["name"] == "bridge"][0]
    script = bridge["cmd"][2]  # ["bash", "-lc", script]
    assert f"project_root:={mod.REPO_ROOT}" in script


def test_census_script_path_is_under_repo_root():
    assert mod.CENSUS_SCRIPT.startswith(str(mod.REPO_ROOT))
    assert mod.CENSUS_SCRIPT.endswith("tools/gpsr_interface_census.py")


def test_resolve_ros2_ws_prefers_repo_root_install_when_present(tmp_path):
    setup = tmp_path / "ros2_ws" / "install" / "setup.bash"
    setup.parent.mkdir(parents=True)
    setup.write_text("# fake setup.bash\n")

    result = mod.resolve_ros2_ws(tmp_path)

    assert result == setup


def test_resolve_ros2_ws_falls_back_to_main_checkout_when_absent(tmp_path):
    # tmp_path has no ros2_ws/install at all -- the common worktree case.
    result = mod.resolve_ros2_ws(tmp_path)

    assert result == mod.MAIN_CHECKOUT_ROS2_WS
    assert str(result).endswith("ros2_ws/install/setup.bash")


def test_resolve_ros_vendor_prefers_repo_root_when_present(tmp_path):
    vendor = tmp_path / ".ros-vendor" / "humble" / "local_setup.bash"
    vendor.parent.mkdir(parents=True)
    vendor.write_text("# fake local_setup.bash\n")

    result = mod.resolve_ros_vendor(tmp_path)

    assert result == vendor


def test_resolve_ros_vendor_falls_back_to_main_checkout_when_absent(tmp_path):
    result = mod.resolve_ros_vendor(tmp_path)

    assert result == mod.MAIN_CHECKOUT_ROS_VENDOR


def test_census_sources_use_resolved_ros2_ws_and_vendor():
    # CENSUS_SOURCES must reference the resolved (possibly-fallback) paths,
    # never the bare relative "ros2_ws/install/setup.bash" /
    # ".ros-vendor/humble/local_setup.bash" strings that only worked when
    # REPO_ROOT was hardcoded to a fully-built main checkout.
    assert str(mod.resolve_ros2_ws(mod.REPO_ROOT)) in mod.CENSUS_SOURCES
    assert str(mod.resolve_ros_vendor(mod.REPO_ROOT)) in mod.CENSUS_SOURCES
    assert "ros2_ws/install/setup.bash" not in mod.CENSUS_SOURCES
    assert ".ros-vendor/humble/local_setup.bash" not in mod.CENSUS_SOURCES


def test_bridge_stage_sources_use_resolved_ros2_ws():
    bridge = [s for s in mod.stage_commands(_cfg()) if s["name"] == "bridge"][0]
    script = bridge["cmd"][2]
    assert f"source {mod.resolve_ros2_ws(mod.REPO_ROOT)}" in script
    assert "source ros2_ws/install/setup.bash" not in script
