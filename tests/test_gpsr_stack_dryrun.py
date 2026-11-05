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
    assert names == ["sim", "bridge", "vision", "nav_extras", "rtf_monitor"]


def test_live_includes_stage4_with_gpu():
    stages = mod.stage_commands(_cfg(manipulation="live", manip_gpu=1))
    manip = [s for s in stages if s["name"] == "manipulation"][0]
    assert manip["env"]["CUDA_VISIBLE_DEVICES"] == "1"


def test_sim_stage_env():
    sim = mod.stage_commands(_cfg())[0]
    env = sim["env"]
    assert env["ROS_DOMAIN_ID"] == "42"
    assert "TINKER_SIM_ARENA_CAMERA" not in env
    assert "TINKER_SIM_DISABLE_WRIST_CAMERA" not in env
    assert "--scenario" in sim["cmd"] and "gpsr-rcw2026-bench" in sim["cmd"]


# --- Arena camera is an evidence-run opt-in (RTF: 0.24 with it, 0.68+ without) ---

def test_sim_stage_arena_off_by_default_mock():
    env = mod.stage_commands(_cfg(manipulation="mock"))[0]["env"]
    assert "TINKER_SIM_ARENA_CAMERA" not in env
    assert "TINKER_SIM_ARENA_CAMERA_HZ" not in env
    assert "TINKER_SIM_DISABLE_WRIST_CAMERA" not in env


def test_sim_stage_arena_off_by_default_live():
    # Split from the mock case so the mock half is unaffected by this repo's
    # known environmental gap: stage_commands(manipulation="live") resolves
    # the model bundle manifest via resolve_model_bundle_manifest(), which
    # raises "produced model bundle not found" when this worktree hasn't
    # produced one (per docs/gpsr-sim-runbook.md's generation recipe) -- an
    # environmental precondition, not something the arena-off assertion
    # below is exercising.
    env = mod.stage_commands(_cfg(manipulation="live", manip_gpu=1))[0]["env"]
    assert "TINKER_SIM_ARENA_CAMERA" not in env
    assert "TINKER_SIM_ARENA_CAMERA_HZ" not in env
    assert "TINKER_SIM_DISABLE_WRIST_CAMERA" not in env


def test_sim_stage_arena_on_for_evidence_runs():
    env = mod.stage_commands(_cfg(evidence=True))[0]["env"]
    assert env["TINKER_SIM_ARENA_CAMERA"] == "1"
    assert env["TINKER_SIM_ARENA_CAMERA_HZ"] == "2"


def test_cli_evidence_flag_reaches_config(monkeypatch):
    seen = {}
    monkeypatch.setattr(mod, "_run_up", lambda cfg, dry_run: seen.update(cfg=cfg) or 0)
    assert mod.main(["up", "--evidence", "--dry-run"]) == 0
    assert seen["cfg"].evidence is True
    assert mod.main(["up", "--dry-run"]) == 0
    assert seen["cfg"].evidence is False


def test_wrist_camera_required_in_both_modes():
    """Wrist topics required in both mock and live modes."""
    stacks_mock = mod._gate_census_stacks(_cfg(manipulation="mock"))
    assert "sim cameras wrist" in stacks_mock["sim"]
    assert "sim cameras" in stacks_mock["sim"]

    stacks_live = mod._gate_census_stacks(_cfg(manipulation="live", manip_gpu=1))
    assert "sim cameras wrist" in stacks_live["sim"]
    assert "sim cameras" in stacks_live["sim"]


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
    assert names == ["sim", "bridge", "vision", "manipulation", "nav_extras", "rtf_monitor"]


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
    assert by_name["rtf_monitor"] == "nav"


def test_vision_stage_has_four_popens_including_named_camera_server_and_pan_tilt():
    stages = mod.stage_commands(_cfg())
    vision = [s for s in stages if s["name"] == "vision"][0]
    cmd = vision["cmd"]
    assert isinstance(cmd[0], list), "vision stage must be multiple Popens"
    flat = [" ".join(c) for c in cmd]
    assert any("vision_bringup" in c for c in flat)
    assert any("__node:=head_camera_server" in c for c in flat)
    # The RealSense is read through its own service endpoint; the sim's
    # /camera/xarm_camera topics must be bridged by a second camera server.
    wrist = [c for c in flat if "__node:=wrist_camera_server" in c]
    assert len(wrist) == 1
    assert "color_topic:=/camera/xarm_camera/color/image_raw" in wrist[0]
    assert "depth_topic:=/camera/xarm_camera/aligned_depth_to_color/image_raw" in wrist[0]
    assert any("pan_tilt" in c and "state_publisher" in c for c in flat)
    assert any("pan_tilt_tf_publisher.py" in c for c in flat)


def test_pan_tilt_state_publisher_stays_off_joint_states():
    """pick_and_place's live-manip readiness contract requires exactly one
    /joint_states publisher (the joint_state_broadcaster), so the pan/tilt
    state publisher must be remapped to the side topic the TF publisher
    consumes -- in every mode, so hybrid and live stacks behave identically.
    """
    for cfg in (_cfg(), _cfg(manipulation="live", manip_gpu=0)):
        stages = mod.stage_commands(cfg)
        vision = [s for s in stages if s["name"] == "vision"][0]
        flat = [" ".join(c) for c in vision["cmd"]]
        stitcher = [c for c in flat if "state_publisher" in c and "pan_tilt " in c]
        assert stitcher, "pan_tilt state_publisher process missing"
        assert "joint_state_topic:=/pan_tilt/joint_states" in stitcher[0]
        tf_pub = [c for c in flat if "pan_tilt_tf_publisher.py" in c]
        assert tf_pub, "pan_tilt_tf_publisher process missing"
        assert "joint_state_topic:=/pan_tilt/joint_states" in tf_pub[0]


def test_every_stage_has_required_keys():
    for s in mod.stage_commands(_cfg(manipulation="live", manip_gpu=1)):
        assert set(("name", "cmd", "env", "cwd", "gate")) <= set(s.keys())


def test_stage_commands_pure_no_side_effects(tmp_path, monkeypatch):
    # stage_commands must never spawn a subprocess. It is NOT filesystem-read
    # -free in live mode any more: resolve_model_bundle_manifest(REPO_ROOT)
    # stats outputs/ompl-overlay/model-bundle/model-bundle.json to build the
    # manipulation stage's model_bundle_manifest:= argument. This test's real
    # REPO_ROOT happens to have that file (see the precedence/raise tests
    # below for the read itself), so it only guards the subprocess half.
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


def test_run_census_times_out_instead_of_hanging_bring_up(monkeypatch):
    # 2026-09-30: a census hung 30+ min on broken DDS discovery and bring-up
    # never reached its own gate deadline.
    seen = {}

    def fake_run(argv, **kwargs):
        seen["timeout"] = kwargs.get("timeout")
        raise mod.subprocess.TimeoutExpired(argv, kwargs.get("timeout"), output=b"partial\n")

    monkeypatch.setattr(mod.subprocess, "run", fake_run)
    rc, output = mod._run_census()
    assert seen["timeout"] == mod.CENSUS_TIMEOUT_S
    assert rc == 124 and "partial" in output and "timed out" in output


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


def test_run_status_returns_zero_even_when_census_reports_nonzero(monkeypatch):
    # _run_census's returncode is the whole-graph result, which is non-zero
    # by construction under --manipulation mock (tk25_manipulation's
    # interfaces are permanently absent). `status` is informational, so it
    # must not propagate that as a false-negative exit code (review finding:
    # `gpsr-stack status && ...` was a permanent false negative).
    monkeypatch.setattr(mod, "_run_census", lambda: (1, "MISSING [tk25_manipulation]\n"))
    rc = mod._run_status(_cfg())
    assert rc == 0


# --- Task 50 fix round: teardown survivor polling (runbook cumotion kill) --

def test_poll_and_reap_survivors_returns_immediately_when_group_is_empty(monkeypatch):
    monkeypatch.setattr(mod, "_pgrep_group", lambda pgid: "")
    killpg_calls = []
    monkeypatch.setattr(mod.os, "killpg", lambda pgid, sig: killpg_calls.append((pgid, sig)))

    mod._poll_and_reap_survivors(4242, timeout_s=1.0, poll_s=0.01)

    assert killpg_calls == []


def test_poll_and_reap_survivors_rekills_and_reports_persistent_members(monkeypatch, capsys):
    # Simulate cumotion_goal_set_planner_node still showing up in pgrep
    # after the first SIGKILL -- the runbook's "outlives a careless
    # teardown" case.
    monkeypatch.setattr(mod, "_pgrep_group", lambda pgid: "9999 cumotion_goal_set_planner_node")
    killpg_calls = []
    monkeypatch.setattr(mod.os, "killpg", lambda pgid, sig: killpg_calls.append((pgid, sig)))

    mod._poll_and_reap_survivors(4242, timeout_s=0.05, poll_s=0.01)

    assert killpg_calls == [(4242, mod.signal.SIGKILL)]
    err = capsys.readouterr().err
    assert "4242" in err
    assert "cumotion_goal_set_planner_node" in err


def test_poll_and_reap_survivors_uses_pgrep_group_never_a_name_pattern(monkeypatch):
    # Guard against regressing to a name-pattern sweep: the survivor check
    # must go through _pgrep_group (pgrep -g <pgid>), not pgrep -f/pkill.
    captured = {}

    def fake_run(argv, **kwargs):
        captured["argv"] = argv
        class _Result:
            stdout = ""
        return _Result()

    monkeypatch.setattr(mod.subprocess, "run", fake_run)

    mod._poll_and_reap_survivors(4242, timeout_s=1.0, poll_s=0.01)

    assert captured["argv"] == ["pgrep", "-g", "4242", "-l"]


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


# --- Task 50: model-bundle manifest resolution ---


def _write_produced_bundle(repo_root) -> Path:
    bundle_dir = Path(repo_root) / "outputs" / "ompl-overlay" / "model-bundle"
    bundle_dir.mkdir(parents=True)
    bundle = bundle_dir / "model-bundle.json"
    bundle.write_text("{}")
    return bundle


def _write_artifact_manifest(repo_root) -> Path:
    """Set up the OLD (removed) fallback shape: artifacts/asset-manifest.json
    pointing at a robot-dir manifest.json. Used only to prove the resolver no
    longer falls back to it even when it is present (precedence test).
    """
    asset_dir = Path(repo_root) / "artifacts"
    asset_dir.mkdir()
    asset_manifest = asset_dir / "asset-manifest.json"
    robot_dir = asset_dir / "robot" / "tinker2" / "somehash"
    robot_dir.mkdir(parents=True)
    manifest = robot_dir / "manifest.json"
    manifest.write_text("{}")
    asset_manifest.write_text(
        """{
  "schema_version": 1,
  "generated_robot_usds": [
    {
      "path": "artifacts/robot/tinker2/somehash/robot.usd",
      "sha256": "fake"
    }
  ]
}"""
    )
    return manifest


def test_resolve_model_bundle_manifest_returns_produced_bundle(tmp_path):
    """resolve_model_bundle_manifest returns the produced bundle when present."""
    bundle = _write_produced_bundle(tmp_path)

    result = mod.resolve_model_bundle_manifest(tmp_path)

    assert result == bundle.resolve()
    assert result.is_absolute()
    assert result.name == "model-bundle.json"


def test_resolve_model_bundle_manifest_prefers_produced_bundle_over_artifact_manifest(tmp_path):
    """The produced bundle takes precedence even when a robot-artifact
    manifest.json also exists -- this is the whole point of c2c21e9 and had
    no direct test (review finding: produced-bundle precedence)."""
    bundle = _write_produced_bundle(tmp_path)
    _write_artifact_manifest(tmp_path)

    result = mod.resolve_model_bundle_manifest(tmp_path)

    assert result == bundle.resolve()
    assert result.name != "manifest.json"


def test_resolve_model_bundle_manifest_raises_with_generation_recipe_when_absent(tmp_path):
    """No silent fallback to the robot-artifact manifest.json: when the
    produced bundle is absent, raise with the exact generation recipe."""
    import pytest

    with pytest.raises(RuntimeError) as excinfo:
        mod.resolve_model_bundle_manifest(tmp_path)

    message = str(excinfo.value)
    assert "model-bundle.json" in message
    assert "ros2 run tinker_sim_bridge model_limits" in message
    assert "ros2 run tinker_sim_bridge model_bundle" in message


def test_resolve_model_bundle_manifest_raises_even_when_artifact_manifest_present(tmp_path):
    """The robot-artifact manifest.json is never a valid fallback any more,
    even when it exists and is well-formed."""
    import pytest

    _write_artifact_manifest(tmp_path)

    with pytest.raises(RuntimeError, match="model_limits"):
        mod.resolve_model_bundle_manifest(tmp_path)


def test_live_manipulation_stage_includes_model_bundle_manifest():
    """Live-mode manipulation stage includes model_bundle_manifest:= argument,
    pointing at the produced bundle (this test runs against the real
    REPO_ROOT, which this checkout has already produced -- see
    outputs/ompl-overlay/model-bundle/model-bundle.json)."""
    stages = mod.stage_commands(_cfg(manipulation="live", manip_gpu=1))
    manip = [s for s in stages if s["name"] == "manipulation"][0]
    script = manip["cmd"][0][2]  # ["bash", "-lc", script]
    assert "model_bundle_manifest:=" in script
    assert script.endswith("model-bundle.json")


# --- a stage child that died before its gate passed fails the bring-up ---

def test_check_stage_deaths_raises_on_child_died_at_startup(tmp_path):
    import pytest

    (tmp_path / "03-vision-0.log").write_text(
        "[generalist_node-1] ModuleNotFoundError: No module named 'torch'\n"
        "[ERROR] [generalist_node-1]: process has died [pid 1893676, exit code 1, cmd 'x'].\n")
    (tmp_path / "03-vision-1.log").write_text("[INFO] camera server ready\n")
    with pytest.raises(RuntimeError, match=r"generalist_node-1 \(exit code 1\)"):
        mod._check_stage_deaths(tmp_path, "03-vision")


def test_check_stage_deaths_ignores_other_stages_and_healthy_logs(tmp_path):
    (tmp_path / "03-vision-0.log").write_text("[INFO] Generalist detection node ready\n")
    (tmp_path / "04-manipulation-0.log").write_text(
        "[ERROR] [move_group-1]: process has died [pid 1, exit code -11, cmd 'x'].\n")
    mod._check_stage_deaths(tmp_path, "03-vision")  # no raise


def test_bridge_stage_gets_the_head_camera_aim_like_the_sim_stage():
    # 2026-10-01: only the sim stage had TINKER_SIM_HEAD_CAMERA_AIM, so the
    # bridge's camera_color_optical_frame alias stayed identity and every
    # map-frame head detection projected upward.
    stages = {s["name"]: s for s in mod.stage_commands(_cfg())}
    assert stages["sim"]["env"]["TINKER_SIM_HEAD_CAMERA_AIM"] == mod.HEAD_CAMERA_AIM
    assert stages["bridge"]["env"]["TINKER_SIM_HEAD_CAMERA_AIM"] == mod.HEAD_CAMERA_AIM


# --- Lidar presets (RTF budget, docs/superpowers/specs/2026-10-11-gpsr-sim-rtf-budget-design.md) ---

def _sim_stage(**kw):
    return [s for s in mod.stage_commands(_cfg(**kw)) if s["name"] == "sim"][0]


def _contract_scale():
    import json
    raw = json.loads((mod.REPO_ROOT / "simulation" / "sensors" / "hardware-parity.json").read_text())
    ray = raw["lidar"]["raycast"]
    return str(ray["channels"]), str(ray["columns"])


def test_default_preset_is_battery_with_pinned_channels_not_devs_default(monkeypatch):
    # dev's run_sim default (unset) is 44 ch; the battery preset pins 24.
    monkeypatch.delenv("TINKER_SIM_LIDAR_CHANNELS", raising=False)
    sim = _sim_stage()
    assert "--raycast-lidar" not in sim["cmd"]
    assert "--map-lidar" not in sim["cmd"]
    assert sim["cmd"][-1] == "--headless"
    assert sim["env"]["TINKER_SIM_LIDAR_CHANNELS"] == "24"
    assert "TINKER_SIM_LIDAR_COLUMNS" not in sim["env"]


def test_parity_preset_sets_contract_scale_explicitly(monkeypatch):
    monkeypatch.delenv("TINKER_SIM_LIDAR_CHANNELS", raising=False)
    monkeypatch.delenv("TINKER_SIM_LIDAR_COLUMNS", raising=False)
    sim = _sim_stage(lidar="parity")
    channels, columns = _contract_scale()
    assert sim["env"]["TINKER_SIM_LIDAR_CHANNELS"] == channels
    assert sim["env"]["TINKER_SIM_LIDAR_COLUMNS"] == columns
    assert "--raycast-lidar" not in sim["cmd"]
    assert "--map-lidar" not in sim["cmd"]


def test_parity_preset_isolates_exported_scale_vars(monkeypatch):
    monkeypatch.setenv("TINKER_SIM_LIDAR_CHANNELS", "16")
    monkeypatch.setenv("TINKER_SIM_LIDAR_COLUMNS", "100")
    sim = _sim_stage(lidar="parity")
    channels, columns = _contract_scale()
    assert sim["env"]["TINKER_SIM_LIDAR_CHANNELS"] == channels
    assert sim["env"]["TINKER_SIM_LIDAR_COLUMNS"] == columns


def test_parity_preset_errors_clearly_when_contract_missing(monkeypatch, tmp_path):
    import pytest
    monkeypatch.setattr(mod, "REPO_ROOT", tmp_path)
    with pytest.raises(RuntimeError, match="hardware-parity.json"):
        mod.stage_commands(_cfg(lidar="parity"))


def test_no_preset_passes_raycast_lidar_flag():
    for preset in mod.LIDAR_PRESETS:
        assert "--raycast-lidar" not in _sim_stage(lidar=preset)["cmd"]


def test_battery_blank_channel_export_falls_back_to_default(monkeypatch):
    monkeypatch.setenv("TINKER_SIM_LIDAR_CHANNELS", "  ")
    assert _sim_stage()["env"]["TINKER_SIM_LIDAR_CHANNELS"] == str(mod.BATTERY_LIDAR_CHANNELS)


def test_default_banner_shows_battery_channels(monkeypatch, capsys):
    monkeypatch.delenv("TINKER_SIM_LIDAR_CHANNELS", raising=False)
    assert mod.main(["up", "--manipulation", "mock", "--dry-run"]) == 0
    assert "lidar preset: battery (channels=24)" in capsys.readouterr().out


def test_planar_preset_opts_out_with_map_lidar():
    sim = _sim_stage(lidar="planar")
    assert "--raycast-lidar" not in sim["cmd"]
    assert sim["cmd"][-2:] == ["--", "--map-lidar"]
    assert "TINKER_SIM_LIDAR_CHANNELS" not in sim["env"]


def test_battery_preset_respects_caller_channel_export(monkeypatch):
    monkeypatch.setenv("TINKER_SIM_LIDAR_CHANNELS", "32")
    assert _sim_stage()["env"]["TINKER_SIM_LIDAR_CHANNELS"] == "32"


def test_unknown_preset_is_refused():
    import pytest
    with pytest.raises(ValueError, match="lidar preset"):
        mod.stage_commands(_cfg(lidar="fast"))


def test_physics_rate_is_never_set_by_the_stack():
    for preset in mod.LIDAR_PRESETS:
        assert "TINKER_SIM_PHYSICS_HZ" not in _sim_stage(lidar=preset)["env"]


def test_cli_lidar_flag_reaches_config_and_banner(capsys):
    rc = mod.main(["up", "--manipulation", "mock", "--dry-run", "--lidar", "planar"])
    assert rc == 0
    out = capsys.readouterr().out
    assert "lidar preset: planar" in out
    assert "--raycast-lidar" not in out


# --- RTF monitor stage, run dir, private domain, vision GPU ---

def test_rtf_monitor_stage_runs_the_tool_under_ros():
    stage = mod.stage_commands(_cfg())[-1]
    assert stage["name"] == "rtf_monitor"
    script = stage["cmd"][-1]
    assert stage["cmd"][:2] == ["bash", "-lc"]
    assert "source /opt/ros/humble/setup.bash" in script
    assert "exec python3 tools/rtf_monitor.py" in script


def test_domain_override(monkeypatch):
    monkeypatch.setenv("GPSR_ROS_DOMAIN_ID", "219")
    for stage in mod.stage_commands(_cfg()):
        assert stage["env"]["ROS_DOMAIN_ID"] == "219"
    monkeypatch.delenv("GPSR_ROS_DOMAIN_ID")
    assert mod.stage_commands(_cfg())[0]["env"]["ROS_DOMAIN_ID"] == "42"
    monkeypatch.setenv("GPSR_ROS_DOMAIN_ID", "  ")
    assert mod.stage_commands(_cfg())[0]["env"]["ROS_DOMAIN_ID"] == "42"


def test_vision_gpu_override(monkeypatch):
    monkeypatch.delenv("GPSR_VISION_GPU", raising=False)
    vision = [s for s in mod.stage_commands(_cfg()) if s["name"] == "vision"][0]
    assert "CUDA_VISIBLE_DEVICES" not in vision["env"]
    monkeypatch.setenv("GPSR_VISION_GPU", "1")
    vision = [s for s in mod.stage_commands(_cfg()) if s["name"] == "vision"][0]
    assert vision["env"]["CUDA_VISIBLE_DEVICES"] == "1"
    assert vision["env"]["QT_QPA_PLATFORM"] == "offscreen"


def test_run_up_passes_run_dir_to_every_stage(tmp_path, monkeypatch):
    import os
    envs = []

    def _popen(argv, **kw):
        envs.append(kw["env"])
        return _FakePopenProc(os.getpid())

    monkeypatch.setattr(mod.subprocess, "Popen", _popen)
    monkeypatch.setattr(mod, "_wait_for_gate", lambda gate: None)
    assert mod._run_up(_cfg(log_dir=str(tmp_path)), dry_run=False) == 0
    run_dirs = {e["GPSR_RUN_DIR"] for e in envs}
    assert len(run_dirs) == 1
    run_dir = Path(run_dirs.pop())
    assert run_dir.is_absolute() and run_dir.parent == tmp_path.resolve()
