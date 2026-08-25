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
                manip_gpu=None, log_dir="gpsr_stack_logs")
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


def test_scenario_json_has_two_actors():
    import json
    data = json.loads((Path(__file__).resolve().parents[1] /
                       "simulation/scenarios/gpsr-rcw2026-bench.json").read_text())
    assert len(data["actors"]) == 2


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
