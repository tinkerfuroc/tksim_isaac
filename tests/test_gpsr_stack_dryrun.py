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
