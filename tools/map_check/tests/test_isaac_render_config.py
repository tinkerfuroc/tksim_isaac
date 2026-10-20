"""isaac_render's SimulationApp config and its no-GPU fail-fast, without Isaac (Ruling 11)."""
import json
import os
import subprocess
import sys
import textwrap
from pathlib import Path

from map_check.render import isaac_render
import pytest

from map_check.render.isaac_render import (NO_GPU_EXIT, app_config, emit_view, first_image_budget,
                                         no_device_message, slow_start_message)
from map_check.render.jobfile import JobFile, JobPose

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG.parents[1] / "validation"))
import run_sim as rs  # noqa: E402


def test_no_active_gpu_in_extra_args():
    args = app_config()["extra_args"]
    assert not any("activeGpu" in a for a in args)
    assert not any("multiGpu" in a for a in args)


def test_gpu_settings_mirror_run_sim():
    ours, sim = app_config(), rs._streaming_application_config([])
    for k in ("multi_gpu", "max_gpu_count", "physics_gpu"):
        assert ours[k] == sim[k]
    assert (ours["multi_gpu"], ours["max_gpu_count"], ours["physics_gpu"]) == (False, 1, -1)
    physics = [a for a in sim["extra_args"] if a.startswith("--/physics/")]
    assert physics == ["--/physics/useGpu=false", "--/physics/cudaDevice=-1"]
    assert [a for a in ours["extra_args"] if a.startswith("--/physics/")] == physics


def test_no_device_message():
    assert no_device_message(1, 0) == "Isaac could not create a GPU device on GPU 1 (see the job log)"
    assert no_device_message(1, 1) is None
    assert no_device_message(0, None) is None  # unknown: the first-image backstop decides


def test_emit_view_runs_listeners(capsys, monkeypatch):
    seen = []
    monkeypatch.setattr(isaac_render, "_on_view", [lambda: seen.append(1)])
    emit_view("a", "behind", "00_a_behind.png")
    assert seen == [1]
    assert json.loads(capsys.readouterr().out)["file"] == "00_a_behind.png"


FAKE_ISAACSIM = textwrap.dedent("""
    import json, os
    class SimulationApp:
        def __init__(self, config):
            json.dump(config, open(os.environ["FAKE_CONFIG_OUT"], "w"))
        def close(self):
            pass
""")

FAKE_GFF = textwrap.dedent("""
    import os
    class _Factory:
        def get_device_count(self):
            return int(os.environ["FAKE_DEVICES"])
    def get_gpu_foundation_factory_interface():
        return _Factory()
""")


def _run_main(tmp_path, devices: int):
    fake = tmp_path / "fake"
    (fake / "isaacsim").mkdir(parents=True)
    (fake / "isaacsim" / "__init__.py").write_text(FAKE_ISAACSIM)
    (fake / "omni" / "gpu_foundation_factory").mkdir(parents=True)
    (fake / "omni" / "__init__.py").write_text("")
    (fake / "omni" / "gpu_foundation_factory" / "__init__.py").write_text(FAKE_GFF)
    job = tmp_path / "job.json"
    JobFile(job_id="j", map="m", arena_usd="/a.usd", robot_usd="/r.usd", robot_urdf="/r.urdf",
            z0=0.08, gpu=1, out_dir=str(tmp_path / "out"), arm_deg=[0.0] * 7,
            poses=[JobPose(key="saved:a", name="a", x=0.0, y=0.0, yaw=0.0, views=["behind"])],
            overview=True, deadline=4_000_000_000.0).write(job)
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "AMENT_PREFIX_PATH")}
    env.update(PYTHONPATH=os.pathsep.join([str(fake), str(PKG)]), FAKE_DEVICES=str(devices),
               FAKE_CONFIG_OUT=str(tmp_path / "config.json"))
    p = subprocess.run([sys.executable, "-m", "map_check.render.isaac_render", str(job)],
                       env=env, capture_output=True, text=True, timeout=60)
    config = json.loads((tmp_path / "config.json").read_text())
    events = [json.loads(line) for line in p.stdout.splitlines() if line.startswith("{")]
    return p, config, events


def test_main_fails_fast_without_gpu_device(tmp_path):
    p, config, events = _run_main(tmp_path, devices=0)
    assert p.returncode == NO_GPU_EXIT, p.stderr
    assert events == [{"event": "error",
                       "message": "Isaac could not create a GPU device on GPU 1 (see the job log)"}]
    assert config == app_config()


def test_main_goes_on_with_a_gpu_device(tmp_path):
    """With a device, main proceeds to the Renderer (which needs real Isaac: an error event)."""
    p, _, events = _run_main(tmp_path, devices=1)
    assert p.returncode == 1, p.stderr
    assert events[-1]["event"] == "error" and "GPU device" not in events[-1]["message"]


@pytest.mark.parametrize("left,budget", [
    (150.0, 135.0),     # 1 view (140 s timeout + 10 s grace): just under the job's deadline
    (270.0, 255.0),     # 7 views: the 300 s floor is capped at left - 15
    (400.0, 300.0),     # the floor
    (1000.0, 500.0),    # half the time left
    (1810.0, 905.0),    # the longest job
])
def test_first_image_budget(left, budget):
    now = 1_792_000_000.0
    got = first_image_budget(now + left, now)
    assert got == pytest.approx(budget)
    assert now + got <= now + left - 15.0  # never past the job's own deadline


@pytest.mark.parametrize("left", [15.0, 10.0, 0.0, -5.0])
def test_no_backstop_when_job_deadline_is_that_close(left):
    assert first_image_budget(100.0 + left, 100.0) is None


def test_slow_start_message_is_not_the_no_device_message():
    msg = slow_start_message(1, 255.4)
    assert msg == ("No image after 255 s: Isaac may have no GPU device on GPU 1, or startup is "
                   "slow (see the job log)")
    assert "could not create a GPU device" not in msg
    assert "could not create a GPU device" in no_device_message(1, 0)
