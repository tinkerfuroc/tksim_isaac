"""Render job queue and runner, driven by a fake render command (no Isaac, no GPU)."""
import json
import os
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest

from map_check.bench import BenchStore, JobStatus
from map_check.config import Settings
from map_check.gpu import BusyReason, GpuStatus
from map_check.jobs import GpuBusy, JobRunner, default_isaac_command
from map_check.overlap import Flag

ARM = [-87.4, -58.8, -2.4, 8.0, 12.3, -68.5, -8.2]
TERMINAL = ("done", "failed", "interrupted", "timeout")

FAKE = textwrap.dedent("""
    import json, os, subprocess, sys, time
    job = json.load(open(sys.argv[-1]))
    cfg = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "cfg.json")))
    os.makedirs(job["out_dir"], exist_ok=True)
    print("Isaac noise line", flush=True)
    if cfg.get("hang"):
        child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(1000)"])
        with open(cfg["pids"], "w") as f:
            json.dump([os.getpid(), child.pid], f)
        time.sleep(1000)
    views = [("overview", "overview")] + [(p["key"], v) for p in job["poses"] for v in p["views"]]
    for i, (key, view) in enumerate(views):
        time.sleep(cfg.get("sleep", 0))
        name = f"{i:02d}_{view}.png"
        open(os.path.join(job["out_dir"], name), "wb").close()
        print(json.dumps({"event": "view", "key": key, "view": view, "file": name}), flush=True)
    print(json.dumps({"event": "done"}), flush=True)
    sys.exit(cfg.get("exit", 0))
""")


class FakeRobot:
    """Stands in for overlap.RobotModel: flags pose 'a', nothing else."""
    urdf = Path("/fake/robot.urdf")
    usd = Path("/fake/robot.usd")
    z0 = 0.0775

    def flags(self, pose, arm_deg, pan_deg, tilt_deg, boxes, bounds):
        if pose.name == "a":
            return [Flag("overlap", "base", "wall", -0.03, "base overlaps wall by 3 cm")]
        return []


def free_gpu(own_pids=frozenset()):
    return GpuStatus(gpus=[0, 1], busy=[])


def busy_gpu(own_pids=frozenset()):
    return GpuStatus(gpus=[0, 1], busy=[BusyReason(0, 4242, "gpsr-stack", "scripts/gpsr-stack")])


def make_map(settings, name, poses=("a", "b")):
    """A Map Check arena and Map Studio poses for `name`, all poses ticked with behind only."""
    vdir = settings.arena_root / name / "v1"
    vdir.mkdir(parents=True)
    (vdir / "arena.usd").write_text("#usda 1.0\n")
    (vdir / "boxes.json").write_text(json.dumps([
        {"kind": "wall", "shape_id": None, "x0": 0, "x1": 5, "y0": 0, "y1": 0.1, "z0": 0, "z1": 0.6}]))
    (vdir / "manifest.json").write_text(json.dumps({"kind": "arena", "built_by": "map_check"}))
    (settings.arena_root / name / "current.json").write_text(
        json.dumps({"manifest": str(vdir / "manifest.json"), "schema_version": 1}))
    settings.maps_dir.mkdir(parents=True, exist_ok=True)
    lines = [f"map: {name}", "frame_id: map", "poses:"]
    lines += [f"  {p}: {{x: {1.0 + i}, y: 1.0, yaw: 0.0}}" for i, p in enumerate(poses)]
    (settings.maps_dir / f"{name}.poses.yaml").write_text("\n".join(lines) + "\n")
    BenchStore(settings.state_dir).apply(name, [{
        "op": "set_rows", "rows": [f"saved:{p}" for p in poses], "fields": {"included": True}}])


@pytest.fixture
def settings(tmp_path):
    arm = tmp_path / "constants.json"
    arm.write_text(json.dumps({"arm_pos_navigating": ARM}))
    s = Settings(maps_dir=tmp_path / "maps", arena_root=tmp_path / "arena",
                 state_dir=tmp_path / "state", repo_root=tmp_path, arm_poses_json=arm)
    make_map(s, "m1")
    make_map(s, "m2")
    return s


def fake_command(tmp_path, **cfg):
    d = tmp_path / f"fake{len(list(tmp_path.glob('fake*')))}"
    d.mkdir()
    (d / "render.py").write_text(FAKE)
    (d / "cfg.json").write_text(json.dumps(cfg))
    return lambda jobfile: [sys.executable, str(d / "render.py"), str(jobfile)]


@pytest.fixture
def runners():
    made = []
    yield made
    for r in made:
        r.shutdown()


def runner(runners, settings, bench, command, **kw):
    kw.setdefault("gpu_probe", free_gpu)
    kw.setdefault("robot_model", FakeRobot())
    r = JobRunner(settings, bench, command, **kw)
    runners.append(r)
    return r


def wait_for(fn, timeout=20.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        v = fn()
        if v:
            return v
        time.sleep(0.05)
    raise AssertionError("timed out waiting")


def finished(r, map_name):
    return wait_for(lambda: (st := r.status(map_name)) and st.status in TERMINAL and st)


def alive(pid):
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
    except OSError:
        return False
    return stat[stat.rindex(")") + 2] != "Z"


def test_runs_and_records(tmp_path, settings, runners):
    bench = BenchStore(settings.state_dir)
    r = runner(runners, settings, bench, fake_command(tmp_path))
    job = r.submit("m1", override=False)
    assert job.total_views == 3
    st = finished(r, "m1")
    assert st.status == "done", st.message
    assert st.done_views == 3
    assert bench.get("m1").job.status == "done"
    rec = bench.get("m1").last_render
    assert rec.job_id == job.id and rec.arm_pose == "arm_pos_navigating"
    assert rec.overview_url == f"/renders/m1/{job.id}/00_overview.png"
    assert [it["name"] for it in rec.items] == ["a", "b"]
    assert [v["url"] for v in rec.items[1]["views"]] == [f"/renders/m1/{job.id}/02_behind.png"]
    assert rec.items[0]["flags"][0].text == "base overlaps wall by 3 cm"
    assert rec.items[1]["flags"] == []
    out = settings.state_dir / "renders" / "m1" / job.id
    assert (out / "02_behind.png").is_file()
    # the job file carries the arm pose, the robot and the chosen GPU
    jf = json.loads(next((settings.state_dir / "jobs").glob(f"{job.id}*.json")).read_text())
    assert jf["arm_deg"] == ARM and jf["gpu"] == 1 and jf["out_dir"] == str(out)
    assert [p["views"] for p in jf["poses"]] == [["behind"], ["behind"]]


def test_busy_refused_without_override(tmp_path, settings, runners):
    bench = BenchStore(settings.state_dir)
    r = runner(runners, settings, bench, fake_command(tmp_path), gpu_probe=busy_gpu)
    with pytest.raises(GpuBusy) as e:
        r.submit("m1", override=False)
    assert "gpsr-stack (pid 4242) on GPU 0" in str(e.value)
    assert e.value.reasons[0].pid == 4242
    assert bench.get("m1").job is None
    r.submit("m1", override=True)
    assert finished(r, "m1").status == "done"


def test_queue_fifo(tmp_path, settings, runners):
    bench = BenchStore(settings.state_dir)
    r = runner(runners, settings, bench, fake_command(tmp_path, sleep=0.3))
    first = r.submit("m1", override=False)
    second = r.submit("m2", override=False)
    assert first.status == "running"
    assert second.status == "queued" and second.position == 1
    assert bench.get("m2").job.status == "queued"
    a, b = finished(r, "m1"), finished(r, "m2")
    assert a.status == b.status == "done"
    assert a.finished_at <= b.started_at
    assert b.position is None


def test_same_map_second_click_queues(tmp_path, settings, runners):
    bench = BenchStore(settings.state_dir)
    r = runner(runners, settings, bench, fake_command(tmp_path, sleep=0.3))
    first = r.submit("m1", override=False)
    second = r.submit("m1", override=False)
    assert second.status == "queued" and second.position == 1 and second.id != first.id
    # the map shows the job that is rendering, not the one waiting behind it
    assert r.status("m1").id == first.id
    wait_for(lambda: r.status("m1").id == second.id and r.status("m1").status == "done")
    assert bench.get("m1").last_render.job_id == second.id


def test_timeout_kills_group(tmp_path, settings, runners):
    pids_file = tmp_path / "pids.json"
    bench = BenchStore(settings.state_dir)
    r = runner(runners, settings, bench, fake_command(tmp_path, hang=True, pids=str(pids_file)),
               timeout_s=2.0)
    r.submit("m1", override=False)
    st = finished(r, "m1")
    assert st.status == "timeout"
    pids = json.loads(pids_file.read_text())
    wait_for(lambda: not any(alive(p) for p in pids), timeout=5)
    assert bench.get("m1").last_render is None


def test_timeout_default_formula(settings):
    assert JobRunner.timeout_for(3) == 180
    assert JobRunner.timeout_for(500) == 1800


def test_failed_job_reports_error(tmp_path, settings, runners):
    d = tmp_path / "bad"
    d.mkdir()
    (d / "render.py").write_text(
        'import json\nprint(json.dumps({"event": "error", "message": "no robot"}), flush=True)\n'
        'raise SystemExit(3)\n')
    bench = BenchStore(settings.state_dir)
    r = runner(runners, settings, bench, lambda jf: [sys.executable, str(d / "render.py"), str(jf)])
    r.submit("m1", override=False)
    st = finished(r, "m1")
    assert st.status == "failed" and "no robot" in st.message


def test_nothing_ticked_refused(tmp_path, settings, runners):
    bench = BenchStore(settings.state_dir)
    bench.apply("m1", [{"op": "set_rows", "rows": ["saved:a", "saved:b"],
                        "fields": {"included": False}}])
    r = runner(runners, settings, bench, fake_command(tmp_path))
    with pytest.raises(ValueError, match="no poses"):
        r.submit("m1", override=False)


def test_restart_marks_interrupted(tmp_path, settings, runners):
    # T8 store: a job stored as running reads back as interrupted
    BenchStore(settings.state_dir).set_job(
        "m2", JobStatus(id="old", map="m2", status="running", queued_at=1.0))
    r = runner(runners, settings, BenchStore(settings.state_dir), fake_command(tmp_path))
    assert r.status("m2").status == "interrupted"

    # a server that died mid-job leaves its render running; the next one kills it
    pids_file = tmp_path / "pids.json"
    dead = runner(runners, settings, BenchStore(settings.state_dir),
                  fake_command(tmp_path, hang=True, pids=str(pids_file)))
    dead.submit("m1", override=False)
    pids = wait_for(lambda: pids_file.exists() and json.loads(pids_file.read_text() or "null"))
    assert all(alive(p) for p in pids)
    fresh = runner(runners, settings, BenchStore(settings.state_dir), fake_command(tmp_path))
    assert fresh.status("m1").status == "interrupted"
    wait_for(lambda: not any(alive(p) for p in pids), timeout=15)


def test_shutdown_kills_running_job(tmp_path, settings, runners):
    pids_file = tmp_path / "pids.json"
    bench = BenchStore(settings.state_dir)
    r = runner(runners, settings, bench, fake_command(tmp_path, hang=True, pids=str(pids_file)))
    r.submit("m1", override=False)
    queued = r.submit("m2", override=False)
    pids = wait_for(lambda: pids_file.exists() and json.loads(pids_file.read_text() or "null"))
    r.shutdown()
    assert not any(alive(p) for p in pids)
    assert bench.get("m1").job.status == "interrupted"
    assert bench.get("m2").job.status == "interrupted" and bench.get("m2").job.id == queued.id


def test_default_isaac_command(settings):
    cmd = default_isaac_command(Path("/x/job.json"), settings)
    assert cmd == [str(settings.isaac_python), "-m", "map_check.render.isaac_render", "/x/job.json"]


def test_child_env(tmp_path, settings, runners):
    d = tmp_path / "envdump"
    d.mkdir()
    (d / "render.py").write_text(
        'import json, os, sys\njob = json.load(open(sys.argv[-1]))\n'
        f'json.dump(dict(os.environ), open({str(tmp_path / "env.json")!r}, "w"))\n'
        'print(json.dumps({"event": "done"}), flush=True)\n')
    bench = BenchStore(settings.state_dir)
    r = runner(runners, settings, bench, lambda jf: [sys.executable, str(d / "render.py"), str(jf)])
    r.submit("m1", override=False)
    finished(r, "m1")
    env = json.loads((tmp_path / "env.json").read_text())
    assert env["OMNI_KIT_ACCEPT_EULA"] == "YES"
    assert str(Path(__file__).resolve().parents[1]) in env["PYTHONPATH"].split(os.pathsep)
    assert env.get("CUDA_VISIBLE_DEVICES") == os.environ.get("CUDA_VISIBLE_DEVICES")


def test_isaac_render_imports_without_isaac():
    """The Isaac script parses as Python and defers every Isaac import to run time."""
    p = subprocess.run([sys.executable, "-c", "import map_check.render.isaac_render as m; "
                        "print(callable(m.main))"],
                       capture_output=True, text=True, cwd=Path(__file__).resolve().parents[1])
    assert p.stdout.strip() == "True", p.stderr
