"""Render job queue and runner, driven by a fake render command (no Isaac, no GPU)."""
import json
import os
import signal
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest

from map_check.bench import BenchStore, JobStatus
from map_check.config import Settings
from map_check.gpu import BusyReason, GpuStatus
from map_check.jobs import (KIT_NO_GPU, UNSET_ENV, GpuBusy, JobRunner, _proc_start,
                            default_isaac_command)
from map_check.overlap import Flag
from map_check.render.cameras import overview_top
from map_check.render.isaac_render import overview_labels
from map_check.render.jobfile import JobPose

ARM = [-87.4, -58.8, -2.4, 8.0, 12.3, -68.5, -8.2]
TERMINAL = ("done", "failed", "interrupted", "timeout")

FAKE = textwrap.dedent("""
    import json, os, subprocess, sys, time
    job = json.load(open(sys.argv[-1]))
    cfg = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "cfg.json")))
    os.makedirs(job["out_dir"], exist_ok=True)
    print("Isaac noise line", flush=True)
    for stream, line in cfg.get("kit_lines", []):
        print(line, file=sys.stderr if stream == "err" else sys.stdout, flush=True)
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
    for stream, line in cfg.get("after_done", []):  # Kit shutting down
        time.sleep(0.2)
        print(line, file=sys.stderr if stream == "err" else sys.stdout, flush=True)
    time.sleep(cfg.get("linger", 0))
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


def test_same_map_second_render_refused_until_done(tmp_path, settings, runners):
    bench = BenchStore(settings.state_dir)
    r = runner(runners, settings, bench, fake_command(tmp_path, sleep=0.3))
    first = r.submit("m1", override=False)
    other = r.submit("m2", override=False)  # queued behind it
    for m in ("m1", "m2"):  # running, and queued
        with pytest.raises(ValueError, match=f"^A render for {m} is already queued or running.$"):
            r.submit(m, override=True)
    assert r.status("m1").id == first.id and r.status("m2").id == other.id
    assert finished(r, "m1").status == "done"
    again = r.submit("m1", override=False)  # finished: a new render is fine
    assert again.id != first.id
    wait_for(lambda: r.status("m1").id == again.id and r.status("m1").status == "done")
    assert bench.get("m1").last_render.job_id == again.id


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


def leftover_render(tmp_path, settings, server_pid, server_start):
    """A render left running in its own session, with active.json naming its server.

    Started through a parent that exits at once, so, like a real leftover, it is
    an orphan that this process neither owns nor reaps.
    """
    jobs = settings.state_dir / "jobs"
    jobs.mkdir(parents=True, exist_ok=True)
    jobfile = jobs / "leftover.json"
    jobfile.write_text(json.dumps({"out_dir": str(tmp_path / "leftover"), "poses": []}))
    pids_file = tmp_path / "leftover-pids.json"
    cmd = fake_command(tmp_path, hang=True, pids=str(pids_file))(jobfile)
    spawn = subprocess.run(
        [sys.executable, "-c", "import subprocess, sys; "
         "print(subprocess.Popen(sys.argv[1:], start_new_session=True, stdout=subprocess.DEVNULL, "
         "stderr=subprocess.DEVNULL).pid)", *cmd],
        capture_output=True, text=True, check=True, timeout=10)
    pid = int(spawn.stdout)
    (jobs / "active.json").write_text(json.dumps({
        "pid": pid, "jobfile": str(jobfile), "map": "m1",
        "server_pid": server_pid, "server_start": server_start}))
    BenchStore(settings.state_dir).set_job(
        "m1", JobStatus(id="left", map="m1", status="running", queued_at=1.0))
    pids = wait_for(lambda: pids_file.exists() and json.loads(pids_file.read_text() or "null"))
    return pid, pids


def test_restart_marks_interrupted(tmp_path, settings, runners):
    # T8 store: a job stored as running reads back as interrupted
    BenchStore(settings.state_dir).set_job(
        "m2", JobStatus(id="old", map="m2", status="running", queued_at=1.0))
    old = JobRunner(settings, BenchStore(settings.state_dir), fake_command(tmp_path),
                    gpu_probe=free_gpu, robot_model=FakeRobot())
    assert old.status("m2").status == "interrupted"
    old.shutdown()  # releases the state dir

    # the server died mid-job: its render still runs and active.json names a dead server
    gone = subprocess.Popen([sys.executable, "-c", "pass"])
    gone.wait()
    pid, pids = leftover_render(tmp_path, settings, gone.pid, "1")
    assert all(alive(p) for p in pids)
    fresh = runner(runners, settings, BenchStore(settings.state_dir), fake_command(tmp_path))
    assert fresh.status("m1").status == "interrupted"
    assert pids[0] == pid
    wait_for(lambda: not any(alive(p) for p in pids), timeout=15)
    assert not (settings.state_dir / "jobs" / "active.json").exists()


def test_leftover_of_live_server_untouched(tmp_path, settings, runners):
    pid, pids = leftover_render(tmp_path, settings, os.getpid(), _proc_start(os.getpid()))
    try:
        runner(runners, settings, BenchStore(settings.state_dir), fake_command(tmp_path))
        time.sleep(0.3)
        assert all(alive(p) for p in pids)
        assert (settings.state_dir / "jobs" / "active.json").exists()
    finally:
        os.killpg(pid, signal.SIGKILL)


def test_second_runner_refused(tmp_path, settings, runners):
    pids_file = tmp_path / "pids.json"
    first = runner(runners, settings, BenchStore(settings.state_dir),
                   fake_command(tmp_path, hang=True, pids=str(pids_file)))
    first.submit("m1", override=False)
    pids = wait_for(lambda: pids_file.exists() and json.loads(pids_file.read_text() or "null"))
    with pytest.raises(RuntimeError, match="another Map Check server is using"):
        JobRunner(settings, BenchStore(settings.state_dir), fake_command(tmp_path),
                  gpu_probe=free_gpu, robot_model=FakeRobot())
    assert all(alive(p) for p in pids)
    assert first.status("m1").status == "running"
    assert (settings.state_dir / "jobs" / "active.json").exists()


def test_exception_after_start_kills_render(tmp_path, settings, runners, monkeypatch):
    bench = BenchStore(settings.state_dir)
    r = runner(runners, settings, bench, fake_command(tmp_path, sleep=0.3))
    real, seen = r._write_active, []

    def boom(pid, *args):
        if not seen:
            seen.append(pid)
            raise RuntimeError("boom")
        return real(pid, *args)

    monkeypatch.setattr(r, "_write_active", boom)
    first = r.submit("m1", override=False)
    st = finished(r, "m1")
    assert st.id == first.id and st.status == "failed" and "boom" in st.message
    wait_for(lambda: not alive(seen[0]), timeout=5)
    second = r.submit("m1", override=False)
    wait_for(lambda: r.status("m1").id == second.id and r.status("m1").status == "done")


def test_overview_labels_by_row_key():
    poses = [JobPose("saved:a", "a", 1.0, 1.0, 0.0, ["behind"]),
             JobPose("pasted:a", "a", 1.1, 1.0, 0.0, ["behind"])]
    labs = overview_labels(overview_top((0.0, 5.0, 0.0, 5.0)), poses, {"pasted:a"},
                           lambda t: (10 * len(t), 20))
    assert [lab["key"] for lab in labs] == ["saved:a", "pasted:a"]
    assert [lab["text"] for lab in labs] == ["a", "a (pasted)"]
    assert [lab["flagged"] for lab in labs] == [False, True]
    assert labs[0]["box"] != labs[1]["box"]


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


def test_child_env(tmp_path, settings, runners, monkeypatch):
    for k in UNSET_ENV:
        monkeypatch.setenv(k, "/opt/ros/humble/inherited")
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
    assert env["PYTHONPATH"] == str(Path(__file__).resolve().parents[1])
    assert not any(k in env for k in UNSET_ENV if k != "PYTHONPATH")
    assert env["CUDA_VISIBLE_DEVICES"] == "1"  # free_gpu: both free, GPU 1 preferred


def test_child_env_pins_chosen_gpu(tmp_path, settings, runners, monkeypatch):
    """CUDA_VISIBLE_DEVICES carries the chosen GPU, overriding whatever the server had."""
    from map_check.gpu import BusyReason as BR
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "0,1")
    d = tmp_path / "envdump"
    d.mkdir()
    (d / "render.py").write_text(
        'import json, os, sys\njob = json.load(open(sys.argv[-1]))\n'
        f'json.dump({{"env": os.environ.get("CUDA_VISIBLE_DEVICES"), "gpu": job["gpu"]}}, '
        f'open({str(tmp_path / "gpu.json")!r}, "w"))\n'
        'print(json.dumps({"event": "done"}), flush=True)\n')

    def gpu1_busy(own_pids=frozenset()):  # GPU 1 in use, overridden: GPU 0 is chosen
        return GpuStatus(gpus=[0, 1], busy=[BR(1, 99, "isaac-sim", "isaac-sim")])
    bench = BenchStore(settings.state_dir)
    r = runner(runners, settings, bench, lambda jf: [sys.executable, str(d / "render.py"), str(jf)],
               gpu_probe=gpu1_busy)
    r.submit("m1", override=True)
    assert finished(r, "m1").status == "done"
    got = json.loads((tmp_path / "gpu.json").read_text())
    assert got == {"env": "0", "gpu": 0}


def test_job_file_has_deadline(tmp_path, settings, runners):
    bench = BenchStore(settings.state_dir)
    r = runner(runners, settings, bench, fake_command(tmp_path), timeout_s=50.0, kill_grace_s=2.0)
    before = time.time()
    job = r.submit("m1", override=False)
    finished(r, "m1")
    jf = json.loads((settings.state_dir / "jobs" / f"{job.id}.json").read_text())
    assert before + 52.0 <= jf["deadline"] <= time.time() + 52.0


def test_isaac_render_imports_without_isaac():
    """The Isaac script parses as Python and defers every Isaac import to run time."""
    p = subprocess.run([sys.executable, "-c", "import map_check.render.isaac_render as m; "
                        "print(callable(m.main))"],
                       capture_output=True, text=True, cwd=Path(__file__).resolve().parents[1])
    assert p.stdout.strip() == "True", p.stderr


# Verbatim from the failed 05:03:37 render's log (20261020T050337-cc14fb.log, line 66; that
# file is gone): activeGpu=0 under CUDA_VISIBLE_DEVICES=1 left Kit with no device.
KIT_FAILED_LINE = ("2026-10-20T09:03:41Z [3,107ms] [Error] [omni.gpu_foundation_factory.plugin] "
                   "Failed to create any GPU devices, including an attempt with compatibility mode.")
# Verbatim from the healthy 07:09:42 render's log (20261020T070942-bc6a8a.log): Kit warns
# about the GPU it can't see, then renders on the one it can.
KIT_HEALTHY_LINES = [
    "[1.305s] [ext: omni.gpu_foundation-0.0.0] startup",
    "2026-10-20T11:09:45Z [1,525ms] [Warning] [omni.rtx] CUDA_VISIBLE_DEVICES environment variable is set.",
    "2026-10-20T11:09:46Z [2,438ms] [Warning] [omni.rtx] Skipping NVIDIA graphics device because "
    "the matching CUDA device could not be found: NVIDIA GeForce RTX 2080 Ti",
    "2026-10-20T11:09:46Z [2,439ms] [Warning] [omni.rtx] Skipping NVIDIA graphics device because "
    "the matching CUDA device could not be found: NVIDIA GeForce RTX 2080 Ti",
    "| 1   | NVIDIA GeForce RTX 2080 Ti       | Yes: 0 |     | 11510   MB | 10de      | 0          |",
]


def test_kit_line_matches_the_real_log():
    assert KIT_NO_GPU in KIT_FAILED_LINE
    assert not any(KIT_NO_GPU in line for line in KIT_HEALTHY_LINES)


@pytest.mark.parametrize("stream", ["err", "out"])
def test_kit_no_gpu_line_fails_job_fast_and_kills_group(tmp_path, settings, runners, stream):
    pids_file = tmp_path / "pids.json"
    bench = BenchStore(settings.state_dir)
    cmd = fake_command(tmp_path, hang=True, pids=str(pids_file),
                       kit_lines=[(stream, line) for line in KIT_HEALTHY_LINES[:3]]
                       + [(stream, KIT_FAILED_LINE)])
    r = runner(runners, settings, bench, cmd, timeout_s=120.0, kill_grace_s=2.0)
    t0 = time.monotonic()
    job = r.submit("m1", override=False)
    st = finished(r, "m1")
    assert time.monotonic() - t0 < 10  # not the 120 s timeout
    assert st.status == "failed"
    assert st.message == "Isaac could not create a GPU device on GPU 1 (see the job log)"
    pids = json.loads(pids_file.read_text())
    wait_for(lambda: not any(alive(p) for p in pids), timeout=5)
    log = (settings.state_dir / "jobs" / f"{job.id}.log").read_text()
    assert KIT_FAILED_LINE in log  # the reader still logs what Kit said


def test_kit_gpu_warnings_alone_do_not_stop_render(tmp_path, settings, runners):
    bench = BenchStore(settings.state_dir)
    cmd = fake_command(tmp_path, sleep=0.2, kit_lines=[("err", line) for line in KIT_HEALTHY_LINES]
                       + [("out", line) for line in KIT_HEALTHY_LINES[2:4]])
    r = runner(runners, settings, bench, cmd)
    job = r.submit("m1", override=False)
    st = finished(r, "m1")
    assert st.status == "done", st.message
    assert st.done_views == 3
    log = (settings.state_dir / "jobs" / f"{job.id}.log").read_text()
    assert "Skipping NVIDIA graphics device" in log  # stderr reaches the log through the reader


def test_finished_render_wins_over_late_kit_no_gpu_line(tmp_path, settings, runners):
    """Kit may print the no-GPU line while shutting down after "done": the job stays done."""
    bench = BenchStore(settings.state_dir)
    cmd = fake_command(tmp_path, after_done=[("err", KIT_FAILED_LINE)], linger=0.5)
    r = runner(runners, settings, bench, cmd)
    job = r.submit("m1", override=False)
    st = finished(r, "m1")
    assert st.status == "done" and st.message == "", st.message
    assert st.done_views == 3
    rec = bench.get("m1").last_render
    assert rec is not None and rec.job_id == job.id
    log = (settings.state_dir / "jobs" / f"{job.id}.log").read_text()
    assert KIT_FAILED_LINE in log  # the late line was seen, and ignored
