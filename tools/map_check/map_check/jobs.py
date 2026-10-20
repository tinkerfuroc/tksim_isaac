"""Render jobs: one FIFO queue across maps, one Isaac process at a time.

A job snapshots the map's ticked rows and arm pose when it is submitted. When
it reaches the head of the queue the worker computes the overlap flags, writes
the job file (state_dir/jobs/<id>.json) and runs the render command in its own
process group. Progress lines from the command count views; every status change
goes to the shared bench, and a finished job replaces the map's last render.
"""
from __future__ import annotations

import copy
import fcntl
import functools
import json
import os
import secrets
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from urllib.parse import quote

from . import robot
from .arena_store import ArenaStore
from .bench import BenchStore, JobStatus, RenderRecord, RowState
from .config import Settings
from .gpu import BusyReason, choose_gpu, describe, gpu_status
from .names import check_map_name
from .overlap import Flag, RobotModel, arena_bounds
from .poses import Pose
from .render.jobfile import OVERVIEW, VIEWS, JobFile, JobPose, parse_event

PACKAGE_ROOT = Path(__file__).resolve().parents[1]  # tools/map_check
ISAAC_MODULE = "map_check.render.isaac_render"
TIMEOUT_BASE_S, TIMEOUT_PER_VIEW_S, TIMEOUT_MAX_S = 120, 20, 1800
KILL_GRACE_S = 10.0  # SIGTERM to SIGKILL
EXIT_GRACE_S = 30.0  # how long the render may take to exit after "done"
ACTIVE = "active.json"  # the running job's pid, so a restarted server can stop it
LOCK = "runner.lock"  # held by the one JobRunner using a state dir
# Unset for the render, as scripts/gpsr-stack PREAMBLE_UNSET and
# scripts/gripper-close-probe do before Isaac: ROS-sourced paths break it.
UNSET_ENV = ("PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
             "ROS_PACKAGE_PATH", "LD_LIBRARY_PATH", "PKG_CONFIG_PATH")
CAPTIONS = {"behind": "behind-left", "top": "top-down"}


class GpuBusy(Exception):
    """A sim run is using the GPUs and the render was not overridden."""

    def __init__(self, reasons: list[BusyReason]):
        super().__init__(f"GPU busy: {describe(reasons)}")
        self.reasons = list(reasons)


def default_isaac_command(jobfile: Path, settings: Settings | None = None) -> list[str]:
    settings = settings or Settings.from_env(os.environ)
    return [str(settings.isaac_python), "-m", ISAAC_MODULE, str(jobfile)]


def job_env(gpu: int) -> dict[str, str]:
    """The server's environment minus ROS paths, plus the Kit EULA and map_check on the path.

    The render sees only the chosen GPU, as scripts/gpsr-stack pins its stages: with
    CUDA_VISIBLE_DEVICES set, Isaac renders on its device 0 (isaac_render passes
    --/renderer/activeGpu=0).
    """
    env = {k: v for k, v in os.environ.items() if k not in UNSET_ENV}
    env["CUDA_VISIBLE_DEVICES"] = str(int(gpu))
    env["OMNI_KIT_ACCEPT_EULA"] = "YES"
    env["PYTHONPATH"] = str(PACKAGE_ROOT)
    return env


def _proc_start(pid: int) -> str | None:
    """The process's start time (clock ticks since boot) from /proc, or None if it's gone."""
    try:
        stat = Path(f"/proc/{pid}/stat").read_text(errors="replace")
        return stat[stat.rindex(")") + 2:].split()[19]
    except (OSError, ValueError, IndexError):
        return None


def _caption(view: str, state: RowState) -> str:
    if view == "head":
        return f"head cam · pan {state.pan_deg:g}° · tilt {state.tilt_deg:g}°"
    return CAPTIONS[view]


def _kill_group(pgid: int, grace: float, reap=None) -> None:
    """SIGTERM the process group, then SIGKILL whatever is left after grace seconds."""
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return
    end = time.monotonic() + grace
    while time.monotonic() < end:
        if reap:
            reap()  # a zombie leader still counts as a group member
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.05)
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass


@dataclass
class _Job:
    status: JobStatus
    override: bool
    arm_pose: str
    arm_deg: list[float]
    rows: list[tuple[str, Pose, RowState]]
    boxes: list
    arena_dir: Path
    picked: bool = False
    proc: subprocess.Popen | None = None
    files: dict[tuple[str, str], str] = field(default_factory=dict)  # (key, view) -> file
    flags: dict[str, list[Flag]] = field(default_factory=dict)
    error: str = ""
    done: threading.Event = field(default_factory=threading.Event)


class JobRunner:
    def __init__(self, settings: Settings, bench: BenchStore, command_factory=default_isaac_command,
                 *, gpu_probe=gpu_status, robot_model=None, timeout_s: float | None = None,
                 kill_grace_s: float = KILL_GRACE_S):
        if command_factory is default_isaac_command:
            command_factory = functools.partial(default_isaac_command, settings=settings)
        self.settings, self.bench = settings, bench
        self._command, self._gpu_probe = command_factory, gpu_probe
        self._robot_model, self._models = robot_model, {}
        self._timeout_s, self._grace = timeout_s, kill_grace_s
        self._arenas = ArenaStore(settings)
        self._jobs_dir = settings.state_dir / "jobs"
        self._renders_dir = settings.state_dir / "renders"
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._queue: deque[_Job] = deque()
        self._current: _Job | None = None
        self._finished: dict[str, _Job] = {}
        self._stopping = False
        self._lock_fd = self._take_lock()
        try:
            self._stop_leftover()
        except BaseException:
            os.close(self._lock_fd)
            raise
        self._worker = threading.Thread(target=self._work, name="map-check-render", daemon=True)
        self._worker.start()

    @staticmethod
    def timeout_for(views: int) -> float:
        return min(TIMEOUT_MAX_S, TIMEOUT_BASE_S + TIMEOUT_PER_VIEW_S * views)

    # -- public --
    def submit(self, map_name: str, override: bool) -> JobStatus:
        """Queue a render of the map's ticked rows; GpuBusy when busy and not overridden."""
        check_map_name(map_name)
        view = self.bench.view(map_name, self.settings.maps_dir)
        rows = [(r.key, r.pose, copy.deepcopy(r.state)) for r in view.rows if r.state.included]
        if not rows:
            raise ValueError("no poses ticked for render")
        arm_pose = self.bench.get(map_name).arm_pose
        arm = robot.arm_poses(self.settings.arm_poses_json)
        if arm_pose not in arm:
            raise ValueError(f"unknown arm pose {arm_pose!r}")
        try:
            _, boxes, arena_dir = self._arenas.load_current(map_name)
        except (OSError, ValueError, KeyError, TypeError) as e:
            raise ValueError(f"no Map Check arena for {map_name}: {e}") from e
        if not override:
            busy = self._gpu_probe(own_pids=self._own_pids()).busy
            if busy:
                raise GpuBusy(busy)
        views = 1 + sum(getattr(s, v) for _, _, s in rows for v in VIEWS)
        job_id = f"{time.strftime('%Y%m%dT%H%M%S')}-{secrets.token_hex(3)}"
        job = _Job(JobStatus(id=job_id, map=map_name, status="queued", queued_at=time.time(),
                             total_views=views),
                   override, arm_pose, arm[arm_pose], rows, boxes, Path(arena_dir))
        with self._lock:
            if self._stopping:
                raise RuntimeError("the render queue is shut down")
            self._queue.append(job)
            self._advance_locked()
            return copy.copy(job.status)

    def status(self, map_name: str) -> JobStatus | None:
        """The map's running job, else its first queued one, else its last finished one."""
        with self._lock:
            st = self._relevant_locked(map_name)
            if st is not None:
                return copy.copy(st)
        return self.bench.get(map_name).job

    def shutdown(self) -> None:
        """Stop the worker, kill the running render's process group, mark jobs interrupted."""
        with self._lock:
            if self._stopping:
                return
            self._stopping = True
            jobs = ([self._current] if self._current else []) + list(self._queue)
            proc = self._current.proc if self._current else None
            self._queue.clear()
            self._current = None
            now = time.time()
            for j in jobs:
                j.status.status, j.status.position = "interrupted", None
                j.status.finished_at, j.status.message = now, "the server stopped"
                self._finished[j.status.map] = j
            for m in {j.status.map for j in jobs}:
                self._publish_locked(m)
            self._cv.notify_all()
        if proc is not None:
            _kill_group(proc.pid, self._grace, proc.poll)
        self._worker.join(timeout=self._grace + 5)
        os.close(self._lock_fd)  # releases the state dir for the next server

    # -- queue (call with the lock held) --
    def _relevant_locked(self, map_name: str) -> JobStatus | None:
        if self._current and self._current.status.map == map_name:
            return self._current.status
        for j in self._queue:
            if j.status.map == map_name:
                return j.status
        j = self._finished.get(map_name)
        return j.status if j else None

    def _publish_locked(self, map_name: str) -> None:
        st = self._relevant_locked(map_name)
        if st is not None:
            self.bench.set_job(map_name, st)

    def _advance_locked(self) -> None:
        changed = set()
        if self._current is None and self._queue:
            self._current = self._queue.popleft()
            st = self._current.status
            st.status, st.started_at = "running", time.time()
            changed.add(st.map)
            self._cv.notify_all()
        for i, j in enumerate(self._queue, 1):
            if j.status.position != i:
                j.status.position = i
                changed.add(j.status.map)
        if self._current:
            self._current.status.position = None
        for m in changed:
            self._publish_locked(m)

    def _own_pids(self) -> frozenset[int]:
        cur = self._current
        return frozenset({cur.proc.pid}) if cur and cur.proc else frozenset()

    # -- worker --
    def _work(self) -> None:
        while True:
            with self._cv:
                while not self._stopping and (self._current is None or self._current.picked):
                    self._cv.wait()
                if self._stopping:
                    return
                job = self._current
                job.picked = True
            try:
                status, message = self._run(job)
            except Exception as e:  # noqa: BLE001 - any failure ends this job, not the worker
                status, message = "failed", f"{type(e).__name__}: {e}"
            with self._lock:
                if self._stopping:
                    return  # shutdown already marked it interrupted
                st = job.status
                st.status, st.message, st.finished_at = status, message, time.time()
                self._finished[st.map] = job
                self._current = None
                self._publish_locked(st.map)
                self._advance_locked()

    def _model(self):
        if self._robot_model is not None:
            return self._robot_model
        files = robot.resolve_robot(self.settings.robot_pointer)
        key = (files.urdf, files.usd)
        if key not in self._models:
            self._models[key] = RobotModel(files.urdf, files.usd, self.settings.state_dir / "cache",
                                           repo_root=self.settings.repo_root,
                                           margin=self.settings.overlap_margin)
        return self._models[key]

    def _run(self, job: _Job) -> tuple[str, str]:
        st = job.status
        model = self._model()
        bounds = arena_bounds(job.boxes)
        for key, pose, s in job.rows:
            job.flags[key] = model.flags(pose, job.arm_deg, s.pan_deg, s.tilt_deg, job.boxes, bounds)
        gpus = self._gpu_probe(own_pids=self._own_pids())
        if gpus.busy and not job.override:
            return "failed", f"the GPU became busy while queued: {describe(gpus.busy)}"
        out_dir = self._renders_dir / st.map / st.id
        out_dir.mkdir(parents=True, exist_ok=True)
        timeout = self._timeout_s or self.timeout_for(st.total_views)
        gpu = choose_gpu(gpus)
        jf = JobFile(
            job_id=st.id, map=st.map, arena_usd=str(job.arena_dir / "arena.usd"),
            robot_usd=str(model.usd), robot_urdf=str(model.urdf), z0=float(model.z0),
            gpu=gpu, out_dir=str(out_dir), arm_deg=list(job.arm_deg),
            poses=[JobPose(key=k, name=p.name, x=p.x, y=p.y, yaw=p.yaw,
                           views=[v for v in VIEWS if getattr(s, v)],
                           pan_deg=s.pan_deg, tilt_deg=s.tilt_deg) for k, p, s in job.rows],
            # The render's own backstop (render.watchdog): start + the job timeout, plus the
            # kill grace so the server's timeout, reported as "timeout", always acts first.
            overview=True, deadline=time.time() + timeout + self._grace)
        # Beyond the JobFile schema: what the overview marks, computed here, not in Isaac.
        doc = jf.to_dict()
        doc["flagged"] = [k for k, _, _ in job.rows if job.flags[k]]
        doc["bounds"] = list(bounds)
        self._jobs_dir.mkdir(parents=True, exist_ok=True)
        jobfile = self._jobs_dir / f"{st.id}.json"
        jobfile.write_text(json.dumps(doc, indent=2) + "\n")
        log_path = self._jobs_dir / f"{st.id}.log"

        with open(log_path, "ab") as log:
            proc = subprocess.Popen(self._command(jobfile), stdin=subprocess.DEVNULL,
                                    stdout=subprocess.PIPE, stderr=log, start_new_session=True,
                                    env=job_env(gpu))
            reader = None
            try:
                started = time.monotonic()
                with self._lock:
                    job.proc = proc
                    stopping = self._stopping
                if stopping:
                    return "interrupted", "the server stopped"
                self._write_active(proc.pid, jobfile, st.map)
                reader = threading.Thread(target=self._read, args=(job, proc, log), daemon=True)
                reader.start()
                timed_out, done_at = False, None
                while True:
                    try:
                        proc.wait(timeout=0.1)
                        break
                    except subprocess.TimeoutExpired:
                        pass
                    now = time.monotonic()
                    if job.done.is_set():  # finished; don't wait long for Isaac to exit
                        done_at = done_at or now
                        if now - done_at > EXIT_GRACE_S:
                            break
                    elif now - started > timeout:
                        timed_out = True
                        break
            finally:  # on every path, exceptions included: nothing of the render keeps running
                _kill_group(proc.pid, self._grace, proc.poll)
                proc.wait()
                if reader is not None:
                    reader.join(timeout=5)
                proc.stdout.close()
                (self._jobs_dir / ACTIVE).unlink(missing_ok=True)

        if timed_out:
            return "timeout", f"no result after {timeout:g} s; the render was stopped"
        if not job.done.is_set():
            return "failed", job.error or f"the render exited with code {proc.returncode} (log: {log_path})"
        self.bench.set_last_render(st.map, self._record(job))
        return "done", ""

    def _read(self, job: _Job, proc: subprocess.Popen, log) -> None:
        for raw in proc.stdout:
            ev = parse_event(raw.decode(errors="replace"))
            if ev is None:
                log.write(raw)
                continue
            if ev["event"] == "done":
                job.done.set()
            elif ev["event"] == "error":
                job.error = str(ev.get("message", ""))
            else:
                name = Path(str(ev.get("file", ""))).name
                if not name:
                    continue
                with self._lock:
                    job.files[(str(ev.get("key", "")), str(ev.get("view", "")))] = name
                    job.status.done_views += 1
                    if not self._stopping:
                        self._publish_locked(job.status.map)

    def _record(self, job: _Job) -> RenderRecord:
        st = job.status

        def url(name):
            return f"/renders/{quote(st.map)}/{quote(st.id)}/{quote(name)}"

        overview = next((f for (k, v), f in job.files.items() if v == OVERVIEW), None)
        items = []
        for key, pose, s in job.rows:
            views = [{"view": v, "url": url(job.files[(key, v)]), "caption": _caption(v, s)}
                     for v in VIEWS if (key, v) in job.files]
            items.append({"key": key, "name": pose.name, "flags": job.flags.get(key, []),
                          "views": views})
        return RenderRecord(job_id=st.id, finished_at=time.time(), arm_pose=job.arm_pose,
                            items=items, overview_url=url(overview) if overview else None)

    # -- a render left running by a server that died --
    def _write_active(self, pid: int, jobfile: Path, map_name: str) -> None:
        tmp = self._jobs_dir / f".{ACTIVE}.tmp"
        tmp.write_text(json.dumps({"pid": pid, "jobfile": str(jobfile), "map": map_name,
                                   "server_pid": os.getpid(),
                                   "server_start": _proc_start(os.getpid())}))
        os.replace(tmp, self._jobs_dir / ACTIVE)

    def _take_lock(self) -> int:
        self._jobs_dir.mkdir(parents=True, exist_ok=True)
        fd = os.open(self._jobs_dir / LOCK, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(fd)
            raise RuntimeError(
                f"another Map Check server is using {self.settings.state_dir}") from None
        return fd

    def _stop_leftover(self) -> None:
        """Kill the render a dead server left running (the lock holder only)."""
        path = self._jobs_dir / ACTIVE
        try:
            doc = json.loads(path.read_text())
            pid, jobfile, map_name = int(doc["pid"]), str(doc["jobfile"]), str(doc["map"])
            server_pid, server_start = int(doc["server_pid"]), doc["server_start"]
        except (OSError, ValueError, KeyError, TypeError):
            return
        if server_start is not None and _proc_start(server_pid) == str(server_start):
            return  # its server still runs: not ours to stop
        try:
            self.bench.get(map_name)  # loading marks its running job interrupted
        except ValueError:
            pass
        try:
            args = Path(f"/proc/{pid}/cmdline").read_bytes().decode(errors="replace").split("\0")
            ours = jobfile in args and os.getpgid(pid) == pid
        except (OSError, ProcessLookupError):
            ours = False
        if ours:
            _kill_group(pid, self._grace)
        path.unlink(missing_ok=True)
