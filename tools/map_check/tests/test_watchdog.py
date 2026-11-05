"""The render's own deadline and parent-death guard (render.watchdog), without Isaac."""
import json
import os
import signal
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest

from map_check.render import watchdog
from map_check.render.watchdog import EXIT_CODE, Watchdog, expired

PKG = Path(__file__).resolve().parents[1]


class Fake:
    """Injected clock, parent pid and exit for one Watchdog."""

    def __init__(self, now=1000.0, ppid=4242):
        self.now, self.ppid, self.exits, self.reports = now, ppid, [], []

    def dog(self, deadline=1010.0, start_ppid=4242, **kw):
        return Watchdog(deadline, start_ppid, clock=lambda: self.now, getppid=lambda: self.ppid,
                        exit=self.exits.append, report=self.reports.append, **kw)


def test_expired_reasons():
    assert expired(10.0, 7, now=9.9, ppid=7) is None
    assert "deadline" in expired(10.0, 7, now=10.0, ppid=7)
    assert "pid 7" in expired(10.0, 7, now=0.0, ppid=1)
    assert "server" in expired(10.0, 7, now=99.0, ppid=1)  # a dead parent wins over the deadline


def test_keeps_going_before_deadline_with_same_parent():
    f = Fake()
    d = f.dog()
    assert d.check() is None and f.exits == [] and f.reports == []


def test_exits_once_deadline_passes():
    f = Fake()
    d = f.dog()
    f.now = 1010.0
    assert "deadline" in d.check()
    assert f.exits == [EXIT_CODE] and "deadline" in f.reports[0]


def test_exits_when_parent_changes():
    f = Fake()
    d = f.dog()
    f.ppid = 1  # reparented: the server died
    assert "server" in d.check()
    assert f.exits == [EXIT_CODE]


def test_report_failure_never_blocks_exit():
    f = Fake(now=2000.0)

    def broken(_):
        raise BrokenPipeError

    d = Watchdog(1010.0, 4242, clock=lambda: f.now, getppid=lambda: f.ppid, exit=f.exits.append,
                 report=broken)
    d.check()
    assert f.exits == [EXIT_CODE]


def test_thread_polls_until_expired():
    f = Fake()
    sleeps = []

    def sleep(s):  # each poll advances the injected clock by 4 s
        sleeps.append(s)
        f.now += 4.0

    d = f.dog(sleep=sleep, interval=0.5).start()
    d.thread.join(5)
    assert not d.thread.is_alive()
    assert f.exits == [EXIT_CODE] and f.now == 1012.0 and sleeps == [0.5, 0.5, 0.5]


def test_parent_death_signal_available_on_linux():
    if not sys.platform.startswith("linux"):
        pytest.skip("prctl is Linux only")
    code = "from map_check.render.watchdog import set_parent_death_signal as s; print(s())"
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, cwd=PKG)
    assert out.stdout.strip() == "True", out.stderr


GUARDED = textwrap.dedent("""
    import json, os, sys, time
    from map_check.render.jobfile import error_event
    from map_check.render.watchdog import guard
    start_ppid = os.getppid()
    print(json.dumps({"pid": os.getpid()}), flush=True)
    guard(float(sys.argv[1]), start_ppid, report=lambda why: print(error_event(why), flush=True))
    time.sleep(60)
""")


def _env():
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "AMENT_PREFIX_PATH")}
    env["PYTHONPATH"] = str(PKG)
    return env


def test_guarded_process_exits_at_its_deadline(tmp_path):
    script = tmp_path / "guarded.py"
    script.write_text(GUARDED)
    t0 = time.monotonic()
    p = subprocess.run([sys.executable, str(script), str(time.time() + 0.5)], env=_env(),
                       capture_output=True, text=True, timeout=30)
    assert p.returncode == EXIT_CODE, p.stderr
    assert time.monotonic() - t0 < 10
    events = [json.loads(line) for line in p.stdout.splitlines()]
    assert events[-1]["event"] == "error" and "deadline" in events[-1]["message"]


def test_guarded_process_dies_with_its_parent(tmp_path):
    """The server is SIGKILLed (no atexit, no cleanup): its render must not outlive it."""
    script = tmp_path / "guarded.py"
    script.write_text(GUARDED)
    parent = textwrap.dedent(f"""
        import subprocess, sys, time
        child = subprocess.Popen([sys.executable, {str(script)!r}, str(time.time() + 600)],
                                 stdout=sys.stdout, start_new_session=True)
        time.sleep(60)
    """)
    proc = subprocess.Popen([sys.executable, "-c", parent], env=_env(), stdout=subprocess.PIPE,
                            text=True)
    child_pid = None
    try:
        child_pid = json.loads(proc.stdout.readline())["pid"]
        time.sleep(0.3)  # let the guard arm
        assert _alive(child_pid)
        os.kill(proc.pid, signal.SIGKILL)
        proc.wait(10)
        end = time.monotonic() + 10
        while _alive(child_pid) and time.monotonic() < end:
            time.sleep(0.05)
        assert not _alive(child_pid), "the guarded render outlived its parent"
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        if child_pid and _alive(child_pid):
            os.kill(child_pid, signal.SIGKILL)


def _alive(pid):
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
    except OSError:
        return False
    return stat[stat.rindex(")") + 2] != "Z"


def test_guard_rechecks_after_prctl(monkeypatch):
    """A parent that died before prctl sends no signal: guard notices at once."""
    exits = []
    monkeypatch.setattr(watchdog, "set_parent_death_signal", lambda: True)
    monkeypatch.setattr(watchdog.os, "getppid", lambda: 1)
    monkeypatch.setattr(watchdog.os, "_exit", exits.append)
    monkeypatch.setattr(watchdog.Watchdog, "start", lambda self: self)
    watchdog.guard(time.time() + 600, start_ppid=4242)
    assert exits == [EXIT_CODE]


def test_cancel_stands_down_and_exit_code_is_configurable():
    f = Fake()
    d = f.dog(exit_code=4)
    d.cancel()
    f.now = 99999.0
    assert d.check() is None and f.exits == []
    d2 = f.dog(exit_code=4)
    assert d2.check() is not None and f.exits == [4]


def test_cancelled_thread_ends_without_exit():
    f = Fake()
    d = f.dog(sleep=lambda s: d.cancel(), interval=0.01)
    d.start().thread.join(5)
    assert not d.thread.is_alive() and f.exits == []


def test_cancel_racing_the_exit_wins():
    """cancel() lands after the reason is computed but before the exit: no exit, no report."""
    f = Fake()

    def clock():  # the deadline has passed, and the first image arrives at this very moment
        d.cancel()
        return 2000.0

    d = Watchdog(1010.0, 4242, clock=clock, getppid=lambda: f.ppid, exit=f.exits.append,
                 report=f.reports.append)
    assert d.check() is None
    assert f.exits == [] and f.reports == [] and d.reason is None


def test_cancel_waits_for_an_exit_in_progress():
    """A cancel() during report/exit blocks on the lock; the exit that began still happens."""
    import threading
    f = Fake(now=2000.0)
    in_report, release = threading.Event(), threading.Event()

    def report(_):
        in_report.set()
        release.wait(5)

    d = Watchdog(1010.0, 4242, clock=lambda: f.now, getppid=lambda: f.ppid, exit=f.exits.append,
                 report=report)
    t = threading.Thread(target=d.check)
    t.start()
    assert in_report.wait(5)
    c = threading.Thread(target=d.cancel)
    c.start()
    c.join(0.2)
    assert c.is_alive()  # held off until the exit decision completes
    release.set()
    t.join(5)
    c.join(5)
    assert f.exits == [EXIT_CODE] and not c.is_alive()
