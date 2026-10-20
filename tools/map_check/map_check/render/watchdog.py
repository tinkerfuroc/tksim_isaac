"""A render's own deadline, independent of the Map Check server that started it.

The server starts the Isaac render in its own session (start_new_session=True) and
normally enforces the job timeout itself. If the server dies without cleaning up
(SIGKILL, OOM, a lost ssh session), nothing else would stop a hung Isaac holding a
GPU, so the render watches for itself: it exits once the job file's absolute
deadline passes or its parent process changes, and asks the kernel to SIGKILL it
when the parent dies (PR_SET_PDEATHSIG, Linux only).

Pure Python and stdlib only, so it is tested without Isaac.
"""
from __future__ import annotations

import os
import signal
import sys
import threading
import time

EXIT_CODE = 3
PR_SET_PDEATHSIG = 1  # <linux/prctl.h>
CHECK_INTERVAL_S = 1.0


def set_parent_death_signal(sig: int = signal.SIGKILL) -> bool:
    """Ask the kernel to send sig when the parent dies; False where unsupported.

    The kernel tracks the parent *thread* that forked us: the server's render worker
    thread lives as long as the server, so this matches the server's lifetime.
    """
    if not sys.platform.startswith("linux"):
        return False
    try:
        import ctypes

        libc = ctypes.CDLL(None, use_errno=True)
        return libc.prctl(PR_SET_PDEATHSIG, int(sig), 0, 0, 0) == 0
    except (OSError, AttributeError, ValueError):
        return False


def expired(deadline: float, start_ppid: int, now: float, ppid: int) -> str | None:
    """Why the render must stop now, or None to keep going."""
    if ppid != start_ppid:
        return f"the Map Check server (pid {start_ppid}) is gone"
    if now >= deadline:
        return "the render passed its deadline"
    return None


class Watchdog:
    """Polls expired() on a daemon thread and calls exit(EXIT_CODE) when it says stop.

    clock, getppid, exit, sleep and report are injectable for tests.
    """

    def __init__(self, deadline: float, start_ppid: int, *, clock=None, getppid=None, exit=None,
                 sleep=None, report=None, interval: float = CHECK_INTERVAL_S):
        self.deadline, self.start_ppid = float(deadline), int(start_ppid)
        self._clock, self._getppid = clock or time.time, getppid or os.getppid
        self._exit, self._sleep = exit or os._exit, sleep or time.sleep
        self._report, self._interval = report, interval
        self.reason: str | None = None
        self.thread = threading.Thread(target=self._run, name="map-check-watchdog", daemon=True)

    def check(self) -> str | None:
        """One poll: exits (through the injected exit) and returns the reason when expired."""
        reason = expired(self.deadline, self.start_ppid, self._clock(), self._getppid())
        if reason is not None:
            self.reason = reason
            if self._report is not None:
                try:  # the reader may be gone with the server; never let that block the exit
                    self._report(reason)
                except Exception:  # noqa: BLE001
                    pass
            self._exit(EXIT_CODE)
        return reason

    def _run(self) -> None:
        while self.check() is None:
            self._sleep(self._interval)

    def start(self) -> "Watchdog":
        self.thread.start()
        return self


def guard(deadline: float, start_ppid: int, report=None) -> Watchdog:
    """Arm both guards; start_ppid is os.getppid() read as early as possible in the process.

    Re-checks the parent right after prctl: if it died before the call, no signal will come.
    """
    set_parent_death_signal()
    dog = Watchdog(deadline, start_ppid, report=report)
    dog.check()  # exits at once if the parent already changed or the deadline already passed
    return dog.start()
