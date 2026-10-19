"""GPU busy detection: nvidia-smi compute apps plus a /proc scan for sim processes."""
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

PATTERNS = ("run_sim.py", "gpsr-stack", "isaac-sim", "omni.kit")
CMD_MAX = 200
SMI_TIMEOUT_S = 5


@dataclass(frozen=True)
class BusyReason:
    gpu: int | None
    pid: int
    name: str
    cmd: str


@dataclass
class GpuStatus:
    gpus: list[int] = field(default_factory=list)
    busy: list[BusyReason] = field(default_factory=list)


def _smi(run, query: str) -> list[list[str]] | None:
    try:
        p = run(["nvidia-smi", f"--query-{query}", "--format=csv,noheader"],
                capture_output=True, text=True, timeout=SMI_TIMEOUT_S)
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return None
    if p.returncode != 0:
        return None
    return [[c.strip() for c in line.split(",")] for line in p.stdout.splitlines() if line.strip()]


def _read_cmdline(proc_root: Path, pid: str, limit: int | None = None) -> str | None:
    try:
        raw = (proc_root / pid / "cmdline").read_bytes()
    except OSError:
        return None
    return " ".join(p for p in raw.decode(errors="replace").split("\0") if p)[:limit]


def _ppid(proc_root: Path, pid: str) -> int | None:
    try:
        stat = (proc_root / pid / "stat").read_text(errors="replace")
        return int(stat[stat.rindex(")") + 1:].split()[1])
    except (OSError, ValueError, IndexError):
        return None


def _excluded(proc_root: Path, pids: list[str], own_pids) -> set[int]:
    """own_pids plus every descendant found in proc_root."""
    parent = {int(p): _ppid(proc_root, p) for p in pids if p.isdigit()}
    out = set(own_pids)
    changed = True
    while changed:
        changed = False
        for pid, pp in parent.items():
            if pid not in out and pp in out:
                out.add(pid)
                changed = True
    return out


def gpu_status(run=subprocess.run, proc_root: Path = Path("/proc"),
               own_pids=frozenset()) -> GpuStatus:
    proc_root = Path(proc_root)
    try:
        pids = [d.name for d in proc_root.iterdir() if d.name.isdigit()]
    except OSError:
        pids = []
    skip = _excluded(proc_root, pids, own_pids)

    status = GpuStatus()
    uuid_to_idx: dict[str, int] = {}
    rows = _smi(run, "gpu=index,uuid")
    if rows is not None:
        for row in rows:
            try:
                uuid_to_idx[row[1]] = int(row[0])
            except (IndexError, ValueError):
                continue
        status.gpus = sorted(uuid_to_idx.values())
        apps = _smi(run, "compute-apps=gpu_uuid,pid,process_name") or []
        for row in apps:
            try:
                pid = int(row[1])
            except (IndexError, ValueError):
                continue
            if pid in skip:
                continue
            name = Path(row[2]).name if len(row) > 2 else ""
            status.busy.append(BusyReason(uuid_to_idx.get(row[0]), pid, name,
                                          _read_cmdline(proc_root, str(pid), CMD_MAX) or ""))

    seen = {r.pid for r in status.busy}
    for p in sorted(pids, key=int):
        pid = int(p)
        if pid in skip or pid in seen:
            continue
        cmd = _read_cmdline(proc_root, p)
        if not cmd:
            continue
        for pat in PATTERNS:
            if pat in cmd:
                status.busy.append(BusyReason(None, pid, pat, cmd[:CMD_MAX]))
                break
    return status


def choose_gpu(status: GpuStatus) -> int:
    """First free GPU, preferring 1; if all are busy, the one with the fewest apps.

    status.gpus == [] means the GPU list is unknown (nvidia-smi unavailable); the
    fallback to GPU 0 is then arbitrary.
    """
    gpus = status.gpus or [0]
    count = {g: sum(1 for r in status.busy if r.gpu == g) for g in gpus}
    free = [g for g in gpus if count[g] == 0]
    if free:
        return 1 if 1 in free else free[0]
    return min(gpus, key=lambda g: (count[g], g))


def describe(busy: list[BusyReason]) -> str:
    return "; ".join(
        f"{r.name} (pid {r.pid})" + (f" on GPU {r.gpu}" if r.gpu is not None else "")
        for r in busy)
