"""validation/gpsr_run_record.sh: syntax, dry-run command lines, and signal forwarding.

Signal forwarding is verified with a fake ``python3`` on PATH that records its argv
and the first signal it receives, then exits; the wrapper must forward the signal
to BOTH children exactly once, wait for both, and exit 0.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "validation" / "gpsr_run_record.sh"

FAKE_PY = r"""#!/usr/bin/env bash
# Fake python3: log argv, wait for one signal, log it, exit.
log="$FAKE_LOG_DIR/$$.argv"; printf '%s\n' "$@" > "$log"
sig_log="$FAKE_LOG_DIR/$$.sig"
n=0
on_sig() { n=$((n+1)); echo "$1 $n" >> "$sig_log"; }
trap 'on_sig TERM' TERM
trap 'on_sig INT' INT
# Stay alive until signalled; after the first signal linger briefly (a second
# signal in this window would be logged) and exit cleanly.
while [ "$n" = 0 ]; do sleep 0.05; done
sleep 0.3
exit 0
"""


def _wait_for(pred, timeout=10.0, interval=0.05):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(interval)
    return pred()


def test_bash_syntax_ok():
    subprocess.run(["bash", "-n", str(SCRIPT)], check=True)


def test_dry_run_prints_both_commands(tmp_path):
    run_dir = tmp_path / "run 01"
    out = subprocess.run(
        ["bash", str(SCRIPT), "--dry-run", str(run_dir), "--topic", "/arena/image=arena"],
        capture_output=True, text=True, check=True,
    ).stdout
    lines = out.strip().splitlines()
    assert len(lines) == 2
    rec, probe = lines
    assert rec.startswith("recorder: ") and "validation/gpsr_run_recorder.py" in rec
    assert "--interval 1.0" in rec and "--topic /arena/image=arena" in rec
    assert probe.startswith("probe: ") and "tools/gpsr_run_truth_probe.py" in probe
    assert "--topic" not in probe
    assert not run_dir.exists()  # dry-run starts nothing and creates nothing


def test_usage_error_without_run_dir():
    r = subprocess.run(["bash", str(SCRIPT)], capture_output=True, text=True)
    assert r.returncode == 2
    assert "usage" in r.stderr


@pytest.mark.parametrize("sig", [signal.SIGTERM, signal.SIGINT])
def test_forwards_signal_to_both_children_once_and_exits_zero(tmp_path, sig):
    shim = tmp_path / "bin"
    shim.mkdir()
    fake = shim / "python3"
    fake.write_text(FAKE_PY)
    fake.chmod(0o755)
    log_dir = tmp_path / "logs"
    log_dir.mkdir()
    run_dir = tmp_path / "run"

    env = dict(os.environ, PATH=f"{shim}{os.pathsep}{os.environ.get('PATH', '')}",
               FAKE_LOG_DIR=str(log_dir))
    env.pop("GPSR_RECORD_PYTHON", None)
    proc = subprocess.Popen(
        ["bash", str(SCRIPT), str(run_dir), "--topic", "/arena/image=arena"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True,
    )
    try:
        assert _wait_for(lambda: len(list(log_dir.glob("*.argv"))) == 2), "both children did not start"
        assert run_dir.is_dir()
        # Like the bench: signal the wrapper's process group.
        os.killpg(os.getpgid(proc.pid), sig)
        out, _ = proc.communicate(timeout=15)
    finally:
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait()

    assert proc.returncode == 0, out
    argvs = sorted(p.read_text() for p in log_dir.glob("*.argv"))
    assert any("gpsr_run_recorder.py" in a and "--interval\n1.0" in a and "/arena/image=arena" in a
               for a in argvs)
    assert any("gpsr_run_truth_probe.py" in a for a in argvs)
    sig_logs = list(log_dir.glob("*.sig"))
    assert len(sig_logs) == 2, out
    name = "TERM" if sig == signal.SIGTERM else "INT"
    for p in sig_logs:
        assert p.read_text().splitlines() == [f"{name} 1"], (p.read_text(), out)
    assert "both recorders exited" in out


def test_exits_zero_when_children_finish_on_their_own(tmp_path):
    shim = tmp_path / "bin"
    shim.mkdir()
    fake = shim / "python3"
    fake.write_text("#!/usr/bin/env bash\nexit 0\n")
    fake.chmod(0o755)
    env = dict(os.environ, PATH=f"{shim}{os.pathsep}{os.environ.get('PATH', '')}")
    env.pop("GPSR_RECORD_PYTHON", None)
    r = subprocess.run(["bash", str(SCRIPT), str(tmp_path / "run")], env=env,
                       capture_output=True, text=True, timeout=15)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "both recorders exited" in r.stdout


def test_extents_json_env_forwarded_to_probe_only_when_file_exists(tmp_path):
    extents = tmp_path / "pose_audit.json"
    extents.write_text("{}")
    base_env = dict(os.environ)
    base_env.pop("GPSR_TRUTH_EXTENTS_JSON", None)
    out = subprocess.run(
        ["bash", str(SCRIPT), "--dry-run", str(tmp_path / "run")],
        env=dict(base_env, GPSR_TRUTH_EXTENTS_JSON=str(extents)),
        capture_output=True, text=True, check=True,
    ).stdout
    rec, probe = out.strip().splitlines()
    assert f"--extents-json {extents}" in probe
    assert "--extents-json" not in rec

    for env in (dict(base_env, GPSR_TRUTH_EXTENTS_JSON=str(tmp_path / "missing.json")), base_env):
        out = subprocess.run(
            ["bash", str(SCRIPT), "--dry-run", str(tmp_path / "run")],
            env=env, capture_output=True, text=True, check=True,
        ).stdout
        assert "--extents-json" not in out


def test_gpsr_record_python_env_override_is_honoured(tmp_path):
    out = subprocess.run(
        ["bash", str(SCRIPT), "--dry-run", str(tmp_path)],
        env=dict(os.environ, GPSR_RECORD_PYTHON=sys.executable),
        capture_output=True, text=True, check=True,
    ).stdout
    assert sys.executable in out
