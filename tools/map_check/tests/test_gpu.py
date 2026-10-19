import subprocess

import pytest

from map_check.gpu import BusyReason, GpuStatus, choose_gpu, describe, gpu_status

SMI_GPUS = "0, GPU-aaa\n1, GPU-bbb\n"


def fake_run(apps="", gpus=SMI_GPUS):
    def run(cmd, **kw):
        assert kw == {"capture_output": True, "text": True, "timeout": 5}
        out = gpus if "--query-gpu=index,uuid" in cmd else apps
        return subprocess.CompletedProcess(cmd, 0, out, "")
    return run


def add_proc(root, pid, cmdline, ppid=1):
    d = root / str(pid)
    d.mkdir()
    (d / "cmdline").write_bytes("\0".join(cmdline).encode() + b"\0")
    (d / "stat").write_text(f"{pid} (my) proc) S {ppid} 0 0\n")


def test_free(tmp_path):
    add_proc(tmp_path, 10, ["bash"])
    st = gpu_status(run=fake_run(), proc_root=tmp_path)
    assert st.gpus == [0, 1] and st.busy == []
    assert choose_gpu(st) == 1


def test_compute_app_busy(tmp_path):
    st = gpu_status(run=fake_run("GPU-aaa, 4242, /usr/bin/gpsr-stack\n"), proc_root=tmp_path)
    assert st.busy == [BusyReason(gpu=0, pid=4242, name="gpsr-stack", cmd="")]
    assert choose_gpu(st) == 1
    assert describe(st.busy) == "gpsr-stack (pid 4242) on GPU 0"


def test_all_busy_fewest(tmp_path):
    apps = "GPU-aaa, 1, a\nGPU-bbb, 2, b\nGPU-bbb, 3, c\n"
    assert choose_gpu(gpu_status(run=fake_run(apps), proc_root=tmp_path)) == 0


def test_cmdline_busy(tmp_path):
    add_proc(tmp_path, 123, ["bash", "scripts/gpsr-stack", "up"])
    st = gpu_status(run=fake_run(), proc_root=tmp_path)
    assert len(st.busy) == 1
    r = st.busy[0]
    assert (r.gpu, r.pid, r.name) == (None, 123, "gpsr-stack")
    assert r.cmd == "bash scripts/gpsr-stack up"
    assert describe(st.busy) == "gpsr-stack (pid 123)"


def test_own_job_ignored(tmp_path):
    add_proc(tmp_path, 555, ["python", "run_sim.py"])
    add_proc(tmp_path, 556, ["python", "omni.kit"], ppid=555)
    add_proc(tmp_path, 557, ["python", "run_sim.py"], ppid=556)
    add_proc(tmp_path, 600, ["python", "run_sim.py"])
    st = gpu_status(run=fake_run(), proc_root=tmp_path, own_pids=frozenset({555}))
    assert [r.pid for r in st.busy] == [600]


def test_nvidia_smi_missing(tmp_path):
    add_proc(tmp_path, 123, ["x", "run_sim.py"])

    def run(cmd, **kw):
        raise FileNotFoundError

    st = gpu_status(run=run, proc_root=tmp_path)
    assert st.gpus == [] and [r.pid for r in st.busy] == [123]


def test_nvidia_smi_nonzero_and_timeout(tmp_path):
    bad = lambda cmd, **kw: subprocess.CompletedProcess(cmd, 9, "", "err")
    assert gpu_status(run=bad, proc_root=tmp_path) == GpuStatus(gpus=[], busy=[])

    def slow(cmd, **kw):
        raise subprocess.TimeoutExpired(cmd, 5)

    assert gpu_status(run=slow, proc_root=tmp_path).gpus == []


def test_marker_past_truncation_limit(tmp_path):
    add_proc(tmp_path, 321, ["python", "x" * 300, "run_sim.py"])
    st = gpu_status(run=fake_run(), proc_root=tmp_path)
    assert [r.pid for r in st.busy] == [321]
    assert st.busy[0].name == "run_sim.py" and len(st.busy[0].cmd) == 200
