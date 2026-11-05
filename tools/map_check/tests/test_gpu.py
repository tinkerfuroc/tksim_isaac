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
    add_proc(tmp_path, 123, ["python", "run_sim.py"])

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
    add_proc(tmp_path, 321, ["python", "/" + "x" * 300 + "/run_sim.py"])
    st = gpu_status(run=fake_run(), proc_root=tmp_path)
    assert [r.pid for r in st.busy] == [321]
    assert st.busy[0].name == "run_sim.py" and len(st.busy[0].cmd) == 200


def test_sim_launch_forms_busy(tmp_path):
    add_proc(tmp_path, 11, ["/usr/bin/python3", "-u", "/repo/validation/run_sim.py", "--headless"])
    add_proc(tmp_path, 12, ["/bin/bash", "/repo/scripts/gpsr-stack", "up"])
    add_proc(tmp_path, 13, ["/isaac-sim/kit/kit", "/isaac-sim/apps/isaacsim.exp.full.kit"])
    st = gpu_status(run=fake_run(), proc_root=tmp_path)
    assert [(r.pid, r.name) for r in st.busy] == [(11, "run_sim.py"), (12, "gpsr-stack"),
                                                  (13, "isaac-sim")]


def test_text_mentioning_marker_not_busy(tmp_path):
    # An orphaned log monitor once blocked every render: its grep pattern and its
    # `zsh -c` script both mention gpsr-stack, but neither runs a sim.
    add_proc(tmp_path, 21, ["/usr/bin/zsh", "-c", "tail -F battery.log | grep 'gpsr-stack up rc'"])
    add_proc(tmp_path, 22, ["ugrep", "-E", "--line-buffered", "PREFLIGHT|gpsr-stack up rc"])
    add_proc(tmp_path, 23, ["tail", "-F", "/runs/run_sim.py.log"])
    add_proc(tmp_path, 24, ["python3", "-c", "import run_sim.py"])
    assert gpu_status(run=fake_run(), proc_root=tmp_path).busy == []


def test_isaacsim_and_other_map_check_renders_busy(tmp_path):
    add_proc(tmp_path, 31, ["/repo/.venv/bin/python", "-m", "isaacsim", "--headless"])
    add_proc(tmp_path, 32, ["/repo/.venv/bin/python", "-m", "map_check.render.isaac_render",
                            "/other/state/jobs/j1.json"])
    add_proc(tmp_path, 33, ["/opt/isaacsim/python.sh", "standalone.py"])
    st = gpu_status(run=fake_run(), proc_root=tmp_path)
    assert [(r.pid, r.name) for r in st.busy] == [(31, "isaacsim"), (32, "isaac_render"),
                                                  (33, "isaacsim")]


def test_own_render_not_busy(tmp_path):
    """This server's own job (and what it spawns) matches isaac_render but is excluded."""
    add_proc(tmp_path, 41, ["/repo/.venv/bin/python", "-m", "map_check.render.isaac_render",
                            "/state/jobs/mine.json"])
    add_proc(tmp_path, 42, ["/repo/.venv/bin/python", "-m", "isaacsim.kit"], ppid=41)
    add_proc(tmp_path, 43, ["/repo/.venv/bin/python", "-m", "map_check.render.isaac_render",
                            "/other/jobs/theirs.json"])
    st = gpu_status(run=fake_run(), proc_root=tmp_path, own_pids=frozenset({41}))
    assert [r.pid for r in st.busy] == [43]
