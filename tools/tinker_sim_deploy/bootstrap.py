from __future__ import annotations

import os
import shutil
import tempfile
from pathlib import Path
from typing import Any

from .checkout import ensure_isaac_lab, ensure_isaacsim_ros_workspaces
from .config import Config
from .preflight import Check, collect
from .process import deployment_env, run
from .provenance import verify
from .report import build_report, write_report
from .ros_boundary import locate_internal_humble
from .ros_vendor import ensure_ros_vendor


CONTAMINATING_VARIABLES = (
    "AMENT_PREFIX_PATH",
    "CMAKE_PREFIX_PATH",
    "COLCON_CURRENT_PREFIX",
    "COLCON_PREFIX_PATH",
    "LD_LIBRARY_PATH",
    "PYTHONHOME",
    "PYTHONPATH",
    "ROS_PACKAGE_PATH",
    "ROS_PYTHON_VERSION",
    "ROS_VERSION",
    "VIRTUAL_ENV",
)


def _require_eula() -> None:
    if os.environ.get("TINKER_ACCEPT_OMNIVERSE_EULA") != "Y":
        raise RuntimeError(
            "Omniverse EULA acceptance is required. Review NVIDIA's terms, then set "
            "TINKER_ACCEPT_OMNIVERSE_EULA=Y in deployment configuration."
        )


def _uv_version(config: Config) -> str:
    if shutil.which("uv") is None:
        raise RuntimeError("uv is not installed; install the pinned 0.10 release before bootstrap")
    result = run(["uv", "--version"], check=True)
    version = result.stdout.strip().split()[-1]
    actual = tuple(int(part) for part in version.split(".")[:2])
    if actual != config.uv_minor:
        raise RuntimeError(
            f"uv {config.runtime['uv']}.x is required to consume this lock; found {version}"
        )
    return version


def base_environment(config: Config) -> dict[str, str]:
    env = deployment_env(
        config.root,
        config.path("uv_cache"),
        config.path("uv_python"),
        config.path("isaac_cache"),
    )
    for variable in CONTAMINATING_VARIABLES:
        env.pop(variable, None)
    env.update(
        {
            "ACCEPT_EULA": "Y",
            "OMNI_KIT_ACCEPT_EULA": "YES",
            "ROS_DISTRO": config.ros["distro"],
            "ROS_DOMAIN_ID": str(os.environ.get("ROS_DOMAIN_ID", config.ros["domain_id"])),
            "RMW_IMPLEMENTATION": str(
                os.environ.get("RMW_IMPLEMENTATION", config.ros["rmw_implementation"])
            ),
        }
    )
    env.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
    env["TINKER_SIM_DDS_PROFILE"] = "local"
    return env


def _test(name: str, command: list[str], config: Config, env: dict[str, str]) -> dict[str, Any]:
    result = run(command, cwd=config.root, env=env, capture=False)
    return {"status": "pass" if result.ok else "fail", "returncode": result.returncode}


def execute(
    config: Config,
    *,
    mode: str,
    skip_preflight: bool = False,
    skip_validation: bool = False,
    skip_prewarm: bool = False,
) -> Path:
    _require_eula()
    _uv_version(config)
    offline = mode == "offline"
    checks: list[Check] = [] if skip_preflight else collect(config)
    failures = [check for check in checks if check.status == "fail"]
    if failures:
        names = ", ".join(check.name for check in failures)
        raise RuntimeError(f"host preflight failed: {names}")
    env = base_environment(config)
    for path_name in (
        "uv_cache",
        "uv_python",
        "isaac_cache",
        "reports",
        "artifacts",
        "ros_deb_cache",
        "ros_vendor",
    ):
        config.path(path_name).mkdir(parents=True, exist_ok=True)
    ensure_isaac_lab(config, offline=offline)
    ensure_isaacsim_ros_workspaces(config, offline=offline)
    ensure_ros_vendor(config, offline=offline)
    verify(config, require_python=offline)
    if not offline:
        run(
            ["uv", "python", "install", config.runtime["python"]],
            cwd=config.root,
            env=env,
            check=True,
            capture=False,
        )
        verify(config, require_python=True)
    sync = ["uv", "sync", "--frozen"]
    if offline:
        sync.append("--offline")
    run(sync, cwd=config.root, env=env, check=True, capture=False)
    internal_humble = locate_internal_humble(config, env)
    env["LD_LIBRARY_PATH"] = str(internal_humble)
    tests: dict[str, Any] = {
        "locked_sync": {"status": "pass", "mode": mode},
    }
    if not offline:
        with tempfile.TemporaryDirectory(
            prefix="offline-sync-audit-",
            dir=config.path("artifacts"),
        ) as temporary:
            offline_env = dict(env)
            offline_env["UV_PROJECT_ENVIRONMENT"] = str(Path(temporary) / ".venv")
            offline_env["UV_LINK_MODE"] = "hardlink"
            tests["offline_cache_sync"] = _test(
                "offline_cache_sync",
                ["uv", "sync", "--frozen", "--offline"],
                config,
                offline_env,
            )
    tests["python_boundary"] = _test(
        "python_boundary",
        [
            "uv",
            "run",
            "--frozen",
            "--no-sync",
            "python",
            "-c",
            (
                "import sys;"
                "bad=[p for p in sys.path if '/opt/ros/' in p or 'python3.10' in p];"
                "print({'contaminating_paths':bad});"
                "raise SystemExit(bool(bad))"
            ),
        ],
        config,
        env,
    )
    if not skip_validation:
        tests["compatibility"] = _test(
            "compatibility",
            [
                "uv",
                "run",
                "--frozen",
                "--no-sync",
                "isaacsim",
                "isaacsim.exp.compatibility_check",
                "--/app/quitAfter=10",
                "--no-window",
            ],
            config,
            env,
        )
        tests["headless_physx_10000"] = _test(
            "headless_physx_10000",
            [
                "uv",
                "run",
                "--frozen",
                "--no-sync",
                "python",
                "validation/headless_smoke.py",
                "--steps",
                "10000",
            ],
            config,
            env,
        )
        tests["rtx_camera_lidar"] = _test(
            "rtx_camera_lidar",
            [
                "uv",
                "run",
                "--frozen",
                "--no-sync",
                "python",
                "validation/rtx_sensor_smoke.py",
            ],
            config,
            env,
        )
        tests["nvenc"] = _test(
            "nvenc",
            [
                "ffmpeg",
                "-hide_banner",
                "-f",
                "lavfi",
                "-i",
                "color=black:s=1280x720:d=1",
                "-c:v",
                "h264_nvenc",
                "-f",
                "null",
                "-",
            ],
            config,
            env,
        )
        tests["webrtc_startup"] = _test(
            "webrtc_startup",
            [
                "uv",
                "run",
                "--frozen",
                "--no-sync",
                "python",
                "validation/webrtc_smoke.py",
            ],
            config,
            env,
        )
    if not skip_prewarm:
        tests["cache_prewarm"] = _test(
            "cache_prewarm",
            [
                "uv",
                "run",
                "--frozen",
                "--no-sync",
                "python",
                "validation/prewarm.py",
            ],
            config,
            env,
        )
    report = build_report(config, env, mode=mode, preflight=checks, tests=tests)
    path = write_report(config, report)
    failed_tests = [name for name, value in tests.items() if value["status"] != "pass"]
    if failed_tests:
        raise RuntimeError(
            f"deployment validation failed ({', '.join(failed_tests)}); report: {path}"
        )
    return path
