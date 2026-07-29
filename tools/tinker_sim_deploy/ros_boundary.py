from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping

from .config import Config
from .process import deployment_env, run


ROS_PATH_VARIABLES = (
    "PYTHONPATH",
    "AMENT_PREFIX_PATH",
    "CMAKE_PREFIX_PATH",
    "COLCON_PREFIX_PATH",
    "ROS_PACKAGE_PATH",
    "LD_LIBRARY_PATH",
)


def contamination(environment: Mapping[str, str]) -> dict[str, list[str]]:
    bad: dict[str, list[str]] = {}
    for variable in ROS_PATH_VARIABLES:
        entries = [
            item
            for item in environment.get(variable, "").split(os.pathsep)
            if item and ("/opt/ros/" in item or "python3.10" in item)
        ]
        if entries:
            bad[variable] = entries
    return bad


def locate_internal_humble(config: Config, env: Mapping[str, str]) -> Path:
    probe = (
        "import pathlib,site;"
        "roots=[pathlib.Path(p) for p in site.getsitepackages()];"
        "c=[];"
        "[c.extend(r.glob('isaacsim/exts/isaacsim.ros2.*')) for r in roots];"
        "print('\\n'.join(str(p) for p in c))"
    )
    result = run(
        ["uv", "run", "--frozen", "--no-sync", "python", "-c", probe],
        cwd=config.root,
        env=env,
        check=True,
    )
    for extension in result.stdout.splitlines():
        candidate = Path(extension) / "humble"
        if (candidate / "lib").is_dir() and (candidate / "rclpy" / "rclpy").is_dir():
            return candidate.resolve()
    raise RuntimeError("Isaac Sim internal Humble libraries were not found in the locked environment")


def clean_isaac_environment(
    config: Config,
    base: Mapping[str, str] | None = None,
    *,
    dds_profile: str = "local",
) -> dict[str, str]:
    source = dict(base or os.environ)
    dirty = contamination(source)
    if dirty:
        details = "; ".join(f"{key}={','.join(value)}" for key, value in dirty.items())
        raise RuntimeError(
            "refusing to launch Isaac from a Python 3.10/system ROS environment: " + details
        )
    env = source
    env.update(
        deployment_env(
            config.root,
            config.path("uv_cache"),
            config.path("uv_python"),
            config.path("isaac_cache"),
        )
    )
    internal_humble = locate_internal_humble(config, env)
    internal_lib = internal_humble / "lib"
    internal_python = internal_humble / "rclpy"
    existing_ld = env.get("LD_LIBRARY_PATH", "")
    existing_python = env.get("PYTHONPATH", "")
    env.update(
        {
            "ROS_DISTRO": config.ros["distro"],
            "ROS_DOMAIN_ID": str(env.get("ROS_DOMAIN_ID", config.ros["domain_id"])),
            "RMW_IMPLEMENTATION": str(
                env.get("RMW_IMPLEMENTATION", config.ros["rmw_implementation"])
            ),
            "LD_LIBRARY_PATH": os.pathsep.join(
                part for part in (existing_ld, str(internal_lib)) if part
            ),
            "PYTHONPATH": os.pathsep.join(
                part for part in (existing_python, str(internal_python)) if part
            ),
        }
    )
    selected_profile = config.dds_profile(dds_profile)
    if selected_profile is None:
        env.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
    else:
        env["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(selected_profile)
    env["TINKER_SIM_DDS_PROFILE"] = dds_profile
    return env
