from __future__ import annotations

import json
import os
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .config import Config, sha256_file
from .preflight import Check
from .process import run


def _json_probe(config: Config, env: dict[str, str]) -> dict[str, Any]:
    probe = """
import importlib.metadata as m
import json
import pathlib
import sys
names = ["isaacsim", "isaaclab", "torch", "torchvision", "torchaudio", "pillow"]
versions = {}
for name in names:
    try:
        versions[name] = m.version(name)
    except m.PackageNotFoundError:
        versions[name] = None
print(json.dumps({
    "python": sys.version,
    "python_executable": str(pathlib.Path(sys.executable).resolve()),
    "packages": versions,
    "sys_path": sys.path,
}))
""".strip()
    result = run(
        ["uv", "run", "--frozen", "--no-sync", "python", "-c", probe],
        cwd=config.root,
        env=env,
        check=True,
    )
    return json.loads(result.stdout)


def build_report(
    config: Config,
    env: dict[str, str],
    *,
    mode: str,
    preflight: list[Check],
    tests: dict[str, Any],
) -> dict[str, Any]:
    probe = _json_probe(config, env)
    python_path = Path(probe["python_executable"])
    lab = run(["git", "-C", str(config.path("isaac_lab")), "rev-parse", "HEAD"])
    gpu = run(
        [
            "nvidia-smi",
            "--query-gpu=name,memory.total,driver_version",
            "--format=csv,noheader",
        ]
    )
    lock_path = config.root / "uv.lock"
    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "mode": mode,
        "success": not any(check.status == "fail" for check in preflight)
        and all(test.get("status") == "pass" for test in tests.values()),
        "host": {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "gpu": gpu.stdout.strip() if gpu.ok else None,
        },
        "runtime": {
            **probe,
            "python_sha256": sha256_file(python_path),
            "uv": run(["uv", "--version"]).stdout.strip(),
            "lock_sha256": sha256_file(lock_path),
            "isaac_lab_commit": lab.stdout.strip() if lab.ok else None,
        },
        "ros": {
            "domain_id": env["ROS_DOMAIN_ID"],
            "rmw_implementation": env["RMW_IMPLEMENTATION"],
            "dds_profile": env.get("TINKER_SIM_DDS_PROFILE", "local"),
            "fastdds_profile": env.get("FASTRTPS_DEFAULT_PROFILES_FILE"),
        },
        "preflight": [check.__dict__ for check in preflight],
        "tests": tests,
    }


def write_report(config: Config, report: dict[str, Any]) -> Path:
    target_dir = config.path("reports")
    target_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    target = target_dir / f"deployment-{stamp}.json"
    target.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    latest = target_dir / "latest.json"
    latest.write_text(target.read_text(encoding="utf-8"), encoding="utf-8")
    return target
