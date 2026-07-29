from __future__ import annotations

import json
from pathlib import Path

from .config import Config, sha256_file
from .process import run


def ensure_ros_vendor(config: Config, *, offline: bool) -> Path:
    manifest_path = config.root / config.raw["dependencies"]["ros_debian_packages"]["manifest"]
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    cache = config.path("ros_deb_cache")
    vendor = config.path("ros_vendor")
    cache.mkdir(parents=True, exist_ok=True)
    vendor.mkdir(parents=True, exist_ok=True)
    for package in manifest["packages"]:
        archive = cache / package["filename"]
        if not archive.is_file() and not offline:
            run(
                ["apt", "download", f"{package['name']}={package['version']}"],
                cwd=cache,
                check=True,
                capture=False,
            )
        if not archive.is_file():
            raise RuntimeError(f"offline ROS package is missing: {archive}")
        actual = sha256_file(archive)
        if actual != package["sha256"]:
            raise RuntimeError(
                f"ROS package hash mismatch for {package['name']}: {actual}"
            )
        run(["dpkg-deb", "-x", str(archive), str(vendor)], check=True)
    marker = vendor / "tinker-sim-vendor.json"
    marker.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "manifest_sha256": sha256_file(manifest_path),
                "packages": manifest["packages"],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    setup = vendor / "local_setup.bash"
    setup.write_text(
        """# generated from the checksum-verified isolated ROS Debian cache
_tinker_vendor_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_tinker_vendor_prefix="${_tinker_vendor_root}/opt/ros/humble"
export AMENT_PREFIX_PATH="${_tinker_vendor_prefix}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
export CMAKE_PREFIX_PATH="${_tinker_vendor_prefix}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${_tinker_vendor_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONPATH="${_tinker_vendor_prefix}/local/lib/python3.10/dist-packages:${_tinker_vendor_prefix}/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
unset _tinker_vendor_prefix _tinker_vendor_root
""",
        encoding="utf-8",
    )
    return vendor / "opt/ros/humble"
