from __future__ import annotations

import ctypes
import json
import os
import platform
import re
import shutil
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable

from .config import Config
from .process import CommandResult, run


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    actual: str
    required: str
    detail: str = ""


def _version_tuple(value: str) -> tuple[int, ...]:
    return tuple(int(part) for part in re.findall(r"\d+", value))


def _os_release(path: Path = Path("/etc/os-release")) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value.strip().strip('"')
    return values


def _ram_gb() -> float:
    pages = os.sysconf("SC_PHYS_PAGES")
    page_size = os.sysconf("SC_PAGE_SIZE")
    return pages * page_size / 1_000_000_000


def _glibc() -> str:
    libc = ctypes.CDLL("libc.so.6")
    libc.gnu_get_libc_version.restype = ctypes.c_char_p
    return libc.gnu_get_libc_version().decode()


def _gpu_query(command_runner: Callable[..., CommandResult]) -> CommandResult:
    return command_runner(
        [
            "nvidia-smi",
            "--query-gpu=name,memory.total,driver_version",
            "--format=csv,noheader,nounits",
        ]
    )


def collect(config: Config, command_runner: Callable[..., CommandResult] = run) -> list[Check]:
    expected = config.platform
    checks: list[Check] = []
    os_info = _os_release()
    os_actual = f"{os_info.get('ID', '?')} {os_info.get('VERSION_ID', '?')}"
    os_ok = (
        os_info.get("ID") == expected["os_id"]
        and os_info.get("VERSION_ID") == expected["os_version"]
    )
    checks.append(
        Check(
            "operating_system",
            "pass" if os_ok else "fail",
            os_actual,
            f"{expected['os_id']} {expected['os_version']}",
        )
    )

    architecture = platform.machine()
    checks.append(
        Check(
            "architecture",
            "pass" if architecture == expected["architecture"] else "fail",
            architecture,
            expected["architecture"],
        )
    )

    glibc = _glibc()
    checks.append(
        Check(
            "glibc",
            "pass" if _version_tuple(glibc) >= _version_tuple(expected["minimum_glibc"]) else "fail",
            glibc,
            f">={expected['minimum_glibc']}",
        )
    )

    disk_gb = shutil.disk_usage(config.root).free / 1_000_000_000
    checks.append(
        Check(
            "free_disk",
            "pass" if disk_gb >= expected["minimum_disk_gb"] else "fail",
            f"{disk_gb:.1f} GB",
            f">={expected['minimum_disk_gb']} GB",
        )
    )

    ram_gb = _ram_gb()
    ram_status = "fail" if ram_gb < expected["minimum_ram_gb"] else (
        "warn" if ram_gb < expected["recommended_ram_gb"] else "pass"
    )
    checks.append(
        Check(
            "ram",
            ram_status,
            f"{ram_gb:.1f} GB",
            f">={expected['minimum_ram_gb']} GB; {expected['recommended_ram_gb']} GB supported",
        )
    )

    if shutil.which("nvidia-smi") is None:
        checks.extend(
            [
                Check("gpu", "fail", "nvidia-smi not found", "RTX GPU"),
                Check("driver", "fail", "unknown", f">={expected['supported_driver']}"),
                Check("vram", "fail", "unknown", f">={expected['minimum_vram_gb']} GB"),
            ]
        )
    else:
        result = _gpu_query(command_runner)
        if not result.ok or not result.stdout.strip():
            detail = result.stderr.strip() or "no GPU returned"
            checks.extend(
                [
                    Check("gpu", "fail", "unavailable", "RTX GPU", detail),
                    Check("driver", "fail", "unknown", f">={expected['supported_driver']}"),
                    Check("vram", "fail", "unknown", f">={expected['minimum_vram_gb']} GB"),
                ]
            )
        else:
            first_gpu = result.stdout.strip().splitlines()[0]
            name, memory, driver = [part.strip() for part in first_gpu.split(",", 2)]
            checks.append(
                Check("gpu", "pass" if "RTX" in name.upper() else "fail", name, "RTX GPU")
            )
            memory_mib = int(float(memory))
            memory_gb = memory_mib * 1024**2 / 1_000_000_000
            checks.append(
                Check(
                    "vram",
                    "pass" if memory_gb >= expected["minimum_vram_gb"] else "fail",
                    f"{memory_gb:.1f} GB ({memory_mib} MiB)",
                    f">={expected['minimum_vram_gb']} GB",
                )
            )
            checks.append(
                Check(
                    "driver",
                    "pass" if _version_tuple(driver) >= _version_tuple(expected["supported_driver"]) else "warn",
                    driver,
                    f">={expected['supported_driver']}",
                    "Older drivers are experimental and are not release-qualified."
                    if _version_tuple(driver) < _version_tuple(expected["supported_driver"])
                    else "",
                )
            )

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        checks.append(Check("nvenc", "fail", "ffmpeg not found", "h264_nvenc encoder"))
    else:
        encoders = command_runner([ffmpeg, "-hide_banner", "-encoders"])
        has_nvenc = encoders.ok and "h264_nvenc" in encoders.stdout
        checks.append(
            Check(
                "nvenc",
                "pass" if has_nvenc else "fail",
                "h264_nvenc available" if has_nvenc else "h264_nvenc unavailable",
                "h264_nvenc encoder",
                "Preflight verifies encoder registration; validation performs a hardware encode.",
            )
        )
    return checks


def as_json(checks: list[Check]) -> str:
    return json.dumps(
        {
            "ok": not any(check.status == "fail" for check in checks),
            "checks": [asdict(check) for check in checks],
        },
        indent=2,
        sort_keys=True,
    )
