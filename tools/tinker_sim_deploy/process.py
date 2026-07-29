from __future__ import annotations

import os
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence


@dataclass(frozen=True)
class CommandResult:
    command: tuple[str, ...]
    returncode: int
    stdout: str
    stderr: str

    @property
    def ok(self) -> bool:
        return self.returncode == 0


def run(
    command: Sequence[str],
    *,
    cwd: Path | None = None,
    env: Mapping[str, str] | None = None,
    check: bool = False,
    capture: bool = True,
) -> CommandResult:
    completed = subprocess.run(
        list(command),
        cwd=cwd,
        env=dict(env) if env is not None else None,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.PIPE if capture else None,
        check=False,
    )
    result = CommandResult(
        command=tuple(command),
        returncode=completed.returncode,
        stdout=completed.stdout or "",
        stderr=completed.stderr or "",
    )
    if check and not result.ok:
        rendered = " ".join(command)
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(f"command failed ({result.returncode}): {rendered}\n{detail}")
    return result


def deployment_env(
    root: Path, uv_cache: Path, uv_python: Path, isaac_cache: Path
) -> dict[str, str]:
    env = dict(os.environ)
    env.update(
        {
            "UV_CACHE_DIR": str(uv_cache),
            "UV_PYTHON_INSTALL_DIR": str(uv_python),
            "XDG_CACHE_HOME": str(isaac_cache),
            "OV_CACHE_ROOT": str(isaac_cache / "ov"),
            "ISAACSIM_CACHE_PATH": str(isaac_cache),
        }
    )
    return env
