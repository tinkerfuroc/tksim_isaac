from __future__ import annotations

from pathlib import Path

from .config import Config
from .process import run


ISAAC_LAB_REPOSITORY = "https://github.com/isaac-sim/IsaacLab.git"
ISAACSIM_ROS_WORKSPACES_REPOSITORY = (
    "https://github.com/isaac-sim/IsaacSim-ros_workspaces.git"
)


def _head(path: Path) -> str | None:
    if not (path / ".git").exists():
        return None
    result = run(["git", "-C", str(path), "rev-parse", "HEAD"])
    return result.stdout.strip() if result.ok else None


def ensure_isaac_lab(config: Config, *, offline: bool) -> str:
    path = config.path("isaac_lab")
    commit = config.lab_commit
    current = _head(path)
    if current == commit:
        return current
    if offline:
        actual = current or "missing checkout"
        raise RuntimeError(
            f"offline Isaac Lab checkout mismatch: expected {commit}, found {actual}"
        )
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        run(
            ["git", "clone", "--filter=blob:none", "--no-checkout", ISAAC_LAB_REPOSITORY, str(path)],
            check=True,
            capture=False,
        )
    elif not (path / ".git").exists():
        raise RuntimeError(f"{path} exists but is not an Isaac Lab Git checkout")
    run(["git", "-C", str(path), "fetch", "--depth=1", "origin", commit], check=True, capture=False)
    run(["git", "-C", str(path), "checkout", "--detach", commit], check=True, capture=False)
    actual = _head(path)
    if actual != commit:
        raise RuntimeError(f"Isaac Lab checkout verification failed: {actual} != {commit}")
    return actual


def ensure_isaacsim_ros_workspaces(config: Config, *, offline: bool) -> str:
    dependency = config.raw["dependencies"]["isaacsim_ros_workspaces"]
    path = config.path("isaacsim_ros_workspaces")
    commit = dependency["commit"]
    current = _head(path)
    if current == commit:
        return current
    if offline:
        actual = current or "missing checkout"
        raise RuntimeError(
            "offline IsaacSim ROS workspace mismatch: "
            f"expected {commit}, found {actual}"
        )
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        run(
            [
                "git",
                "clone",
                "--filter=blob:none",
                "--no-checkout",
                ISAACSIM_ROS_WORKSPACES_REPOSITORY,
                str(path),
            ],
            check=True,
            capture=False,
        )
    elif not (path / ".git").exists():
        raise RuntimeError(f"{path} exists but is not a Git checkout")
    run(
        ["git", "-C", str(path), "fetch", "--depth=1", "origin", commit],
        check=True,
        capture=False,
    )
    run(
        ["git", "-C", str(path), "checkout", "--detach", commit],
        check=True,
        capture=False,
    )
    actual = _head(path)
    if actual != commit:
        raise RuntimeError(
            f"IsaacSim ROS workspace verification failed: {actual} != {commit}"
        )
    return actual
