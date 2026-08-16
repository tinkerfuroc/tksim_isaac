from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from . import bundle
from .assets import verify_assets
from .bootstrap import execute
from .config import Config
from .preflight import as_json, collect
from .process import deployment_env, run
from .provenance import verify
from .ros_boundary import clean_isaac_environment
from .workspace import capture_workspace_lock, export_tinker2, verify_workspace_lock


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="tinker-sim")
    parser.add_argument(
        "--root",
        type=Path,
        help="deployment project root (defaults to the root containing this tool)",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    preflight = commands.add_parser("preflight", help="check host requirements")
    preflight.add_argument("--json", action="store_true", help="emit machine-readable output")

    bootstrap = commands.add_parser("bootstrap", help="provision and validate the runtime")
    bootstrap.add_argument("--mode", choices=("online", "offline"), default="online")
    bootstrap.add_argument("--skip-preflight", action="store_true")
    bootstrap.add_argument("--skip-validation", action="store_true")
    bootstrap.add_argument("--skip-prewarm", action="store_true")

    create = commands.add_parser("bundle-create", help="create a deterministic offline bundle")
    create.add_argument("output", type=Path)

    restore = commands.add_parser("bundle-restore", help="restore and verify an offline bundle")
    restore.add_argument("bundle", type=Path)
    restore.add_argument("destination", type=Path)
    restore.add_argument("--profile", choices=("whole_robot", "physics_only"), default="whole_robot")

    launch = commands.add_parser("launch", help="launch through the isolated Isaac ROS boundary")
    launch.add_argument(
        "--sensor-profile",
        choices=(
            "physics-only",
            "sensor-rich",
            "navigation-parity",
            "manipulation-core",
            "streaming",
        ),
        default="physics-only",
    )
    launch.add_argument("--profile", choices=("parity", "oracle"), default="parity")
    launch.add_argument("--scenario", default="empty")
    launch.add_argument("--seed", type=int, default=0)
    launch.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    launch.add_argument("--ros", action=argparse.BooleanOptionalAction, default=False)
    launch.add_argument("--duration", type=float, default=0.0)
    launch.add_argument("--qualification", action="store_true")
    launch.add_argument("--dds-profile", choices=("local", "lan"), default="local")
    launch.add_argument(
        "--camera-pointcloud",
        action="store_true",
        help="publish /camera/depth_registered/points under sensor-rich",
    )
    launch.add_argument(
        "--arena-colors",
        action="store_true",
        help="color the occupancy walls with the deterministic palette",
    )
    launch.add_argument("isaac_args", nargs=argparse.REMAINDER)

    lock = commands.add_parser("workspace-lock", help="capture read-only Tinker source hashes")
    lock.add_argument("--workspace", type=Path, default=None)
    verify_lock = commands.add_parser("workspace-verify", help="verify the Tinker source lock")
    verify_lock.add_argument("--workspace", type=Path, default=None)
    artifact = commands.add_parser("artifact-export", help="export a content-addressed Tinker 2 artifact")
    artifact.add_argument("--workspace", type=Path, default=None)

    export = commands.add_parser(
        "conda-export", help="export recovery requirements from the authoritative uv lock"
    )
    export.add_argument("output", type=Path)
    return parser


def _print_checks(checks: list[object]) -> None:
    for check in checks:
        detail = f" — {check.detail}" if check.detail else ""
        print(
            f"{check.status.upper():4} {check.name:18} "
            f"actual={check.actual!s} required={check.required!s}{detail}"
        )


def _streaming_lock(root: Path) -> Path:
    lock = root / ".cache" / "streaming-viewer.lock"
    lock.parent.mkdir(parents=True, exist_ok=True)
    if lock.exists():
        try:
            pid = int(lock.read_text(encoding="utf-8").strip())
            os.kill(pid, 0)
        except (ValueError, ProcessLookupError):
            lock.unlink(missing_ok=True)
        else:
            raise RuntimeError(f"one streaming viewer is already active (simulator pid {pid})")
    descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        stream.write(f"{os.getpid()}\n")
    return lock


def _camera_stream_arguments(args) -> list[str]:
    """Optional sensor-rich camera flags forwarded to run_sim."""
    flags = []
    if args.camera_pointcloud:
        flags.append("--camera-pointcloud")
    if args.arena_colors:
        flags.append("--arena-colors")
    return flags


def _launch(config: Config, args: argparse.Namespace) -> int:
    env = clean_isaac_environment(config, dds_profile=args.dds_profile)
    if os.environ.get("TINKER_ACCEPT_OMNIVERSE_EULA") == "Y":
        # Kit prompts interactively without these; headless launches would
        # abort at EOF. Mirrors bootstrap.base_environment.
        env["ACCEPT_EULA"] = "Y"
        env["OMNI_KIT_ACCEPT_EULA"] = "YES"
    streaming = args.sensor_profile == "streaming"
    lock: Path | None = _streaming_lock(config.root) if streaming else None
    common = ["uv", "run", "--frozen", "--no-sync"]
    if streaming:
        command = common + [
            "isaacsim",
            "isaacsim.exp.full.streaming",
            "--no-window",
            "--/app/livestream/allowResize=false",
            "--/physics/useGpu=false",
            "--/physics/cudaDevice=-1",
        ]
    else:
        command = common + [
            "python",
            "validation/run_sim.py",
            "--sensor-profile",
            args.sensor_profile,
            "--profile",
            args.profile,
            "--scenario",
            args.scenario,
            "--seed",
            str(args.seed),
        ]
        if args.headless:
            command.append("--headless")
        if args.ros:
            command.append("--ros")
        if args.duration > 0.0:
            command.extend(["--duration", str(args.duration)])
        if args.qualification:
            command.append("--qualification")
        command.extend(_camera_stream_arguments(args))
    command.extend(args.isaac_args)
    try:
        process = subprocess.Popen(command, cwd=config.root, env=env)
        try:
            return process.wait()
        except KeyboardInterrupt:
            # SIGINT reaches the whole foreground process group, so the
            # simulator has already received it.  Let Kit close and preserve
            # the child's clean status instead of emitting a wrapper traceback.
            try:
                return process.wait(timeout=30.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    return process.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    return process.wait()
    finally:
        if lock is not None:
            lock.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    config = Config.load(args.root)
    try:
        if args.command == "preflight":
            checks = collect(config)
            print(as_json(checks) if args.json else "", end="" if args.json else "")
            if not args.json:
                _print_checks(checks)
            return 1 if any(check.status == "fail" for check in checks) else 0
        if args.command == "bootstrap":
            report = execute(
                config,
                mode=args.mode,
                skip_preflight=args.skip_preflight,
                skip_validation=args.skip_validation,
                skip_prewarm=args.skip_prewarm,
            )
            print(report)
            return 0
        if args.command == "bundle-create":
            if os.environ.get("TINKER_ACCEPT_OMNIVERSE_EULA") != "Y":
                raise RuntimeError("TINKER_ACCEPT_OMNIVERSE_EULA=Y is required")
            uv = shutil.which("uv")
            if uv is None:
                raise RuntimeError("uv executable was not found")
            verify(config, require_python=True)
            verify_assets(config)
            prewarm_marker = config.path("isaac_cache") / "prewarm.json"
            if not prewarm_marker.is_file():
                raise RuntimeError(
                    "Isaac extension/asset cache has not been prewarmed; run online bootstrap first"
                )
            with tempfile.TemporaryDirectory(
                prefix="offline-audit-", dir=config.path("uv_cache").parent
            ) as audit:
                env = deployment_env(
                    config.root,
                    config.path("uv_cache"),
                    config.path("uv_python"),
                    config.path("isaac_cache"),
                )
                env["UV_PROJECT_ENVIRONMENT"] = str(Path(audit) / ".venv")
                env["UV_LINK_MODE"] = "hardlink"
                env["ACCEPT_EULA"] = "Y"
                run(
                    ["uv", "sync", "--frozen", "--offline"],
                    cwd=config.root,
                    env=env,
                    check=True,
                    capture=False,
                )
            output = bundle.create(config, args.output.resolve(), Path(uv).resolve())
            print(output)
            return 0
        if args.command == "bundle-restore":
            restored = bundle.restore(args.bundle.resolve(), args.destination.resolve(), profile=args.profile)
            print(restored)
            return 0
        if args.command == "launch":
            return _launch(config, args)
        if args.command in {"workspace-lock", "workspace-verify", "artifact-export"}:
            workspace_value = args.workspace or (Path(os.environ["TINKER_WS"]) if os.environ.get("TINKER_WS") else None)
            if workspace_value is None:
                raise RuntimeError("external Tinker workspace is required; pass --workspace or set TINKER_WS")
            lock_path = config.path("artifacts") / "provenance" / "tinker2-source-lock.json"
            if args.command == "workspace-lock":
                print(json.dumps(capture_workspace_lock(workspace_value, lock_path), indent=2))
                return 0
            if args.command == "workspace-verify":
                mismatches = verify_workspace_lock(workspace_value, lock_path)
                if mismatches:
                    raise RuntimeError("workspace source-lock mismatch: " + ", ".join(mismatches[:12]))
                print(lock_path)
                return 0
            result = export_tinker2(workspace_value, config.path("artifacts"), lock_path)
            print(result.artifact_dir)
            return 0
        if args.command == "conda-export":
            result = run(
                [
                    "uv",
                    "export",
                    "--frozen",
                    "--no-dev",
                    "--format",
                    "requirements-txt",
                    "--no-emit-project",
                ],
                cwd=config.root,
                check=True,
            )
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(result.stdout, encoding="utf-8")
            print(args.output)
            return 0
    except RuntimeError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    return 2
