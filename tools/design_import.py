#!/usr/bin/env python3
# tools/design_import.py
"""Candidate robot design importer CLI.

designs/<name>/{robot.urdf|robot.urdf.xacro, design.yaml, meshes/} ->
artifacts/robot/<name>/<hash>/{robot.urdf, robot.usd, robot-profile.yaml,
manifest.json, source-lock.json} + current.json.

Every Isaac call sits behind ``ConverterHooks`` so ``run_import``/``main``
run under system Python with ``--stub-converter`` (see tests).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import traceback
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(REPO_ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "tools"))

from tinker_designs.clean import CleanError, canonical_bytes, expand_xacro, package_share_dirs, resolve_mesh_uris, strip_gazebo
from tinker_designs.contract import ContractError, require_contract
from tinker_designs.derive import DeriveError, derive_profile
from tinker_designs.heuristics import draft_design
from tinker_designs.lock import design_source_lock
from tinker_designs.model import parse_urdf
from tinker_designs.schema import Design, DesignError, load_design
from tinker_sim_deploy.workspace import ArtifactExportError, publish_robot_artifact

EXIT_RENDER, EXIT_CONTRACT, EXIT_IMPORT, EXIT_PUBLISH = 2, 3, 4, 5
DESIGN_CANONICALIZER = "tinker-designs-canonical-v1"


class ImportStageError(RuntimeError):
    """The Isaac import hook failed or produced no USD."""


class ConverterHooks(Protocol):
    def import_urdf(self, urdf_path: Path, usd_path: Path) -> None: ...


class StubHooks:
    """--stub-converter: writes a placeholder USD so the pipeline runs without Kit."""

    def import_urdf(self, urdf_path: Path, usd_path: Path) -> None:
        usd_path.write_bytes(b"#usda 1.0\n# stub conversion of " + urdf_path.name.encode() + b"\n")


@dataclass(frozen=True)
class ImportResult:
    artifact_dir: Path | None
    manifest: dict | None
    profile: dict
    canonical_urdf: bytes


def render(design_dir: Path, design: Design, *, packages: Mapping[str, Path], work_dir: Path,
           resolve: bool = True, xacro_runner=subprocess.run) -> tuple[ET.Element, Path | None, list[Path]]:
    """Clean the source into (canonical root with package:// intact, Isaac input path, resolved upstream files).

    With ``resolve=False`` (the --no-import loop) no mesh URI is touched, so the
    fast loop never needs a sourced ROS environment; the Isaac path is None.
    """
    source = design_dir / design.source
    data = expand_xacro(source, runner=xacro_runner) if source.suffix == ".xacro" else source.read_bytes()
    root = parse_urdf(data)
    strip_gazebo(root)
    canonical_root = ET.fromstring(canonical_bytes(root))
    if not resolve:
        return canonical_root, None, []
    isaac_root = ET.fromstring(canonical_bytes(root))
    resolved = resolve_mesh_uris(isaac_root, design_dir=design_dir, packages=packages)
    isaac_path = work_dir / "robot.isaac.urdf"
    isaac_path.write_bytes(ET.tostring(isaac_root, encoding="utf-8", xml_declaration=True))
    return canonical_root, isaac_path, resolved


def _label(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def run_import(design_dir: Path, repo_root: Path, hooks: ConverterHooks | None, *, no_import: bool = False,
               packages: Mapping[str, Path] | None = None, work_dir: Path | None = None,
               artifacts: Path | None = None) -> ImportResult:
    design_dir = Path(design_dir).resolve()
    repo_root = Path(repo_root).resolve()
    artifacts = repo_root / "artifacts" if artifacts is None else Path(artifacts)
    design = load_design(design_dir)
    packages = package_share_dirs() if packages is None else packages
    owned_work = work_dir is None
    work_dir = Path(tempfile.mkdtemp(prefix="design-import-")) if work_dir is None else Path(work_dir)
    try:
        root, isaac_path, resolved = render(design_dir, design, packages=packages, work_dir=work_dir, resolve=not no_import)
        require_contract(root, design)
        profile = derive_profile(root, design)
        canonical = canonical_bytes(root)
        if no_import:
            return ImportResult(None, None, profile, canonical)
        if hooks is None:
            raise ImportStageError("no converter hooks (pass --stub-converter or run under Isaac)")
        usd_path = work_dir / "robot.usd"
        try:
            hooks.import_urdf(isaac_path, usd_path)
        except Exception as error:
            raise ImportStageError(f"URDF import failed: {error}") from error
        if not usd_path.is_file() or usd_path.stat().st_size == 0:
            raise ImportStageError(f"importer wrote no USD at {usd_path}")
        file_bytes: dict[str, bytes] = {
            "robot.urdf": canonical,
            "robot.usd": usd_path.read_bytes(),
            "robot-profile.yaml": yaml.safe_dump(profile, sort_keys=True).encode("utf-8"),
        }
        meshes = design_dir / "meshes"
        if meshes.is_dir():
            for path in sorted(meshes.rglob("*")):
                if path.is_file():
                    file_bytes[f"meshes/{path.relative_to(meshes).as_posix()}"] = path.read_bytes()
        source_lock = design_source_lock(design_dir, repo_root, resolved)
        lock = json.loads(source_lock)
        source_bytes = (design_dir / design.source).read_bytes()
        driven = profile["wheels"]["driven"]
        extra = {
            "qualification": "design_candidate",
            "kinematics": {
                "front_left_joint": driven[0], "front_right_joint": driven[1],
                "wheel_radius_m": profile["wheels"]["radius_m"], "wheel_track_m": profile["wheels"]["track_m"],
                "footprint": profile["footprint"],
            },
            "profile": profile,
            "provenance": {
                "source_lock_sha256": hashlib.sha256(source_lock).hexdigest(),
                "source_identity": lock["source_identity"],
                "source_files": lock["files"],
                "design_dir": _label(design_dir, repo_root),
            },
        }
        result = publish_robot_artifact(
            artifacts,
            robot=design.name, file_bytes=file_bytes, canonical_urdf=canonical, source_lock_bytes=source_lock,
            canonicalizer=DESIGN_CANONICALIZER, manifest_extra=extra,
            source_path=_label(design_dir / design.source, repo_root), source_sha256=hashlib.sha256(source_bytes).hexdigest(),
        )
        return ImportResult(result.artifact_dir, result.manifest, profile, canonical)
    finally:
        if owned_work:
            shutil.rmtree(work_dir, ignore_errors=True)


def write_init(design_dir: Path, name: str) -> Path:
    design_dir = Path(design_dir)
    target = design_dir / "design.yaml"
    if target.exists():
        raise FileExistsError(f"{target} already exists; edit it or delete it first")
    source = next((design_dir / candidate for candidate in ("robot.urdf", "robot.urdf.xacro") if (design_dir / candidate).is_file()), None)
    if source is None:
        raise DesignError(f"{design_dir} has no robot.urdf or robot.urdf.xacro")
    data = expand_xacro(source) if source.suffix == ".xacro" else source.read_bytes()
    draft = draft_design(parse_urdf(data), name)
    target.write_text(yaml.safe_dump(draft, sort_keys=False), encoding="utf-8")
    return target


def _package_root_pair(item: str) -> tuple[str, Path]:
    package, _, path = item.partition("=")
    if not package or not path:
        raise argparse.ArgumentTypeError(f"--package-root expects PKG=PATH, got {item!r}")
    return package, Path(path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--design", type=Path, required=True, help="designs/<name> directory")
    parser.add_argument("--init", action="store_true", help="write a draft design.yaml from the URDF and exit")
    parser.add_argument("--no-import", action="store_true", help="stop after render + contract + derive")
    parser.add_argument("--stub-converter", action="store_true", help="skip Isaac; write a placeholder USD")
    parser.add_argument("--artifacts", type=Path, default=REPO_ROOT / "artifacts")
    parser.add_argument("--package-root", action="append", default=[], metavar="PKG=PATH", type=_package_root_pair)
    args = parser.parse_args(argv)

    design_dir = args.design.resolve()
    if args.init:
        try:
            print(write_init(design_dir, design_dir.name))
            return 0
        except (FileExistsError, DesignError, CleanError) as error:
            print(f"design_import: {error}")
            return EXIT_RENDER

    packages = dict(package_share_dirs())
    packages.update(dict(args.package_root))

    def _run(hooks) -> int:
        try:
            result = run_import(design_dir, REPO_ROOT, hooks, no_import=args.no_import, packages=packages, artifacts=args.artifacts)
        except (ContractError, DeriveError) as error:
            print(f"design_import: contract failed\n{error}")
            return EXIT_CONTRACT
        except (DesignError, CleanError, ValueError) as error:
            print(f"design_import: render failed\n{error}")
            return EXIT_RENDER
        except ImportStageError as error:
            print(f"design_import: import failed\n{error}")
            return EXIT_IMPORT
        except ArtifactExportError as error:
            print(f"design_import: publish failed\n{error}")
            return EXIT_PUBLISH
        if result.artifact_dir is None:
            print(f"design_import: {design_dir.name} passes render + contract (mass {result.profile['mass_kg']} kg, track {result.profile['wheels']['track_m']} m)")
        else:
            print(result.artifact_dir)
        return 0

    if args.no_import:
        return _run(None)
    if args.stub_converter:
        return _run(StubHooks())

    from isaacsim import SimulationApp  # noqa: E402  (GPU/Kit only from here)

    real_argv, sys.argv = sys.argv, sys.argv[:1]
    try:
        app = SimulationApp({"headless": True})
    finally:
        sys.argv = real_argv
    try:
        from design_convert import IsaacHooks

        code = _run(IsaacHooks())
    except BaseException:
        traceback.print_exc()
        app.close()
        return EXIT_IMPORT
    app.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
