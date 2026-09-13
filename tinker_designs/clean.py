# tinker_designs/clean.py
"""Turn a design's URDF/xacro into what Isaac's importer accepts.

Ported from tk26_sim's render_for_isaac.sh: xacro expansion, <gazebo> strip,
package:// -> file:// resolution. Uses the stdlib only (lxml is not in the venv).
"""
from __future__ import annotations

import os
import subprocess
import xml.etree.ElementTree as ET
from collections.abc import Callable, Mapping
from pathlib import Path

from tinker_sim_deploy.workspace import finalize_canonical


class CleanError(RuntimeError):
    """A render/clean stage failed; the message lists every problem found."""


def expand_xacro(xacro_path: Path, *, runner: Callable[..., subprocess.CompletedProcess] = subprocess.run) -> bytes:
    command = ["xacro", str(xacro_path)]
    completed = runner(command, capture_output=True, check=False)
    if completed.returncode != 0:
        raise CleanError(f"xacro failed ({completed.returncode}):\n{completed.stderr.decode('utf-8', 'replace')}")
    return completed.stdout


def strip_gazebo(root: ET.Element) -> int:
    removed = 0
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "gazebo":
                parent.remove(child)
                removed += 1
    return removed


def package_share_dirs(env: Mapping[str, str] = os.environ) -> dict[str, Path]:
    found: dict[str, Path] = {}
    for prefix in (item for item in env.get("AMENT_PREFIX_PATH", "").split(":") if item):
        share = Path(prefix) / "share"
        if not share.is_dir():
            continue
        for candidate in share.iterdir():
            if candidate.is_dir() and (candidate / "package.xml").is_file():
                found.setdefault(candidate.name, candidate)
    return found


def resolve_mesh_uris(root: ET.Element, *, design_dir: Path, packages: Mapping[str, Path]) -> list[Path]:
    resolved: list[Path] = []
    problems: list[str] = []
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename", "")
        if uri.startswith("file://"):
            target = Path(uri[len("file://"):])
        elif uri.startswith("package://"):
            package, _, relative = uri[len("package://"):].partition("/")
            share = packages.get(package)
            if share is None:
                problems.append(f"unknown package {package!r} in {uri!r}")
                continue
            target = Path(share) / relative
        else:
            target = Path(design_dir) / uri
        if not target.is_file():
            problems.append(f"missing mesh file {target} (from {uri!r})")
            continue
        mesh.set("filename", f"file://{target}")
        resolved.append(target)
    if problems:
        raise CleanError("unresolved mesh URIs:\n  " + "\n  ".join(problems))
    return resolved


def canonical_bytes(root: ET.Element) -> bytes:
    xml = ET.tostring(root, encoding="unicode")
    canonical = ET.canonicalize(xml_data=xml, with_comments=False, strip_text=False)
    return (canonical.rstrip("\n") + "\n").encode("utf-8")
