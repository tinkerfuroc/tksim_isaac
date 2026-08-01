"""Shared current-artifact accessor for the model-bundle overlay.

The authoritative resolver lives in the simulator deployment tooling
(``tools/tinker_sim_deploy/runtime.py``); this module locates the checkout and
re-exports exactly that one resolver so runtime selection, bundle resolution,
and preflight identity all read ``current.json`` through the same
implementation.  Legacy (unversioned pointer + schema-2 manifest) and schema-4
publication shapes are dispatched by the shared resolver, never re-implemented
here.

This module stays import-time ROS-free: the deployment import happens lazily
inside the resolver call, so importing ``tinker_sim_bridge.model_bundle`` or
``model_preflight`` never touches ``tools/``.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

from .model_contract import ModelContractError


def _simulation_root() -> Path | None:
    env = os.environ.get("TINKER_SIM_ROOT")
    if env:
        root = Path(env)
        if (root / "tools" / "tinker_sim_deploy").is_dir():
            return root
    module = Path(__file__).resolve()
    for ancestor in (module, *module.parents):
        if (ancestor / "tools" / "tinker_sim_deploy").is_dir() and (ancestor / "artifacts").is_dir():
            return ancestor
    cwd = Path.cwd()
    if (cwd / "tools" / "tinker_sim_deploy").is_dir():
        return cwd
    return None


def _shared_resolver(project_root=None):
    """Locate the authoritative ``tinker_sim_deploy`` resolver.

    *project_root*, when supplied, is the preferred source of
    ``tools/tinker_sim_deploy``: a copied ROS install outside the simulator
    checkout can still resolve the real runtime tooling when the caller passes
    or derives the simulator project root from the manifest artifact path.  The
    module-tree/environment/cwd discovery remains the fallback.
    """
    root = None
    if project_root is not None:
        candidate = Path(project_root)
        if (candidate / "tools" / "tinker_sim_deploy").is_dir():
            root = candidate
    if root is None:
        root = _simulation_root()
    if root is None:
        raise ModelContractError(
            "artifact_current",
            "cannot locate the simulator checkout to load the shared current-artifact resolver; "
            "set TINKER_SIM_ROOT, pass the simulator project root, or run from the simulator repository",
        )
    tools = root / "tools"
    if str(tools) not in sys.path:
        sys.path.insert(0, str(tools))
    try:
        from tinker_sim_deploy.runtime import ArtifactResolution, resolve_current_artifact
    except ImportError as exc:
        raise ModelContractError(
            "artifact_current",
            "cannot import the shared current-artifact resolver from {}".format(tools),
        ) from exc
    return root, resolve_current_artifact, ArtifactResolution


def resolve_current_artifact(project_root):
    """Resolve the selected canonical Tinker 2 artifact through ``current.json``.

    The return value mirrors ``tinker_sim_deploy.runtime.ArtifactResolution``
    and carries ``artifact_dir``, ``manifest``, ``source_lock``, and
    ``robot_urdf``.  The authoritative resolver is located through
    *project_root* first (then module-tree/environment/cwd).  Failures surface
    as typed ``ModelContractError`` so the model overlay can classify them
    consistently.
    """
    _, resolver, _resolution_type = _shared_resolver(project_root)
    try:
        return resolver(Path(project_root))
    except ModelContractError:
        raise
    except Exception as exc:
        raise ModelContractError(
            "artifact_current", "cannot resolve current artifact: {}".format(exc), field=str(Path(project_root))
        ) from exc


__all__ = ["resolve_current_artifact"]
