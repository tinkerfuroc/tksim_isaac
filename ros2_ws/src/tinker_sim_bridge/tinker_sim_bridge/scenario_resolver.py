"""Deterministic qualification-scenario resolution with package-share fallback.

The integrated OMPL overlay launch resolves a qualification scenario first from
the source checkout (``<project_root>/simulation/scenarios/<id>.json``) and,
when that is unavailable, from the installed package share
(``<share>/scenarios/<id>.json``).  If both canonical sources exist but their
bytes disagree the resolution is refused so the acceptance contract cannot be
silently satisfied by a drifted copy.

This module is ROS-free at import time so the resolver is unit-testable under
the simulator venv without a sourced Humble environment.
"""
from __future__ import annotations

from pathlib import Path


class ScenarioResolutionError(RuntimeError):
    """Typed scenario-resolution failure (unsafe id, missing, or ambiguous)."""


def resolve_scenario_file(root: Path | str, scenario: str, share: Path | str) -> Path:
    """Resolve one qualification scenario with a deterministic package-share fallback.

    *root* is the simulator source checkout, *scenario* is the canonical scenario
    id, and *share* is the installed ``tinker_sim_bridge`` package-share path.
    Prefers the source-tree file; falls back to the installed share; refuses when
    both exist but differ in bytes.
    """
    root = Path(root)
    share = Path(share)
    if not scenario or "/" in scenario or "\\" in scenario or scenario in {".", ".."}:
        raise ScenarioResolutionError("unsafe scenario id: {!r}".format(scenario))
    source = root / "simulation" / "scenarios" / "{}.json".format(scenario)
    installed = share / "scenarios" / "{}.json".format(scenario)
    if source.is_file():
        if installed.is_file() and installed.read_bytes() != source.read_bytes():
            raise ScenarioResolutionError(
                "scenario {!r} differs between source checkout ({}) and installed package share ({}); "
                "refusing ambiguous resolution".format(scenario, source, installed)
            )
        return source
    if installed.is_file():
        return installed
    raise ScenarioResolutionError(
        "scenario {!r} not found in source checkout ({}) or installed package share ({})".format(
            scenario, source, installed
        )
    )


__all__ = ["ScenarioResolutionError", "resolve_scenario_file"]
