"""Bounded pure model-bundle preflight and manifest/provider-entry validator.

The preflight validates manifest schema, absolute paths, exact hashes, the
selected-subgraph contract, installed/source artifact identity, prefix, mount,
groups, end-effector parent, resolved touch links, limits, collision geometry,
and finite JSON output.  It returns a typed preflight result for every
mismatch, artifact/path state, timeout, or safety classification and atomically
writes a report only for the fully ready result.

This module is ROS-free at import time and runs under both simulator CPython
3.12 and system Humble CPython 3.10.
"""
from __future__ import annotations

import argparse
import json
import os
import tempfile
import time
from pathlib import Path
from typing import Mapping, Sequence

import yaml

from .current_artifact import resolve_current_artifact as _resolve_current_artifact
from .model_contract import (
    ARTIFACT_NAMES,
    GROUPS,
    ModelContractError,
    canonical_contract,
    canonical_json,
    contract_fingerprint,
    sha256_file,
    validate_bundle_manifest,
)

PREFLIGHT_SCHEMA = 1
_READY = "ready"
_MISMATCH = "mismatch"
_INVALID = "invalid"
_TIMEOUT = "timeout"
_ERROR = "error"
MAX_ARTIFACT_BYTES = 64 * 1024 * 1024


class _PreflightTimeout(Exception):
    pass


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def write_report(report_path: Path | str, result: Mapping[str, object]) -> Path:
    """Atomically publish a fully-ready preflight report."""
    report_path = Path(report_path)
    if not report_path.is_absolute():
        raise ModelContractError("report_path", "report path must be an absolute path", field=str(report_path))
    data = canonical_json(result)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".{}.".format(report_path.name), dir=str(report_path.parent))
    temporary_path = Path(temporary)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, report_path)
        _fsync_directory(report_path.parent)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise
    return report_path


def _result(
    status: str,
    checks: list[dict[str, object]],
    manifest: Mapping[str, object] | None,
    elapsed_ms: float,
    *,
    fingerprint: str | None = None,
) -> dict[str, object]:
    return {
        "schema_version": PREFLIGHT_SCHEMA,
        "status": status,
        "ready": status == _READY,
        "checks": checks,
        "model_bundle_manifest": str(manifest.get("_manifest_path", "")) if isinstance(manifest, dict) else "",
        "structural_fingerprint": fingerprint,
        "elapsed_ms": elapsed_ms,
    }


def _load_manifest(manifest_path: Path) -> dict[str, object]:
    raw = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ModelContractError("invalid_manifest", "manifest must contain a JSON object", field=str(manifest_path))
    raw["_manifest_path"] = str(manifest_path)
    return raw


def _recompute_contract(manifest: Mapping[str, object]) -> dict[str, object]:
    artifacts = manifest["artifacts"]
    if not isinstance(artifacts, dict):
        raise ModelContractError("invalid_manifest", "artifacts must be an object", field="artifacts")
    sim_xml = Path(artifacts["simulator_full_urdf"]["path"]).read_text(encoding="utf-8")
    plan_xml = Path(artifacts["planning_urdf"]["path"]).read_text(encoding="utf-8")
    srdf_xml = Path(artifacts["planning_srdf"]["path"]).read_text(encoding="utf-8")
    try:
        limits_root = yaml.safe_load(Path(artifacts["joint_limits"]["path"]).read_bytes())
    except (OSError, yaml.YAMLError) as exc:
        raise ModelContractError("invalid_limits", "unable to parse joint_limits: {}".format(exc), field="joint_limits") from exc
    if not isinstance(limits_root, dict) or set(limits_root) != {"joint_limits"} or not isinstance(limits_root["joint_limits"], dict):
        raise ModelContractError("invalid_limits", "joint_limits YAML must contain exactly a joint_limits mapping", field="joint_limits")
    try:
        kinematics_root = yaml.safe_load(Path(artifacts["kinematics"]["path"]).read_bytes())
    except (OSError, yaml.YAMLError) as exc:
        raise ModelContractError("invalid_kinematics", "unable to parse kinematics: {}".format(exc), field="kinematics") from exc
    if not isinstance(kinematics_root, dict):
        raise ModelContractError("invalid_kinematics", "kinematics YAML must contain a mapping", field="kinematics")
    normalization = manifest["normalization"]
    if not isinstance(normalization, dict):
        raise ModelContractError("invalid_manifest", "normalization must be an object", field="normalization")
    computed = canonical_contract(
        sim_xml,
        plan_xml,
        srdf_xml,
        limits_root["joint_limits"],
        kinematics_root,
        prefix=normalization["prefix"],
        mount=normalization["mount"],
    )
    # Mirror the landed production consumer: group names/content and exact
    # selected-link ordering must agree with the recomputed resolved graph.
    groups = normalization.get("groups")
    if isinstance(groups, dict):
        names = {str(item) for item in groups.values()}
    elif isinstance(groups, list):
        names = {str(item) for item in groups}
    else:
        raise ModelContractError("invalid_manifest", "normalization.groups must be a mapping or list", field="normalization.groups")
    if names != set(GROUPS.values()):
        raise ModelContractError("invalid_manifest", "normalization.groups must name xarm7 and xarm_gripper", field="normalization.groups")
    if tuple(normalization["selected_links"]) != tuple(computed["selected_links"]):
        raise ModelContractError(
            "semantic_mismatch", "normalization.selected_links does not match resolved graph", field="normalization.selected_links"
        )
    return computed


def _effective_project_root(explicit, manifest) -> Path | None:
    if explicit is not None:
        return Path(explicit)
    sim_path = Path(manifest["artifacts"]["simulator_full_urdf"]["path"])
    for ancestor in (sim_path, *sim_path.parents):
        if (ancestor / "artifacts" / "robot" / "tinker2" / "current.json").is_file():
            return ancestor
    env = os.environ.get("TINKER_SIM_ROOT")
    if env:
        return Path(env)
    cwd = Path.cwd()
    if (cwd / "artifacts" / "robot" / "tinker2" / "current.json").is_file():
        return cwd
    return None


def _check_artifact_identity(
    checks: list[dict[str, object]],
    manifest: Mapping[str, object],
    project_root: Path | None,
) -> None:
    """Append a typed installed/source artifact-identity check (never omitted).

    A fully-ready preflight must prove equality with the authoritative
    ``current.json`` selection.  Outside-tree simulator artifacts are therefore
    compared against a resolvable authoritative selection and fail if different;
    if no authoritative root can be resolved the check fails closed.  There is
    never an ``ok=true`` "not applicable" result.
    """
    sim_path = Path(manifest["artifacts"]["simulator_full_urdf"]["path"]).resolve()
    root = _effective_project_root(project_root, manifest)
    if root is None:
        checks.append(
            {
                "name": "artifact_identity",
                "ok": False,
                "detail": "cannot resolve an authoritative current-artifact selection; set TINKER_SIM_ROOT or run from the simulator checkout",
            }
        )
        return
    try:
        resolved = _resolve_current_artifact(root)
        expected = (resolved.artifact_dir / "robot.urdf").resolve()
    except (ModelContractError, RuntimeError) as exc:
        checks.append(
            {
                "name": "artifact_identity",
                "ok": False,
                "detail": "cannot resolve current artifact selection: {}".format(exc),
            }
        )
        return
    try:
        sim_sha = sha256_file(sim_path)
        expected_sha = sha256_file(expected)
    except OSError as exc:
        checks.append(
            {
                "name": "artifact_identity",
                "ok": False,
                "detail": "unable to read artifacts for identity comparison: {}".format(exc),
            }
        )
        return
    ok = sim_sha == expected_sha
    detail = "current.json selects {} (sha256 {}); manifest references {} (sha256 {})".format(
        expected, expected_sha, sim_path, sim_sha
    )
    if not ok:
        detail += " (identity mismatch)"
    checks.append({"name": "artifact_identity", "ok": ok, "detail": detail})


def preflight_manifest(
    manifest_path: Path | str,
    *,
    timeout: float | None,
    project_root: Path | str | None = None,
) -> dict[str, object]:
    """Run a bounded preflight over one model-bundle manifest.

    *timeout* is a wall-clock budget in seconds (``None`` disables the bound).
    The installed/source artifact identity check always runs: *project_root*
    may name a simulator checkout explicitly, otherwise it is derived from the
    manifest's simulator artifact path or the environment, and the check fails
    closed (never omitted) when no authoritative selection can be resolved.
    """
    manifest_path = Path(manifest_path)
    if not manifest_path.is_absolute():
        raise ModelContractError("manifest_path", "model bundle manifest must be an absolute path", field=str(manifest_path))
    if timeout is not None and timeout <= 0.0:
        raise ModelContractError("timeout", "preflight timeout must be positive", field="timeout")
    started = time.monotonic()
    deadline = None if timeout is None else started + timeout
    checks: list[dict[str, object]] = []

    def elapsed_ms() -> float:
        return round((time.monotonic() - started) * 1000.0, 1)

    def _bounded() -> None:
        if deadline is not None and time.monotonic() > deadline:
            raise _PreflightTimeout()

    try:
        try:
            if manifest_path.stat().st_size > MAX_ARTIFACT_BYTES:
                checks.append(
                    {
                        "name": "manifest_schema",
                        "ok": False,
                        "detail": "manifest exceeds the {} MiB artifact bound".format(MAX_ARTIFACT_BYTES // (1024 * 1024)),
                    }
                )
                return _result(_INVALID, checks, {"_manifest_path": str(manifest_path)}, elapsed_ms())
        except OSError as exc:
            checks.append({"name": "manifest_schema", "ok": False, "detail": "unable to stat manifest: {}".format(exc)})
            return _result(_INVALID, checks, {"_manifest_path": str(manifest_path)}, elapsed_ms())
        try:
            manifest = _load_manifest(manifest_path)
        except (OSError, json.JSONDecodeError) as exc:
            checks.append({"name": "manifest_schema", "ok": False, "detail": "unable to read manifest: {}".format(exc)})
            return _result(_INVALID, checks, {"_manifest_path": str(manifest_path)}, elapsed_ms())
        checks.append(
            {
                "name": "manifest_schema",
                "ok": True,
                "detail": "schema_version={}".format(manifest.get("schema_version")),
            }
        )
        try:
            validate_bundle_manifest(manifest)
        except ModelContractError as exc:
            checks.append({"name": "manifest_structure", "ok": False, "detail": "{}: {}".format(exc.code, exc)})
            return _result(_INVALID, checks, manifest, elapsed_ms())
        checks.append({"name": "manifest_structure", "ok": True, "detail": "structural validation passed"})

        for name in ARTIFACT_NAMES:
            _bounded()
            entry = manifest["artifacts"][name]
            path = Path(entry["path"])
            if not path.is_absolute() or not path.is_file() or path.is_symlink():
                checks.append(
                    {
                        "name": "artifact_path_{}".format(name),
                        "ok": False,
                        "detail": "not an existing absolute regular file: {}".format(path),
                    }
                )
                continue
            try:
                size = path.stat().st_size
            except OSError as exc:
                checks.append(
                    {
                        "name": "artifact_path_{}".format(name),
                        "ok": False,
                        "detail": "unable to stat {}: {}".format(path, exc),
                    }
                )
                continue
            if size > MAX_ARTIFACT_BYTES:
                checks.append(
                    {
                        "name": "artifact_path_{}".format(name),
                        "ok": False,
                        "detail": "{} exceeds the {} MiB artifact bound".format(path, MAX_ARTIFACT_BYTES // (1024 * 1024)),
                    }
                )
                continue
            checks.append({"name": "artifact_path_{}".format(name), "ok": True, "detail": str(path)})
            _bounded()
            actual = sha256_file(path)
            ok = actual == entry["sha256"]
            detail = "sha256 {}".format(actual) if ok else "declared {} does not match bytes {}".format(entry["sha256"], actual)
            checks.append({"name": "artifact_hash_{}".format(name), "ok": ok, "detail": detail})
        if any(not item["ok"] for item in checks):
            return _result(_MISMATCH, checks, manifest, elapsed_ms())

        _bounded()
        try:
            computed = _recompute_contract(manifest)
        except (ModelContractError, OSError, yaml.YAMLError) as exc:
            code = exc.code if isinstance(exc, ModelContractError) else "artifact_parse"
            checks.append({"name": "contract", "ok": False, "detail": "{}: {}".format(code, exc)})
            return _result(_MISMATCH, checks, manifest, elapsed_ms())
        _bounded()
        checks.append(
            {
                "name": "contract",
                "ok": computed == manifest["contract"],
                "detail": "recomputed selected-subgraph contract matches manifest"
                if computed == manifest["contract"]
                else "recomputed contract differs from declared manifest contract",
            }
        )
        fingerprint = contract_fingerprint(computed)
        checks.append(
            {
                "name": "fingerprint",
                "ok": fingerprint == manifest["structural_fingerprint"],
                "detail": "structural fingerprint {}".format(fingerprint),
            }
        )
        try:
            json.dumps(manifest, allow_nan=False)
            checks.append({"name": "finite_json", "ok": True, "detail": "report serializes without non-finite values"})
        except (ValueError, TypeError) as exc:
            checks.append({"name": "finite_json", "ok": False, "detail": str(exc)})

        _check_artifact_identity(checks, manifest, project_root)

        if any(not item["ok"] for item in checks):
            return _result(_MISMATCH, checks, manifest, elapsed_ms())
        return _result(
            _READY,
            checks,
            manifest,
            elapsed_ms(),
            fingerprint=manifest["structural_fingerprint"],
        )
    except _PreflightTimeout:
        checks.append({"name": "timeout", "ok": False, "detail": "exceeded configured timeout of {} seconds".format(timeout)})
        return _result(_TIMEOUT, checks, {"_manifest_path": str(manifest_path)}, elapsed_ms())
    except Exception as exc:  # pragma: no cover - defensive safety classification
        checks.append({"name": "error", "ok": False, "detail": "{}: {}".format(type(exc).__name__, exc)})
        return _result(_ERROR, checks, {"_manifest_path": str(manifest_path)}, elapsed_ms())


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="model_preflight",
        description="Bounded model-bundle preflight; writes a report only when fully ready.",
    )
    parser.add_argument("--model-bundle-manifest", required=True, metavar="PATH", help="absolute model-bundle manifest path")
    parser.add_argument("--report", required=True, metavar="PATH", help="absolute report path (written only on ready)")
    parser.add_argument("--timeout", type=float, default=60.0, metavar="SECONDS", help="wall-clock preflight budget")
    args = parser.parse_args(argv)

    # The project root is derived inside preflight_manifest (explicit input,
    # manifest artifact path, then environment/cwd), and the artifact-identity
    # check fails closed when no authoritative selection can be resolved.
    result = preflight_manifest(
        Path(args.model_bundle_manifest),
        timeout=args.timeout,
        project_root=None,
    )
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    if result["status"] == _READY:
        write_report(Path(args.report), result)
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
