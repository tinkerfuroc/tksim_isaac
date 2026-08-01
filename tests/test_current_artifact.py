"""Shared current-artifact resolver tests (legacy + schema-4 dispatch).

One authoritative resolver (``tinker_sim_deploy.runtime.resolve_current_artifact``)
must resolve the same on-disk ``current.json`` identically through runtime
selection, model-bundle resolution, and preflight identity.  These tests cover
the legacy (unversioned pointer + schema-2 manifest) and schema-4 publication
shapes plus malformed/cross-robot/inconsistent-pointer mutations.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests"))

import pytest

from model_fixtures import write_legacy_current

from test_artifact_export import _make_workspace
from tinker_sim_deploy.runtime import resolve_current_artifact as runtime_resolve
from tinker_sim_deploy.workspace import capture_workspace_lock, export_tinker2
from tinker_sim_bridge.current_artifact import resolve_current_artifact as overlay_resolve
from tinker_sim_bridge.model_bundle import resolve_simulator_full_urdf
from tinker_sim_bridge.model_contract import ModelContractError

MOUNT = {"parent": "world", "child": "base_link", "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
_URDF_BYTES = b"<robot/>"


def _assert_all_resolve_identically(root: Path, expected_urdf: Path) -> None:
    runtime = runtime_resolve(root)
    assert (runtime.artifact_dir / "robot.urdf") == expected_urdf
    assert overlay_resolve(root).artifact_dir == runtime.artifact_dir
    assert resolve_simulator_full_urdf(root) == expected_urdf


def _publish_schema4(root: Path) -> Path:
    """Publish a real schema-4 artifact tree and return the selected robot.urdf."""
    workspace = root / "workspace"
    artifacts = root / "artifacts"
    _make_workspace(workspace)
    lock = artifacts / "provenance/tinker2-source-lock.json"
    capture_workspace_lock(workspace, lock)
    result = export_tinker2(workspace, artifacts, lock)
    return result.artifact_dir / "robot.urdf"


def test_legacy_dispatch_resolves_identically_through_all_boundaries(tmp_path: Path) -> None:
    artifact_id = "36ac0317025d20a5"
    urdf = write_legacy_current(tmp_path, artifact_id, _URDF_BYTES)
    _assert_all_resolve_identically(tmp_path, urdf)


def test_schema4_dispatch_resolves_identically_through_all_boundaries(tmp_path: Path) -> None:
    urdf = _publish_schema4(tmp_path)
    _assert_all_resolve_identically(tmp_path, urdf)


def test_real_on_disk_legacy_selector_resolves_identically() -> None:
    current = ROOT / "artifacts" / "robot" / "tinker2" / "current.json"
    if not current.is_file():
        pytest.skip("simulator artifact tree is not present in this checkout")
    data = json.loads(current.read_text(encoding="utf-8"))
    artifact_id = data["artifact_id"]
    expected = ROOT / "artifacts" / "robot" / "tinker2" / artifact_id / "robot.urdf"
    if not expected.is_file():
        pytest.skip("selected artifact generation is not present in this checkout")
    _assert_all_resolve_identically(ROOT, expected)


def test_legacy_missing_artifact_id_rejected(tmp_path: Path) -> None:
    write_legacy_current(tmp_path, "36ac0317025d20a5", _URDF_BYTES)
    (tmp_path / "artifacts" / "robot" / "tinker2" / "current.json").write_text(json.dumps({"manifest": "x"}), encoding="utf-8")
    with pytest.raises(Exception) as error:
        runtime_resolve(tmp_path)
    assert "artifact_id" in str(error.value)
    with pytest.raises(ModelContractError) as typed:
        overlay_resolve(tmp_path)
    assert typed.value.code == "artifact_current"


def test_cross_robot_binding_rejected(tmp_path: Path) -> None:
    artifact_id = "36ac0317025d20a5"
    write_legacy_current(tmp_path, artifact_id, _URDF_BYTES)
    manifest_path = tmp_path / "artifacts" / "robot" / "tinker2" / artifact_id / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["robot"] = "tinker1"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(Exception) as error:
        runtime_resolve(tmp_path)
    assert "does not bind" in str(error.value) or "robot" in str(error.value)


def test_inconsistent_pointer_rejected(tmp_path: Path) -> None:
    artifact_id = "36ac0317025d20a5"
    write_legacy_current(tmp_path, artifact_id, _URDF_BYTES)
    # current.json manifest points at a different artifact generation.
    other = "aaaa1111aaaa1111"
    (tmp_path / "artifacts" / "robot" / "tinker2" / "current.json").write_text(
        json.dumps({"artifact_id": artifact_id, "manifest": "artifacts/robot/tinker2/{}/manifest.json".format(other)}),
        encoding="utf-8",
    )
    with pytest.raises(Exception) as error:
        runtime_resolve(tmp_path)
    assert "manifest path" in str(error.value)


def test_manifest_artifact_id_binding_rejected(tmp_path: Path) -> None:
    artifact_id = "36ac0317025d20a5"
    write_legacy_current(tmp_path, artifact_id, _URDF_BYTES)
    manifest_path = tmp_path / "artifacts" / "robot" / "tinker2" / artifact_id / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["artifact_id"] = "bbbb2222bbbb2222"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(Exception) as error:
        runtime_resolve(tmp_path)
    assert "does not bind" in str(error.value)


def test_unsupported_schema_rejected(tmp_path: Path) -> None:
    write_legacy_current(tmp_path, "36ac0317025d20a5", _URDF_BYTES)
    (tmp_path / "artifacts" / "robot" / "tinker2" / "current.json").write_text(
        json.dumps({"schema_version": 99}), encoding="utf-8"
    )
    with pytest.raises(Exception) as error:
        runtime_resolve(tmp_path)
    assert "unsupported" in str(error.value)
    with pytest.raises(ModelContractError) as typed:
        overlay_resolve(tmp_path)
    assert typed.value.code == "artifact_current"


def test_legacy_urdf_hash_mismatch_rejected(tmp_path: Path) -> None:
    artifact_id = "36ac0317025d20a5"
    urdf = write_legacy_current(tmp_path, artifact_id, _URDF_BYTES)
    urdf.write_bytes(b"<robot><tampered/></robot>")
    with pytest.raises(Exception) as error:
        runtime_resolve(tmp_path)
    assert "hash does not match" in str(error.value) or "provenance" in str(error.value)


def test_schema4_urdf_missing_payload_rejected(tmp_path: Path) -> None:
    urdf = _publish_schema4(tmp_path)
    urdf.unlink()
    with pytest.raises(Exception):
        runtime_resolve(tmp_path)
