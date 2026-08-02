from __future__ import annotations

import ast
import copy
import hashlib
import importlib
import json
import os
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))
sys.path.insert(0, str(ROOT / "simulation"))

import yaml

from tinker_sim_deploy.config import Config
from tinker_sim_deploy.provenance import verify
from tinker_sim_bridge.fixture_contract import revision_digest
from tinker_sim_bridge.fixture_planning_scene import (
    canonical_fixture_status,
    fixture_descriptor_sha256,
    fixture_owned_ids,
)
from tinker_sim_bridge.integrated_readiness import (
    build_integrated_mapping,
    public_integrated_mapping,
    sha256_json,
)
from tinker_sim_bridge.model_bundle import (
    build_manifest,
    stable_manifest_evidence,
)
from tinker_sim_bridge.model_limits import synthesize_joint_limits
from tinker_sim_bridge.model_preflight import (
    preflight_manifest,
    stable_preflight_evidence,
)
from tinker_sim_bridge.scenario_resolver import (
    ScenarioResolutionError,
    resolve_scenario_file,
)

# ---------------------------------------------------------------------------
# fix-round-1 source-backed constants (recorded, immutable)
# ---------------------------------------------------------------------------
_SIMULATOR_FULL_URDF_SOURCE = (
    "integration/model-bundle-r2/simulator_full_urdf/source-tinker-full.urdf"
)
_SIMULATOR_FULL_URDF_OUTPUT = "3e2361635c296defa9821bc00e89840506eaa57c27d8b8187b4bdb4e78c05fe6"
_SIMULATOR_FULL_URDF_SOURCE_SHA = "1e4aae81d4fa3b90dbdcaeab4aa86e4cfc647243b297202d56c7597589239504"
_V1_CANONICALIZER_COMMIT = "8a4e72465cb90e980632f205eb5c60684008d649"
_TK26_SIM_COMMIT = "18296c0140efcfd20935a05b258f10bf9153bf94"
_TK25_BASIC_COMMIT = "6576d3a52f7c1da12d234cdacd613ab308879783"
_TK25_MANIP_COMMIT = "39d96a176904c0b7966b11333c5517b3b54b6ae3"
_MANIFEST_STABLE_SHA = "82018ba5a32ba6cbb422ce305790456d818a472b9053d61a7c69e8484dd6beba"
_PREFLIGHT_STABLE_SHA = "cbca4cf67a9dedc636ad6460f5e557e8c290fa47046c004a0886357ac5624f7c"
_JOINT_LIMITS_SEMANTIC_SHA = "b62b2839407b66ad0ea852845bdd6c9ac2d2ed44e1b00ddf39c8d1c82523d87c"
_TASK_RANGE_START_SUBJECT = "feat: implement canonical model bundle producer and bounded preflight"
_TASK_RANGE_END_SUBJECT = "fix: tighten OMPL smoke adjudication"
# Exact Task 3-7 implementation commits in `git rev-list <start>..<end>` order
# (newest first).  Mutating task_range start/end to a different 13-commit pair
# must fail against this recorded ordered list.
_TASK_RANGE_COMMITS = [
    "f34de5f4cd472e2dbb50d65eb53e089bb1c84891",
    "0c2a475fb50df6952505e11952c03bc2bac25bd2",
    "e3d31fedd9f90761ef8c143f714f4509efe91594",
    "1903aee77ad739d2623a952f6e9fe360fb866102",
    "382a6721c544ac23825f829f50843fae71c5fcda",
    "bd67d511005a43080bfb84a4f89e86f5cbbcb16e",
    "67984ef48332c9406a8fd2c5fea021a92bd7faeb",
    "4e414a8793d164268167b3a94574a0cc8b09b6f2",
    "62db4d2872952f3afc8cf57e8e77bf7ed9ad04b0",
    "0e9f3496ebce6d3129e7cfa6d892a90813ee0f9f",
    "5472b5f88b8b2531eac52a585a000ea2f741261a",
    "1b31c5db03df069d474eaa4ea0ba756debfaa234",
    "c72d79dd7ba0442b7fd5708a6689627f6b097ec2",
]

# ---------------------------------------------------------------------------
# fix-round-3 locked clean-checkout class identity (immutable)
# ---------------------------------------------------------------------------
# The exact pytest node set of Task8OMPLOverlayProvenanceTest (64 tests) is
# load-bearing: the clean-checkout seam collects the class in a fresh tracked
# clone and requires exactly these 64 node IDs and this canonical SHA-256 over
# the JSON-canonical sorted node list.  A deleted test, an added test, or a
# rename that preserves the count cannot preserve the hash, so silently reduced
# coverage fails closed.
_TASK8_CLASS_TEST_COUNT = 64
_TASK8_CLASS_NODE_SET_SHA = (
    "0488b3048283d8fb4af3f72d2fdec2e1b83d1215de2bf55f0e714ec48fb99929"
)

# The only legitimate clean-checkout skips are the four host-runtime /
# environment diagnostics, keyed by exact test name with the expected reason
# category (a substring of the real skip message).  Any additional skip, or a
# legitimate diagnostic broadened into a skip, changes this set and fails.
_TASK8_CLEAN_CHECKOUT_SKIPS = {
    "test_clean_checkout_static_acceptance_seam": "nested clean-checkout seam invocation",
    "test_installed_package_data_byte_identity": "install prefix not present",
    "test_current_artifact_matches_reproducible_derivation": "host is not provisioned",
    "test_model_bundle_stable_projection_reproducible": "non-provisioned host",
}

# The fixture status field contract is asserted against this independently
# documented 12-field literal (canonical order/encoding), never re-derived from
# the Task 5 function whose key set it also validates separately.
_EXPECTED_FIXTURE_STATUS_FIELDS = [
    "schema_version=1",
    "state",
    "scenario",
    "owner=sim_fixture",
    "revision",
    "revision_digest",
    "monotonic sequence",
    "finite published_at",
    "declared-order owned_ids",
    "target_source_id",
    "target_handoff=pick_and_place/object_mesh",
    "fixture_descriptor_sha256",
]
_FIXTURE_STATUS_FIELD_KEYS = [
    "schema_version",
    "state",
    "scenario",
    "owner",
    "revision",
    "revision_digest",
    "sequence",
    "published_at",
    "owned_ids",
    "target_source_id",
    "target_handoff",
    "fixture_descriptor_sha256",
]

_EXPECTED_18_LAUNCH_ARGS = [
    "model_bundle_manifest",
    "provider_manifest_path",
    "provider_manifest_sha256",
    "use_sim_time",
    "show_rviz",
    "start_move_group",
    "start_task_server",
    "safety_required",
    "required_fixture_revision",
    "required_fixture_revision_digest",
    "required_fixture_owned_ids",
    "scenario_status_path",
    "required_scenario_id",
    "required_scenario_seed",
    "required_scenario_identities",
    "required_model_fingerprint",
    "required_fixture_descriptor_sha256",
    "execution_profile",
]
_EXPECTED_STRICT_SIM_INPUTS = [
    "scenario_status_path",
    "required_scenario_id",
    "required_scenario_seed",
    "required_scenario_identities",
    "required_model_fingerprint",
    "required_fixture_descriptor_sha256",
]
_EXPECTED_PRODUCTION_IMPORTS = {
    "os", "sys", "pathlib", "hashlib", "json", "ast", "importlib",
    "re", "typing", "ament_index_python", "launch", "launch_ros", "yaml", "pytest",
}
_EXPECTED_PRODUCTION_NODE_PACKAGES = {"moveit_ros_move_group", "rviz2", "pick_and_place"}
_EXPECTED_PRODUCTION_NODE_EXECUTABLES = {"move_group", "rviz2", "pick_and_place"}
_EXPECTED_PRODUCTION_CONTROLLER_RESOURCES = {
    "xarm7_traj_controller", "xarm_gripper", "follow_joint_trajectory", "gripper_action",
}
_EXPECTED_SIM_COMPAT_KEYS = (
    "use_cumotion_object_attachment",
    "use_cumotion_goalset",
    "use_cumotion_straight_approach",
    "esdf_freshness_wait_enabled",
)
_EXPECTED_TASK_ACTION_ENDPOINTS = {
    "/pickup_action", "/place_action", "/cartesian_move_action",
    "/joint_move_action", "/fold_action",
}


def _git_blob(repo: Path, commit: str, path: str) -> bytes:
    result = subprocess.run(
        ["git", "-C", str(repo), "show", "{}:{}".format(commit, path)],
        capture_output=True,
    )
    if result.returncode != 0:
        raise AssertionError(
            "cannot read {}:{} from {}: {}".format(
                commit, path, repo, result.stderr.decode("utf-8", errors="replace")
            )
        )
    return result.stdout


def _git_stdout(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args], capture_output=True, text=True, check=False
    )
    if result.returncode != 0:
        raise AssertionError(
            "git {} failed in {}: {}".format(" ".join(args), repo, result.stderr)
        )
    return result.stdout.strip()


def _canonicalize_simulator_full_urdf(source_bytes: bytes) -> bytes:
    """Deterministically execute the pinned v1 URDF canonicalizer.

    The pinned ``workspace.py`` at ``_V1_CANONICALIZER_COMMIT`` and its pinned
    sibling ``config.py`` are materialized as a real temporary package and
    imported by a unique package name (no source-text surgery, no sys.modules
    collision with the active ``tinker_sim_deploy``), so the exact reviewed
    git-object implementation is executed.
    """
    with tempfile.TemporaryDirectory(prefix="v1canon-") as tmp:
        package_name = "tinker_sim_v1canon"
        package_dir = Path(tmp) / package_name
        package_dir.mkdir(parents=True)
        (package_dir / "__init__.py").write_text("")
        (package_dir / "config.py").write_bytes(
            _git_blob(ROOT, _V1_CANONICALIZER_COMMIT, "tools/tinker_sim_deploy/config.py")
        )
        (package_dir / "workspace.py").write_bytes(
            _git_blob(ROOT, _V1_CANONICALIZER_COMMIT, "tools/tinker_sim_deploy/workspace.py")
        )
        sys.path.insert(0, tmp)
        try:
            workspace = importlib.import_module(package_name + ".workspace")
            return workspace.canonicalize_urdf(source_bytes)
        finally:
            sys.path.remove(tmp)
            for name in (package_name, package_name + ".config", package_name + ".workspace"):
                sys.modules.pop(name, None)


def _canonical_json(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _sha256_json(value: object) -> str:
    return hashlib.sha256(_canonical_json(value)).hexdigest()


def _node_set_sha(node_ids: list[str]) -> str:
    """Canonical SHA-256 over the JSON-canonical sorted pytest node list.

    Delete+add substitutions cannot preserve this hash even when the count is
    unchanged, so the clean-checkout seam can lock the exact executed test set.
    """
    return hashlib.sha256(_canonical_json(sorted(node_ids))).hexdigest()


def _check_collection_acceptance(collected: list[str], tree: ET.ElementTree) -> None:
    """Deterministic clean-checkout acceptance validator (fix round 4).

    This is the *single* validator used by the clean-checkout seam for the real
    clone output AND by the inline mutation cases, so a negative assertion always
    exercises the exact code path applied to real output (never a direct set
    comparison).  It raises :class:`AssertionError` with a specific message on
    any deviation from the locked contract:

    - exactly ``_TASK8_CLASS_TEST_COUNT`` collected node IDs and the canonical
      ``_TASK8_CLASS_NODE_SET_SHA`` (delete / rename+add substitution fails);
    - JUnit root is ``<testsuites>``/``<testsuite>`` with exactly ONE
      ``<testsuite>`` (multiple suites are rejected rather than summed);
    - suite counters ``tests``/``failures``/``errors``/``skipped`` match;
    - exactly ``_TASK8_CLASS_TEST_COUNT`` direct ``<testcase>`` children with
      unique names (no duplicate/nested-testcase ambiguity);
    - no unexpected ``failure``/``error``/``xfailure`` child statuses;
    - the exact ``_TASK8_CLEAN_CHECKOUT_SKIPS`` identities with the expected
      reason category substrings.
    """
    # -- collection gate --
    if len(collected) != _TASK8_CLASS_TEST_COUNT:
        raise AssertionError(
            "Task8 class must collect exactly {} tests, found {}".format(
                _TASK8_CLASS_TEST_COUNT, len(collected)
            )
        )
    node_sha = _node_set_sha(sorted(collected))
    if node_sha != _TASK8_CLASS_NODE_SET_SHA:
        raise AssertionError(
            "Task8 collected-node set changed (sha {} != locked {}); update the "
            "locked hash only if the class change is intentional".format(
                node_sha, _TASK8_CLASS_NODE_SET_SHA
            )
        )
    # -- JUnit root / exactly one suite (non-xdist) --
    root = tree.getroot()
    if root.tag not in ("testsuites", "testsuite"):
        raise AssertionError("unexpected JUnit root element {!r}".format(root.tag))
    suites = [s for s in tree.iter("testsuite")]
    if len(suites) != 1:
        raise AssertionError(
            "exactly one <testsuite> required (non-xdist invocation), found {}".format(
                len(suites)
            )
        )
    suite = suites[0]
    # -- suite counters --
    counters = {k: int(suite.attrib[k]) for k in ("tests", "failures", "errors", "skipped")}
    if counters["tests"] != _TASK8_CLASS_TEST_COUNT:
        raise AssertionError(
            "testsuite tests={} != {}".format(counters["tests"], _TASK8_CLASS_TEST_COUNT)
        )
    if counters["failures"] != 0 or counters["errors"] != 0:
        raise AssertionError(
            "testsuite failures/errors must be zero, got {}".format(counters)
        )
    if counters["skipped"] != len(_TASK8_CLEAN_CHECKOUT_SKIPS):
        raise AssertionError(
            "testsuite skipped={} != {}".format(
                counters["skipped"], len(_TASK8_CLEAN_CHECKOUT_SKIPS)
            )
        )
    # -- direct testcase children: exact count, unique names, no status surprises --
    cases = [c for c in suite if c.tag == "testcase"]
    names = [c.attrib.get("name", "") for c in cases]
    if len(cases) != _TASK8_CLASS_TEST_COUNT:
        raise AssertionError(
            "testsuite must contain {} direct <testcase> elements, found {}".format(
                _TASK8_CLASS_TEST_COUNT, len(cases)
            )
        )
    if len(set(names)) != len(names):
        raise AssertionError("duplicate testcase names must be rejected: {}".format(names))
    for case in cases:
        unexpected = [ch.tag for ch in case if ch.tag in ("failure", "error", "xfailure")]
        if unexpected:
            raise AssertionError(
                "unexpected status children {} on testcase {!r}".format(
                    unexpected, case.attrib.get("name")
                )
            )
    # -- exact skip identities + reason categories --
    skip_details: dict[str, str] = {}
    for case in cases:
        skip_node = case.find("skipped")
        if skip_node is not None:
            skip_details[case.attrib["name"]] = skip_node.attrib.get("message", "")
    if set(skip_details) != set(_TASK8_CLEAN_CHECKOUT_SKIPS):
        raise AssertionError(
            "skip set differs from the locked set: {}".format(sorted(skip_details))
        )
    for name, reason in skip_details.items():
        if _TASK8_CLEAN_CHECKOUT_SKIPS[name] not in reason:
            raise AssertionError(
                "skip {!r} must carry the expected reason category, got {!r}".format(
                    name, reason
                )
            )


def _resolver_probe_script() -> str:
    """Source of the fresh isolated child that proves the pinned-resolver proof.

    Run with ``sys.executable -I`` so no ``tinker_sim_deploy*`` module cache is
    inherited and no ``PYTHONPATH``/user-site contamination leaks in.  The child
    explicitly constructs ``sys.path`` (pinned materialized ``tools`` first,
    then the decoy working-tree package, then the minimum bridge dependency),
    asserts ``tinker_sim_deploy`` is not pre-loaded, imports
    ``tinker_sim_deploy.runtime`` and records its resolved ``__file__``, then
    runs the real unmodified Task 3 ``preflight_manifest`` against the
    reconstructed root and emits exactly one machine-readable JSON object on
    stdout.  Any failure emits ``{"ok": false, ...}`` and exits non-zero.
    """
    return r'''
import hashlib
import json
import sys
from pathlib import Path


def _emit(payload: dict[str, object]) -> int:
    json.dump(payload, sys.stdout, sort_keys=True, separators=(",", ":"))
    sys.stdout.write("\n")
    sys.stdout.flush()
    return 0 if payload.get("ok") else 1


def _main() -> int:
    root = Path(sys.argv[1])
    manifest = Path(sys.argv[2])
    bridge = Path(sys.argv[3])
    decoy_tools = Path(sys.argv[4])
    mode = sys.argv[5]
    if "tinker_sim_deploy" in sys.modules:
        return _emit({
            "ok": False,
            "mode": mode,
            "error": "tinker_sim_deploy is already loaded in the fresh child",
        })
    if mode == "positive":
        # Pinned materialized path FIRST; the decoy working-tree package is
        # later on sys.path so any import of tinker_sim_deploy must resolve to
        # the pinned bytes (the decoy sentinel would fail loudly if reached).
        sys.path = [str(root / "tools"), str(decoy_tools), str(bridge)] + sys.path
    elif mode == "negative":
        # Reverse/remove the pinned path: only the broken decoy resolver is
        # visible, so the child must fail and prove the positive is load-bearing.
        sys.path = [str(decoy_tools), str(bridge)] + sys.path
    else:
        return _emit({"ok": False, "mode": mode, "error": "unknown mode"})
    try:
        import tinker_sim_deploy.runtime as runtime
        module_file = str(Path(runtime.__file__).resolve())

        from tinker_sim_bridge.model_preflight import (
            preflight_manifest,
            stable_preflight_evidence,
        )
        result = preflight_manifest(manifest, timeout=30.0, project_root=root)
        stable = stable_preflight_evidence(result)
        stable_sha = hashlib.sha256(
            json.dumps(stable, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        identity_checks = [c for c in result["checks"] if c["name"] == "artifact_identity"]
        return _emit({
            "ok": True,
            "mode": mode,
            "ready": bool(result["ready"]),
            "check_count": len(result["checks"]),
            "check_names": [c["name"] for c in result["checks"]],
            "artifact_identity_ok": bool(identity_checks[0]["ok"]) if identity_checks else None,
            "stable_preflight_sha256": stable_sha,
            "module_file": module_file,
            "module_in_root": module_file.startswith(str(Path(root).resolve())),
            "loaded_modules": {
                name: str(Path(mod.__file__).resolve())
                for name, mod in sorted(sys.modules.items())
                if (name == "tinker_sim_deploy" or name.startswith("tinker_sim_deploy."))
                and getattr(mod, "__file__", None)
            },
        })
    except Exception as exc:
        return _emit({
            "ok": False,
            "mode": mode,
            "error": "{}: {}".format(type(exc).__name__, exc),
        })


if __name__ == "__main__":
    sys.exit(_main())
'''


class ProvenanceTest(unittest.TestCase):
    def test_checked_in_release_inputs_match_manifest(self) -> None:
        manifest = verify(Config.load(ROOT), require_python=True)
        self.assertEqual(manifest["environment"]["resolved_packages"], 219)


class Task8OMPLOverlayProvenanceTest(unittest.TestCase):
    """Task 8: deterministic acceptance-contract provenance (fix rounds 1-4).

    These assertions recompute every derived hash/contract from immutable git
    objects (``git show <recorded-commit>:<path>``) and committed simulator
    source, never from mutable working trees or contract literals alone, and fail
    on mutations: altered argument order/count, booleans, strict keys,
    production imports/packages/executables, simulator provider entries,
    handoff, client scope, model-bundle source evidence, stable hashes, task-range
    boundaries/commits, fixture status publication, artifact path policy, and
    installed data registration.

    Fix round 2 separates the static acceptance gate from the provisioned-host
    runtime diagnostics:

    - the 16-check preflight runs the real unmodified ``preflight_manifest``
      against a self-contained reconstructed project root (committed source +
      pinned git objects + a Task 3-compatible legacy ``current.json``), so
      ``ready=true`` and the stable preflight hash reproduce on a clean checkout
      with no gitignored ``outputs/``/``artifacts/`` dependency;
    - the real ``current.json`` selector is a separate provisioned-host
      readiness diagnostic: when present it must select the reproduced artifact
      (stale selection fails), and when absent the host is reported
      ``not_provisioned`` without failing or silently skipping the static suite;
    - top-level ``evidence.preflight`` carries the load-bearing stable hashes and
      nests the raw host snapshot under ``host_snapshot``.

    The pre-existing uv-environment provenance failure is deliberately NOT
    masked here: ``ProvenanceTest.test_checked_in_release_inputs_match_manifest``
    continues to verify the pinned uv 0.10.8 toolchain and fails on this host
    where installed ``uv 0.12.0`` differs.  The assertions in this class are
    independent of that environment drift.
    """

    CONTRACT_PATH = ROOT / "integration/ompl-overlay-contract.json"
    PROVIDER_MANIFEST_PATH = ROOT / "ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json"
    SETUP_PY = ROOT / "ros2_ws/src/tinker_sim_bridge/setup.py"
    PACKAGE_XML = ROOT / "ros2_ws/src/tinker_sim_bridge/package.xml"
    BRIDGE_DIR = ROOT / "ros2_ws/src/tinker_sim_bridge"
    SCENARIO_DIR = ROOT / "simulation/scenarios"
    MODEL_BUNDLE_PATH = ROOT / "outputs/ompl-overlay/model-bundle-r2/model-bundle.json"
    PREFLIGHT_PATH = ROOT / "outputs/ompl-overlay/model-bundle-r2/preflight-report.json"
    CURRENT_JSON = ROOT / "artifacts/robot/tinker2/current.json"
    PRODUCTION_REPO = Path("/home/tinker/tk25_ws/src/tk25_manipulation")
    PRODUCTION_BASIC_REPO = Path("/home/tinker/tk25_ws/src/tk25_basic")
    PRODUCTION_SIM_REPO = Path("/home/tinker/tk25_ws/src/tk26_sim")
    SIMULATOR_LOCK = ROOT / "integration/source-locks.json"
    SOURCE_EVIDENCE_URDF = ROOT / _SIMULATOR_FULL_URDF_SOURCE
    SCENARIOS = (
        "qualification-moveit-plan-joint",
        "qualification-moveit-plan-pose",
        "qualification-moveit-plan-blocked",
        "qualification-moveit-execute-joint",
        "qualification-moveit-execute-pose",
        "qualification-moveit-cartesian-retreat",
        "qualification-moveit-gripper",
        "qualification-moveit-cancel",
        "qualification-moveit-safety",
        "qualification-pick-place-positive",
        "qualification-pick-place-blocked-approach",
        "qualification-pick-place-unreachable-grasp",
        "qualification-pick-place-malformed-back",
        "qualification-pick-place-cancel-approach",
        "qualification-pick-place-cancel-transport",
        "qualification-pick-place-safety-transport",
        "qualification-pick-place-occupied-place",
    )

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _load_contract(self) -> dict[str, object]:
        return json.loads(self.CONTRACT_PATH.read_text(encoding="utf-8"))

    def _provider_manifest(self) -> dict[str, object]:
        return json.loads(self.PROVIDER_MANIFEST_PATH.read_text(encoding="utf-8"))

    def _git(self, repo: Path, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["git", "-C", str(repo), *args],
            capture_output=True,
            text=True,
            check=False,
        )

    def _sha256_file(self, path: Path) -> str:
        return _sha256_file(path)

    # ------------------------------------------------------------------
    # source-backed production helpers (immutable git objects)
    # ------------------------------------------------------------------
    def _production_launch_source(self) -> str:
        return _git_blob(
            self.PRODUCTION_REPO, _TK25_MANIP_COMMIT,
            "src/mobile_bringup/launch/manipulation_planning_task_only.launch.py",
        ).decode("utf-8")

    def _production_launch_arguments(self) -> list[str]:
        tree = ast.parse(self._production_launch_source())
        names: list[str] = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                func = node.func
                name = None
                if isinstance(func, ast.Name) and func.id == "DeclareLaunchArgument":
                    name = node.args[0].value
                elif isinstance(func, ast.Attribute) and func.attr == "DeclareLaunchArgument":
                    name = node.args[0].value
                if isinstance(name, str):
                    names.append(name)
        return names

    def _production_sim_ompl_literal_false(self) -> dict[str, bool]:
        tree = ast.parse(self._production_launch_source())
        found: dict[str, bool] = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.Dict):
                for key, value in zip(node.keys, node.values):
                    if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
                        continue
                    if key.value in set(_EXPECTED_SIM_COMPAT_KEYS) and isinstance(value, ast.Constant):
                        found[key.value] = bool(value.value)
        return found

    def _production_sim_ompl_literal_true(self) -> dict[str, object]:
        tree = ast.parse(self._production_launch_source())
        found: dict[str, object] = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.Dict):
                for key, value in zip(node.keys, node.values):
                    if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
                        continue
                    if key.value in {"safety_required", "fixture_revision_required", "use_sim_time"} and isinstance(value, ast.Constant):
                        found[key.value] = value.value
        return found

    def _production_strict_sim_inputs_keys(self) -> list[str]:
        tree = ast.parse(self._production_launch_source())
        # strict_sim_inputs is first initialized to {} then populated; take the
        # last non-empty assignment in the pinned launch.
        keys: list[str] | None = None
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign):
                targets = node.targets
                if len(targets) == 1 and isinstance(targets[0], ast.Name) and targets[0].id == "strict_sim_inputs":
                    if isinstance(node.value, ast.Dict):
                        candidate = [ast.literal_eval(key) for key in node.value.keys]
                        if candidate:
                            keys = candidate
        if keys is None:
            raise AssertionError("strict_sim_inputs assignment not found in pinned production launch")
        return keys

    def _production_helpers_source(self) -> str:
        return _git_blob(
            self.PRODUCTION_REPO, _TK25_MANIP_COMMIT,
            "src/mobile_bringup/test/launch_contract_helpers.py",
        ).decode("utf-8")

    def _parse_frozenset(self, source: str, name: str) -> set[str]:
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id == name:
                        value = node.value
                        # ALLOWED_*_IMPORTS = frozenset({...})
                        if isinstance(value, ast.Call) and isinstance(value.func, ast.Name) and value.func.id == "frozenset" and value.args:
                            value = value.args[0]
                        if isinstance(value, ast.Set):
                            return {ast.literal_eval(item) for item in value.elts}
        raise AssertionError("{} not found in launch_contract_helpers.py".format(name))

    def _parse_tuple(self, source: str, name: str) -> tuple[str, ...]:
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id == name:
                        return tuple(ast.literal_eval(item) for item in node.value.elts)
        raise AssertionError("{} not found in launch_contract_helpers.py".format(name))

    def _simulator_overlay_provider_set(self) -> dict[str, list[str]]:
        manifest = self._provider_manifest()
        packages: set[str] = set()
        executables: set[str] = set()
        for entry in list(manifest["persistent_nodes"]) + list(manifest["one_shot_processes"]):
            if entry.get("package"):
                packages.add(str(entry["package"]))
            if entry.get("executable"):
                executables.add(str(entry["executable"]))
        packages.add("controller_manager")
        return {
            "packages": sorted(packages),
            "executables": sorted(executables),
        }

    def _handoff_constants_from_production(self) -> dict[str, str]:
        ownership = _git_blob(
            self.PRODUCTION_REPO, _TK25_MANIP_COMMIT,
            "src/pick_and_place/include/scene_ownership.hpp",
        ).decode("utf-8")
        transaction = _git_blob(
            self.PRODUCTION_REPO, _TK25_MANIP_COMMIT,
            "src/pick_and_place/include/planning_scene_transaction.hpp",
        ).decode("utf-8")
        main = _git_blob(
            self.PRODUCTION_REPO, _TK25_MANIP_COMMIT,
            "src/pick_and_place/src/pick_and_place.cpp",
        ).decode("utf-8")
        return {
            "ownership": ownership,
            "transaction": transaction,
            "main": main,
        }

    # ------------------------------------------------------------------
    # model-bundle reconstruction (clean-checkout reproducible)
    # ------------------------------------------------------------------
    def _production_blob(self, repo: Path, commit: str, path: str) -> bytes:
        return _git_blob(repo, commit, path)

    def _reconstruct_simulator_full_urdf(self, tmpdir: Path) -> Path:
        if not self.SOURCE_EVIDENCE_URDF.is_file():
            raise AssertionError(
                "committed simulator full-URDF source evidence is missing: {} -- "
                "the model-bundle acceptance evidence is not reproducible".format(self.SOURCE_EVIDENCE_URDF)
            )
        source = self.SOURCE_EVIDENCE_URDF.read_bytes()
        self.assertEqual(_sha256_file(self.SOURCE_EVIDENCE_URDF), _SIMULATOR_FULL_URDF_SOURCE_SHA)
        canonical = _canonicalize_simulator_full_urdf(source)
        out = tmpdir / "robot.urdf"
        out.write_bytes(canonical)
        self.assertEqual(_sha256_file(out), _SIMULATOR_FULL_URDF_OUTPUT)
        return out

    def _reconstruct_artifacts(self, tmpdir: Path) -> dict[str, Path]:
        sim_urdf = self._reconstruct_simulator_full_urdf(tmpdir)
        planning_urdf = tmpdir / "xarm7.urdf"
        planning_urdf.write_bytes(self._production_blob(
            self.PRODUCTION_BASIC_REPO, _TK25_BASIC_COMMIT,
            "src/cumotion_description/config/xarm7.urdf",
        ))
        planning_srdf = tmpdir / "xarm7.srdf"
        planning_srdf.write_bytes(self._production_blob(
            self.PRODUCTION_BASIC_REPO, _TK25_BASIC_COMMIT,
            "src/cumotion_description/config/xarm7.srdf",
        ))
        kinematics = tmpdir / "kinematics.yaml"
        kinematics.write_bytes(self._production_blob(
            self.PRODUCTION_REPO, _TK25_MANIP_COMMIT,
            "src/xarm_ros2/xarm_moveit_config/config/xarm7/kinematics.yaml",
        ))
        arm_limits = tmpdir / "arm-joint-limits.yaml"
        arm_limits.write_bytes(self._production_blob(
            self.PRODUCTION_REPO, _TK25_MANIP_COMMIT,
            "src/xarm_ros2/xarm_moveit_config/config/xarm7/joint_limits.yaml",
        ))
        gripper_limits = tmpdir / "gripper-joint-limits.yaml"
        gripper_limits.write_bytes(self._production_blob(
            self.PRODUCTION_REPO, _TK25_MANIP_COMMIT,
            "src/xarm_ros2/xarm_moveit_config/config/xarm_gripper/joint_limits.yaml",
        ))
        joint_limits = tmpdir / "joint_limits.yaml"
        joint_limits.write_bytes(yaml.safe_dump(
            synthesize_joint_limits(arm_limits, gripper_limits),
            sort_keys=True, default_flow_style=False,
        ).encode("utf-8"))
        return {
            "simulator_full_urdf": sim_urdf,
            "planning_urdf": planning_urdf,
            "planning_srdf": planning_srdf,
            "joint_limits": joint_limits,
            "kinematics": kinematics,
        }

    def _reconstruct_manifest(self, tmpdir: Path) -> dict[str, object]:
        artifacts = self._reconstruct_artifacts(tmpdir)
        mount = {"parent": "world", "child": "base_link", "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
        return build_manifest(
            simulator_full_urdf=artifacts["simulator_full_urdf"],
            planning_urdf=artifacts["planning_urdf"],
            planning_srdf=artifacts["planning_srdf"],
            joint_limits=artifacts["joint_limits"],
            kinematics=artifacts["kinematics"],
            prefix="",
            mount=mount,
        )

    def _materialize_tinker_sim_deploy(
        self, target: Path, commit: str | None = None
    ) -> None:
        """Materialize ``tools/tinker_sim_deploy`` into *target* from immutable
        git objects at the recorded simulator implementation identity.

        The file set is derived from ``git ls-tree`` at the pinned commit and
        every blob is written with ``git show <commit>:<path>`` -- never read
        from the live working tree -- so uncommitted local edits to
        ``tools/tinker_sim_deploy`` cannot influence the reconstructed
        preflight.  A non-existent commit or an empty file set fails closed.
        """
        if commit is None:
            commit = str(
                self._load_contract()["repositories"]["simulator"]["implementation_identity"]
            )
        listing = self._git(
            ROOT, "ls-tree", "-r", "--name-only", commit, "tools/tinker_sim_deploy/"
        )
        self.assertEqual(
            listing.returncode, 0, listing.stderr
        )
        paths = [p for p in listing.stdout.split() if p]
        self.assertGreater(
            len(paths), 0,
            "pinned tools/tinker_sim_deploy file set must not be empty at {}".format(commit),
        )
        deploy_dir = target / "tools" / "tinker_sim_deploy"
        deploy_dir.mkdir(parents=True, exist_ok=True)
        for path in paths:
            blob = _git_blob(ROOT, commit, path)
            relative = path[len("tools/tinker_sim_deploy/"):]
            out = deploy_dir / relative
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_bytes(blob)

    def _make_decoy_resolver(self, target: Path) -> Path:
        """Materialize a temporary decoy working-tree resolver package.

        The decoy is a full materialized ``tools/tinker_sim_deploy`` whose
        ``runtime.py`` is replaced by a sentinel that raises on import, so any
        use of the decoy fails loudly instead of silently reproducing the pinned
        evidence.  It lives in a temp directory (never the active checkout) and
        proves the pinned path wins path precedence and is load-bearing.
        """
        self._materialize_tinker_sim_deploy(target)
        runtime = target / "tools/tinker_sim_deploy/runtime.py"
        runtime.write_text(
            "raise RuntimeError('decoy resolver engaged: the pinned materialized resolver must win')\n"
        )
        return target / "tools"

    def _run_resolver_probe(
        self,
        recon_root: Path,
        manifest_path: Path,
        decoy_tools: Path,
        mode: str,
    ) -> subprocess.CompletedProcess[str]:
        """Run the fresh isolated pinned-resolver probe in a child interpreter.

        Uses ``sys.executable -I`` so the child has no inherited
        ``tinker_sim_deploy*`` module cache and no ``PYTHONPATH``/user-site
        contamination.  stdout carries exactly one JSON result; stderr and
        non-zero exits are propagated to the caller for actionable diagnostics.
        No tracked active-checkout file is ever written.
        """
        with tempfile.TemporaryDirectory(prefix="mb-probe-") as pdir:
            script = Path(pdir) / "probe.py"
            script.write_text(_resolver_probe_script())
            env = os.environ.copy()
            env.pop("TINKER_SIM_ROOT", None)
            env.pop("PYTHONPATH", None)
            return subprocess.run(
                [
                    sys.executable, "-I", str(script),
                    str(recon_root), str(manifest_path), str(self.BRIDGE_DIR),
                    str(decoy_tools), mode,
                ],
                capture_output=True,
                text=True,
                env=env,
                check=False,
            )

    def _prove_pinned_resolver_in_fresh_subprocess(
        self, root: dict[str, object], data: dict[str, object]
    ) -> None:
        """Fix-round-4 pinned-resolver proof, executed in a fresh subprocess.

        The parent process pre-imports ``tinker_sim_deploy`` from the live
        working tree, so in-process ``sys.path`` surgery cannot prove the
        reconstructed preflight executes the pinned materialized resolver (the
        module cache masks it).  A fresh ``-I`` child with an empty
        ``tinker_sim_deploy`` cache, the materialized root's ``tools`` first on
        ``sys.path``, and a temp decoy working-tree package later on the path:

        - **positive**: the child loads ``tinker_sim_deploy.runtime`` from the
          materialized root (never the active checkout), runs the real Task 3
          preflight, and emits ready/16-check/exact-stable-hash JSON that the
          parent validates;
        - **negative**: with the pinned path removed only the decoy (broken)
          resolver is visible, so the child fails and proves the positive is
          load-bearing.

        The active checkout's tracked files are never modified.
        """
        with tempfile.TemporaryDirectory(prefix="mb-decoy-") as dtmp:
            decoy_tools = self._make_decoy_resolver(Path(dtmp))
            recon_root = Path(root["tmp"])
            manifest_path = Path(root["manifest_path"])
            positive = self._run_resolver_probe(
                recon_root, manifest_path, decoy_tools, "positive"
            )
            self.assertEqual(
                positive.returncode, 0,
                "positive pinned-resolver probe must succeed in the fresh child:\n"
                + positive.stdout + positive.stderr,
            )
            payload = json.loads(positive.stdout)
            self.assertTrue(payload["ok"], payload)
            self.assertTrue(payload["ready"], payload)
            self.assertEqual(payload["check_count"], data["check_count"])
            self.assertEqual(payload["check_names"], data["check_names"])
            self.assertTrue(payload["artifact_identity_ok"], payload)
            self.assertEqual(payload["stable_preflight_sha256"], data["stable_sha256"])
            module_file = Path(payload["module_file"])
            self.assertTrue(
                module_file.is_relative_to(recon_root),
                "resolver must load from the materialized root, got {}".format(module_file),
            )
            self.assertFalse(
                module_file.is_relative_to(ROOT),
                "resolver must never load from the active checkout, got {}".format(module_file),
            )
            loaded = payload["loaded_modules"]
            self.assertIn("tinker_sim_deploy.runtime", loaded)
            # Every tinker_sim_deploy module loaded in the fresh child must come
            # from the materialized reconstructed root, never the active checkout.
            for name, mod_path in loaded.items():
                self.assertTrue(
                    Path(mod_path).is_relative_to(recon_root),
                    "module {} must come from the materialized root, got {}".format(name, mod_path),
                )
            negative = self._run_resolver_probe(
                recon_root, manifest_path, decoy_tools, "negative"
            )
            self.assertNotEqual(
                negative.returncode, 0,
                "negative pinned-resolver probe must fail (decoy detected):\n"
                + negative.stdout + negative.stderr,
            )
            self.assertIn(
                "decoy", negative.stdout + negative.stderr,
                "negative probe must surface the decoy sentinel diagnostic",
            )

    def _reconstruct_project_root(self, tmpdir: Path) -> dict[str, object]:
        """Reconstruct a self-contained Task 3-compatible project root.

        Returns ``{"tmp", "manifest", "manifest_path"}``.  The temp root carries
        its own ``tools/tinker_sim_deploy`` (so the shared current-artifact
        resolver is self-contained) and a Task 3-compatible legacy
        ``artifacts/robot/tinker2/<id>/`` with a ``current.json`` selector that
        points at the reconstructed canonical simulator URDF.  The real
        unmodified ``preflight_manifest`` therefore runs ``ready=true`` with all
        16 checks -- including ``artifact_identity`` -- on a clean checkout with
        no dependency on the gitignored ``outputs/``/``artifacts/`` trees.
        """
        artifacts = self._reconstruct_artifacts(tmpdir)
        artifact_id = "36ac0317025d20a5"
        artifact_dir = tmpdir / "artifacts/robot/tinker2" / artifact_id
        artifact_dir.mkdir(parents=True, exist_ok=True)
        urdf = artifact_dir / "robot.urdf"
        urdf.write_bytes(artifacts["simulator_full_urdf"].read_bytes())
        urdf_sha = _sha256_file(urdf)
        (artifact_dir / "manifest.json").write_text(_canonical_json({
            "robot": "tinker2",
            "artifact_id": artifact_id,
            "schema_version": 2,
            "canonicalization": {
                "output_sha256": urdf_sha,
                "algorithm": "tinker2-urdf-canonical-v1",
            },
            "files": [
                {"path": "artifacts/robot/tinker2/{}/robot.urdf".format(artifact_id), "sha256": urdf_sha}
            ],
        }).decode("utf-8"))
        (tmpdir / "artifacts/robot/tinker2/current.json").write_text(_canonical_json({
            "artifact_id": artifact_id,
            "manifest": "artifacts/robot/tinker2/{}/manifest.json".format(artifact_id),
        }).decode("utf-8"))
        # The shared resolver is materialized from immutable git objects at the
        # recorded simulator implementation identity -- never copied from the
        # live working tree -- so local edits cannot influence acceptance.
        self._materialize_tinker_sim_deploy(tmpdir)
        manifest = build_manifest(
            simulator_full_urdf=artifacts["simulator_full_urdf"],
            planning_urdf=artifacts["planning_urdf"],
            planning_srdf=artifacts["planning_srdf"],
            joint_limits=artifacts["joint_limits"],
            kinematics=artifacts["kinematics"],
            prefix="",
            mount={"parent": "world", "child": "base_link", "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
        )
        manifest_path = tmpdir / "model-bundle.json"
        manifest_path.write_bytes(_canonical_json(manifest))
        return {"tmp": tmpdir, "manifest": manifest, "manifest_path": manifest_path}

    def _task7_action_client_scope(self) -> dict[str, object]:
        """Ground the Task 7 action-client restriction in pinned Task 7 source.

        AST-count the real ``ActionClient(...)`` constructions in
        ``validation/ompl_plan_smoke.py`` at the recorded simulator
        implementation identity: exactly one MoveGroup client on ``/move_action``
        and no execute-trajectory/controller/task action client.
        """
        sim_identity = str(self._load_contract()["repositories"]["simulator"]["implementation_identity"])
        source = _git_blob(ROOT, sim_identity, "validation/ompl_plan_smoke.py").decode("utf-8")
        tree = ast.parse(source)
        calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "ActionClient"
        ]
        self.assertEqual(len(calls), 1, "Task 7 smoke must construct exactly one ActionClient")
        args = calls[0].args
        self.assertGreaterEqual(len(args), 3, "ActionClient requires node, type, action")
        self.assertEqual(args[2].id, "MOVE_ACTION")
        self.assertIn('MOVE_ACTION = "/move_action"', source)
        return {"count": 1, "action": "/move_action", "type": "MoveGroup"}

    # ------------------------------------------------------------------
    # deterministic canonical contract
    # ------------------------------------------------------------------
    def test_contract_is_deterministic_canonical_json(self) -> None:
        raw = self.CONTRACT_PATH.read_text(encoding="utf-8")
        data = json.loads(raw)
        canonical = json.dumps(data, sort_keys=True, separators=(",", ":"))
        self.assertEqual(raw, canonical, "contract must be canonical compact JSON")
        self.assertFalse(raw.endswith("\n"), "contract must not carry a trailing newline")
        self.assertEqual(data["schema_version"], 1)
        self.assertEqual(data["contract_id"], "simulator-ompl-overlay-acceptance")

    def test_contract_has_no_timestamps_and_no_task8_self_reference(self) -> None:
        raw = self.CONTRACT_PATH.read_text(encoding="utf-8")
        for token in ('"timestamp"', '"created_at"', '"generated_at"'):
            self.assertNotIn(token, raw, f"contract must not record a {token} field")
        self.assertNotIn("task8_commit", raw)
        self.assertNotIn("task_8_commit", raw)
        data = self._load_contract()
        self.assertNotIn("task8_commit", data)
        self.assertNotIn("commit", data)

    def test_contract_required_sections_present(self) -> None:
        data = self._load_contract()
        required = {
            "repositories",
            "production_overlay",
            "ros_policy",
            "provider_manifest",
            "model_bundle",
            "typed_contract",
            "fixture_contract",
            "scenarios",
            "evidence",
            "build_commands",
            "source_locks",
        }
        self.assertTrue(required.issubset(set(data)))

    # ------------------------------------------------------------------
    # repository identities
    # ------------------------------------------------------------------
    def test_contract_repository_identities_match_git_history(self) -> None:
        data = self._load_contract()
        sim = data["repositories"]["simulator"]
        prod = data["repositories"]["production"]

        sim_id = str(sim["implementation_identity"])
        resolved = self._git(ROOT, "rev-parse", sim_id + "^{commit}")
        self.assertEqual(resolved.returncode, 0, "simulator identity not in history")
        self.assertEqual(resolved.stdout.strip(), sim_id)

        # The recorded implementation identity must actually be an ancestor-or-equal
        # of the current simulator HEAD (merge-base --is-ancestor).
        ancestor = self._git(ROOT, "merge-base", "--is-ancestor", sim_id, "HEAD")
        self.assertEqual(ancestor.returncode, 0, "simulator identity must be an ancestor of HEAD")

        for key, commit in (
            ("runtime_hardening", str(prod["runtime_hardening"]["head_commit"])),
            ("task2_launch", str(prod["task2_launch"]["head_commit"])),
        ):
            result = self._git(self.PRODUCTION_REPO, "rev-parse", commit + "^{commit}")
            self.assertEqual(result.returncode, 0, f"production {key} commit missing")
            self.assertEqual(result.stdout.strip(), commit)

        hardening_end = str(prod["runtime_hardening"]["end"])
        task2_end = str(prod["task2_launch"]["end"])
        ancestor = self._git(
            self.PRODUCTION_REPO, "merge-base", "--is-ancestor", hardening_end, task2_end
        )
        self.assertEqual(ancestor.returncode, 0, "runtime-hardening end must be an ancestor of Task 2 launch end")

    def test_contract_encodes_production_ranges_from_actual_history(self) -> None:
        data = self._load_contract()
        rh = data["repositories"]["production"]["runtime_hardening"]
        t2 = data["repositories"]["production"]["task2_launch"]
        self.assertEqual(rh["range"], "{}..{}".format(rh["start"], rh["end"]))
        self.assertEqual(t2["range"], "{}..{}".format(t2["start"], t2["end"]))
        self.assertEqual(data["repositories"]["production"]["implementation_identity"], t2["end"])
        for key in (rh["start"], rh["end"], t2["start"], t2["end"]):
            self.assertNotEqual(key, "HEAD")

    def test_contract_task_range_count_semantics(self) -> None:
        data = self._load_contract()
        tr = data["repositories"]["simulator"]["task_range"]
        start = str(tr["start"])
        end = str(tr["end"])
        # Recorded count must equal the exclusive start..end git count.
        count = self._git(ROOT, "rev-list", "--count", "{}..{}".format(start, end))
        self.assertEqual(count.returncode, 0)
        self.assertEqual(int(count.stdout.strip()), int(tr["count"]))
        # The semantics field must explicitly define the exclusive-start meaning.
        self.assertIn("exclusive", str(tr["count_semantics"]))
        self.assertIn("rev-list --count", str(tr["count_semantics"]))

    def test_contract_task_range_exact_boundary_and_commits(self) -> None:
        data = self._load_contract()
        tr = data["repositories"]["simulator"]["task_range"]
        start = str(tr["start"])
        end = str(tr["end"])
        # Ordered: start is a strict ancestor of end.
        self.assertEqual(
            self._git(ROOT, "merge-base", "--is-ancestor", start, end).returncode,
            0,
            "task_range.start must be an ancestor of task_range.end",
        )
        self.assertNotEqual(start, end)
        # Boundary subjects identify the intended Task 3 / Task 7 implementation
        # commits (canonical model bundle producer -> OMPL smoke adjudication).
        self.assertEqual(
            self._git(ROOT, "log", "--format=%s", "-1", start).stdout.strip(),
            tr["boundary_subjects"]["start"],
        )
        self.assertEqual(
            self._git(ROOT, "log", "--format=%s", "-1", end).stdout.strip(),
            tr["boundary_subjects"]["end"],
        )
        self.assertEqual(tr["boundary_subjects"]["start"], _TASK_RANGE_START_SUBJECT)
        self.assertEqual(tr["boundary_subjects"]["end"], _TASK_RANGE_END_SUBJECT)
        # The exact ordered range must equal the recorded Task 3-7 commit list --
        # not merely another 13-commit pair.
        commits = self._git(ROOT, "rev-list", "{}..{}".format(start, end)).stdout.split()
        self.assertEqual(commits, list(tr["commits"]))
        self.assertEqual(commits, _TASK_RANGE_COMMITS)
        self.assertEqual(len(commits), int(tr["count"]))
        self.assertEqual(commits[0], end)

    def test_contract_host_paths_are_environment_identities(self) -> None:
        data = self._load_contract()
        note = str(data["repositories"]["path_scope"])
        self.assertIn("environment identities", note)
        self.assertIn("qualification workspace", note)

    def test_contract_clean_dirty_policy_recorded(self) -> None:
        data = self._load_contract()
        sim_policy = str(data["repositories"]["simulator"]["dirty_policy"])
        prod_policy = str(data["repositories"]["production"]["dirty_policy"])
        self.assertIn("clean", sim_policy.lower())
        self.assertIn("read-only", prod_policy.lower())

    # ------------------------------------------------------------------
    # typed contract vs live source
    # ------------------------------------------------------------------
    def test_contract_typed_contract_matches_integrated_readiness_source(self) -> None:
        data = self._load_contract()
        contract = data["typed_contract"]
        mapping = build_integrated_mapping()

        self.assertEqual(contract["report_revision"], mapping["report_revision"])
        self.assertEqual(contract["actions"], mapping["actions"])
        self.assertEqual(contract["services"], mapping["services"])
        self.assertEqual(contract["publishers"], mapping["publishers"])
        self.assertEqual(contract["joint_names"], mapping["joint_names"])
        self.assertEqual(contract["touch_links"], mapping["touch_links"])
        self.assertEqual(contract["tf"], mapping["tf"])
        self.assertEqual(contract["controller_resources"], mapping["controller_resources"])
        self.assertEqual(contract["final_simulation_state"], mapping["final_simulation_state"])

        self.assertEqual(contract["runtime_contract_sha256"], sha256_json(mapping))
        pub = public_integrated_mapping()
        self.assertEqual(contract["public_report_separation"]["public_integrated"], pub)
        self.assertEqual(
            contract["public_report_separation"]["public_integrated_sha256"],
            sha256_json(pub),
        )

    def test_contract_public_report_vs_full_readiness_separation(self) -> None:
        data = self._load_contract()
        sep = data["typed_contract"]["public_report_separation"]
        self.assertEqual(sep["public_integrated"], {"execution_profile": "sim_ompl"})
        self.assertNotEqual(
            sep["public_integrated_sha256"],
            data["typed_contract"]["runtime_contract_sha256"],
        )

    def test_contract_full_eight_touch_links(self) -> None:
        data = self._load_contract()
        touch = data["typed_contract"]["touch_links"]
        expected = [
            "xarm_gripper_base_link",
            "left_outer_knuckle",
            "left_finger",
            "left_inner_knuckle",
            "right_inner_knuckle",
            "right_outer_knuckle",
            "right_finger",
            "link_tcp",
        ]
        self.assertEqual(touch, expected)
        self.assertEqual(len(touch), 8)
        self.assertEqual(data["model_bundle"]["semantic_contract"]["touch_links"], expected)

    def test_contract_isaac_joint_commands_depth_50_and_external_owner(self) -> None:
        data = self._load_contract()
        pubs = data["typed_contract"]["publishers"]
        self.assertEqual(pubs["/isaac_joint_commands"]["depth"], 50)
        self.assertEqual(pubs["/isaac_joint_commands"]["source"], "/tinker_sim_command_gateway")
        self.assertEqual(pubs["/isaac_joint_commands"]["type"], "sensor_msgs/msg/JointState")
        self.assertEqual(pubs["/isaac_joint_commands"]["cardinality"], 1)
        self.assertEqual(pubs["/sim/safety/operator"]["source"], "/tinker_integrated_gate_executor")
        self.assertEqual(pubs["/sim/safety/operator"]["depth"], 1)
        self.assertEqual(pubs["/sim/safety/operator"]["durability"], "TRANSIENT_LOCAL")
        # The external owner is explicitly marked late-bound, not a live provider.
        self.assertIn("external", str(data["typed_contract"]["external_publishers_note"]).lower())

    # ------------------------------------------------------------------
    # provider manifest
    # ------------------------------------------------------------------
    def test_contract_provider_manifest_matches_committed_file(self) -> None:
        contract = self._load_contract()["provider_manifest"]
        manifest = self._provider_manifest()

        self.assertEqual(contract["schema_version"], manifest["schema_version"])
        self.assertEqual(contract["owner"], manifest["owner"])
        self.assertEqual(contract["canonical_self_hash"], manifest["provider_manifest_sha256"])
        self.assertEqual(contract["raw_sha256"], self._sha256_file(self.PROVIDER_MANIFEST_PATH))
        self.assertEqual(contract["cardinality_source"], manifest["cardinality_source"])
        self.assertEqual(contract["persistent_nodes"], manifest["persistent_nodes"])
        self.assertEqual(contract["one_shot_processes"], manifest["one_shot_processes"])
        self.assertEqual(contract["controller_resources"], manifest["controller_resources"])
        self.assertEqual(contract["publishers"], manifest["publishers"])

    def test_provider_manifest_canonical_self_hash_recomputed(self) -> None:
        manifest = self._provider_manifest()
        recorded = manifest["provider_manifest_sha256"]
        recomputed = sha256_json(
            {k: v for k, v in manifest.items() if k != "provider_manifest_sha256"}
        )
        self.assertEqual(recorded, recomputed)

    def test_provider_manifest_distinguishes_provider_classes(self) -> None:
        data = self._load_contract()
        pm = data["provider_manifest"]
        live = self._provider_manifest()
        self.assertEqual(pm["persistent_nodes"], live["persistent_nodes"])
        self.assertEqual(
            {e["key"] for e in pm["persistent_nodes"]},
            {e["key"] for e in live["persistent_nodes"]},
        )
        one_shot_keys = {e["key"] for e in pm["one_shot_processes"]}
        self.assertEqual(one_shot_keys, {"scenario_runner", "controller_reconciler"})
        controller_names = {e["resource_name"] for e in pm["controller_resources"]}
        self.assertEqual(
            controller_names, {"joint_state_broadcaster", "xarm7_traj_controller"}
        )
        publisher_topics = {e["topic"] for e in pm["publishers"]}
        self.assertEqual(
            publisher_topics,
            {
                "/joint_states",
                "/isaac_joint_commands",
                "/sim/controller/gripper_commands",
                "/sim/safety/operator",
                "/sim/hardware/safety_stop",
                "/sim/status/planning_scene_fixture",
                "/sim/status/integrated_manipulation",
            },
        )
        persistent_nodes = {e["node"] for e in pm["persistent_nodes"]}
        self.assertNotIn("/tinker_integrated_gate_executor", persistent_nodes)
        # The external /sim/safety/operator publisher is explicitly marked late-bound.
        external = pm["external_late_bound_publishers"]
        self.assertEqual(external[0]["topic"], "/sim/safety/operator")
        self.assertEqual(external[0]["source"], "/tinker_integrated_gate_executor")

    def test_contract_all_typed_endpoints_recorded(self) -> None:
        data = self._load_contract()
        actions = data["typed_contract"]["actions"]
        self.assertEqual(
            set(actions),
            {
                "/move_action",
                "/execute_trajectory",
                "/xarm_gripper/gripper_action",
                "/xarm7_traj_controller/follow_joint_trajectory",
                "/pickup_action",
                "/place_action",
                "/cartesian_move_action",
                "/joint_move_action",
                "/fold_action",
            },
        )
        self.assertEqual(actions["/move_action"]["type"], "moveit_msgs/action/MoveGroup")
        self.assertEqual(actions["/execute_trajectory"]["type"], "moveit_msgs/action/ExecuteTrajectory")
        self.assertEqual(actions["/pickup_action"]["type"], "tinker_arm_msgs/action/Pick")
        self.assertEqual(actions["/place_action"]["type"], "tinker_arm_msgs/action/Place")
        self.assertEqual(actions["/cartesian_move_action"]["type"], "tinker_arm_msgs/action/CartesianMove")
        self.assertEqual(actions["/joint_move_action"]["type"], "tinker_arm_msgs/action/JointMove")
        self.assertEqual(actions["/fold_action"]["type"], "tinker_arm_msgs/action/Fold")

        services = data["typed_contract"]["services"]
        self.assertEqual(services["/arm_joint_service"]["type"], "tinker_arm_msgs/srv/ArmJointService")
        self.assertEqual(services["/arm_joint_service"]["source"], "/pick_and_place")
        self.assertEqual(
            services["/controller_manager/list_controllers"]["type"],
            "controller_manager_msgs/srv/ListControllers",
        )
        self.assertEqual(
            services["/get_planning_scene"]["type"], "moveit_msgs/srv/GetPlanningScene"
        )
        self.assertEqual(
            services["/apply_planning_scene"]["type"], "moveit_msgs/srv/ApplyPlanningScene"
        )
        self.assertEqual(
            services["/sim/ready/physics"],
            {"type": "std_srvs/srv/Trigger", "source": "/tinker_sim_physics_ready_gate"},
        )
        self.assertEqual(
            services["/sim/ready/fixture"],
            {"type": "std_srvs/srv/Trigger", "source": "/fixture_planning_scene"},
        )

    # ------------------------------------------------------------------
    # scenarios and fixture identities
    # ------------------------------------------------------------------
    def test_contract_scenario_identities_recomputed(self) -> None:
        data = self._load_contract()
        expected = {
            "qualification-moveit-plan-joint": {
                "decl": "a3dab04c7dc7905ea56466b5f0ae9219c3b9250c5fb9c2ce90cf4acdcb0f68eb",
                "rev": "2026-08-01-moveit-qualification-joint",
                "rev_digest": "d684a3d2270ab6d935b8e5c94dd5d4512760e06a1d09a41582177680536ccd8d",
                "owned": ["sim_fixture/pedestal", "sim_fixture/public_target"],
                "fd": "7f89ab08d2cf74ae0726bb9edc1bfc3cf6e1f6ea7d8c5662760d3551530ab9de",
                "target": "sim_fixture/public_target",
            },
            "qualification-moveit-plan-pose": {
                "decl": "04ade8e7339b9fc3cf72e9b8ae0d0c88325ffbc72332efe55413be184f5579c3",
                "rev": "2026-08-01-moveit-qualification-pose",
                "rev_digest": "fb2abd517a6c6d2f5d34ec099f3e62a245a968e52cd3223bb5ac911d5382af67",
                "owned": ["sim_fixture/pedestal", "sim_fixture/public_target"],
                "fd": "de3af2e05493e0fc3a11c8526c84da1065648f7134f996c9a331819c54df84f4",
                "target": "sim_fixture/public_target",
            },
            "qualification-moveit-plan-blocked": {
                "decl": "3e4f846117131bca8febcd7b09dff4ad0793a8b055fc4efe0c3b6897247b794a",
                "rev": "2026-08-01-moveit-qualification-blocked",
                "rev_digest": "d6d25cfe04fa7e641d6140c20431c24a5cd150005d0c7f9c13735402c93281ac",
                "owned": [
                    "sim_fixture/pedestal",
                    "sim_fixture/public_target",
                    "sim_fixture/plan_blocker",
                ],
                "fd": "d1a1923ceb45394dd70b4248541eac3379a12921d1d244c7d3d5de44157578e4",
                "target": "sim_fixture/public_target",
            },
            "qualification-moveit-execute-joint": {
                "decl": "5c122da43753ffd3e5c4257525cf9a99d9730558550b0bce8ffe8b6000f59de0",
                "rev": "2026-08-02-moveit-execute-joint",
                "rev_digest": "0386a418d3cc1a8ad9d60571544adeb1dce3e835fac86361f2de858c543a2803",
                "owned": ["sim_fixture/pedestal", "sim_fixture/public_target"],
                "fd": "09ac8025ef2dde4f66f0d1e3bed6a31a6d747623c75478622f08767285fd947c",
                "target": "sim_fixture/public_target",
            },
            "qualification-moveit-execute-pose": {
                "decl": "586ad918a9be5b8171a5172461c6b12f17e6e365ff85852c55f01cdd227d7cdd",
                "rev": "2026-08-02-moveit-execute-pose",
                "rev_digest": "909614fb232baac5c7481898f9239990a94fe235101a8485ee8af5c212c0db09",
                "owned": ["sim_fixture/pedestal", "sim_fixture/public_target"],
                "fd": "e7baa8bfd3c84e994e0ee95b3260d7df5ed0183db2cb5caa52041bc6a380cb67",
                "target": "sim_fixture/public_target",
            },
            "qualification-moveit-cartesian-retreat": {
                "decl": "477928d8fbecc2fe9286756b02cc3340f5a027b8a18e587ed5b2850924287898",
                "rev": "2026-08-02-moveit-execute-cartesian-retreat",
                "rev_digest": "554a5d2234a5a9388b69f2f055f00a4205d3184f62472c774905a148c66c9092",
                "owned": ["sim_fixture/pedestal", "sim_fixture/public_target"],
                "fd": "7cd5b1d5d7c90d361c6573b96db1ea2c70a9245c690e82bedd47cb3ebec137d8",
                "target": "sim_fixture/public_target",
            },
            "qualification-moveit-gripper": {
                "decl": "660a3caf5678aac63ccd23f872adf6da00a1a8386ffe6a399107e1689dda3d1c",
                "rev": "2026-08-02-moveit-execute-gripper",
                "rev_digest": "3ff431fc73f0c3118a31b7b632f3944a3c689bd17839e623f06ef9bf51396451",
                "owned": ["sim_fixture/pedestal", "sim_fixture/public_target"],
                "fd": "84af8885ff11acfe77970ab9eda086fb69f896e34931cded9bf9a9c5646e5c7b",
                "target": "sim_fixture/public_target",
            },
            "qualification-moveit-cancel": {
                "decl": "50d004a293e4acdc511dd82499a3e1292786d68d83f722ea9eda31a0a73f8a0b",
                "rev": "2026-08-02-moveit-execute-cancel",
                "rev_digest": "7d65de800c7e27f2896815057c6e0ede2bf3a02921f275a2b229be3756f6962b",
                "owned": ["sim_fixture/pedestal", "sim_fixture/public_target"],
                "fd": "a22d99cd23b2ba8741edc18371368a2752ace6c79d3e0dd46ce88594812d8887",
                "target": "sim_fixture/public_target",
            },
            "qualification-moveit-safety": {
                "decl": "c9e6718fb4d6ee6ab7f9b69da15dfc79f4c56eaebada02202f31cae8f6436572",
                "rev": "2026-08-02-moveit-execute-safety",
                "rev_digest": "053a1c688870776a610046df70d2e24937af22d24df80254c8eb2267d55eec08",
                "owned": ["sim_fixture/pedestal", "sim_fixture/public_target"],
                "fd": "ebed4693c7875192cde1b1a3a9f219da49bac7938d9b0620fe487fca8ab4c96d",
                "target": "sim_fixture/public_target",
            },
            "qualification-pick-place-positive": {
                "decl": "936f0820d93ce686a37323af6df1d63bdce182f1f412ed7fba161e90ad9ccdcb",
                "rev": "qualification-v1",
                "rev_digest": "3afe7f1e1210a29c31364fabd6a955991a30cf5aaf9183c98503f2412d002a9b",
                "owned": ["sim_fixture/pedestal", "sim_fixture/qualification_cube"],
                "fd": "a254f2990a2215b8292872f400ee78b0b2c680fa17e8ea360c72b9939d34138c",
                "target": "sim_fixture/qualification_cube",
            },
            "qualification-pick-place-blocked-approach": {
                "decl": "88b40f1e05c93af767e9c40947c58e420d740ab543c2ddbb56434038f6da0935",
                "rev": "qualification-v1",
                "rev_digest": "4bf4696ee23fc7645d4affd05b0baf5ff85d8a229a0f998cb893ff91b0c56f21",
                "owned": [
                    "sim_fixture/pedestal",
                    "sim_fixture/qualification_cube",
                    "sim_fixture/plan_blocker",
                ],
                "fd": "677694785eab24a50f59a8438b09b8d6051865f4e120c46a03caa4e9f956eff9",
                "target": "sim_fixture/qualification_cube",
            },
            "qualification-pick-place-unreachable-grasp": {
                "decl": "359ce6215ce638087bd54838638542bd17f3dff626b42a0f03ab8890817f15ef",
                "rev": "qualification-v1",
                "rev_digest": "4f70567d8a2242de74a5643b10d8b4d030fe9ea6574e13c8224bee3a232f5e09",
                "owned": ["sim_fixture/pedestal", "sim_fixture/qualification_cube"],
                "fd": "f32a0610b1be074ba9a8b7290ce01dcd2b608107a4b445bc856381fb9eb24770",
                "target": "sim_fixture/qualification_cube",
            },
            "qualification-pick-place-malformed-back": {
                "decl": "25d6eb0870f4adbc27f24bc46cdcc5bb01bb8c5fbf661371a4a34094ec8d0d29",
                "rev": "qualification-v1",
                "rev_digest": "3afe7f1e1210a29c31364fabd6a955991a30cf5aaf9183c98503f2412d002a9b",
                "owned": ["sim_fixture/pedestal", "sim_fixture/qualification_cube"],
                "fd": "a254f2990a2215b8292872f400ee78b0b2c680fa17e8ea360c72b9939d34138c",
                "target": "sim_fixture/qualification_cube",
            },
            "qualification-pick-place-cancel-approach": {
                "decl": "e317daa41a76b6df8785fe31c6dcba17c2f9aacc1d6a2001cb4cbe1a920b4b8b",
                "rev": "qualification-v1",
                "rev_digest": "3afe7f1e1210a29c31364fabd6a955991a30cf5aaf9183c98503f2412d002a9b",
                "owned": ["sim_fixture/pedestal", "sim_fixture/qualification_cube"],
                "fd": "a254f2990a2215b8292872f400ee78b0b2c680fa17e8ea360c72b9939d34138c",
                "target": "sim_fixture/qualification_cube",
            },
            "qualification-pick-place-cancel-transport": {
                "decl": "9662f3bc27a47c51b079f6444e3cd4f2ac679a1710567e17cacb0f9e089a8347",
                "rev": "qualification-v1",
                "rev_digest": "3afe7f1e1210a29c31364fabd6a955991a30cf5aaf9183c98503f2412d002a9b",
                "owned": ["sim_fixture/pedestal", "sim_fixture/qualification_cube"],
                "fd": "a254f2990a2215b8292872f400ee78b0b2c680fa17e8ea360c72b9939d34138c",
                "target": "sim_fixture/qualification_cube",
            },
            "qualification-pick-place-safety-transport": {
                "decl": "97b47b64f48c10a154251e80f33f923517b3bddf26bcb5143956b8536fa9f486",
                "rev": "qualification-v1",
                "rev_digest": "3afe7f1e1210a29c31364fabd6a955991a30cf5aaf9183c98503f2412d002a9b",
                "owned": ["sim_fixture/pedestal", "sim_fixture/qualification_cube"],
                "fd": "a254f2990a2215b8292872f400ee78b0b2c680fa17e8ea360c72b9939d34138c",
                "target": "sim_fixture/qualification_cube",
            },
            "qualification-pick-place-occupied-place": {
                "decl": "78020660eb64dd9597a293559e462c3062565018254ca0eed8567f88aac24621",
                "rev": "qualification-v1",
                "rev_digest": "fb30f65d2378355095084c343cbe23ca057ff4fd659716f2c7b6758a528c7589",
                "owned": [
                    "sim_fixture/pedestal",
                    "sim_fixture/qualification_cube",
                    "sim_fixture/place_occupant",
                ],
                "fd": "2787a6bb062ac15aa0db43b2e0a1cc0e6cd8c72d184e212db5c7d3f95f86038d",
                "target": "sim_fixture/qualification_cube",
            },
        }
        for sid, exp in expected.items():
            raw = json.loads((self.SCENARIO_DIR / f"{sid}.json").read_text(encoding="utf-8"))
            declaration = {str(k): v for k, v in raw.items() if k not in {"id", "seed"}}
            recomputed_decl = sha256_json({"id": sid, "seed": 7, "declaration": declaration})
            self.assertEqual(recomputed_decl, exp["decl"])
            ps = raw["planning_scene"]
            recomputed_rev_digest = revision_digest(ps)
            recomputed_owned = list(str(i) for i in fixture_owned_ids(ps))
            recomputed_fd = fixture_descriptor_sha256(ps)
            self.assertEqual(recomputed_rev_digest, exp["rev_digest"])
            self.assertEqual(recomputed_owned, exp["owned"])
            self.assertEqual(recomputed_fd, exp["fd"])

            recorded = data["scenarios"][sid]
            self.assertEqual(recorded["scenario_declaration_sha256"], exp["decl"])
            self.assertEqual(recorded["planning_scene"]["revision"], exp["rev"])
            self.assertEqual(recorded["planning_scene"]["revision_digest"], exp["rev_digest"])
            self.assertEqual(recorded["planning_scene"]["owned_ids"], exp["owned"])
            self.assertEqual(recorded["planning_scene"]["fixture_descriptor_sha256"], exp["fd"])
            self.assertEqual(recorded["planning_scene"]["target_source_id"], exp["target"])
            self.assertEqual(
                recorded["planning_scene"]["target_handoff"], "pick_and_place/object_mesh"
            )

    def test_contract_fixture_ownership_and_handoff(self) -> None:
        data = self._load_contract()
        fc = data["fixture_contract"]
        self.assertEqual(fc["namespace"], "sim_fixture/*")
        self.assertEqual(fc["target_source_id"], "sim_fixture/public_target")
        self.assertEqual(fc["target_handoff"], "pick_and_place/object_mesh")
        self.assertIn("pick_and_place/object_mesh", str(fc["task_owned_lifecycle"]))
        self.assertIn("fixture_owned_ids", str(fc["parser_encoding"]))

    # ------------------------------------------------------------------
    # model bundle -- reproducible from committed inputs (clean checkout)
    # ------------------------------------------------------------------
    def test_simulator_full_urdf_derivation_reproducible(self) -> None:
        if not self.SOURCE_EVIDENCE_URDF.is_file():
            self.fail("committed simulator full-URDF source evidence missing; derivation not reproducible")
        source = self.SOURCE_EVIDENCE_URDF.read_bytes()
        self.assertEqual(
            _sha256_file(self.SOURCE_EVIDENCE_URDF), _SIMULATOR_FULL_URDF_SOURCE_SHA,
            "committed source evidence must equal the recorded external source sha256",
        )
        # Cross-check the committed copy against the external tracked source.
        external = self.PRODUCTION_SIM_REPO / "_generated/tinker_full.full.urdf"
        if external.is_file():
            self.assertEqual(_sha256_file(external), _SIMULATOR_FULL_URDF_SOURCE_SHA)
        canonical = _canonicalize_simulator_full_urdf(source)
        self.assertEqual(
            hashlib.sha256(canonical).hexdigest(), _SIMULATOR_FULL_URDF_OUTPUT,
            "v1 canonicalizer must reproduce the selected simulator full URDF byte-for-byte",
        )

    def test_model_bundle_reconstructed_from_committed_inputs(self) -> None:
        data = self._load_contract()["model_bundle"]
        with tempfile.TemporaryDirectory(prefix="mb-recon-") as tmp:
            manifest = self._reconstruct_manifest(Path(tmp))
            self.assertEqual(manifest["schema_version"], data["schema_version"])
            self.assertEqual(manifest["producer"], data["producer"])
            # The structural fingerprint is path-independent and must match.
            self.assertEqual(manifest["structural_fingerprint"], data["structural_fingerprint"])
            # The full selected-subgraph contract must match.
            self.assertEqual(manifest["contract"], data["semantic_contract"])
            # The stable manifest projection is byte-reproducible.
            self.assertEqual(
                _sha256_json(stable_manifest_evidence(manifest)),
                data["stable_manifest_sha256"],
            )
            # Every artifact content hash must match the pinned contract.
            for name, entry in manifest["artifacts"].items():
                self.assertEqual(entry["sha256"], data["artifacts"][name]["sha256"])
            # The contract's source-evidence output equals the regenerated simulator URDF.
            self.assertEqual(
                data["source_evidence"]["simulator_full_urdf"]["output_sha256"],
                manifest["artifacts"]["simulator_full_urdf"]["sha256"],
            )

    def test_model_bundle_stable_projection_reproducible(self) -> None:
        # The committed-input reconstruction is the mandatory clean-checkout gate
        # (never skipped, no gitignored-tree dependency).
        with tempfile.TemporaryDirectory(prefix="mb-stable-") as tmp:
            manifest = self._reconstruct_manifest(Path(tmp))
            self.assertEqual(
                _sha256_json(stable_manifest_evidence(manifest)),
                self._load_contract()["model_bundle"]["stable_manifest_sha256"],
            )
        # When the provisioned on-disk manifest exists, its stable projection must
        # equal the contract's stable hash too (identical bytes, host-independent).
        # When it is absent, record an explicit non-provisioned diagnostic instead
        # of silently passing (never a silent no-op).
        if self.MODEL_BUNDLE_PATH.is_file():
            on_disk = json.loads(self.MODEL_BUNDLE_PATH.read_text(encoding="utf-8"))
            self.assertEqual(
                _sha256_json(stable_manifest_evidence(on_disk)),
                self._load_contract()["model_bundle"]["stable_manifest_sha256"],
            )
        else:
            self.skipTest(
                "non-provisioned host: provisioned outputs/ompl-overlay/model-bundle-r2/"
                "model-bundle.json is absent; the committed-input reconstruction above is "
                "the mandatory passing gate, and this on-disk cross-check is a host diagnostic"
            )

    def test_preflight_stable_evidence_reconstructed(self) -> None:
        data = self._load_contract()["model_bundle"]["preflight_report"]
        with tempfile.TemporaryDirectory(prefix="mb-recon-") as tmp:
            root = self._reconstruct_project_root(Path(tmp))
            # The real unmodified preflight runs against a self-contained temp
            # project root whose legacy current.json selects the reconstructed
            # canonical simulator URDF; artifact_identity is therefore resolvable
            # with no dependency on the gitignored outputs/artifacts trees.
            result = preflight_manifest(root["manifest_path"], timeout=30.0, project_root=root["tmp"])
            self.assertEqual(result["status"], "ready")
            self.assertTrue(result["ready"])
            self.assertEqual(len(result["checks"]), data["check_count"])
            self.assertEqual(
                [c["name"] for c in result["checks"]], data["check_names"]
            )
            # artifact_identity must be ok=true (the current.json selection
            # matches the reconstructed artifact).
            identity = next(c for c in result["checks"] if c["name"] == "artifact_identity")
            self.assertTrue(identity["ok"])
            # The stable preflight projection excludes elapsed_ms and host paths.
            self.assertEqual(
                _sha256_json(stable_preflight_evidence(result)),
                data["stable_sha256"],
            )
            self.assertNotIn("elapsed_ms", stable_preflight_evidence(result))
            self.assertNotIn("model_bundle_manifest", stable_preflight_evidence(result))
            # Stale-selector fail-closed: a provisioned selector that points at a
            # stale/missing artifact must make the preflight not-ready and the
            # artifact_identity check fail (never ready).
            current_path = Path(tmp) / "artifacts/robot/tinker2/current.json"
            original_current = current_path.read_bytes()
            try:
                stale_id = "deadbeef00000000"
                current_path.write_bytes(_canonical_json({
                    "artifact_id": stale_id,
                    "manifest": "artifacts/robot/tinker2/{}/manifest.json".format(stale_id),
                }))
                stale = preflight_manifest(
                    root["manifest_path"], timeout=30.0, project_root=Path(tmp)
                )
                self.assertFalse(stale["ready"])
                stale_identity = next(
                    c for c in stale["checks"] if c["name"] == "artifact_identity"
                )
                self.assertFalse(stale_identity["ok"])
            finally:
                current_path.write_bytes(original_current)
            # Pinned-resolver independence is proven in a FRESH ISOLATED
            # SUBPROCESS (fix round 4).  The parent process pre-imports
            # ``tinker_sim_deploy`` from the live working tree, so in-process
            # ``sys.path`` surgery -- and therefore the previous dirty live-tree
            # edit probe -- is masked by Python's module cache and cannot prove
            # the reconstruction executes the pinned materialized resolver.  A
            # fresh ``-I`` child with no inherited ``tinker_sim_deploy`` cache
            # loads the resolver from the materialized temp root; a temp decoy
            # working-tree package proves the pinned path wins path precedence
            # (positive) and is load-bearing (negative).  The active checkout's
            # tracked files are never written.
            self._prove_pinned_resolver_in_fresh_subprocess(root, data)
        # Pinned-blob mismatch/missing fails closed: a nonexistent path at the
        # recorded identity raises, and a wrong-but-real pin materializes bytes
        # that differ from the recorded resolver (a changed pin is detected).
        sim_identity = str(
            self._load_contract()["repositories"]["simulator"]["implementation_identity"]
        )
        with self.assertRaises(AssertionError):
            _git_blob(ROOT, sim_identity, "tools/tinker_sim_deploy/definitely-not-a-file.py")
        with tempfile.TemporaryDirectory(prefix="mb-bogus-") as tmp3:
            with self.assertRaises(AssertionError):
                self._materialize_tinker_sim_deploy(Path(tmp3), commit="0" * 40)
        with tempfile.TemporaryDirectory(prefix="mb-mispin-") as tmp4:
            self._materialize_tinker_sim_deploy(
                Path(tmp4), commit="1b31c5db03df069d474eaa4ea0ba756debfaa234"
            )
            self.assertNotEqual(
                (Path(tmp4) / "tools/tinker_sim_deploy/runtime.py").read_bytes(),
                _git_blob(ROOT, sim_identity, "tools/tinker_sim_deploy/runtime.py"),
                "a wrong pin must not silently reproduce the recorded resolver bytes",
            )

    def test_current_artifact_matches_reproducible_derivation(self) -> None:
        # Provisioned-host selector guard: when the real current.json exists, it
        # must select bytes equal to the reproducible derivation (stale selection
        # fails).  When it is absent the host is reported not_provisioned as a
        # runtime-readiness diagnostic -- this is not a static acceptance failure.
        if not self.CURRENT_JSON.is_file():
            self.skipTest(
                "host is not provisioned: artifacts/robot/tinker2/current.json is absent; "
                "this is a host-runtime readiness diagnostic (the host is not runtime-ready "
                "until it is provisioned), not a static acceptance failure"
            )
        current = json.loads(self.CURRENT_JSON.read_text(encoding="utf-8"))
        artifact_id = str(current["artifact_id"])
        selected = ROOT / "artifacts/robot/tinker2" / artifact_id / "robot.urdf"
        if not selected.is_file():
            self.fail("selected artifact generation missing; stale-current guard cannot verify")
        self.assertEqual(
            _sha256_file(selected), _SIMULATOR_FULL_URDF_OUTPUT,
            "current.json selects an artifact that does not match the reproducible derivation (stale selection)",
        )
        # The raw on-disk manifest hash remains recorded, but is labeled host-scoped.
        if self.MODEL_BUNDLE_PATH.is_file():
            data = self._load_contract()["model_bundle"]
            self.assertEqual(data["manifest_sha256"], self._sha256_file(self.MODEL_BUNDLE_PATH))
            self.assertIn("not reproducible", str(data["manifest_sha256_scope"]).lower())
        # Top-level evidence must agree with the model_bundle stable hashes.
        ev = self._load_contract()["evidence"]["preflight"]
        mb = self._load_contract()["model_bundle"]
        self.assertEqual(ev["stable_manifest_sha256"], mb["stable_manifest_sha256"])
        self.assertEqual(ev["stable_preflight_sha256"], mb["preflight_report"]["stable_sha256"])

    def test_top_level_evidence_stable_hashes_reproducible(self) -> None:
        """Stable hashes are the load-bearing top-level acceptance evidence."""
        ev = self._load_contract()["evidence"]["preflight"]
        mb = self._load_contract()["model_bundle"]
        self.assertEqual(ev["stable_manifest_sha256"], mb["stable_manifest_sha256"])
        self.assertEqual(ev["stable_preflight_sha256"], mb["preflight_report"]["stable_sha256"])
        # Recompute both independently from a self-contained reconstruction
        # (clean-checkout reproducible, no provisioned-host dependency).
        with tempfile.TemporaryDirectory(prefix="mb-ev-") as tmp:
            root = self._reconstruct_project_root(Path(tmp))
            self.assertEqual(
                _sha256_json(stable_manifest_evidence(root["manifest"])),
                ev["stable_manifest_sha256"],
            )
            result = preflight_manifest(root["manifest_path"], timeout=30.0, project_root=root["tmp"])
            self.assertEqual(
                _sha256_json(stable_preflight_evidence(result)),
                ev["stable_preflight_sha256"],
            )
        # Raw hashes are removed from the load-bearing surface and nested under an
        # explicitly non-load-bearing host snapshot with a stated scope.
        self.assertNotIn("model_bundle_manifest_sha256", ev)
        self.assertNotIn("preflight_report_sha256", ev)
        snapshot = ev["host_snapshot"]
        self.assertIn("not reproducible", str(snapshot["scope"]).lower())
        # The real current.json selector is a provisioned-host runtime-readiness
        # diagnostic, distinct from the static acceptance gate.
        selector = ev["current_selector"]
        self.assertIn("provisioned", str(selector["role"]).lower())
        self.assertIn("not runtime-ready", str(selector["scope"]).lower())

    def test_joint_limits_semantic_identity_toolchain_aware(self) -> None:
        data = self._load_contract()["model_bundle"]
        # Serialization policy is recorded (emitter + role of the raw YAML bytes).
        policy = data["serialization_policy"]
        self.assertIn("yaml.safe_dump", policy["emitter"])
        self.assertIn("joint_limits_semantic_sha256", policy["role"])
        # The semantic joint-limit identity is independent of YAML formatting and
        # is recomputed from the reconstructed manifest's canonical mapping.
        with tempfile.TemporaryDirectory(prefix="jl-sem-") as tmp:
            manifest = self._reconstruct_manifest(Path(tmp))
            self.assertEqual(
                _sha256_json(manifest["contract"]["joint_limits"]),
                data["joint_limits_semantic_sha256"],
            )
            # The raw YAML bytes remain the toolchain-scoped runtime artifact hash.
            artifacts = self._reconstruct_artifacts(Path(tmp))
            self.assertEqual(
                _sha256_file(artifacts["joint_limits"]),
                data["artifacts"]["joint_limits"]["sha256"],
            )
        # A changed limit is caught by the semantic identity (mutation seam).
        mutated = self._mutate_contract(
            lambda d: d["model_bundle"]["semantic_contract"]["joint_limits"]["joint2"].update(
                {"yaml": {"max_position": 9.0}}
            )
        )
        self.assertNotEqual(
            _sha256_json(mutated["model_bundle"]["semantic_contract"]["joint_limits"]),
            data["joint_limits_semantic_sha256"],
        )

    def test_model_artifact_path_relative_policy(self) -> None:
        """Every model artifact path_relative follows the reconstructed
        manifest/source policy, not only its content hash."""
        data = self._load_contract()["model_bundle"]
        psc = data["production_source_commits"]
        artifacts = data["artifacts"]
        for name, repo in (
            ("planning_urdf", "tk25_basic"),
            ("planning_srdf", "tk25_basic"),
            ("kinematics", "tk25_manipulation"),
        ):
            # Production-sourced artifacts use the workspace-relative source path.
            self.assertEqual(
                artifacts[name]["path_relative"],
                "src/{}/{}".format(repo, psc[name]["path_relative"]),
            )
            # The pinned production blob hash is recomputed from the recorded commit.
            repo_path = self.PRODUCTION_BASIC_REPO if repo == "tk25_basic" else self.PRODUCTION_REPO
            blob = _git_blob(repo_path, str(psc[name]["commit"]), str(psc[name]["path_relative"]))
            self.assertEqual(hashlib.sha256(blob).hexdigest(), psc[name]["sha256"])
        # Synthesized joint limits live under the provisioned outputs tree.
        self.assertEqual(
            artifacts["joint_limits"]["path_relative"],
            "outputs/ompl-overlay/model-bundle-r2/joint_limits.yaml",
        )
        # The simulator full URDF is the selected current artifact.
        self.assertEqual(
            artifacts["simulator_full_urdf"]["path_relative"],
            data["source_evidence"]["simulator_full_urdf"]["selected_artifact_path_relative"],
        )

    # ------------------------------------------------------------------
    # production overlay -- recomputed from pinned production git objects
    # ------------------------------------------------------------------
    def test_contract_production_18_argument_contract_recomputed(self) -> None:
        data = self._load_contract()["production_overlay"]
        self.assertEqual(data["package"], "mobile_bringup")
        self.assertEqual(data["launch_file"], "manipulation_planning_task_only.launch.py")
        self.assertEqual(
            data["launch_path_relative"],
            "src/mobile_bringup/launch/manipulation_planning_task_only.launch.py",
        )
        # The exact ordered DeclareLaunchArgument names are recomputed from the
        # pinned production launch file, not taken from the contract itself.
        self.assertEqual(self._production_launch_arguments(), _EXPECTED_18_LAUNCH_ARGS)
        self.assertEqual(data["launch_arguments"], self._production_launch_arguments())
        self.assertEqual(len(data["launch_arguments"]), 18)
        # The recorded launch file must exist in the production tree at the pinned commit.
        blob = _git_blob(
            self.PRODUCTION_REPO, _TK25_MANIP_COMMIT,
            "src/mobile_bringup/launch/manipulation_planning_task_only.launch.py",
        )
        self.assertGreater(len(blob), 0)

    def test_contract_literal_false_compatibility_values_recomputed(self) -> None:
        data = self._load_contract()["production_overlay"]
        compat = data["sim_compatibility_parameters_literal_false"]
        # The literal-false sim_ompl block is parsed from the pinned production launch.
        self.assertEqual(set(compat), set(_EXPECTED_SIM_COMPAT_KEYS))
        for key in _EXPECTED_SIM_COMPAT_KEYS:
            self.assertIs(compat[key], False)
            self.assertIs(type(compat[key]), bool)
        # Recomputed from source must agree.
        recomputed = self._production_sim_ompl_literal_false()
        self.assertEqual(set(recomputed), set(_EXPECTED_SIM_COMPAT_KEYS))
        for key in _EXPECTED_SIM_COMPAT_KEYS:
            self.assertIs(recomputed[key], False)

    def test_contract_literal_true_and_strict_inputs_recomputed(self) -> None:
        data = self._load_contract()["production_overlay"]
        true_params = data["sim_task_parameters_literal_true"]
        self.assertEqual(
            true_params,
            {
                "execution_profile": "sim_ompl",
                "fixture_revision_required": True,
                "safety_required": True,
                "use_sim_time": True,
            },
        )
        for value in true_params.values():
            if isinstance(value, bool):
                self.assertIs(value, True)
        # strict_sim_inputs keys recomputed from the pinned launch.
        recomputed_strict = self._production_strict_sim_inputs_keys()
        self.assertEqual(recomputed_strict, _EXPECTED_STRICT_SIM_INPUTS)
        self.assertEqual(data["strict_sim_inputs"], _EXPECTED_STRICT_SIM_INPUTS)
        # The strict keys are exactly the six scenario/identity/fixture inputs and
        # are separate from the literal-true task parameters.
        self.assertEqual(
            set(data["strict_sim_inputs"]) & set(true_params),
            set(),
        )
        # Execution profiles come from the pinned launch argument choices.
        launch = self._production_launch_source()
        self.assertIn("choices=['hardware', 'sim_ompl']", launch)

    def test_contract_production_allowlists_recomputed(self) -> None:
        data = self._load_contract()["production_overlay"]["production_allowlists"]
        helpers = self._production_helpers_source()
        self.assertEqual(
            set(data["import_allowlist"]), self._parse_frozenset(helpers, "ALLOWED_TOP_LEVEL_IMPORTS")
        )
        self.assertEqual(
            set(data["import_allowlist"]), _EXPECTED_PRODUCTION_IMPORTS,
            "production import allowlist must include re and importlib and match the enforced list",
        )
        self.assertEqual(
            set(data["node_packages"]), self._parse_frozenset(helpers, "ALLOWED_NODE_PACKAGES")
        )
        self.assertEqual(
            set(data["node_packages"]), _EXPECTED_PRODUCTION_NODE_PACKAGES,
            "production node-package allowlist must include rviz2",
        )
        self.assertEqual(
            set(data["node_executables"]), self._parse_frozenset(helpers, "ALLOWED_NODE_EXECUTABLES")
        )
        self.assertEqual(
            set(data["node_executables"]), _EXPECTED_PRODUCTION_NODE_EXECUTABLES,
            "production node-executable allowlist must include rviz2",
        )
        self.assertEqual(
            set(data["controller_resources"]),
            self._parse_frozenset(helpers, "ALLOWED_CONTROLLER_RESOURCES"),
        )
        self.assertEqual(
            set(data["controller_resources"]), _EXPECTED_PRODUCTION_CONTROLLER_RESOURCES
        )
        self.assertEqual(
            tuple(data["sim_compat_keys"]), self._parse_tuple(helpers, "SIM_COMPAT_KEYS")
        )
        self.assertEqual(
            tuple(data["sim_compat_keys"]), _EXPECTED_SIM_COMPAT_KEYS
        )
        self.assertEqual(
            set(data["task_node_action_endpoints"]),
            set(self._parse_tuple(helpers, "TASK_NODE_ACTION_ENDPOINTS")),
        )
        self.assertEqual(
            set(data["task_node_action_endpoints"]), _EXPECTED_TASK_ACTION_ENDPOINTS
        )

    def test_contract_simulator_provider_set_derived_from_manifest(self) -> None:
        data = self._load_contract()["production_overlay"]["simulator_overlay_provider_set"]
        derived = self._simulator_overlay_provider_set()
        self.assertEqual(data["packages"], derived["packages"])
        self.assertEqual(data["executables"], derived["executables"])
        self.assertNotIn("rviz2", data["executables"])
        self.assertIn("derived_from", data)

    def test_contract_handoff_and_action_client_scope_from_pinned_source(self) -> None:
        data = self._load_contract()["production_overlay"]
        sources = self._handoff_constants_from_production()
        # The canonical task object id is owned/created by pick_and_place itself.
        self.assertIn("pick_and_place/object_mesh", sources["ownership"])
        self.assertIn("pick_and_place/object_mesh", sources["transaction"])
        self.assertIn("pick_and_place/object_mesh", sources["main"])
        # The fixture adapter never owns task objects (task_owned_lifecycle).
        lifecycle = str(data["task_owned_lifecycle"])
        self.assertIn("pick_and_place creates and owns", lifecycle)
        self.assertIn("fixture adapter never owns", lifecycle)
        # The smoke action-client restriction is grounded in the Task 7 source.
        ac = data["action_client_allowlist"]
        self.assertIn("/move_action", str(ac["smoke"]))

    def test_task7_action_client_scope_from_pinned_source(self) -> None:
        """Ground the Task 7 action-client restriction in pinned Task 7 source
        (AST-count the real ActionClient constructions at the recorded simulator
        implementation identity), not a membership self-check."""
        derived = self._task7_action_client_scope()
        self.assertEqual(derived["count"], 1)
        self.assertEqual(derived["action"], "/move_action")
        self.assertEqual(derived["type"], "MoveGroup")
        data = self._load_contract()["production_overlay"]["action_client_allowlist"]
        self.assertIn("/move_action", str(data["smoke"]))
        self.assertIn("only action client", str(data["smoke"]).lower())
        self.assertIn("MoveGroup", str(data["smoke"]))
        # No execute-trajectory / controller / task action client exists (exactly
        # one ActionClient call in the whole pinned smoke module).
        sim_identity = str(self._load_contract()["repositories"]["simulator"]["implementation_identity"])
        source = _git_blob(ROOT, sim_identity, "validation/ompl_plan_smoke.py").decode("utf-8")
        self.assertIn('MOVE_ACTION = "/move_action"', source)

    def test_fixture_status_publication_recomputed(self) -> None:
        """Recompute fixture_contract.status_publication from Task 5 source and
        constants (fields/topic/type/source/QoS/rate), with a mutation seam."""
        data = self._load_contract()["fixture_contract"]["status_publication"]
        status = canonical_fixture_status(
            scenario="sim_fixture/public_target",
            revision="2026-08-01-moveit-qualification-joint",
            revision_digest="0" * 64,
            sequence=1,
            published_at=1.0,
            owned_ids=["sim_fixture/pedestal", "sim_fixture/public_target"],
            target_source_id="sim_fixture/public_target",
            target_handoff="pick_and_place/object_mesh",
            descriptor_sha256="0" * 64,
            state="FIXTURE_READY",
        )
        # The contract field list is asserted against the independently
        # documented 12-field literal (canonical order/encoding), NOT re-derived
        # from the function output, so a coordinated source+contract change to
        # the key set is detected.
        self.assertEqual(list(data["fields"]), _EXPECTED_FIXTURE_STATUS_FIELDS)
        # Source-shape validation: the Task 5 function must itself emit exactly
        # this key set in this order, so a source change fails even if the
        # contract changed in lockstep.
        self.assertEqual(list(status), _FIXTURE_STATUS_FIELD_KEYS)
        self.assertEqual(status["schema_version"], 1)
        self.assertEqual(status["owner"], "sim_fixture")
        # topic/type/source/qos/rate grounded in the Task 5 node source + launch.
        node_src = (
            self.BRIDGE_DIR / "tinker_sim_bridge/fixture_planning_scene_node.py"
        ).read_text(encoding="utf-8")
        self.assertEqual(data["topic"], "/sim/status/planning_scene_fixture")
        self.assertIn('_STATUS_TOPIC = "/sim/status/planning_scene_fixture"', node_src)
        self.assertEqual(data["type"], "std_msgs/msg/String")
        self.assertIn("from std_msgs.msg import String", node_src)
        self.assertEqual(data["source"], "/fixture_planning_scene")
        self.assertEqual(data["qos"], {"depth": 1, "durability": "TRANSIENT_LOCAL", "reliability": "RELIABLE"})
        self.assertIn("depth=1", node_src)
        self.assertIn("ReliabilityPolicy.RELIABLE", node_src)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", node_src)
        self.assertEqual(data["rate_hz"], 5)
        self.assertIn('declare_parameter("heartbeat_period", 0.2)', node_src)
        launch_src = (
            self.BRIDGE_DIR / "launch/integrated_ompl_manipulation.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn('name="fixture_planning_scene"', launch_src)

    # ------------------------------------------------------------------
    # scenario package-share fallback (Important 3)
    # ------------------------------------------------------------------
    def test_scenario_resolver_package_share_fallback(self) -> None:
        with tempfile.TemporaryDirectory(prefix="scen-fb-") as tmp:
            root = Path(tmp) / "root"
            share = Path(tmp) / "share"
            (share / "scenarios").mkdir(parents=True)
            scenario = "qualification-moveit-plan-joint"
            # Copy the real scenario into the share, source tree absent.
            installed = share / "scenarios" / f"{scenario}.json"
            source_scenario = self.SCENARIO_DIR / f"{scenario}.json"
            installed.write_bytes(source_scenario.read_bytes())
            resolved = resolve_scenario_file(root, scenario, share)
            self.assertEqual(resolved, installed)
            self.assertEqual(resolved.read_bytes(), source_scenario.read_bytes())
        # Unsafe ids and missing scenarios fail closed.
        with tempfile.TemporaryDirectory(prefix="scen-fb-") as tmp:
            share = Path(tmp) / "share"
            (share / "scenarios").mkdir(parents=True)
            with self.assertRaises(ScenarioResolutionError):
                resolve_scenario_file(Path(tmp), "../evil", share)
            with self.assertRaises(ScenarioResolutionError):
                resolve_scenario_file(Path(tmp), "qualification-moveit-plan-joint", share)

    def test_scenario_resolver_rejects_byte_disagreement(self) -> None:
        with tempfile.TemporaryDirectory(prefix="scen-fb-") as tmp:
            root = Path(tmp) / "root"
            share = Path(tmp) / "share"
            (share / "scenarios").mkdir(parents=True)
            (root / "simulation/scenarios").mkdir(parents=True)
            scenario = "qualification-moveit-plan-joint"
            (share / "scenarios" / f"{scenario}.json").write_text("{}")
            (root / "simulation/scenarios" / f"{scenario}.json").write_text("{DIFFERENT}")
            with self.assertRaises(ScenarioResolutionError) as ctx:
                resolve_scenario_file(root, scenario, share)
            self.assertIn("differs", str(ctx.exception))

    # ------------------------------------------------------------------
    # build commands / docs
    # ------------------------------------------------------------------
    def test_contract_build_commands_never_raw_colcon(self) -> None:
        data = self._load_contract()["build_commands"]
        self.assertIn("tkbuild", data["production"])
        self.assertIn("-j2 -l2", data["production"])
        self.assertIn("--parallel-workers 2", data["production"])
        self.assertIn("build-humble-overlay", data["simulator"])
        self.assertIn("-j2 -l2", data["simulator"])
        self.assertIn("TINKER_WS=/home/tinker/tk25_ws", data["simulator"])
        self.assertIn("never raw colcon", data["policy"].lower())

    def test_docs_never_publish_raw_colcon_build(self) -> None:
        for doc in (
            ROOT / "README.md",
            ROOT / "docs/acceptance.md",
            ROOT / "integration/MANIPULATION.md",
            self.BRIDGE_DIR / "README.md",
        ):
            text = doc.read_text(encoding="utf-8")
            self.assertNotIn("colcon build", text, f"{doc.relative_to(ROOT)} must not instruct raw colcon build")

    def test_docs_document_pytest_plugin_autoload_workaround(self) -> None:
        docs = (
            ROOT / "README.md",
            ROOT / "integration/MANIPULATION.md",
            self.BRIDGE_DIR / "README.md",
        )
        mentions = sum(
            1 for doc in docs if "PYTEST_DISABLE_PLUGIN_AUTOLOAD" in doc.read_text(encoding="utf-8")
        )
        self.assertGreaterEqual(mentions, 1, "at least one doc must document PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")

    # ------------------------------------------------------------------
    # setup.py / package.xml / installed data (Important 3)
    # ------------------------------------------------------------------
    def test_setup_py_registers_all_required_assets(self) -> None:
        setup_text = self.SETUP_PY.read_text(encoding="utf-8")

        for launch_name in (
            "launch/fixtures.launch.py",
            "launch/integrated_ompl_manipulation.launch.py",
            "launch/manipulation.launch.py",
            "launch/navigation.launch.py",
            "launch/whole_robot.launch.py",
        ):
            self.assertIn('"' + launch_name + '"', setup_text)
            self.assertTrue((self.BRIDGE_DIR / launch_name).is_file())

        for config_name in (
            "config/base_facade.yaml",
            "config/command_gateway.yaml",
            "config/controllers.yaml",
            "config/pointcloud_to_laserscan.yaml",
            "config/tinker_topic_control.ros2_control.xacro",
        ):
            self.assertIn('"' + config_name + '"', setup_text)
            self.assertTrue((self.BRIDGE_DIR / config_name).is_file())

        self.assertIn('"integration/provider-manifest.json"', setup_text)
        self.assertTrue((self.BRIDGE_DIR / "integration/provider-manifest.json").is_file())

    def test_setup_py_registers_contract_and_scenarios(self) -> None:
        setup_text = self.SETUP_PY.read_text(encoding="utf-8")
        # The canonical contract is registered from the bridge integration/ dir,
        # which exposes the simulator-root canonical copy through a tracked
        # source symlink (no second authoritative copy).
        self.assertIn('"integration/ompl-overlay-contract.json"', setup_text)
        self.assertTrue(self.CONTRACT_PATH.is_file())
        self.assertTrue((self.BRIDGE_DIR / "integration/ompl-overlay-contract.json").is_symlink())
        # Scenario sources are registered under share/tinker_sim_bridge/scenarios/
        # through tracked source symlinks to the canonical checkout copies.
        self.assertIn('share/" + package_name + "/scenarios', setup_text)
        self.assertIn("_scenario_sources", setup_text)
        for scenario in self.SCENARIOS:
            self.assertIn(scenario, setup_text)
            self.assertTrue((self.SCENARIO_DIR / f"{scenario}.json").is_file())
            self.assertTrue((self.BRIDGE_DIR / "scenarios" / f"{scenario}.json").is_symlink())
            # The tracked symlink resolves to the canonical source bytes.
            self.assertEqual(
                _sha256_file(self.BRIDGE_DIR / "scenarios" / f"{scenario}.json"),
                _sha256_file(self.SCENARIO_DIR / f"{scenario}.json"),
            )

    def test_setup_py_registers_every_console_script_entrypoint(self) -> None:
        setup_text = self.SETUP_PY.read_text(encoding="utf-8")
        modules = sorted(
            path.stem
            for path in (self.BRIDGE_DIR / "tinker_sim_bridge").glob("*.py")
        )
        registered_targets = []
        for line in setup_text.splitlines():
            line = line.strip().rstrip(",").strip()
            if line.startswith('"') and "= tinker_sim_bridge." in line and ":main" in line:
                target = line.split("=", 1)[1].strip().rstrip('",')
                registered_targets.append(target)
        for module in modules:
            source = (
                self.BRIDGE_DIR / "tinker_sim_bridge" / f"{module}.py"
            ).read_text(encoding="utf-8")
            if "def main(" not in source:
                continue
            self.assertIn(
                "tinker_sim_bridge.{}:main".format(module),
                registered_targets,
                f"no console-script entrypoint targets module {module}",
            )

    def test_package_xml_declares_direct_dependencies(self) -> None:
        xml = self.PACKAGE_XML.read_text(encoding="utf-8")
        required = [
            "control_msgs",
            "controller_manager_msgs",
            "geometry_msgs",
            "moveit_msgs",
            "shape_msgs",
            "tinker_arm_msgs",
            "simulation_interfaces",
            "tinker_sim_interfaces",
            "tinker_audio_msgs",
            "tinker_vision_msgs_26",
            "xarm_msgs",
            "mobile_bringup",
            "pick_and_place",
            "moveit_ros_move_group",
            "controller_manager",
            "robot_state_publisher",
            "std_srvs",
            "sensor_msgs",
            "std_msgs",
            "nav_msgs",
            "tf2_msgs",
            "tf2_ros",
            "rcl_interfaces",
            "rclpy",
            "robot_localization",
        ]
        for dep in required:
            self.assertIn(f">{dep}<", xml, f"package.xml missing direct dependency {dep}")

    def test_installed_package_data_byte_identity(self) -> None:
        install = ROOT / "ros2_ws/install/tinker_sim_bridge"
        if not install.is_dir():
            self.skipTest(
                "install prefix not present; the portable copy-install/wheel gate is "
                "test_copy_install_wheel_data_byte_identity, and this real colcon "
                "installed-prefix check is supplemental -- run the bounded build "
                "MAKEFLAGS='-j2 -l2' TINKER_WS=/home/tinker/tk25_ws ./scripts/build-humble-overlay"
            )
        self._assert_installed_byte_identity(install)

    def test_copy_install_wheel_data_byte_identity(self) -> None:
        """Prove the symlinked acceptance contract and scenario data become real
        byte-identical installed files under a non-symlink copy install.

        Builds a wheel (``bdist_wheel``) from a clean ``git clone`` of the repo
        into a temp ``--dist-dir`` -- no global Python install, no raw colcon --
        then compares the wheel data bytes to the canonical source bytes and
        verifies the package-share scenario fallback resolves from the copied
        install with the source tree absent.
        """
        with tempfile.TemporaryDirectory(prefix="t8-wheel-") as tmp:
            root = Path(tmp) / "co"
            clone = subprocess.run(
                ["git", "clone", "--local", "--quiet", "--no-hardlinks", str(ROOT), str(root)],
                capture_output=True,
            )
            self.assertEqual(clone.returncode, 0, clone.stderr.decode())
            wheels = Path(tmp) / "wheels"
            wheels.mkdir()
            build = subprocess.run(
                [sys.executable, "setup.py", "bdist_wheel", "--dist-dir", str(wheels)],
                cwd=str(root / "ros2_ws/src/tinker_sim_bridge"),
                capture_output=True,
            )
            self.assertEqual(
                build.returncode, 0,
                "wheel build failed:\n" + build.stdout.decode(errors="replace")
                + build.stderr.decode(errors="replace"),
            )
            whls = list(wheels.glob("*.whl"))
            self.assertEqual(len(whls), 1)
            with zipfile.ZipFile(whls[0]) as wheel:
                entries = wheel.namelist()
                prefix = next(
                    name[: name.index("/integration/ompl-overlay-contract.json")]
                    for name in entries
                    if name.endswith("/integration/ompl-overlay-contract.json")
                )
                # The symlinked contract is copied as real bytes into the wheel.
                contract_installed = wheel.read(
                    prefix + "/integration/ompl-overlay-contract.json"
                )
                self.assertEqual(
                    hashlib.sha256(contract_installed).hexdigest(),
                    _sha256_file(root / "integration/ompl-overlay-contract.json"),
                )
                for scenario in self.SCENARIOS:
                    installed = wheel.read(
                        prefix + "/scenarios/{}.json".format(scenario)
                    )
                    self.assertEqual(
                        hashlib.sha256(installed).hexdigest(),
                        _sha256_file(root / "simulation/scenarios" / f"{scenario}.json"),
                    )
                # Package-share scenario fallback from the copied install: source
                # tree absent, installed share resolves the real scenario bytes.
                share = Path(tmp) / "share"
                (share / "scenarios").mkdir(parents=True)
                for scenario in self.SCENARIOS:
                    (share / "scenarios" / f"{scenario}.json").write_bytes(
                        wheel.read(prefix + "/scenarios/{}.json".format(scenario))
                    )
            resolved = resolve_scenario_file(
                Path(tmp) / "absent-root", "qualification-moveit-plan-joint", share
            )
            self.assertEqual(
                resolved.read_bytes(),
                (root / "simulation/scenarios/qualification-moveit-plan-joint.json").read_bytes(),
            )

    def test_clean_checkout_static_acceptance_seam(self) -> None:
        """Clean-checkout regression seam: a fresh tracked-only clone (no
        gitignored outputs/artifacts/install) must run the exact Task 8 class.

        The seam locks the *executed suite and skip set*, not just the exit
        code: it collects the class in the clone and requires exactly the 64
        canonical node IDs (canonical SHA-256 over the sorted node list), then
        executes the class under a machine-readable pytest JUnit XML and asserts
        total=64, failures=0, errors=0, skipped=4, the exact four legitimate
        host-runtime diagnostic skips, and their reason categories -- so a
        deleted test, an accidental broad skip, or a collection/import error
        all fail closed.  Both the collection gate and the JUnit structure are
        validated by the single deterministic ``_check_collection_acceptance``
        helper (fix round 4), and realistic mutated fixtures are fed through
        that same validator and must each be rejected: delete one node, a
        rename/delete+add substitution, an unrelated skip, a removed expected
        skip, a wrong skip reason, a duplicate testcase, a failure/error count,
        and multiple suites.

        Uses a temporary ``git clone`` (a temporary checkout/copy); never
        mutates the active checkout.  The nested invocation is skipped via
        ``T8_SEAM_ACTIVE`` so the seam cannot recurse.
        """
        if os.environ.get("T8_SEAM_ACTIVE") == "1":
            self.skipTest("nested clean-checkout seam invocation")
        with tempfile.TemporaryDirectory(prefix="t8-seam-") as tmp:
            root = Path(tmp) / "co"
            clone = subprocess.run(
                ["git", "clone", "--local", "--quiet", "--no-hardlinks", str(ROOT), str(root)],
                capture_output=True,
            )
            self.assertEqual(clone.returncode, 0, clone.stderr.decode())
            # A fresh clone has none of the gitignored runtime trees.
            self.assertFalse((root / "outputs").exists())
            self.assertFalse((root / "artifacts").exists())
            self.assertFalse((root / "ros2_ws/install").exists())
            env = os.environ.copy()
            env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
            env["T8_SEAM_ACTIVE"] = "1"
            # 1. Collection gate: exactly the canonical 64-node class, in the
            # clone.  A deleted/renamed/added test, or a collection/import
            # error, fails here (collection errors exit non-zero and the count
            # + node-set hash are asserted, not just the exit code).
            collection = subprocess.run(
                [
                    sys.executable, "-m", "pytest", "--collect-only", "-q",
                    "-p", "no:cacheprovider",
                    "tests/test_provenance.py::Task8OMPLOverlayProvenanceTest",
                ],
                cwd=str(root),
                env=env,
                capture_output=True,
            )
            self.assertEqual(
                collection.returncode, 0,
                "collection failed in the clean checkout:\n"
                + collection.stdout.decode(errors="replace")
                + collection.stderr.decode(errors="replace"),
            )
            collected = [
                line.strip()
                for line in collection.stdout.decode(errors="replace").splitlines()
                if line.strip().startswith("tests/test_provenance.py::Task8OMPLOverlayProvenanceTest::")
            ]
            # 2. Machine-readable execution: JUnit XML, not tail parsing.
            junit = Path(tmp) / "junit.xml"
            result = subprocess.run(
                [
                    sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                    "--junitxml={}".format(junit),
                    "tests/test_provenance.py::Task8OMPLOverlayProvenanceTest",
                ],
                cwd=str(root),
                env=env,
                capture_output=True,
            )
            self.assertEqual(
                result.returncode, 0,
                "Task 8 static acceptance must pass on a clean tracked-only checkout "
                "(no gitignored outputs/artifacts/install); only the provisioned-host "
                "runtime diagnostics may skip.\n"
                + result.stdout.decode(errors="replace")
                + result.stderr.decode(errors="replace"),
            )
            tree = ET.parse(str(junit))
            # 3. The deterministic acceptance validator locks the collection gate
            # (exact 64 node IDs + canonical node-set SHA-256, so delete and
            # rename+add substitutions both fail), the JUnit structure (exactly
            # one <testsuite>, suite counters, 64 unique <testcase> children, no
            # unexpected failure/error/xfail statuses), and the exact four
            # host-runtime diagnostic skip identities with their reason
            # categories.  The SAME validator is used on the real clone output
            # here and on the mutated fixtures in step 4, so the negative
            # assertions exercise the exact acceptance code path.
            _check_collection_acceptance(collected, tree)
            # 4. Inline mutation cases feed realistic mutated fixtures through
            # the same validator and must each be rejected (not trivially-true
            # direct set comparisons): delete one node; rename/delete+add while
            # preserving count; add an unrelated skip; remove an expected skip;
            # wrong skip reason; duplicate testcase; failure/error count;
            # multiple suites.
            with self.assertRaises(AssertionError):
                _check_collection_acceptance(collected[:-1], tree)
            renamed = list(collected)
            renamed[0] = (
                "tests/test_provenance.py::Task8OMPLOverlayProvenanceTest::test_fabricated_renamed_node"
            )
            with self.assertRaises(AssertionError):
                _check_collection_acceptance(renamed, tree)
            # c) broaden a passing test into an unrelated skip (count preserved).
            tree_extra_skip = copy.deepcopy(tree)
            passing_case = next(
                c for s in tree_extra_skip.iter("testsuite")
                for c in s if c.tag == "testcase" and c.find("skipped") is None
            )
            passing_case.append(
                ET.Element("skipped", {"message": "unrelated broadened skip"})
            )
            with self.assertRaises(AssertionError):
                _check_collection_acceptance(collected, tree_extra_skip)
            # d) remove one legitimate diagnostic skip.
            tree_removed_skip = copy.deepcopy(tree)
            removed_case = next(
                c for s in tree_removed_skip.iter("testsuite")
                for c in s if c.tag == "testcase" and c.find("skipped") is not None
            )
            removed_case.remove(removed_case.find("skipped"))
            with self.assertRaises(AssertionError):
                _check_collection_acceptance(collected, tree_removed_skip)
            # e) wrong skip reason on one diagnostic.
            tree_wrong_reason = copy.deepcopy(tree)
            for s in tree_wrong_reason.iter("testsuite"):
                for c in s:
                    if c.tag == "testcase":
                        skip_node = c.find("skipped")
                        if skip_node is not None:
                            skip_node.set("message", "a completely unrelated skip reason")
            with self.assertRaises(AssertionError):
                _check_collection_acceptance(collected, tree_wrong_reason)
            # f) duplicate testcase element.
            tree_dup = copy.deepcopy(tree)
            suite_el = next(iter(tree_dup.iter("testsuite")))
            suite_el.append(copy.deepcopy(next(c for c in suite_el if c.tag == "testcase")))
            with self.assertRaises(AssertionError):
                _check_collection_acceptance(collected, tree_dup)
            # g) failure/error count.
            tree_failure = copy.deepcopy(tree)
            next(iter(tree_failure.iter("testsuite"))).set("failures", "1")
            with self.assertRaises(AssertionError):
                _check_collection_acceptance(collected, tree_failure)
            # h) multiple suites must be rejected, not summed.
            tree_multi = copy.deepcopy(tree)
            testsuites_el = next(iter(tree_multi.iter("testsuites")))
            testsuites_el.append(copy.deepcopy(next(iter(tree_multi.iter("testsuite")))))
            with self.assertRaises(AssertionError):
                _check_collection_acceptance(collected, tree_multi)

    def _assert_installed_byte_identity(self, install: Path) -> None:
        pairs = [
            (self.BRIDGE_DIR / "integration/ompl-overlay-contract.json", install / "share/tinker_sim_bridge/integration/ompl-overlay-contract.json"),
            (self.BRIDGE_DIR / "integration/provider-manifest.json", install / "share/tinker_sim_bridge/integration/provider-manifest.json"),
        ]
        for scenario in self.SCENARIOS:
            pairs.append(
                (self.SCENARIO_DIR / f"{scenario}.json", install / "share/tinker_sim_bridge/scenarios" / f"{scenario}.json")
            )
        for source, installed in pairs:
            self.assertTrue(installed.is_file(), f"installed asset missing: {installed}")
            self.assertEqual(
                _sha256_file(installed), _sha256_file(source),
                f"installed bytes differ from source for {installed}",
            )

    # ------------------------------------------------------------------
    # source-lock exclusion
    # ------------------------------------------------------------------
    def test_source_locks_not_prematurely_included(self) -> None:
        data = self._load_contract()
        locks = data["source_locks"]
        self.assertEqual(locks["status"], "excluded_in_task_8")
        self.assertEqual(locks["simulator_lock_path"], "integration/source-locks.json")
        self.assertEqual(
            locks["production_lock_path"],
            "/home/tinker/tk25_ws/src/tk25_manipulation/integration/source-locks.json",
        )
        self.assertTrue(self.SIMULATOR_LOCK.is_file())
        diff = self._git(ROOT, "diff", "--quiet", "--", "integration/source-locks.json")
        self.assertEqual(diff.returncode, 0, "Task 8 must not modify the simulator source-lock file")

    def test_contract_declares_source_lock_exclusion_note(self) -> None:
        data = self._load_contract()
        note = str(data["source_locks"]["note"])
        self.assertIn("Task 9", note)
        self.assertIn("excluded", data["source_locks"]["status"])

    # ------------------------------------------------------------------
    # mutation resistance -- every guard is load-bearing, not tautological
    # ------------------------------------------------------------------
    def _mutate_contract(self, mutate) -> dict[str, object]:
        data = self._load_contract()
        mutate(data)
        return data

    def test_mutation_arg_order_count_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["production_overlay"]["launch_arguments"].reverse()
        )
        self.assertNotEqual(
            mutated["production_overlay"]["launch_arguments"], _EXPECTED_18_LAUNCH_ARGS
        )
        # A dropped argument is also caught (count guard).
        mutated = self._mutate_contract(
            lambda d: d["production_overlay"]["launch_arguments"].pop()
        )
        self.assertEqual(len(mutated["production_overlay"]["launch_arguments"]), 17)

    def test_mutation_literal_false_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["production_overlay"]["sim_compatibility_parameters_literal_false"].update(
                {"use_cumotion_goalset": True}
            )
        )
        self.assertNotEqual(
            set(mutated["production_overlay"]["sim_compatibility_parameters_literal_false"].values()),
            {False, False, False, False},
        )

    def test_mutation_strict_keys_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["production_overlay"]["strict_sim_inputs"].append("required_scenario_seed")
        )
        self.assertNotEqual(
            mutated["production_overlay"]["strict_sim_inputs"], _EXPECTED_STRICT_SIM_INPUTS
        )

    def test_mutation_production_allowlist_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["production_overlay"]["production_allowlists"]["import_allowlist"].remove("re")
        )
        self.assertNotEqual(
            set(mutated["production_overlay"]["production_allowlists"]["import_allowlist"]),
            _EXPECTED_PRODUCTION_IMPORTS,
            "dropping re from the import allowlist must be detected",
        )
        mutated = self._mutate_contract(
            lambda d: d["production_overlay"]["production_allowlists"]["node_packages"].append("gazebo_ros")
        )
        self.assertNotEqual(
            set(mutated["production_overlay"]["production_allowlists"]["node_packages"]),
            _EXPECTED_PRODUCTION_NODE_PACKAGES,
        )

    def test_mutation_provider_set_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["production_overlay"]["simulator_overlay_provider_set"]["executables"].append("rviz2")
        )
        self.assertNotEqual(
            mutated["production_overlay"]["simulator_overlay_provider_set"]["executables"],
            self._simulator_overlay_provider_set()["executables"],
        )

    def test_mutation_handoff_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["production_overlay"].update({"task_owned_lifecycle": "the fixture adapter owns task objects"})
        )
        # The mutated lifecycle claim contradicts the pinned production source.
        self.assertNotIn("pick_and_place creates and owns", str(mutated["production_overlay"]["task_owned_lifecycle"]))
        sources = self._handoff_constants_from_production()
        self.assertIn("pick_and_place/object_mesh", sources["ownership"])
        # A guard recomputing the handoff from source would reject the mutation.
        self.assertNotEqual(
            str(mutated["production_overlay"]["task_owned_lifecycle"]),
            str(self._load_contract()["production_overlay"]["task_owned_lifecycle"]),
        )

    def test_mutation_simulator_full_urdf_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["model_bundle"]["artifacts"]["simulator_full_urdf"].update(
                {"sha256": "0" * 64}
            )
        )
        self.assertNotEqual(
            mutated["model_bundle"]["artifacts"]["simulator_full_urdf"]["sha256"],
            _SIMULATOR_FULL_URDF_OUTPUT,
        )

    def test_mutation_joint_limits_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["model_bundle"]["artifacts"]["joint_limits"].update(
                {"sha256": "0" * 64}
            )
        )
        with tempfile.TemporaryDirectory(prefix="mb-mut-") as tmp:
            artifacts = self._reconstruct_artifacts(Path(tmp))
            self.assertNotEqual(
                _sha256_file(artifacts["joint_limits"]),
                mutated["model_bundle"]["artifacts"]["joint_limits"]["sha256"],
            )

    def test_mutation_top_level_stable_preflight_hashes_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["evidence"]["preflight"].update({"stable_manifest_sha256": "0" * 64})
        )
        self.assertNotEqual(
            mutated["evidence"]["preflight"]["stable_manifest_sha256"],
            self._load_contract()["model_bundle"]["stable_manifest_sha256"],
        )
        with tempfile.TemporaryDirectory(prefix="mb-mut-") as tmp:
            manifest = self._reconstruct_manifest(Path(tmp))
            self.assertNotEqual(
                _sha256_json(stable_manifest_evidence(manifest)),
                mutated["evidence"]["preflight"]["stable_manifest_sha256"],
            )
        mutated = self._mutate_contract(
            lambda d: d["evidence"]["preflight"].update({"stable_preflight_sha256": "0" * 64})
        )
        self.assertNotEqual(
            mutated["evidence"]["preflight"]["stable_preflight_sha256"],
            self._load_contract()["model_bundle"]["preflight_report"]["stable_sha256"],
        )

    def test_mutation_task_range_boundary_detected(self) -> None:
        data = self._load_contract()
        tr = data["repositories"]["simulator"]["task_range"]
        # Moving start/end to a different 13-commit pair breaks the exact list.
        start = str(tr["start"])
        end = str(tr["end"])
        # Replace start with its parent: the range changes and no longer equals
        # the recorded Task 3-7 commit list.
        parent = self._git(ROOT, "rev-parse", "{}^".format(start)).stdout.strip()
        mutated = self._mutate_contract(
            lambda d: d["repositories"]["simulator"]["task_range"].update({"start": parent})
        )
        mut_tr = mutated["repositories"]["simulator"]["task_range"]
        commits = self._git(ROOT, "rev-list", "{}..{}".format(mut_tr["start"], mut_tr["end"])).stdout.split()
        self.assertNotEqual(commits, list(mut_tr["commits"]))
        self.assertNotEqual(commits, _TASK_RANGE_COMMITS)
        # Boundary subject mismatch is also caught.
        mutated = self._mutate_contract(
            lambda d: d["repositories"]["simulator"]["task_range"]["boundary_subjects"].update(
                {"end": "a different subject"}
            )
        )
        self.assertNotEqual(
            mutated["repositories"]["simulator"]["task_range"]["boundary_subjects"]["end"],
            _TASK_RANGE_END_SUBJECT,
        )

    def test_mutation_fixture_status_publication_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["fixture_contract"]["status_publication"].update({"rate_hz": 10})
        )
        self.assertNotEqual(
            mutated["fixture_contract"]["status_publication"]["rate_hz"],
            self._load_contract()["fixture_contract"]["status_publication"]["rate_hz"],
        )
        mutated = self._mutate_contract(
            lambda d: d["fixture_contract"]["status_publication"]["fields"].append("owner=someone")
        )
        self.assertNotEqual(
            mutated["fixture_contract"]["status_publication"]["fields"],
            self._load_contract()["fixture_contract"]["status_publication"]["fields"],
        )

    def test_mutation_action_client_scope_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["production_overlay"]["action_client_allowlist"].update(
                {"smoke": "a task action client on /pickup_action is also constructed"}
            )
        )
        # The pinned Task 7 source still constructs exactly one MoveGroup client.
        derived = self._task7_action_client_scope()
        self.assertEqual(derived["count"], 1)
        self.assertEqual(derived["action"], "/move_action")
        self.assertNotIn("only action client", str(mutated["production_overlay"]["action_client_allowlist"]["smoke"]).lower())

    def test_mutation_artifact_path_relative_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["model_bundle"]["artifacts"]["planning_urdf"].update(
                {"path_relative": "src/other_repo/config/xarm7.urdf"}
            )
        )
        data = self._load_contract()["model_bundle"]
        self.assertNotEqual(
            mutated["model_bundle"]["artifacts"]["planning_urdf"]["path_relative"],
            "src/{}/{}".format(
                "tk25_basic",
                data["production_source_commits"]["planning_urdf"]["path_relative"],
            ),
        )

    def test_mutation_evidence_preflight_ready_detected(self) -> None:
        mutated = self._mutate_contract(
            lambda d: d["evidence"]["preflight"].update({"ready": False})
        )
        # The self-contained reconstruction proves ready=true on a clean checkout.
        with tempfile.TemporaryDirectory(prefix="mb-mut-") as tmp:
            root = self._reconstruct_project_root(Path(tmp))
            result = preflight_manifest(root["manifest_path"], timeout=30.0, project_root=root["tmp"])
            self.assertTrue(result["ready"])
            self.assertEqual(
                _sha256_json(stable_preflight_evidence(result)),
                self._load_contract()["model_bundle"]["preflight_report"]["stable_sha256"],
            )
        self.assertIsNot(
            mutated["evidence"]["preflight"]["ready"],
            self._load_contract()["evidence"]["preflight"]["ready"],
        )

    def test_missing_source_evidence_fails(self) -> None:
        # If the committed source-evidence copy is removed, the derivation must
        # fail with a clear error (never silently skip).  We simulate the missing
        # source by temporarily relocating the committed file.
        if not self.SOURCE_EVIDENCE_URDF.is_file():
            self.fail("source evidence is missing; the clean-checkout guard is already exercised")
        backup = self.SOURCE_EVIDENCE_URDF.with_suffix(".urdf.bak-fix1")
        os.rename(self.SOURCE_EVIDENCE_URDF, backup)
        try:
            with tempfile.TemporaryDirectory(prefix="mb-missing-") as tmp:
                with self.assertRaises(AssertionError) as ctx:
                    self._reconstruct_simulator_full_urdf(Path(tmp))
                self.assertIn("source evidence is missing", str(ctx.exception))
        finally:
            os.rename(backup, self.SOURCE_EVIDENCE_URDF)
        # Restored: the derivation reproduces the canonical output again.
        self.assertEqual(
            _sha256_file(self.SOURCE_EVIDENCE_URDF), _SIMULATOR_FULL_URDF_SOURCE_SHA
        )


if __name__ == "__main__":
    unittest.main()
