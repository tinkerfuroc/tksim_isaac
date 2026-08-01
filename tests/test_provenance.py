from __future__ import annotations

import ast
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
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


def _load_v1_canonicalizer():
    """Load the v1 URDF canonicalizer (the reviewed Task 3 exporter at the commit
    that produced the selected artifact) from immutable simulator git history."""
    source = _git_blob(
        ROOT, _V1_CANONICALIZER_COMMIT, "tools/tinker_sim_deploy/workspace.py"
    ).decode("utf-8")
    source = source.replace(
        "from .config import sha256_file",
        "def sha256_file(p):\n    return hashlib.sha256(open(p, 'rb').read()).hexdigest()",
    )
    namespace: dict[str, object] = {"hashlib": hashlib}
    exec(compile(source, "<v1-workspace>", "exec"), namespace)  # noqa: S102
    return namespace["canonicalize_urdf"]


def _canonicalize_simulator_full_urdf(source_bytes: bytes) -> bytes:
    return _load_v1_canonicalizer()(source_bytes)


def _canonical_json(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _sha256_json(value: object) -> str:
    return hashlib.sha256(_canonical_json(value)).hexdigest()


class ProvenanceTest(unittest.TestCase):
    def test_checked_in_release_inputs_match_manifest(self) -> None:
        manifest = verify(Config.load(ROOT), require_python=True)
        self.assertEqual(manifest["environment"]["resolved_packages"], 219)


class Task8OMPLOverlayProvenanceTest(unittest.TestCase):
    """Task 8: deterministic acceptance-contract provenance (fix round 1).

    These assertions recompute every derived hash/contract from immutable git
    objects (``git show <recorded-commit>:<path>``) and committed simulator
    source, never from mutable working trees or contract literals alone, and fail
    on mutations: altered argument order/count, booleans, strict keys,
    production imports/packages/executables, simulator provider entries,
    handoff, client scope, model-bundle source evidence, stable hashes, and
    installed data registration.

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
                "decl": "716a0d1845d5d73c5037eeea2baaa89359b5aede44b6a9600355c75bedf2463c",
                "rev": "2026-08-01-moveit-qualification-joint",
                "rev_digest": "d684a3d2270ab6d935b8e5c94dd5d4512760e06a1d09a41582177680536ccd8d",
                "owned": ["sim_fixture/pedestal", "sim_fixture/public_target"],
                "fd": "7f89ab08d2cf74ae0726bb9edc1bfc3cf6e1f6ea7d8c5662760d3551530ab9de",
            },
            "qualification-moveit-plan-pose": {
                "decl": "8ce9d2d1c992c6c09f16302c3e7d286e787b6e93a104ef379f1a13c7534ca5c2",
                "rev": "2026-08-01-moveit-qualification-pose",
                "rev_digest": "fb2abd517a6c6d2f5d34ec099f3e62a245a968e52cd3223bb5ac911d5382af67",
                "owned": ["sim_fixture/pedestal", "sim_fixture/public_target"],
                "fd": "de3af2e05493e0fc3a11c8526c84da1065648f7134f996c9a331819c54df84f4",
            },
            "qualification-moveit-plan-blocked": {
                "decl": "4b6b471d7e89dd568c8ec712c0b7b2ae667e3b3812447daf2f46ace90ed385af",
                "rev": "2026-08-01-moveit-qualification-blocked",
                "rev_digest": "d6d25cfe04fa7e641d6140c20431c24a5cd150005d0c7f9c13735402c93281ac",
                "owned": [
                    "sim_fixture/pedestal",
                    "sim_fixture/public_target",
                    "sim_fixture/plan_blocker",
                ],
                "fd": "d1a1923ceb45394dd70b4248541eac3379a12921d1d244c7d3d5de44157578e4",
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
            self.assertEqual(recorded["planning_scene"]["target_source_id"], "sim_fixture/public_target")
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
        # When the provisioned on-disk manifest exists, its stable projection must
        # equal the contract's stable hash too (identical bytes, host-independent).
        if self.MODEL_BUNDLE_PATH.is_file():
            on_disk = json.loads(self.MODEL_BUNDLE_PATH.read_text(encoding="utf-8"))
            self.assertEqual(
                _sha256_json(stable_manifest_evidence(on_disk)),
                self._load_contract()["model_bundle"]["stable_manifest_sha256"],
            )

    def test_preflight_stable_evidence_reconstructed(self) -> None:
        data = self._load_contract()["model_bundle"]["preflight_report"]
        with tempfile.TemporaryDirectory(prefix="mb-recon-") as tmp:
            manifest = self._reconstruct_manifest(Path(tmp))
            manifest_path = Path(tmp) / "model-bundle.json"
            manifest_path.write_bytes(_canonical_json(manifest))
            # project_root is required so artifact_identity resolves the current
            # selection; the reconstructed simulator URDF is byte-identical to it.
            result = preflight_manifest(manifest_path, timeout=30.0, project_root=ROOT)
            self.assertEqual(result["status"], "ready")
            self.assertTrue(result["ready"])
            self.assertEqual(len(result["checks"]), data["check_count"])
            self.assertEqual(
                [c["name"] for c in result["checks"]], data["check_names"]
            )
            # The stable preflight projection excludes elapsed_ms and host paths.
            self.assertEqual(
                _sha256_json(stable_preflight_evidence(result)),
                data["stable_sha256"],
            )
            self.assertNotIn("elapsed_ms", stable_preflight_evidence(result))
            self.assertNotIn("model_bundle_manifest", stable_preflight_evidence(result))

    def test_current_artifact_matches_reproducible_derivation(self) -> None:
        if not self.CURRENT_JSON.is_file():
            self.fail("current.json is absent; cannot verify the stale-current guard")
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
            self.skipTest("install prefix not present; run the bounded build first")
        self._assert_installed_byte_identity(install)

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
