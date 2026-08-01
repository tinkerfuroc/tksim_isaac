from __future__ import annotations

import hashlib
import json
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))
sys.path.insert(0, str(ROOT / "simulation"))

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


class ProvenanceTest(unittest.TestCase):
    def test_checked_in_release_inputs_match_manifest(self) -> None:
        manifest = verify(Config.load(ROOT), require_python=True)
        self.assertEqual(manifest["environment"]["resolved_packages"], 219)


class Task8OMPLOverlayProvenanceTest(unittest.TestCase):
    """Task 8: deterministic acceptance-contract provenance.

    These assertions recompute every derived hash/contract from the real
    committed source and artifacts and fail on mutations: stale setup/package
    registrations, missing data files, provider-manifest drift, wrong
    model/current artifact, wrong endpoint/type/source/cardinality/QoS, wrong
    fixture/order/handoff, wrong compatibility booleans, raw-colcon text,
    dirty-policy violations, and source-lock files being prematurely included.

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
    PRODUCTION_REPO = Path("/home/tinker/tk25_ws/src/tk25_manipulation")
    SIMULATOR_LOCK = ROOT / "integration/source-locks.json"

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
        return hashlib.sha256(path.read_bytes()).hexdigest()

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
        # No runtime timestamp values are recorded.  The only date-like strings
        # allowed are scenario/fixture revision identifiers and the schema field
        # name "published_at" in the fixture status field list (a schema
        # contract, not a recorded timestamp).
        for token in ('"timestamp"', '"created_at"', '"generated_at"'):
            self.assertNotIn(token, raw, f"contract must not record a {token} field")
        # No self-referential final Task 8 commit hash.
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

        # Simulator recorded identity must exist in this repository's history.
        sim_id = str(sim["implementation_identity"])
        resolved = self._git(ROOT, "rev-parse", sim_id + "^{commit}")
        self.assertEqual(resolved.returncode, 0, "simulator identity not in history")
        self.assertEqual(resolved.stdout.strip(), sim_id)
        # The baseline must be an ancestor-or-equal of the current HEAD.
        head = self._git(ROOT, "rev-parse", "HEAD").stdout.strip()
        self.assertNotEqual(head, "")

        # Production commits must exist in the production repository.
        for key, commit in (
            ("runtime_hardening", str(prod["runtime_hardening"]["head_commit"])),
            ("task2_launch", str(prod["task2_launch"]["head_commit"])),
        ):
            result = self._git(self.PRODUCTION_REPO, "rev-parse", commit + "^{commit}")
            self.assertEqual(result.returncode, 0, f"production {key} commit missing")
            self.assertEqual(result.stdout.strip(), commit)

        # Runtime hardening ended before the Task 2 launch began.
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
        # Ranges are recorded as start..end and match the real git history.
        self.assertEqual(rh["range"], "{}..{}".format(rh["start"], rh["end"]))
        self.assertEqual(t2["range"], "{}..{}".format(t2["start"], t2["end"]))
        # The Task 2 launch head is the recorded production implementation identity.
        self.assertEqual(data["repositories"]["production"]["implementation_identity"], t2["end"])
        # No mutable-concurrent-HEAD assumption: the recorded identities are
        # explicit commit hashes, not the literal string "HEAD".
        for key in (rh["start"], rh["end"], t2["start"], t2["end"]):
            self.assertNotEqual(key, "HEAD")

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

        # Derived digests recompute from the canonical source.
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
        # The public report integrated mapping is exactly the one production-canonical key.
        self.assertEqual(sep["public_integrated"], {"execution_profile": "sim_ompl"})
        # The full runtime contract digest differs from the public mapping digest.
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
        # The model bundle records the identical full eight-link touch set.
        self.assertEqual(data["model_bundle"]["semantic_contract"]["touch_links"], expected)

    def test_contract_isaac_joint_commands_depth_50_and_external_owner(self) -> None:
        data = self._load_contract()
        pubs = data["typed_contract"]["publishers"]
        self.assertEqual(pubs["/isaac_joint_commands"]["depth"], 50)
        self.assertEqual(pubs["/isaac_joint_commands"]["source"], "/tinker_sim_command_gateway")
        self.assertEqual(pubs["/isaac_joint_commands"]["type"], "sensor_msgs/msg/JointState")
        self.assertEqual(pubs["/isaac_joint_commands"]["cardinality"], 1)
        # The external future executor owns /sim/safety/operator (not a Task 6 provider).
        self.assertEqual(pubs["/sim/safety/operator"]["source"], "/tinker_integrated_gate_executor")
        self.assertEqual(pubs["/sim/safety/operator"]["depth"], 1)
        self.assertEqual(pubs["/sim/safety/operator"]["durability"], "TRANSIENT_LOCAL")

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
        # Persistent nodes, one-shot processes, logical controller resources,
        # and publishers are recorded as distinct sections and never collapsed.
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
        # The external /sim/safety/operator publisher is NOT a persistent provider
        # in the manifest (it is owned by the future executor).
        persistent_nodes = {e["node"] for e in pm["persistent_nodes"]}
        self.assertNotIn("/tinker_integrated_gate_executor", persistent_nodes)

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
    # model bundle
    # ------------------------------------------------------------------
    def test_contract_model_bundle_hashes_recomputed(self) -> None:
        if not self.MODEL_BUNDLE_PATH.is_file():
            self.skipTest("model-bundle artifact tree not provisioned (gitignored outputs/)")
        data = self._load_contract()["model_bundle"]
        manifest = json.loads(self.MODEL_BUNDLE_PATH.read_text(encoding="utf-8"))

        self.assertEqual(manifest["schema_version"], data["schema_version"])
        self.assertEqual(manifest["producer"], data["producer"])
        self.assertEqual(data["manifest_sha256"], self._sha256_file(self.MODEL_BUNDLE_PATH))
        self.assertEqual(
            data["structural_fingerprint"],
            manifest["structural_fingerprint"],
        )
        self.assertEqual(
            data["structural_fingerprint"],
            sha256_json(manifest["contract"]),
        )
        # Every artifact hash is recomputed from the real artifact bytes.
        for name, entry in manifest["artifacts"].items():
            recorded = data["artifacts"][name]["sha256"]
            artifact = Path(entry["path"])
            self.assertTrue(artifact.is_file(), f"model bundle artifact missing: {artifact}")
            self.assertEqual(recorded, self._sha256_file(artifact))

    def test_contract_preflight_evidence_ready(self) -> None:
        preflight_path = ROOT / "outputs/ompl-overlay/model-bundle-r2/preflight-report.json"
        if not preflight_path.is_file():
            self.skipTest("preflight report not provisioned (gitignored outputs/)")
        data = self._load_contract()["model_bundle"]["preflight_report"]
        report = json.loads(preflight_path.read_text(encoding="utf-8"))
        self.assertEqual(data["ready"], True)
        self.assertEqual(data["ready"], report["ready"])
        self.assertEqual(data["sha256"], self._sha256_file(preflight_path))
        self.assertEqual(data["check_count"], len(report["checks"]))
        self.assertEqual(data["check_names"], [c["name"] for c in report["checks"]])

    def test_contract_current_artifact_identity(self) -> None:
        current = ROOT / "artifacts/robot/tinker2/current.json"
        if not current.is_file():
            self.skipTest("current artifact selector not provisioned (gitignored artifacts/)")
        data = json.loads(current.read_text(encoding="utf-8"))
        artifact_id = data["artifact_id"]
        selected = ROOT / "artifacts/robot/tinker2" / artifact_id / "robot.urdf"
        if not selected.is_file():
            self.skipTest("selected artifact generation not provisioned")
        contract_entry = self._load_contract()["model_bundle"]["artifacts"]["simulator_full_urdf"]
        self.assertIn(artifact_id, contract_entry["path_relative"])
        self.assertEqual(contract_entry["sha256"], self._sha256_file(selected))

    # ------------------------------------------------------------------
    # production overlay / compatibility booleans
    # ------------------------------------------------------------------
    def test_contract_production_overlay_18_argument_contract(self) -> None:
        data = self._load_contract()["production_overlay"]
        self.assertEqual(data["package"], "mobile_bringup")
        self.assertEqual(data["launch_file"], "manipulation_planning_task_only.launch.py")
        self.assertEqual(
            data["launch_path_relative"],
            "src/mobile_bringup/launch/manipulation_planning_task_only.launch.py",
        )
        self.assertEqual(len(data["launch_arguments"]), 18)
        for arg in (
            "model_bundle_manifest",
            "provider_manifest_path",
            "provider_manifest_sha256",
            "execution_profile",
            "start_move_group",
            "start_task_server",
            "required_fixture_owned_ids",
            "required_scenario_identities",
        ):
            self.assertIn(arg, data["launch_arguments"])

    def test_contract_literal_false_compatibility_values(self) -> None:
        data = self._load_contract()["production_overlay"]
        compat = data["sim_compatibility_parameters_literal_false"]
        self.assertEqual(compat["use_cumotion_object_attachment"], False)
        self.assertEqual(compat["use_cumotion_goalset"], False)
        self.assertEqual(compat["use_cumotion_straight_approach"], False)
        self.assertEqual(compat["esdf_freshness_wait_enabled"], False)
        # Literal booleans, never strings.
        for value in compat.values():
            self.assertIs(type(value), bool)
            self.assertIs(value, False)

    def test_contract_allowlists_and_task_owned_lifecycle(self) -> None:
        data = self._load_contract()["production_overlay"]
        self.assertIn("pick_and_place", data["provider_allowlist"]["node_packages"])
        self.assertIn("move_group", data["provider_allowlist"]["executables"])
        self.assertIn("integrated_readiness", data["provider_allowlist"]["executables"])
        self.assertIn("launch", data["provider_allowlist"]["import_allowlist"])
        self.assertIn("tinker_sim_bridge", data["provider_allowlist"]["import_allowlist"])
        self.assertIn("pick_and_place/object_mesh", str(data["task_owned_lifecycle"]))

    # ------------------------------------------------------------------
    # build commands
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

    # ------------------------------------------------------------------
    # setup.py / package.xml registrations
    # ------------------------------------------------------------------
    def test_setup_py_registers_all_required_assets(self) -> None:
        setup_text = self.SETUP_PY.read_text(encoding="utf-8")

        # Every launch file is registered.
        for launch_name in (
            "launch/fixtures.launch.py",
            "launch/integrated_ompl_manipulation.launch.py",
            "launch/manipulation.launch.py",
            "launch/navigation.launch.py",
            "launch/whole_robot.launch.py",
        ):
            self.assertIn('"' + launch_name + '"', setup_text)
            self.assertTrue((self.BRIDGE_DIR / launch_name).is_file())

        # Every config asset is registered.
        for config_name in (
            "config/base_facade.yaml",
            "config/command_gateway.yaml",
            "config/controllers.yaml",
            "config/pointcloud_to_laserscan.yaml",
            "config/tinker_topic_control.ros2_control.xacro",
        ):
            self.assertIn('"' + config_name + '"', setup_text)
            self.assertTrue((self.BRIDGE_DIR / config_name).is_file())

        # The provider manifest is registered.
        self.assertIn('"integration/provider-manifest.json"', setup_text)
        self.assertTrue((self.BRIDGE_DIR / "integration/provider-manifest.json").is_file())

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
        # Direct ROS message/service/action packages imported by the bridge
        # source (no transitive-import assumptions).  xarm_moveit_config is
        # deliberately NOT here: it is consumed transitively through the
        # mobile_bringup production launch, which itself declares it.
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

    # ------------------------------------------------------------------
    # docs: no raw colcon text
    # ------------------------------------------------------------------
    def test_docs_never_publish_raw_colcon_build(self) -> None:
        for doc in (
            ROOT / "README.md",
            ROOT / "docs/acceptance.md",
            ROOT / "integration/MANIPULATION.md",
            self.BRIDGE_DIR / "README.md",
        ):
            text = doc.read_text(encoding="utf-8")
            self.assertNotIn("colcon build", text, f"{doc.relative_to(ROOT)} must not instruct raw colcon build")

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
        # The simulator lock file exists and is unmodified by Task 8.
        self.assertTrue(self.SIMULATOR_LOCK.is_file())
        diff = self._git(ROOT, "diff", "--quiet", "--", "integration/source-locks.json")
        self.assertEqual(diff.returncode, 0, "Task 8 must not modify the simulator source-lock file")

    def test_contract_declares_source_lock_exclusion_note(self) -> None:
        data = self._load_contract()
        note = str(data["source_locks"]["note"])
        self.assertIn("Task 9", note)
        self.assertIn("excluded", data["source_locks"]["status"])


if __name__ == "__main__":
    unittest.main()
