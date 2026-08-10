"""Task 1: integrated qualification schema and 16-scenario matrix tests.

These tests assert the complete immutable scenario/planning-scene/integrated
mappings are retained, all wire digests match ``^(?!0{64}$)[0-9a-f]{64}$``, the
last operation is the unique ``PHYSICS_READY`` operation, and no legacy alias or
report self-digest is accepted.  The public report carries only the one-key
``integrated`` mapping (``{"execution_profile": "sim_ompl"}``); the full
per-scenario ``integrated`` mapping is bound by the scenario declaration SHA-256
and preserved in the scenario declaration.  This module imports neither ROS nor
Isaac Sim, so it runs under the simulator CPython 3.12 venv.
"""
from __future__ import annotations

import copy
import hashlib
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))
sys.path.insert(0, str(ROOT / "tests"))

DIGEST = re.compile(r"^(?!0{64}$)[0-9a-f]{64}$")

from qualification_test_helpers import (  # noqa: E402
    expected_physics_ready_report,
    load_test_scenario,
)
from tinker_sim_bridge.integrated_readiness import (  # noqa: E402
    public_integrated_mapping,
    sha256_json,
)
from tinker_sim_core.scenario import load_named_scenario  # noqa: E402

SCENARIO_DIR = ROOT / "simulation/scenarios"
QUALIFICATION_DIR = ROOT / "simulation/qualification"

ALL_SCENARIOS = (
    "qualification-moveit-plan-joint",
    "qualification-moveit-plan-pose",
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

PICK_PLACE_SCENARIOS = (
    "qualification-pick-place-positive",
    "qualification-pick-place-blocked-approach",
    "qualification-pick-place-unreachable-grasp",
    "qualification-pick-place-malformed-back",
    "qualification-pick-place-cancel-approach",
    "qualification-pick-place-cancel-transport",
    "qualification-pick-place-safety-transport",
    "qualification-pick-place-occupied-place",
)

NEGATIVE_SCENARIOS = {
    "qualification-pick-place-blocked-approach": {
        "required": ["pick_terminal_non_success", "contact_absent", "scene_attach_absent", "lift_m_lt:0.02"],
        "forbidden": ["gripper_close", "scene_attach", "release", "place_goal_sent"],
        "trigger_timeout_s": 10.0,
    },
    "qualification-pick-place-unreachable-grasp": {
        "required": ["pick_terminal_non_success", "contact_absent", "scene_attach_absent", "approach_tcp_delta_lt:0.02"],
        "forbidden": ["gripper_close", "scene_attach", "lift", "release"],
        "trigger_timeout_s": 10.0,
    },
    "qualification-pick-place-malformed-back": {
        "required": ["goal_rejected_pre_send", "no_planning_scene_mutation"],
        "forbidden": ["pick_goal_sent", "move_group_goal_sent", "scene_attach", "contact"],
        "trigger_timeout_s": 5.0,
    },
    "qualification-pick-place-cancel-approach": {
        "required": ["cancel_trigger_after_approach_start", "contact_absent", "scene_attach_absent", "release_absent"],
        "forbidden": ["gripper_close", "scene_attach", "lift_complete", "place_goal_sent"],
        "trigger_timeout_s": 10.0,
    },
    "qualification-pick-place-cancel-transport": {
        "required": ["cancel_trigger_after_lift", "contact_present_before_cancel", "scene_attached_before_cancel", "release_absent", "no_post_cancel_stage"],
        "forbidden": ["gripper_open", "scene_detach", "place_goal_sent", "post_clear_resume"],
        "trigger_timeout_s": 15.0,
    },
    "qualification-pick-place-safety-transport": {
        "required": ["safety_observed_during_transport", "controller_terminal_non_success", "velocity_below_stop_limit", "release_absent", "no_post_clear_resume"],
        "forbidden": ["gripper_open", "scene_detach", "new_goal_after_clear"],
        "trigger_timeout_s": 15.0,
    },
    "qualification-pick-place-occupied-place": {
        "required": ["pick_physical_retained", "place_terminal_non_success", "release_absent", "scene_attached_after_place_failure"],
        "forbidden": ["scene_detach", "target_region_settled", "gripper_open"],
        "trigger_timeout_s": 15.0,
    },
}

FIXTURE_ASSETS = {
    "sim_fixture/pedestal": {
        "source_object_id": "qualification_pedestal",
        "asset_uri": "simulation/assets/primitives/qualification-pedestal.usda",
        "asset_sha256": "c4a5e9812224a217bdba21ab81c679fac6c107c3501263a43335c3c0695a8e19",
        "geometry": {"type": "box", "dimensions": [0.12, 0.12, 0.60]},
        "center_offset_z": 0.30,
    },
    "sim_fixture/qualification_cube": {
        "source_object_id": "qualification_cube",
        "asset_uri": "simulation/assets/primitives/task-object.usda",
        "asset_sha256": "94c6a7a7324fa1de2d5cc0cd258517c619ed219c12f4b7cf538ad3f409ddd010",
        "geometry": {"type": "box", "dimensions": [0.08, 0.08, 0.08]},
        "center_offset_z": 0.04,
    },
    "sim_fixture/plan_blocker": {
        "source_object_id": "qualification_plan_blocker",
        "asset_uri": "simulation/assets/primitives/obstacle.usda",
        "asset_sha256": "e90c3bdf8b7385a6540a499fa5197a01dfb5f470b5f1eb04ff8e5be0550f851f",
        "geometry": {"type": "box", "dimensions": [0.30, 0.30, 0.30]},
        "center_offset_z": 0.15,
    },
    "sim_fixture/place_occupant": {
        "source_object_id": "qualification_place_occupant",
        "asset_uri": "simulation/assets/primitives/task-object.usda",
        "asset_sha256": "94c6a7a7324fa1de2d5cc0cd258517c619ed219c12f4b7cf538ad3f409ddd010",
        "geometry": {"type": "box", "dimensions": [0.08, 0.08, 0.08]},
        "center_offset_z": 0.04,
    },
    "sim_fixture/place_pedestal": {
        "source_object_id": "qualification_place_pedestal",
        "asset_uri": "simulation/assets/primitives/qualification-pedestal.usda",
        "asset_sha256": "c4a5e9812224a217bdba21ab81c679fac6c107c3501263a43335c3c0695a8e19",
        "geometry": {"type": "box", "dimensions": [0.12, 0.12, 0.60]},
        "center_offset_z": 0.30,
    },
}

# Independent literal anchor for the canonical public report key sets (Task 1
# fix round 1): these exact sets are asserted so the serializer and the
# expected-builder cannot drift together.
PUBLIC_REPORT_TOP_LEVEL_KEYS = {
    "schema_version",
    "report_revision",
    "scenario",
    "planning_scene",
    "integrated",
    "identities",
    "operations",
    "final_simulation_state",
}
PUBLIC_REPORT_IDENTITIES_KEYS = {
    "scenario_id",
    "seed",
    "scenario_declaration_sha256",
    "planning_scene_sha256",
    "integrated_sha256",
    "model_fingerprint",
    "provider_manifest_sha256",
}
PUBLIC_INTEGRATED_KEYS = {"execution_profile"}


def canonical_report(scenario_name):
    """Build from the complete immutable mappings of the current scenario."""
    source = load_test_scenario(scenario_name)
    required = ("scenario", "planning_scene", "integrated", "report_identities")
    assert all(key in source for key in required)
    scenario = copy.deepcopy(source["scenario"])
    planning_scene = copy.deepcopy(source["planning_scene_declaration"])
    integrated = copy.deepcopy(source["integrated"])
    identities = copy.deepcopy(source["report_identities"])
    assert identities["scenario_id"] == scenario["id"]
    assert identities["seed"] == scenario["seed"]
    return expected_physics_ready_report(
        scenario_mapping=scenario,
        planning_scene=planning_scene,
        integrated=integrated,
        expected_identities=identities,
    )


def test_positive_report_has_exact_shape_and_external_digest_only():
    report = canonical_report("qualification-pick-place-positive")
    assert set(report) == PUBLIC_REPORT_TOP_LEVEL_KEYS
    assert set(report["identities"]) == PUBLIC_REPORT_IDENTITIES_KEYS
    assert all(
        DIGEST.fullmatch(value)
        for key, value in report["identities"].items()
        if key not in {"scenario_id", "seed"}
    )
    physics_ready = [op for op in report["operations"] if op.get("boundary") == "PHYSICS_READY"]
    assert len(physics_ready) == 1
    assert report["operations"][-1]["state"] == 1
    assert report["final_simulation_state"] == "STATE_PLAYING"
    assert "scenario_report_sha256" not in report


def test_public_report_key_set_literal_anchor():
    """The serializer and the expected-builder must agree with independent
    literal key sets (top-level, identities, and the one-key public integrated
    mapping)."""
    report = canonical_report("qualification-moveit-plan-joint")
    assert set(report) == PUBLIC_REPORT_TOP_LEVEL_KEYS
    assert set(report["identities"]) == PUBLIC_REPORT_IDENTITIES_KEYS
    assert set(report["integrated"]) == PUBLIC_INTEGRATED_KEYS
    assert set(public_integrated_mapping()) == PUBLIC_INTEGRATED_KEYS
    # The full per-scenario mapping is strictly richer than the public one.
    source = load_test_scenario("qualification-pick-place-positive")
    assert PUBLIC_INTEGRATED_KEYS < set(source["integrated"])


def test_blocked_report_is_scenario_specific():
    positive = canonical_report("qualification-pick-place-positive")
    blocked = canonical_report("qualification-pick-place-blocked-approach")
    assert positive != blocked
    assert positive["scenario"] != blocked["scenario"]
    assert positive["planning_scene"] != blocked["planning_scene"]
    # The public one-key integrated mapping is identical; the full per-scenario
    # integrated mappings differ (blocked-approach adds plan_blocker and the
    # negative contract).
    assert positive["integrated"] == blocked["integrated"] == public_integrated_mapping()
    positive_source = load_test_scenario("qualification-pick-place-positive")
    blocked_source = load_test_scenario("qualification-pick-place-blocked-approach")
    assert positive_source["integrated"] != blocked_source["integrated"]
    assert positive["identities"] != blocked["identities"]


@pytest.mark.parametrize("scenario_name", ALL_SCENARIOS)
def test_all_16_scenarios_build_their_own_complete_report(scenario_name):
    report = canonical_report(scenario_name)
    source = load_test_scenario(scenario_name)
    assert report["scenario"] == source["scenario"]
    assert report["planning_scene"] == source["planning_scene"]
    # The public report carries the one-key integrated mapping; the full
    # mapping stays in the scenario declaration and is bound by the declaration
    # SHA-256.
    assert report["integrated"] == public_integrated_mapping()
    assert report["identities"]["integrated_sha256"] == sha256_json(
        public_integrated_mapping()
    )
    assert report["identities"]["scenario_declaration_sha256"] == sha256_json(
        source["scenario"]
    )
    raw = json.loads((SCENARIO_DIR / f"{scenario_name}.json").read_text(encoding="utf-8"))
    assert source["integrated"] == raw["integrated"]


def test_integrated_ompl_config_declares_all_16_scenarios():
    config_path = QUALIFICATION_DIR / "integrated-ompl.json"
    assert config_path.is_file()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    assert config["schema_version"] == 3
    assert config["id"] == "integrated-ompl"
    assert config["execution_profile"] == "sim_ompl"
    assert config["backend"] == "isaac"
    assert config["robot"] == "tinker2"
    assert config["seed"] == 7
    assert config["core_config"] == "simulation/qualification/manipulation-core.json"
    assert config["overlay_contract"] == "integration/ompl-overlay-contract.json"
    stages = config["stages"]
    declared = []
    declared.extend(stages["C"]["scenarios"])
    declared.extend(stages["D"]["scenarios"])
    declared.append(stages["E"]["positive"])
    declared.extend(stages["E"]["negative"])
    assert sorted(declared) == sorted(ALL_SCENARIOS)
    assert len(declared) == 16


@pytest.mark.parametrize("scenario_name", ALL_SCENARIOS)
def test_all_scenarios_load_through_scenario_definition(scenario_name):
    scenario = load_named_scenario(ROOT, scenario_name)
    assert scenario.integrated is not None
    assert scenario.declaration is not None
    integrated = scenario.integrated
    assert integrated["execution_profile"] == "sim_ompl"
    assert integrated["stage"] in {"C", "D", "E", "F"}
    assert integrated["authority"] == "physics_truth"
    assert integrated["acceptance"]["polarity"]
    assert isinstance(integrated["forbidden_endpoints"], list)
    assert integrated["terminal_policy"]


def test_fixture_asset_hashes_match_actual_assets():
    for fixture_id, spec in FIXTURE_ASSETS.items():
        asset_path = ROOT / spec["asset_uri"]
        assert asset_path.is_file()
        actual = hashlib.sha256(asset_path.read_bytes()).hexdigest()
        assert actual == spec["asset_sha256"], f"{fixture_id} asset sha256 mismatch"
        # The committed asset's internal translate gives the deterministic
        # bottom-origin center offset used by the parity contract (F1).
        asset_text = asset_path.read_text(encoding="utf-8")
        import re as _re
        translate = _re.search(r"xformOp:translate = \(([^)]*)\)", asset_text)
        assert translate is not None, f"{spec['asset_uri']} has no translate"
        z = float(translate.group(1).split(",")[2].strip())
        assert z == spec["center_offset_z"], (
            f"{fixture_id} center offset mismatch: asset z={z} expected {spec['center_offset_z']}"
        )
        # Every E-stage pick-place scenario that owns the fixture declares its
        # exact geometry (the C/D-stage pedestal is a separate, larger shape).
        for name in PICK_PLACE_SCENARIOS:
            raw = json.loads((SCENARIO_DIR / f"{name}.json").read_text(encoding="utf-8"))
            by_id = {obj["id"]: obj for obj in raw["planning_scene"]["objects"]}
            if fixture_id in by_id:
                primitive = by_id[fixture_id]["primitive"]
                assert primitive["type"] == spec["geometry"]["type"]
                assert primitive["dimensions"] == spec["geometry"]["dimensions"], (
                    f"{name} {fixture_id} geometry does not match fixture table"
                )


def test_obstacle_asset_hash_bindings_are_consistent_with_asset_bytes():
    """The obstacle fixture is content-addressed: every existing scenario/config
    hash binding for obstacle.usda must track the committed asset bytes.  Any
    change to the asset (e.g. adding the kinematic rigid-body API) forces every
    binding below to be re-pinned in the same change, or this test goes RED."""
    asset_path = ROOT / "simulation/assets/primitives/obstacle.usda"
    actual = hashlib.sha256(asset_path.read_bytes()).hexdigest()
    # Fixture table binding consumed by the parity/spawn contracts.
    assert FIXTURE_ASSETS["sim_fixture/plan_blocker"]["asset_sha256"] == actual
    # Scenario spawn-record bindings that pin the obstacle asset.  A record
    # without an asset_sha256 (e.g. qualification-arm-collision) has no binding
    # to re-pin, so it is intentionally not forced here.
    for name in (
        "qualification-arm-collision",
        "qualification-pick-place-blocked-approach",
    ):
        raw = json.loads((SCENARIO_DIR / f"{name}.json").read_text(encoding="utf-8"))
        for record in raw.get("objects", []) + raw.get("actors", []):
            if str(record.get("asset_uri", "")).endswith("obstacle.usda"):
                if "asset_sha256" in record:
                    assert record["asset_sha256"] == actual, (
                        f"{name}: obstacle asset_sha256 is stale; re-pin to {actual}"
                    )


def test_each_owned_id_appears_exactly_once_in_planning_scene_objects():
    for scenario_name in ALL_SCENARIOS:
        source = load_test_scenario(scenario_name)
        declaration = source["planning_scene_declaration"]
        object_ids = [obj["id"] for obj in declaration["objects"]]
        owned_ids = source["planning_scene"]["owned_ids"]
        assert sorted(object_ids) == sorted(owned_ids)
        assert len(object_ids) == len(set(object_ids))


@pytest.mark.parametrize("scenario_name", sorted(NEGATIVE_SCENARIOS))
def test_negative_scenario_contracts(scenario_name):
    expected = NEGATIVE_SCENARIOS[scenario_name]
    source = load_test_scenario(scenario_name)
    integrated = source["integrated"]
    assert integrated["acceptance"]["polarity"] == "negative"
    expected_negative = integrated["expected_negative"]
    assert expected_negative is not None
    assert expected_negative["required"] == expected["required"]
    assert expected_negative["forbidden"] == expected["forbidden"]
    assert integrated["race_policy"] == "bounded-observable-trigger"
    assert integrated["trigger_timeout_s"] == expected["trigger_timeout_s"]
    assert integrated["forbidden_after_terminal"] == expected["forbidden"]
    assert integrated["expected_physical"] == []


def test_pick_place_geometry_contract():
    config = json.loads(
        (QUALIFICATION_DIR / "integrated-ompl.json").read_text(encoding="utf-8")
    )
    geometry = config["geometry_contract"]
    assert geometry["mount"] == {
        "parent": "world",
        "child": "base_link",
        "xyz": [0.0, 0.0, 0.0],
        "rpy": [0.0, 0.0, 0.0],
    }
    # Bottom-origin root is never reused as a center (F1): the config carries an
    # explicit center and the deterministic local half-extent.
    assert geometry["object_root_xyz"] == [0.65, 0.0, 0.60]
    assert geometry["object_center_xyz"] == [0.65, 0.0, 0.64]
    assert geometry["object_local_center_z"] == 0.04
    assert geometry["object_half_extent_xyz"] == [0.04, 0.04, 0.04]
    assert geometry["grasp_tcp_xyz"] == [0.65, 0.0, 0.72]
    assert geometry["place_target_point"] == {"frame_id": "base_link", "xyz": [0.85, 0.0, 0.72]}
    assert geometry["place_orientation_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    assert geometry["place_region_center_xyz"] == [0.85, 0.0, 0.64]
    assert geometry["place_support_root_xyz"] == [0.85, 0.0, 0.0]
    assert geometry["place_support_center_xyz"] == [0.85, 0.0, 0.30]
    assert geometry["place_support_dimensions"] == [0.12, 0.12, 0.60]

    positive = json.loads(
        (SCENARIO_DIR / "qualification-pick-place-positive.json").read_text(encoding="utf-8")
    )
    ps = positive["planning_scene"]
    cube = next(obj for obj in ps["objects"] if obj["id"] == "sim_fixture/qualification_cube")
    assert cube["pose"]["xyz"] == geometry["object_center_xyz"]
    assert cube["pose"]["quaternion_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    place_support = next(
        obj for obj in ps["objects"] if obj["id"] == "sim_fixture/place_pedestal"
    )
    assert place_support["pose"]["xyz"] == geometry["place_support_center_xyz"]
    assert place_support["primitive"]["dimensions"] == geometry["place_support_dimensions"]


@pytest.mark.parametrize("scenario_name", (
    "qualification-pick-place-positive",
    "qualification-pick-place-blocked-approach",
    "qualification-pick-place-unreachable-grasp",
    "qualification-pick-place-malformed-back",
    "qualification-pick-place-cancel-approach",
    "qualification-pick-place-cancel-transport",
    "qualification-pick-place-safety-transport",
    "qualification-pick-place-occupied-place",
))
def test_e_stage_physical_root_to_planning_scene_center_parity(scenario_name):
    """F1: every top-level physical record with a planning_scene_id maps to its
    PlanningScene center as physical root + the committed asset center offset."""
    raw = json.loads((SCENARIO_DIR / f"{scenario_name}.json").read_text(encoding="utf-8"))
    ps_by_id = {obj["id"]: obj for obj in raw["planning_scene"]["objects"]}
    physical = []
    for record in raw.get("actors", []) + raw.get("objects", []):
        ps_id = record.get("planning_scene_id")
        if ps_id:
            physical.append((ps_id, record))
    assert len(physical) == len(raw["planning_scene"]["objects"])
    for ps_id, record in physical:
        spec = FIXTURE_ASSETS[ps_id]
        root_xyz = record["pose"]["xyz"]
        center = ps_by_id[ps_id]["pose"]["xyz"]
        assert center[2] == pytest.approx(root_xyz[2] + spec["center_offset_z"]), (
            f"{scenario_name} {ps_id}: PS center z {center[2]} != physical root z "
            f"{root_xyz[2]} + local center {spec['center_offset_z']}"
        )
        assert center[0] == pytest.approx(root_xyz[0])
        assert center[1] == pytest.approx(root_xyz[1])


def test_malformed_back_has_six_positions_all_others_follow_strict_seven():
    config = json.loads(
        (QUALIFICATION_DIR / "integrated-ompl.json").read_text(encoding="utf-8")
    )
    strict_back = config["pick_place_profiles"]["strict"]["back_positions"]
    assert len(strict_back) == 7
    assert all(isinstance(value, (int, float)) for value in strict_back)
    malformed = json.loads(
        (SCENARIO_DIR / "qualification-pick-place-malformed-back.json").read_text(encoding="utf-8")
    )
    assert len(malformed["integrated"]["back_positions"]) == 6
    for name in ALL_SCENARIOS:
        if name == "qualification-pick-place-malformed-back":
            continue
        raw = json.loads((SCENARIO_DIR / f"{name}.json").read_text(encoding="utf-8"))
        # Non-malformed scenarios never carry a six-value defect; they inherit
        # the strict seven-value profile from the config.
        assert "back_positions" not in raw["integrated"] or len(raw["integrated"]["back_positions"]) == 7


def test_top_level_spawn_records_carry_full_physical_metadata():
    positive = json.loads(
        (SCENARIO_DIR / "qualification-pick-place-positive.json").read_text(encoding="utf-8")
    )
    objects = {obj["id"]: obj for obj in positive["objects"]}
    actors = {act["id"]: act for act in positive["actors"]}
    cube = objects["qualification_cube"]
    assert cube["role"] == "pick-target"
    assert cube["owner"] == "sim_fixture"
    assert cube["region"] == "source-region"
    assert cube["planning_scene_id"] == "sim_fixture/qualification_cube"
    pedestal = actors["qualification_pedestal"]
    assert pedestal["role"] == "support"
    assert pedestal["owner"] == "sim_fixture"
    assert pedestal["region"] == "source-region"
    assert pedestal["planning_scene_id"] == "sim_fixture/pedestal"
    # F2: every E-stage scenario declares the place-support pedestal.
    place_pedestal = actors["qualification_place_pedestal"]
    assert place_pedestal["role"] == "support"
    assert place_pedestal["owner"] == "sim_fixture"
    assert place_pedestal["region"] == "place-region"
    assert place_pedestal["fixed"] is True
    assert place_pedestal["planning_scene_id"] == "sim_fixture/place_pedestal"
    assert place_pedestal["pose"]["xyz"] == [0.85, 0.0, 0.0]
    assert place_pedestal["asset_sha256"] == FIXTURE_ASSETS["sim_fixture/place_pedestal"]["asset_sha256"]

    occupied = json.loads(
        (SCENARIO_DIR / "qualification-pick-place-occupied-place.json").read_text(encoding="utf-8")
    )
    occupant = {obj["id"]: obj for obj in occupied["objects"]}["qualification_place_occupant"]
    assert occupant["role"] == "occupied-place"
    assert occupant["owner"] == "sim_fixture"
    assert occupant["region"] == "place-region"
    assert occupant["planning_scene_id"] == "sim_fixture/place_occupant"
    # F1: the occupant physical root rests on the place support top (0.60);
    # its PlanningScene center is root + half-extent.
    assert occupant["pose"]["xyz"] == [0.85, 0.0, 0.60]
    occupant_ps = {
        obj["id"]: obj for obj in occupied["planning_scene"]["objects"]
    }["sim_fixture/place_occupant"]
    assert occupant_ps["pose"]["xyz"] == [0.85, 0.0, 0.64]


def test_place_support_top_matches_placement_object_bottom():
    """F2: place support top == placement-object bottom == 0.60, and the place
    center/TCP target relation is achievable from the declared support."""
    for name in (
        "qualification-pick-place-positive",
        "qualification-pick-place-occupied-place",
    ):
        raw = json.loads((SCENARIO_DIR / f"{name}.json").read_text(encoding="utf-8"))
        ps = {obj["id"]: obj for obj in raw["planning_scene"]["objects"]}
        support = ps["sim_fixture/place_pedestal"]
        support_center_z = support["pose"]["xyz"][2]
        support_half_z = support["primitive"]["dimensions"][2] / 2.0
        # Support top (center + half-height) == placement-object bottom == 0.60.
        assert support_center_z + support_half_z == pytest.approx(0.60)
        # Occupant bottom (center - half-extent) rests exactly on the support.
        for ps_id in ("sim_fixture/qualification_cube", "sim_fixture/place_occupant"):
            if ps_id in ps:
                obj_center_z = ps[ps_id]["pose"]["xyz"][2]
                obj_half_z = ps[ps_id]["primitive"]["dimensions"][2] / 2.0
                assert obj_center_z - obj_half_z == pytest.approx(0.60)
        # Place target TCP is achievable above the place region center.
        config = json.loads((QUALIFICATION_DIR / "integrated-ompl.json").read_text(encoding="utf-8"))
        geometry = config["geometry_contract"]
        assert geometry["place_region_center_xyz"][2] - geometry["place_target_point"]["xyz"][2] == pytest.approx(-0.08)


def test_blocked_approach_geometry_deterministically_rejects_before_contact():
    """F3: the blocker covers the declared target TCP without initial contact."""
    raw = json.loads(
        (SCENARIO_DIR / "qualification-pick-place-blocked-approach.json").read_text(encoding="utf-8")
    )
    ps = {obj["id"]: obj for obj in raw["planning_scene"]["objects"]}
    blocker = ps["sim_fixture/plan_blocker"]
    cube = ps["sim_fixture/qualification_cube"]
    blocker_center = blocker["pose"]["xyz"]
    blocker_dims = blocker["primitive"]["dimensions"]
    blocker_half_z = blocker_dims[2] / 2.0
    blocker_bottom = blocker_center[2] - blocker_half_z
    blocker_top = blocker_center[2] + blocker_half_z
    cube_half_z = cube["primitive"]["dimensions"][2] / 2.0
    cube_top = cube["pose"]["xyz"][2] + cube_half_z
    # No initial 3D overlap: blocker bottom clears the cube top by 0.02 m.
    assert blocker_bottom == pytest.approx(0.70)
    assert cube_top == pytest.approx(0.68)
    assert blocker_bottom - cube_top == pytest.approx(0.02)
    # Target TCP lies inside the blocker volume.
    target_tcp_xyz = raw["integrated"]["goal"]["target_tcp_xyz"]
    assert target_tcp_xyz == [0.65, 0.0, 0.72]
    assert blocker_bottom < target_tcp_xyz[2] < blocker_top
    assert target_tcp_xyz[0] == pytest.approx(blocker_center[0])
    # Explicit top-down approach/target contract for Task 4.
    assert raw["integrated"]["goal"]["approach"] == "top-down"
    # Expected negative remains pre-contact/non-success.
    expected_negative = raw["integrated"]["expected_negative"]
    assert "contact_absent" in expected_negative["required"]
    assert "pick_terminal_non_success" in expected_negative["required"]
    assert expected_negative["forbidden"] == [
        "gripper_close", "scene_attach", "release", "place_goal_sent"
    ]


def test_d_stage_requires_no_spawned_task_object():
    """F4: D-stage execute scenarios declare no spawned physical task object;
    they exercise arm/gripper execution against the declared fixtures only."""
    for name in (
        "qualification-moveit-execute-joint",
        "qualification-moveit-execute-pose",
        "qualification-moveit-cartesian-retreat",
        "qualification-moveit-gripper",
        "qualification-moveit-cancel",
        "qualification-moveit-safety",
    ):
        raw = json.loads((SCENARIO_DIR / f"{name}.json").read_text(encoding="utf-8"))
        assert raw["actors"] == []
        assert raw["objects"] == []
        assert raw["integrated"]["stage"] == "D"
        # Execution predicates describe arm/gripper behavior only; no spawned
        # pick-target is required.
        expected_physical = raw["integrated"]["expected_physical"]
        assert isinstance(expected_physical, list) and expected_physical
        assert all("object" not in predicate for predicate in expected_physical)


def test_report_has_no_legacy_alias_or_self_digest():
    report = canonical_report("qualification-moveit-plan-joint")
    for key in (
        "scenario_report_sha256",
        "scenario_sha256",
        "integrated_mapping",
        "runtime_contract_sha256",
    ):
        assert key not in report
    for op in report["operations"]:
        assert "scenario_report_sha256" not in op
    assert "scenario_report_sha256" not in report["identities"]
