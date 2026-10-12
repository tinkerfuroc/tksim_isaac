"""Gesture/pose-aware person asset selection in tools/gpsr_scene.py.

Pure Python (no ROS, no Isaac): the people-artifact manifest is either
passed in through `plan_scene(person_assets=...)` or synthesised under a
tmp asset_root so the tests do not depend on which models the on-disk
artifact currently carries.
"""
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.gpsr_scene import (  # noqa: E402
    LYING_BACK_OFFSET_M,
    LYING_BODY_LENGTH_M,
    LYING_QUATERNION_XYZW,
    PERSON_ASSET_FALLBACKS,
    PERSON_ASSET_URI,
    PERSON_STANDING_KEY,
    choose_person_asset,
    classify_person_phrase,
    lying_pose,
    person_asset_candidates,
    plan_scene,
    scene_plan_from_json,
    scene_plan_to_json,
)
from tools.gpsr_scene import _load_person_asset_uris, _quat_rotate  # noqa: E402

CONSTANTS_PATH = Path(
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json"
)
PLACEMENTS_PATH = ROOT / "simulation" / "scenarios" / "rcw2026-placements.json"

ALL_MODELS = (
    "person_standing",
    "person_sitting",
    "owada_left_hand_raising",
    "owada_right_hand_raising",
    "person_walking",
    "standing_man_casual",
    "sitting_woman",
)
FAKE_HASH = "0" * 64


def _assets(*keys: str) -> dict[str, str]:
    return {k: f"artifacts/people/sobits/{FAKE_HASH}/{k}/person.usd" for k in keys}


FULL_ASSETS = _assets(*ALL_MODELS)
STANDING_ONLY = _assets("person_standing")


@pytest.fixture(scope="module")
def knowledge():
    return json.loads(CONSTANTS_PATH.read_text())


@pytest.fixture(scope="module")
def placements():
    return json.loads(PLACEMENTS_PATH.read_text())


def _persons(plan):
    return [it for it in plan.items if it.kind == "person"]


# --- phrase -> pose label (pure) ----------------------------------------------

@pytest.mark.parametrize(
    "phrase, label",
    [
        ("waving person", "waving"),
        ("person raising their left arm", "raising_left_arm"),
        ("person raising their right arm", "raising_right_arm"),
        ("person pointing to the left", "pointing_left"),
        ("person pointing to the right", "pointing_right"),
        ("sitting person", "sitting"),
        ("standing person", "standing"),
        ("lying person", "lying"),
        ("walking person", "walking"),
        # plural forms from the official generator
        ("waving persons", "waving"),
        ("persons raising their left arm", "raising_left_arm"),
        ("persons pointing to the right", "pointing_right"),
        ("lying persons", "lying"),
        # no gesture/pose named: names, clothing, bare mentions -> default
        ("person wearing a blue t shirt", "standing"),
        ("Liam", "standing"),
        ("meet Sarah at the laundry_desk", "standing"),
        ("", "standing"),
    ],
)
def test_classify_person_phrase(phrase, label):
    assert classify_person_phrase(phrase) == label


def test_classify_is_word_bounded_and_case_insensitive():
    assert classify_person_phrase("heat it in the microwave for the person") == "standing"
    assert classify_person_phrase("the underlying person") == "standing"
    assert classify_person_phrase("WAVING PERSON") == "waving"


# --- pose label -> asset fallback chain (pure) --------------------------------

@pytest.mark.parametrize(
    "label, chain",
    [
        ("waving", ("owada_right_hand_raising", "person_standing")),
        ("raising_left_arm", ("owada_left_hand_raising", "person_standing")),
        ("raising_right_arm", ("owada_right_hand_raising", "person_standing")),
        ("pointing_left", ("owada_left_hand_raising", "person_standing")),
        ("pointing_right", ("owada_right_hand_raising", "person_standing")),
        ("sitting", ("person_sitting", "sitting_woman", "person_standing")),
        ("standing", ("person_standing", "standing_man_casual")),
        ("lying", ("person_standing",)),
        ("walking", ("person_walking", "person_standing")),
    ],
)
def test_person_asset_candidates_chain(label, chain):
    assert person_asset_candidates(label) == chain


def test_every_chain_ends_in_the_standing_person():
    for label in PERSON_ASSET_FALLBACKS:
        assert PERSON_STANDING_KEY in person_asset_candidates(label)
    assert person_asset_candidates("no-such-label") == person_asset_candidates("standing")


@pytest.mark.parametrize(
    "label, expected",
    [
        ("waving", "owada_right_hand_raising"),
        ("raising_left_arm", "owada_left_hand_raising"),
        ("raising_right_arm", "owada_right_hand_raising"),
        ("pointing_left", "owada_left_hand_raising"),
        ("pointing_right", "owada_right_hand_raising"),
        ("sitting", "person_sitting"),
        ("standing", "person_standing"),
        ("lying", "person_standing"),
        ("walking", "person_walking"),
    ],
)
def test_choose_person_asset_with_the_full_manifest_takes_the_first_choice(label, expected):
    key, note = choose_person_asset(label, FULL_ASSETS)
    assert key == expected
    assert note is None


def test_choose_person_asset_falls_back_along_the_chain_with_a_note():
    key, note = choose_person_asset("sitting", _assets("sitting_woman", "person_standing"))
    assert key == "sitting_woman"
    assert "person_sitting" in note and "sitting_woman" in note

    key, note = choose_person_asset("sitting", STANDING_ONLY)
    assert key == "person_standing"
    assert "not in people manifest" in note

    key, note = choose_person_asset("waving", STANDING_ONLY)
    assert key == "person_standing"
    assert "owada_right_hand_raising" in note


def test_choose_person_asset_with_nothing_available_still_names_person_standing():
    key, note = choose_person_asset("walking", {})
    assert key == "person_standing"
    assert note is not None


# --- command -> planned person item --------------------------------------------

@pytest.mark.parametrize(
    "text, room, pose, asset_key",
    [
        ("take a food from the side_table_02 and give it to the waving person in the laundry_room",
         "laundry_room", "waving", "owada_right_hand_raising"),
        ("greet the person raising their left arm in the kitchen",
         "kitchen", "raising_left_arm", "owada_left_hand_raising"),
        ("greet the person raising their right arm in the kitchen",
         "kitchen", "raising_right_arm", "owada_right_hand_raising"),
        ("tell me how many persons pointing to the left are in the bedroom",
         "bedroom", "pointing_left", "owada_left_hand_raising"),
        ("tell me how many persons pointing to the right are in the bedroom",
         "bedroom", "pointing_right", "owada_right_hand_raising"),
        ("tell the name of the sitting person in the kitchen",
         "kitchen", "sitting", "person_sitting"),
        ("tell the name of the standing person in the kitchen",
         "kitchen", "standing", "person_standing"),
        ("tell the name of the lying person in the bedroom",
         "bedroom", "lying", "person_standing"),
        ("follow the walking person in the laundry_room",
         "laundry_room", "walking", "person_walking"),
        ("introduce yourself to Liam in the living_room and tell what day is today",
         "living_room", "standing", "person_standing"),
        ("greet the person wearing a red jacket in the kitchen",
         "kitchen", "standing", "person_standing"),
    ],
)
def test_plan_scene_picks_the_asset_for_the_phrase(knowledge, placements, text, room, pose, asset_key):
    plan = plan_scene(text, knowledge, placements, seed=2026, person_assets=FULL_ASSETS)
    persons = _persons(plan)
    assert len(persons) == 1
    person = persons[0]
    assert person.room == room
    assert person.pose == pose
    assert person.asset_key == asset_key
    assert person.asset_uri == FULL_ASSETS[asset_key]
    assert person.id == f"cmd_person_{room}"
    assert not any("not in people manifest" in n for n in plan.notes)


def test_object_items_carry_their_ycb_asset_key_and_no_pose(knowledge, placements):
    plan = plan_scene("bring me a mug from the kitchen_table", knowledge, placements, seed=1,
                      person_assets=FULL_ASSETS)
    objects = [it for it in plan.items if it.kind == "object"]
    assert objects[0].asset_key == "ycb_025_mug"
    assert objects[0].pose == ""


def test_two_mentions_keep_their_own_descriptors(knowledge, placements):
    text = "tell the gesture of the person at the shelf_02 to the sitting person at the sofa"
    plan = plan_scene(text, knowledge, placements, seed=2026, person_assets=FULL_ASSETS)
    persons = _persons(plan)
    assert [(p.room, p.pose, p.asset_key) for p in persons] == [
        ("kitchen", "standing", "person_standing"),
        ("living_room", "sitting", "person_sitting"),
    ]

    text = "tell the pose of the person raising their left arm at the sofa to the person at the kitchen_table"
    plan = plan_scene(text, knowledge, placements, seed=2026, person_assets=FULL_ASSETS)
    assert [(p.room, p.pose) for p in _persons(plan)] == [
        ("living_room", "raising_left_arm"),
        ("kitchen", "standing"),
    ]


# --- missing-asset fallback ----------------------------------------------------

@pytest.mark.parametrize(
    "text, pose, wanted",
    [
        ("give it to the waving person in the laundry_room", "waving", "owada_right_hand_raising"),
        ("tell the name of the sitting person in the kitchen", "sitting", "person_sitting"),
        ("follow the walking person in the laundry_room", "walking", "person_walking"),
        ("greet the person pointing to the left in the bedroom", "pointing_left", "owada_left_hand_raising"),
    ],
)
def test_missing_asset_falls_back_to_person_standing_with_a_note(knowledge, placements, text, pose, wanted):
    plan = plan_scene(text, knowledge, placements, seed=1, person_assets=STANDING_ONLY)
    person = _persons(plan)[0]
    assert person.pose == pose
    assert person.asset_key == "person_standing"
    assert person.asset_uri == STANDING_ONLY["person_standing"]
    assert any(wanted in n and "not in people manifest" in n and "person_standing" in n for n in plan.notes)


def test_sitting_falls_back_to_sitting_woman_before_standing(knowledge, placements):
    assets = _assets("sitting_woman", "person_standing")
    plan = plan_scene("tell the name of the sitting person in the kitchen", knowledge, placements, seed=1,
                      person_assets=assets)
    person = _persons(plan)[0]
    assert person.asset_key == "sitting_woman"
    assert person.asset_uri == assets["sitting_woman"]
    assert any("person_sitting" in n and "sitting_woman" in n for n in plan.notes)


def test_unreadable_people_manifest_uses_the_legacy_constant_uri(knowledge, placements):
    plan = plan_scene("meet Liam in the laundry_room", knowledge, placements, seed=1, person_assets={})
    person = _persons(plan)[0]
    assert person.asset_key == "person_standing"
    assert person.asset_uri == PERSON_ASSET_URI
    assert any("manifest unreadable" in n for n in plan.notes)


# --- people artifact manifest lookup -------------------------------------------

def _write_people_artifact(root: Path, models: tuple[str, ...], identity: str = FAKE_HASH) -> Path:
    artifact_dir = root / "artifacts" / "people" / "sobits" / identity
    artifact_dir.mkdir(parents=True)
    payload = {"ATTRIBUTION.md": "x"}
    for model in models:
        payload[f"{model}/person.usd"] = "deadbeef"
        payload[f"{model}/textures/{model}/skin.png"] = "cafe"
    (artifact_dir / "manifest.json").write_text(json.dumps(
        {"algorithm": "arena-artifact-1", "id": "sobits", "identity": identity, "kind": "people",
         "payload": payload, "schema_version": 1}))
    (root / "artifacts" / "people" / "sobits" / "current.json").write_text(json.dumps(
        {"manifest": f"artifacts/people/sobits/{identity}/manifest.json", "schema_version": 1}))
    return artifact_dir


def test_load_person_asset_uris_reads_every_model_from_the_current_manifest(tmp_path):
    _write_people_artifact(tmp_path, ALL_MODELS)
    uris = _load_person_asset_uris(tmp_path)
    assert set(uris) == set(ALL_MODELS)
    assert uris["owada_left_hand_raising"] == (
        f"artifacts/people/sobits/{FAKE_HASH}/owada_left_hand_raising/person.usd"
    )


def test_load_person_asset_uris_is_empty_when_the_pointer_or_manifest_is_missing(tmp_path):
    assert _load_person_asset_uris(tmp_path) == {}
    (tmp_path / "artifacts" / "people" / "sobits").mkdir(parents=True)
    (tmp_path / "artifacts" / "people" / "sobits" / "current.json").write_text("{not json")
    assert _load_person_asset_uris(tmp_path) == {}
    (tmp_path / "artifacts" / "people" / "sobits" / "current.json").write_text(
        json.dumps({"manifest": "artifacts/people/sobits/nope/manifest.json"}))
    assert _load_person_asset_uris(tmp_path) == {}


def test_plan_scene_resolves_person_assets_from_the_manifest_under_asset_root(tmp_path, knowledge, placements):
    _write_people_artifact(tmp_path, ("person_standing", "owada_right_hand_raising"))
    plan = plan_scene("give it to the waving person in the laundry_room", knowledge, placements,
                      seed=1, asset_root=tmp_path)
    person = _persons(plan)[0]
    assert person.asset_key == "owada_right_hand_raising"
    assert person.asset_uri == f"artifacts/people/sobits/{FAKE_HASH}/owada_right_hand_raising/person.usd"

    plan = plan_scene("greet the person raising their left arm in the kitchen", knowledge, placements,
                      seed=1, asset_root=tmp_path)
    person = _persons(plan)[0]
    assert person.asset_key == "person_standing"
    assert any("owada_left_hand_raising" in n for n in plan.notes)


def test_the_real_people_artifact_carries_at_least_the_standing_person():
    uris = _load_person_asset_uris(ROOT)
    assert "person_standing" in uris
    assert uris["person_standing"].endswith("/person_standing/person.usd")
    assert (ROOT / uris["person_standing"]).is_file()


# --- lying pose override -------------------------------------------------------

def _face_and_head_directions(quaternion_xyzw):
    # person_standing faces its local -y and its head is along local +z.
    return _quat_rotate(quaternion_xyzw, (0.0, -1.0, 0.0)), _quat_rotate(quaternion_xyzw, (0.0, 0.0, 1.0))


def test_lying_quaternion_puts_the_face_up_and_the_head_horizontal():
    face, head = _face_and_head_directions(LYING_QUATERNION_XYZW)
    assert face == pytest.approx((0.0, 0.0, 1.0))
    assert head[2] == pytest.approx(0.0)
    assert math.hypot(*LYING_QUATERNION_XYZW[:2], LYING_QUATERNION_XYZW[2], LYING_QUATERNION_XYZW[3]) == pytest.approx(1.0)


@pytest.mark.parametrize("room", ["kitchen", "living_room", "bedroom", "laundry_room"])
def test_lying_pose_keeps_the_face_up_for_every_room_yaw(placements, room):
    # The room quaternions in rcw2026-placements.json are rounded to 6
    # decimals (norm off by ~1e-6), hence the 1e-5 tolerance.
    xyz, quat, _note = lying_pose(placements["persons"][room], spot=None, placements=placements)
    face, head = _face_and_head_directions(quat)
    assert face == pytest.approx((0.0, 0.0, 1.0), abs=1e-5)
    assert head[2] == pytest.approx(0.0, abs=1e-5)
    assert sum(c * c for c in quat) == pytest.approx(1.0, abs=1e-5)


def _body_midpoint(xyz, quaternion_xyzw):
    head = _quat_rotate(quaternion_xyzw, (0.0, 0.0, 1.0))
    half = LYING_BODY_LENGTH_M / 2.0
    return tuple(p + h * half for p, h in zip(xyz, head))


def test_lying_on_the_floor_lifts_the_origin_by_the_back_offset(placements):
    # The room's precomputed free-floor `lying` pose (the whole body clear of
    # furniture/walls, tests/test_gpsr_person_placements.py), not the upright xy.
    room_pose = placements["persons"]["bedroom"]
    xyz, _quat, note = lying_pose(room_pose, spot="side_table_02", placements=placements)
    lying = room_pose["lying"]["xyz"]
    assert xyz == pytest.approx((lying[0], lying[1], LYING_BACK_OFFSET_M))
    assert "floor" in note


def test_lying_on_the_floor_without_a_lying_pose_uses_the_room_xy(placements):
    room_pose = {k: v for k, v in placements["persons"]["bedroom"].items() if k != "lying"}
    xyz, _quat, note = lying_pose(room_pose, spot="side_table_02", placements=placements)
    assert xyz == pytest.approx((room_pose["xyz"][0], room_pose["xyz"][1], LYING_BACK_OFFSET_M))
    assert note == "lying on the floor"


def test_lying_on_the_sofa_moves_onto_the_seat_surface(placements):
    room_pose = placements["persons"]["living_room"]
    sofa = placements["spots"]["sofa"]["surface_xyz"]
    xyz, quat, note = lying_pose(room_pose, spot="sofa", placements=placements)
    # Body centred on the seat along the sofa's long (x) axis.
    assert _body_midpoint(xyz, quat)[:2] == pytest.approx((sofa[0], sofa[1]), abs=1e-3)
    assert xyz[2] == pytest.approx(sofa[2] + LYING_BACK_OFFSET_M)
    assert xyz[2] == pytest.approx(0.57)
    assert abs(_quat_rotate(quat, (0.0, 0.0, 1.0))[0]) == pytest.approx(1.0, abs=1e-6)
    assert "sofa" in note


def test_lying_on_an_unknown_bed_spot_degrades_to_the_floor(placements):
    room_pose = placements["persons"]["bedroom"]
    xyz, _quat, note = lying_pose(room_pose, spot="bed", placements=placements)
    assert xyz[2] == pytest.approx(LYING_BACK_OFFSET_M)
    assert "floor" in note


def test_plan_scene_lying_person_in_the_living_room_lies_on_the_sofa(knowledge, placements):
    plan = plan_scene("tell the name of the lying person in the living_room", knowledge, placements,
                      seed=1, person_assets=FULL_ASSETS)
    person = _persons(plan)[0]
    sofa = placements["spots"]["sofa"]["surface_xyz"]
    assert person.pose == "lying"
    assert person.asset_key == "person_standing"
    assert _body_midpoint(person.xyz, person.quaternion_xyzw)[:2] == pytest.approx((sofa[0], sofa[1]), abs=1e-3)
    assert person.xyz[2] == pytest.approx(sofa[2] + LYING_BACK_OFFSET_M)
    face, _head = _face_and_head_directions(person.quaternion_xyzw)
    assert face == pytest.approx((0.0, 0.0, 1.0), abs=1e-5)
    assert any("lying on 'sofa'" in n for n in plan.notes)


def test_plan_scene_lying_person_at_the_sofa_spot_lies_on_the_sofa(knowledge, placements):
    plan = plan_scene("tell the pose of the lying person at the sofa", knowledge, placements,
                      seed=1, person_assets=FULL_ASSETS)
    person = _persons(plan)[0]
    assert person.room == "living_room"
    assert person.xyz[2] == pytest.approx(0.57)


def test_plan_scene_lying_person_elsewhere_lies_on_the_floor(knowledge, placements):
    plan = plan_scene("tell the name of the lying person in the bedroom", knowledge, placements,
                      seed=1, person_assets=FULL_ASSETS)
    person = _persons(plan)[0]
    room_pose = placements["persons"]["bedroom"]
    lying = room_pose["lying"]["xyz"]
    assert person.xyz == pytest.approx((lying[0], lying[1], LYING_BACK_OFFSET_M))
    assert person.quaternion_xyzw != tuple(room_pose["quaternion_xyzw"])
    assert any("lying on the floor" in n for n in plan.notes)


def test_non_lying_persons_keep_the_room_pose_unchanged(knowledge, placements):
    plan = plan_scene("tell the name of the sitting person in the kitchen", knowledge, placements,
                      seed=1, person_assets=FULL_ASSETS)
    person = _persons(plan)[0]
    room_pose = placements["persons"]["kitchen"]
    assert person.xyz == pytest.approx(tuple(room_pose["xyz"]))
    assert person.quaternion_xyzw == pytest.approx(tuple(room_pose["quaternion_xyzw"]))


# --- JSON schema stays backward compatible ------------------------------------

def test_scene_plan_json_round_trips_asset_key_and_pose(knowledge, placements):
    text = "tell the name of the lying person in the living_room"
    plan = plan_scene(text, knowledge, placements, seed=1, person_assets=FULL_ASSETS)
    data = scene_plan_to_json(plan, command_text=text, seed=1)
    assert data["schema_version"] == 1
    assert data["items"][0]["asset_key"] == "person_standing"
    assert data["items"][0]["pose"] == "lying"
    assert scene_plan_from_json(data) == plan


def test_scene_plan_from_json_accepts_plans_written_without_the_new_keys():
    data = {
        "schema_version": 1,
        "command": "meet Liam in the laundry_room",
        "seed": 1,
        "notes": [],
        "items": [{
            "id": "cmd_person_laundry_room", "kind": "person", "name": "person",
            "asset_uri": PERSON_ASSET_URI, "room": "laundry_room", "spot": "",
            "xyz": [-2.931, 4.663, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.831497, -0.55553],
        }],
    }
    plan = scene_plan_from_json(data)
    assert plan.items[0].asset_key == ""
    assert plan.items[0].pose == ""
    assert plan.items[0].asset_uri == PERSON_ASSET_URI
