import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.gpsr_scene import (  # noqa: E402
    CATEGORY_MAP,
    NAME_TO_YCB_DIR,
    PERSON_ASSET_URI,
    plan_scene,
    scene_plan_from_json,
    scene_plan_to_json,
)

CONSTANTS_PATH = Path(
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json"
)
PLACEMENTS_PATH = ROOT / "simulation" / "scenarios" / "rcw2026-placements.json"


@pytest.fixture(scope="module")
def knowledge():
    return json.loads(CONSTANTS_PATH.read_text())


@pytest.fixture(scope="module")
def placements():
    return json.loads(PLACEMENTS_PATH.read_text())


def _names(items):
    return [it.name for it in items]


def _ids(items):
    return [it.id for it in items]


# --- real corpus commands, s2026-001..007 (verbatim texts from
# gpsr_runs/bench/t2-2026/corpus.jsonl) ------------------------------------

def test_s2026_001_counts_three_kitchen_item_members_at_kitchen_table(knowledge, placements):
    text = "tell me how many kitchen items there are on the kitchen_table"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert [it.kind for it in plan.items] == ["object", "object", "object"]
    assert _names(plan.items) == ["bleach", "bowl", "mustard"]
    assert _ids(plan.items) == ["cmd_bleach_0", "cmd_bowl_0", "cmd_mustard_0"]
    for it in plan.items:
        assert it.spot == "kitchen_table"
        assert it.room == "kitchen"
    xs = [it.xyz[0] for it in plan.items]
    assert xs == [pytest.approx(2.5), pytest.approx(2.68), pytest.approx(2.86)]
    assert all(it.xyz[1] == pytest.approx(-3.0) for it in plan.items)
    assert all(it.xyz[2] == pytest.approx(0.734) for it in plan.items)


def test_s2026_002_counts_persons_spawns_no_objects_and_a_bedroom_person(knowledge, placements):
    text = "tell me how many persons pointing to the left are in the bedroom"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert [it.kind for it in plan.items] == ["person"]
    person = plan.items[0]
    assert person.room == "bedroom"
    assert person.id == "cmd_person_bedroom"
    assert person.xyz == pytest.approx((0.28, 1.755, 0.0))


def test_s2026_003_finds_kitchen_item_category_at_the_first_mentioned_room(knowledge, placements):
    # The command names both a search room ("living_room") and a later
    # placement spot ("kitchen_table"); objects resolve to the
    # FIRST-MENTIONED location in text order, so the search room
    # (living_room, at its first search spot: sofa) wins -- not the later
    # kitchen_table placement destination.
    text = "locate a kitchen item in the living_room then grasp it and place it on the kitchen_table"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert _names(plan.items) == ["bleach", "bowl", "mustard"]
    assert all(it.spot == "sofa" for it in plan.items)
    assert all(it.room == "living_room" for it in plan.items)
    assert all(it.spot != "kitchen_table" for it in plan.items)


def test_s2026_004_explicit_pudding_box_at_side_table_02_no_person(knowledge, placements):
    text = "go to the bedroom then locate a pudding_box and fetch it and put it on the side_table_02"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert len(plan.items) == 1
    item = plan.items[0]
    assert item.kind == "object"
    assert item.name == "pudding_box"
    assert item.id == "cmd_pudding_box_0"
    assert item.spot == "side_table_02"
    assert item.room == "bedroom"
    assert item.xyz == pytest.approx((0.43, 1.755, 0.612))


def test_s2026_005_named_person_at_the_named_room_no_objects(knowledge, placements):
    text = "introduce yourself to Liam in the living_room and tell what day is today"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert [it.kind for it in plan.items] == ["person"]
    assert plan.items[0].room == "living_room"
    assert plan.items[0].xyz == pytest.approx((-4.684, -4.089, 0.0))


def test_s2026_006_named_person_room_from_the_explicit_spot_not_the_later_room(knowledge, placements):
    # Two location words appear ("laundry_desk", "kitchen"); the explicit
    # spot's owning room (laundry_room) wins over the later plain room word.
    text = "meet Sarah at the laundry_desk then locate them in the kitchen"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert [it.kind for it in plan.items] == ["person"]
    assert plan.items[0].room == "laundry_room"
    assert plan.items[0].xyz == pytest.approx((-2.931, 4.663, 0.0))


def test_s2026_007_named_person_at_laundry_room(knowledge, placements):
    text = "meet Liam in the laundry_room and say the day of the month"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert [it.kind for it in plan.items] == ["person"]
    assert plan.items[0].room == "laundry_room"
    assert plan.items[0].xyz == pytest.approx((-2.931, 4.663, 0.0))


# --- deterministic category sampling ---------------------------------------

def test_category_sampling_is_deterministic_by_seed(knowledge, placements):
    text = "tell me how many kitchen items there are on the kitchen_table"
    plan_a = plan_scene(text, knowledge, placements, seed=2026)
    plan_b = plan_scene(text, knowledge, placements, seed=2026)
    assert _names(plan_a.items) == _names(plan_b.items)

    plan_other_seed = plan_scene(text, knowledge, placements, seed=7)
    assert len(plan_other_seed.items) == 3


# --- unparseable / no-content text -----------------------------------------

def test_unrecognized_text_returns_an_empty_plan_with_a_note(knowledge, placements):
    plan = plan_scene("asdf qwerty zxcv", knowledge, placements, seed=1)
    assert plan.items == ()
    assert any("no object" in note for note in plan.notes)


def test_empty_text_never_raises(knowledge, placements):
    plan = plan_scene("", knowledge, placements, seed=1)
    assert plan.items == ()


# --- grid layout never duplicates xyz ---------------------------------------

def test_grid_layout_never_duplicates_xyz_for_a_counted_object(knowledge, placements):
    text = "tell me how many spam there are on the kitchen_table"
    plan = plan_scene(text, knowledge, placements, seed=1)
    assert len(plan.items) == 3
    xy_pairs = [(it.xyz[0], it.xyz[1]) for it in plan.items]
    assert len(set(xy_pairs)) == 3


# --- asset resolution --------------------------------------------------------

def test_every_category_map_member_has_a_ycb_asset_mapping():
    all_members = {name for members in CATEGORY_MAP.values() for name in members}
    assert all_members <= set(NAME_TO_YCB_DIR)


def test_resolved_object_items_carry_a_nonempty_asset_uri(knowledge, placements):
    text = "go to the bedroom then locate a pudding_box and fetch it and put it on the side_table_02"
    plan = plan_scene(text, knowledge, placements, seed=1)
    assert plan.items[0].asset_uri.endswith("ycb_008_pudding_box/object.usd")


def test_person_items_use_the_bench_scenario_person_asset(knowledge, placements):
    plan = plan_scene("meet Liam in the laundry_room", knowledge, placements, seed=1)
    assert plan.items[0].asset_uri == PERSON_ASSET_URI


# --- JSON round-trip ----------------------------------------------------------

def test_scene_plan_json_round_trips(knowledge, placements):
    text = "go to the bedroom then locate a pudding_box and fetch it and put it on the side_table_02"
    plan = plan_scene(text, knowledge, placements, seed=1)
    data = scene_plan_to_json(plan, command_text=text, seed=1)
    restored = scene_plan_from_json(data)
    assert restored == plan


# --- fix round 1 -------------------------------------------------------------
# Issue 1: person room must come only from a room/spot NAMED IN THE TEXT, never
# from an object's location that was silently defaulted via default_locations.

def test_person_room_never_leaks_a_defaulted_object_location(knowledge, placements):
    # "banana" names no location; its spawn spot is defaulted (bedroom's
    # side_table_02, per default_locations) -- that default must not leak
    # into the (unrelated) person's room, which falls back to living_room.
    plan = plan_scene("guide me to find a banana", knowledge, placements, seed=1)
    person_items = [it for it in plan.items if it.kind == "person"]
    assert len(person_items) == 1
    assert person_items[0].room == "living_room"

    # "kitchen_table" IS named in the text, so it (and its owning room,
    # kitchen) is fair game for the person's room even with no object named.
    plan2 = plan_scene("guide me to the kitchen_table", knowledge, placements, seed=1)
    person_items2 = [it for it in plan2.items if it.kind == "person"]
    assert len(person_items2) == 1
    assert person_items2[0].room == "kitchen"


# Issue 2: every explicitly named object spawns (1 instance each, in order of
# first appearance in the text); the counting template's x3 applies only to
# the first-mentioned object.

def test_two_named_objects_each_spawn_one_in_mention_order(knowledge, placements):
    text = "grasp the mug and place it next to the bowl"
    plan = plan_scene(text, knowledge, placements, seed=1)
    assert _names(plan.items) == ["mug", "bowl"]
    assert _ids(plan.items) == ["cmd_mug_0", "cmd_bowl_0"]


def test_counting_template_multiplies_only_the_first_named_object(knowledge, placements):
    text = "how many mugs are on the kitchen_table next to the bowl"
    plan = plan_scene(text, knowledge, placements, seed=1)
    assert _names(plan.items) == ["mug", "mug", "mug", "bowl"]
    assert _ids(plan.items) == ["cmd_mug_0", "cmd_mug_1", "cmd_mug_2", "cmd_bowl_0"]


# Issue 3: a malformed placements dict (missing "spots") degrades to an empty
# plan with a note, per the module's "never raises" contract -- not a KeyError.

def test_malformed_placements_missing_spots_key_never_raises(knowledge):
    plan = plan_scene("bring me a mug", knowledge, {}, seed=1)
    assert plan.items == ()
    assert any("no placement entry" in note for note in plan.notes)


# --- fix round 2 --------------------------------------------------------------
# Issue: objects resolve to the FIRST-MENTIONED location in text order among
# all spot and room mentions (position, not spot-vs-room priority). The
# s2026-003 corpus regression itself is covered above by
# test_s2026_003_finds_kitchen_item_category_at_the_first_mentioned_room.

def test_bedroom_first_then_named_spot_still_resolves_to_that_spot(knowledge, placements):
    # "bedroom" is named first; its first search spot is side_table_02,
    # which also happens to be the later-named spot -- same result as
    # before the fix, but now via first-mention-wins rather than
    # spot-priority.
    text = "go to the bedroom then locate a pudding_box and fetch it and put it on the side_table_02"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert len(plan.items) == 1
    item = plan.items[0]
    assert item.name == "pudding_box"
    assert item.spot == "side_table_02"
    assert item.room == "bedroom"


def test_spot_only_text_is_unaffected_by_first_mention_change(knowledge, placements):
    text = "bring me a spam from the laundry_desk"
    plan = plan_scene(text, knowledge, placements, seed=1)

    assert len(plan.items) == 1
    item = plan.items[0]
    assert item.name == "spam"
    assert item.spot == "laundry_desk"
    assert item.room == "laundry_room"


def test_first_mentioned_spot_wins_over_a_later_mentioned_room(knowledge, placements):
    # "kitchen_table" (a spot) is named before "bedroom" (a room); the
    # first-mentioned spot wins even though it appears first among mixed
    # spot/room mentions.
    text = "take the mug from the kitchen_table to the bedroom"
    plan = plan_scene(text, knowledge, placements, seed=1)

    assert len(plan.items) == 1
    item = plan.items[0]
    assert item.name == "mug"
    assert item.spot == "kitchen_table"
    assert item.room == "kitchen"
