import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.gpsr_scene import ScenePlan, SpawnItem  # noqa: E402
from tools.gpsr_spawn import (  # noqa: E402
    apply_plan,
    base_scenario_keys,
    clear_manifest,
    emit_scenario,
    main,
)

PLACEMENTS = json.loads((ROOT / "simulation" / "scenarios" / "rcw2026-placements.json").read_text())
BASE_SCENARIO = json.loads((ROOT / "simulation" / "scenarios" / "gpsr-rcw2026-bench.json").read_text())


class FakeServiceClient:
    def __init__(self, fail_ids=()):
        self.spawned = []
        self.deleted = []
        self.fail_ids = set(fail_ids)

    def spawn(self, item):
        if item.id in self.fail_ids:
            raise RuntimeError(f"spawn failed for {item.id}")
        self.spawned.append(item.id)
        return f"/World/Scenario/{item.id}"

    def delete(self, entity):
        self.deleted.append(entity)
        return True


def _item(id_, name, spot, room, xyz, kind="object",
         asset_uri="artifacts/objects/ycb/x/object.usd"):
    return SpawnItem(id=id_, kind=kind, name=name, asset_uri=asset_uri, room=room, spot=spot,
                     xyz=xyz, quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))


def test_base_scenario_keys_finds_the_four_bench_scenario_items():
    keys = base_scenario_keys(BASE_SCENARIO, PLACEMENTS)
    assert ("ycb_010_tomato_soup_can", "kitchen_table") in keys
    assert ("ycb_025_mug", "kitchen_table") in keys
    assert ("ycb_011_banana", "side_table_02") in keys
    assert ("ycb_024_bowl", "shelf_02") in keys
    assert ("person_standing", "kitchen") in keys
    assert ("person_standing", "living_room") in keys


def test_apply_plan_skips_items_already_in_the_base_scenario():
    plan = ScenePlan(
        items=(_item("cmd_soup_0", "soup", "kitchen_table", "kitchen", (2.5, -3.0, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_010_tomato_soup_can/object.usd"),),
        notes=(),
    )
    client = FakeServiceClient()
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    assert manifest["entities"] == []
    assert manifest["skipped"][0]["id"] == "cmd_soup_0"
    assert client.spawned == []


def test_apply_plan_spawns_a_new_item_and_records_the_entity_name():
    plan = ScenePlan(
        items=(_item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    client = FakeServiceClient()
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    assert manifest["skipped"] == []
    assert len(manifest["entities"]) == 1
    entity = manifest["entities"][0]
    assert entity["ok"] is True
    assert entity["entity_name"] == "/World/Scenario/cmd_spam_0"
    assert client.spawned == ["cmd_spam_0"]


def test_apply_plan_records_a_failed_spawn_without_raising():
    plan = ScenePlan(
        items=(_item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    client = FakeServiceClient(fail_ids={"cmd_spam_0"})
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    assert manifest["entities"][0]["ok"] is False
    assert "spawn failed" in manifest["entities"][0]["error"]


def test_apply_plan_skips_items_already_in_a_previous_manifest():
    plan = ScenePlan(
        items=(_item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    client = FakeServiceClient()
    previous = {
        "entities": [
            {"id": "cmd_spam_0", "asset_key": "ycb_005_spam", "where": "laundry_desk",
             "entity_name": "/World/Scenario/cmd_spam_0", "ok": True}
        ],
        "skipped": [],
    }
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS,
                          previous_manifest=previous)
    assert manifest["skipped"][0]["id"] == "cmd_spam_0"
    assert client.spawned == []


def test_clear_manifest_deletes_every_ok_entity_and_tolerates_missing():
    manifest = {
        "entities": [
            {"id": "cmd_spam_0", "entity_name": "/World/Scenario/cmd_spam_0", "ok": True,
             "asset_key": "ycb_005_spam", "where": "laundry_desk"},
            {"id": "cmd_bad_0", "entity_name": "", "ok": False, "asset_key": "x", "where": "y"},
        ],
        "skipped": [],
    }
    client = FakeServiceClient()
    updated = clear_manifest(manifest, client)
    assert client.deleted == ["/World/Scenario/cmd_spam_0"]
    assert updated["entities"][0]["cleared"] is True
    assert "cleared" not in updated["entities"][1]


def test_emit_scenario_merges_a_new_item_into_a_copy_of_the_base_scenario():
    plan = ScenePlan(
        items=(_item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    merged = emit_scenario([plan], BASE_SCENARIO, PLACEMENTS)
    assert len(BASE_SCENARIO["objects"]) == 4  # original untouched
    assert len(merged["objects"]) == len(BASE_SCENARIO["objects"]) + 1
    new_ids = {o["id"] for o in merged["objects"]} - {o["id"] for o in BASE_SCENARIO["objects"]}
    assert len(new_ids) == 1


def test_emit_scenario_dedupes_against_the_base_scenario():
    plan = ScenePlan(
        items=(_item("cmd_soup_0", "soup", "kitchen_table", "kitchen", (2.5, -3.0, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_010_tomato_soup_can/object.usd"),),
        notes=(),
    )
    merged = emit_scenario([plan], BASE_SCENARIO, PLACEMENTS)
    assert len(merged["objects"]) == len(BASE_SCENARIO["objects"])


def test_emit_scenario_larger_count_wins_between_two_plans():
    small = ScenePlan(
        items=(_item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    big = ScenePlan(
        items=tuple(
            _item(f"cmd_spam_{i}", "spam", "laundry_desk", "laundry_room",
                 (-2.988 + 0.18 * i, 4.525, 0.734),
                 asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd")
            for i in range(3)
        ),
        notes=(),
    )
    merged = emit_scenario([small, big], BASE_SCENARIO, PLACEMENTS)
    spam_items = [o for o in merged["objects"] if "spam" in o["asset_uri"]]
    assert len(spam_items) == 3


def test_cli_plan_writes_a_scene_plan_json(tmp_path):
    out = tmp_path / "scene-plan.json"
    code = main([
        "plan", "--command",
        "go to the bedroom then locate a pudding_box and fetch it and put it on the side_table_02",
        "--seed", "1", "--out", str(out),
    ])
    assert code == 0
    data = json.loads(out.read_text())
    assert data["items"][0]["name"] == "pudding_box"


def test_cli_emit_scenario_merges_plans(tmp_path):
    plan_path = tmp_path / "scene-plan.json"
    main(["plan", "--command", "bring me a spam from the laundry_desk", "--seed", "1",
         "--out", str(plan_path)])
    out = tmp_path / "generated.json"
    code = main([
        "emit-scenario", "--plans", str(plan_path),
        "--base", str(ROOT / "simulation" / "scenarios" / "gpsr-rcw2026-bench.json"),
        "--placements", str(ROOT / "simulation" / "scenarios" / "rcw2026-placements.json"),
        "--out", str(out),
    ])
    assert code == 0
    merged = json.loads(out.read_text())
    assert len(merged["objects"]) >= len(BASE_SCENARIO["objects"])
