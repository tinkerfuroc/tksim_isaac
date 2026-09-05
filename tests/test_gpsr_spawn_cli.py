import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.gpsr_scene import ScenePlan, SpawnItem, scene_plan_to_json  # noqa: E402
from tools.gpsr_spawn import (  # noqa: E402
    EntityAlreadyExists,
    ServiceUnavailable,
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


class _AlreadyExistsClient(FakeServiceClient):
    def spawn(self, item):
        raise EntityAlreadyExists(
            f"spawn_entity failed for {item.id}: Entity '/World/Scenario/{item.id}' already exists "
            "and allow_renaming is false"
        )


def test_apply_plan_treats_an_already_existing_entity_as_present():
    # Live 2026-08-29 (s2026-007): the first spawn_entity reply took >30 s, the tool marked
    # the item failed, and the retry got "already exists" -- but the entity IS in the sim,
    # which is what the caller wanted. Record it ok (and clear-able), with a note.
    plan = ScenePlan(
        items=(_item("cmd_person_laundry_room", "person", None, "laundry_room", (-2.931, 4.663, 0.0),
                     kind="person", asset_uri="artifacts/people/person_standing/person.usd"),),
        notes=(),
    )
    manifest = apply_plan(plan, _AlreadyExistsClient(), base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    entry = manifest["entities"][0]
    assert entry["ok"] is True
    assert entry["entity_name"] == "/World/Scenario/cmd_person_laundry_room"
    assert "already" in entry["note"]
    assert "error" not in entry
    assert manifest["not_attempted"] == []


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


def test_apply_plan_retries_a_previously_failed_item_and_replaces_its_manifest_record():
    # Fix round 1, finding 1: a previous ok:False entry must not count as
    # "already present" -- the item is retried (even under a new id, since
    # a replan can rename it) and the stale failed record is dropped in
    # favour of the fresh attempt's result.
    plan = ScenePlan(
        items=(_item("cmd_spam_1", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    client = FakeServiceClient()
    previous = {
        "entities": [
            {"id": "cmd_spam_0", "asset_key": "ycb_005_spam", "where": "laundry_desk",
             "entity_name": "", "ok": False, "error": "boom"}
        ],
        "skipped": [],
    }
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS,
                          previous_manifest=previous)
    assert client.spawned == ["cmd_spam_1"]
    assert manifest["skipped"] == []
    assert len(manifest["entities"]) == 1
    assert manifest["entities"][0]["id"] == "cmd_spam_1"
    assert manifest["entities"][0]["ok"] is True


def test_apply_plan_stops_after_service_unavailable_and_marks_remaining_not_attempted():
    # Fix round 1, finding 2: a ServiceUnavailable spawn failure is an
    # infra-level outage, not a per-item failure -- apply_plan must stop
    # attempting further items (exactly one spawn call) and record the
    # rest as not_attempted instead.
    plan = ScenePlan(
        items=(
            _item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                 asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),
            _item("cmd_mustard_0", "mustard", "shelf", "laundry_room", (-3.687, 0.309, 1.07),
                 asset_uri="artifacts/objects/ycb/x/ycb_006_mustard_bottle/object.usd"),
        ),
        notes=(),
    )

    class OutageClient:
        def __init__(self):
            self.calls = 0
            self.deleted = []

        def spawn(self, item):
            self.calls += 1
            raise ServiceUnavailable("/spawn_entity timed out")

        def delete(self, entity):
            self.deleted.append(entity)
            return True

    client = OutageClient()
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    assert client.calls == 1
    assert manifest["entities"][0]["id"] == "cmd_spam_0"
    assert manifest["entities"][0]["ok"] is False
    assert len(manifest["not_attempted"]) == 1
    assert manifest["not_attempted"][0]["id"] == "cmd_mustard_0"
    assert manifest["not_attempted"][0]["error"] == "not attempted: service unavailable"


def test_cli_apply_exits_2_on_service_unavailable_and_calls_spawn_once(tmp_path, monkeypatch):
    # Fix round 1, finding 2 (CLI-level): the same outage propagates to a
    # process exit code of 2, and _make_ros_service_client is monkeypatched
    # so this stays ROS-free.
    plan = ScenePlan(
        items=(
            _item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                 asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),
            _item("cmd_mustard_0", "mustard", "shelf", "laundry_room", (-3.687, 0.309, 1.07),
                 asset_uri="artifacts/objects/ycb/x/ycb_006_mustard_bottle/object.usd"),
        ),
        notes=(),
    )
    plan_path = tmp_path / "plan.json"
    plan_path.write_text(json.dumps(scene_plan_to_json(plan, command_text="x", seed=1)))
    manifest_path = tmp_path / "manifest.json"

    class OutageClient:
        def __init__(self):
            self.calls = 0

        def spawn(self, item):
            self.calls += 1
            raise ServiceUnavailable("/spawn_entity timed out")

        def delete(self, entity):
            return True

        def shutdown(self):
            pass

    outage_client = OutageClient()
    monkeypatch.setattr("tools.gpsr_spawn._make_ros_service_client", lambda: outage_client)

    code = main([
        "apply", "--plan", str(plan_path), "--manifest", str(manifest_path),
        "--base-scenario", str(ROOT / "simulation" / "scenarios" / "gpsr-rcw2026-bench.json"),
        "--placements", str(ROOT / "simulation" / "scenarios" / "rcw2026-placements.json"),
    ])
    assert code == 2
    assert outage_client.calls == 1


def test_apply_plan_calls_on_progress_once_per_item_with_cumulative_manifest():
    # Fix round 1, finding 4: on_progress is called after every item with
    # the manifest-so-far (used by the CLI to rewrite the manifest file
    # incrementally).
    plan = ScenePlan(
        items=(
            _item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                 asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),
            _item("cmd_mustard_0", "mustard", "shelf", "laundry_room", (-3.687, 0.309, 1.07),
                 asset_uri="artifacts/objects/ycb/x/ycb_006_mustard_bottle/object.usd"),
        ),
        notes=(),
    )
    client = FakeServiceClient()
    snapshots = []
    apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS,
              on_progress=lambda m: snapshots.append(json.loads(json.dumps(m))))
    assert len(snapshots) == 2
    assert len(snapshots[0]["entities"]) == 1
    assert snapshots[0]["entities"][0]["id"] == "cmd_spam_0"
    assert len(snapshots[1]["entities"]) == 2


def test_clear_manifest_calls_on_progress_once_per_entity():
    # Fix round 1, finding 4 (clear_manifest side).
    manifest = {
        "entities": [
            {"id": "cmd_spam_0", "entity_name": "/World/Scenario/cmd_spam_0", "ok": True,
             "asset_key": "ycb_005_spam", "where": "laundry_desk"},
            {"id": "cmd_mustard_0", "entity_name": "/World/Scenario/cmd_mustard_0", "ok": True,
             "asset_key": "ycb_006_mustard_bottle", "where": "shelf"},
        ],
        "skipped": [],
    }
    client = FakeServiceClient()
    snapshots = []
    clear_manifest(manifest, client,
                   on_progress=lambda m: snapshots.append(json.loads(json.dumps(m))))
    assert len(snapshots) == 2
    assert snapshots[0]["entities"][0]["cleared"] is True
    assert len(snapshots[1]["entities"]) == 2


def test_clear_manifest_preserves_not_attempted_in_return_and_every_progress_snapshot():
    # Fix round 2: clear_manifest used to return/persist only
    # {"entities", "skipped"}, silently dropping a partial-outage apply's
    # "not_attempted" list. It (and every top-level key it doesn't own)
    # must survive unchanged through both the return value and every
    # on_progress snapshot.
    manifest = {
        "entities": [
            {"id": "cmd_spam_0", "entity_name": "/World/Scenario/cmd_spam_0", "ok": True,
             "asset_key": "ycb_005_spam", "where": "laundry_desk"},
        ],
        "skipped": [],
        "not_attempted": [
            {"id": "cmd_mustard_0", "ok": False, "error": "not attempted: service unavailable"}
        ],
    }
    client = FakeServiceClient()
    snapshots = []
    updated = clear_manifest(manifest, client,
                             on_progress=lambda m: snapshots.append(json.loads(json.dumps(m))))
    assert updated["not_attempted"] == manifest["not_attempted"]
    assert len(snapshots) == 1
    assert snapshots[0]["not_attempted"] == manifest["not_attempted"]


def test_apply_plan_records_pose_fields_on_each_entity():
    # Fix round 1, finding 5: manifest entities carry the spawned item's
    # xyz/quaternion_xyzw (spec 2.3: "entity names, poses, skipped items,
    # service results").
    plan = ScenePlan(
        items=(_item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    client = FakeServiceClient()
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    entity = manifest["entities"][0]
    assert entity["xyz"] == [-2.988, 4.525, 0.734]
    assert entity["quaternion_xyzw"] == [0.0, 0.0, 0.0, 1.0]


def test_apply_plan_count_aware_dedupe_spawns_remainder_and_slots_past_a_different_asset():
    # Fix round 4: a counting command (3 instances of the same key) must
    # not be treated as fully-present just because one instance of the
    # key already exists -- dedupe is count-aware. A different-asset
    # object already at the same spot (any asset) still claims grid slot
    # 0, so the 3 new spawns land on slots 1, 2, 3.
    base_scenario_with_a_mug_at_kitchen_table = {
        "objects": [
            {
                "id": "obj_mug",
                "asset_uri": "artifacts/objects/ycb/x/ycb_025_mug/object.usd",
                "pose": {"xyz": [2.5, -3.0, 0.734], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            }
        ],
        "actors": [],
    }
    plan = ScenePlan(
        items=tuple(
            _item(f"cmd_pudding_box_{i}", "pudding_box", "kitchen_table", "kitchen",
                 (2.5 + 0.18 * i, -3.0, 0.734),
                 asset_uri="artifacts/objects/ycb/x/ycb_008_pudding_box/object.usd")
            for i in range(3)
        ),
        notes=(),
    )
    client = FakeServiceClient()
    manifest = apply_plan(plan, client, base_scenario=base_scenario_with_a_mug_at_kitchen_table,
                          placements=PLACEMENTS)
    assert manifest["skipped"] == []
    assert client.spawned == ["cmd_pudding_box_0", "cmd_pudding_box_1", "cmd_pudding_box_2"]
    assert len(manifest["entities"]) == 3
    kitchen_table = PLACEMENTS["spots"]["kitchen_table"]
    surface_xyz, dx, dy = kitchen_table["surface_xyz"], kitchen_table["grid_dx"], kitchen_table["grid_dy"]
    expected_slots = [1, 2, 3]
    for entity, k in zip(manifest["entities"], expected_slots):
        assert entity["ok"] is True
        assert entity["xyz"] == pytest.approx(
            [surface_xyz[0] + (k % 3) * dx, surface_xyz[1] + (k // 3) * dy, surface_xyz[2]]
        )
        # slot 0 (the pre-existing mug's spot) is never reused
        assert entity["xyz"] != pytest.approx(list(surface_xyz))


def test_apply_plan_count_aware_dedupe_against_a_previous_manifest_ok_entity():
    # A previous manifest already has 1 ok pudding_box at kitchen_table
    # (from an earlier apply of the same command); a fresh 3-instance plan
    # must skip only that 1 and spawn the remaining 2.
    plan = ScenePlan(
        items=tuple(
            _item(f"cmd_pudding_box_{i}", "pudding_box", "kitchen_table", "kitchen",
                 (2.5 + 0.18 * i, -3.0, 0.734),
                 asset_uri="artifacts/objects/ycb/x/ycb_008_pudding_box/object.usd")
            for i in range(3)
        ),
        notes=(),
    )
    previous = {
        "entities": [
            {"id": "cmd_pudding_box_0", "asset_key": "ycb_008_pudding_box", "where": "kitchen_table",
             "entity_name": "/World/Scenario/cmd_pudding_box_0", "ok": True}
        ],
        "skipped": [],
    }
    client = FakeServiceClient()
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS,
                          previous_manifest=previous)
    assert len(manifest["skipped"]) == 1
    assert manifest["skipped"][0]["id"] == "cmd_pudding_box_0"
    assert client.spawned == ["cmd_pudding_box_1", "cmd_pudding_box_2"]
    assert len([e for e in manifest["entities"] if e.get("ok")]) == 3  # 1 carried + 2 fresh


def test_apply_plan_count_aware_dedupe_against_a_same_asset_base_scenario_item():
    # The bench scenario already has 1 soup at kitchen_table; a plan
    # asking for 3 soups at kitchen_table must skip only that 1 and spawn
    # the other 2 (not treat the whole key as already-satisfied).
    plan = ScenePlan(
        items=tuple(
            _item(f"cmd_soup_{i}", "soup", "kitchen_table", "kitchen",
                 (2.5 + 0.18 * i, -3.0, 0.734),
                 asset_uri="artifacts/objects/ycb/x/ycb_010_tomato_soup_can/object.usd")
            for i in range(3)
        ),
        notes=(),
    )
    client = FakeServiceClient()
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    assert len(manifest["skipped"]) == 1
    assert manifest["skipped"][0]["id"] == "cmd_soup_0"
    assert client.spawned == ["cmd_soup_1", "cmd_soup_2"]


def test_emit_scenario_count_aware_dedupe_adds_only_the_excess_over_existing_count():
    # Fix round 4: emit_scenario's dedupe must be count-aware like
    # apply_plan's, not presence-only -- the base scenario already has 1
    # soup at kitchen_table; a plan asking for 3 soups there should merge
    # in only the excess 2, not 0 (old presence-only behaviour) and not 3.
    plan = ScenePlan(
        items=tuple(
            _item(f"cmd_soup_{i}", "soup", "kitchen_table", "kitchen",
                 (2.5 + 0.18 * i, -3.0, 0.734),
                 asset_uri="artifacts/objects/ycb/x/ycb_010_tomato_soup_can/object.usd")
            for i in range(3)
        ),
        notes=(),
    )
    merged = emit_scenario([plan], BASE_SCENARIO, PLACEMENTS)
    new_soup_objects = [o for o in merged["objects"] if "tomato_soup" in o["asset_uri"]
                        and o["id"] not in {b["id"] for b in BASE_SCENARIO["objects"]}]
    assert len(new_soup_objects) == 2


def test_emit_scenario_merged_objects_never_share_xyz_with_base_objects_at_that_spot():
    # Fix round 4: the bench scenario already has soup + mug at
    # kitchen_table (slots 0 and thereabouts); a plan merging a new asset
    # into kitchen_table must be re-sloted past both, never landing on
    # either's exact xyz.
    plan = ScenePlan(
        items=(_item("cmd_spam_0", "spam", "kitchen_table", "kitchen", (2.5, -3.0, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    merged = emit_scenario([plan], BASE_SCENARIO, PLACEMENTS)
    base_xyzs = {tuple(o["pose"]["xyz"]) for o in BASE_SCENARIO["objects"]}
    new_objects = [o for o in merged["objects"] if o["id"] not in
                  {b["id"] for b in BASE_SCENARIO["objects"]}]
    assert len(new_objects) == 1
    assert tuple(new_objects[0]["pose"]["xyz"]) not in base_xyzs


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


class _FlakyDeleteClient(FakeServiceClient):
    """delete() raises ServiceUnavailable the first `flaky` times per entity, then succeeds."""

    def __init__(self, flaky: int):
        super().__init__()
        self.flaky = flaky
        self.attempts: dict[str, int] = {}

    def delete(self, entity):
        n = self.attempts.get(entity, 0) + 1
        self.attempts[entity] = n
        if n <= self.flaky:
            raise ServiceUnavailable(f"delete_entity timed out for {entity}")
        return super().delete(entity)


def test_clear_manifest_retries_a_timed_out_delete_once():
    # Live 2026-08-29: the first delete_entity right after orchestrator teardown timed out
    # once (cmd_bleach_0), and a manual re-run cleared it. One retry absorbs that.
    manifest = {"entities": [
        {"id": "cmd_bleach_0", "entity_name": "/World/Scenario/cmd_bleach_0", "ok": True,
         "asset_key": "ycb_021_bleach_cleanser", "where": "sofa"}], "skipped": []}
    client = _FlakyDeleteClient(flaky=1)
    updated = clear_manifest(manifest, client)
    assert client.attempts["/World/Scenario/cmd_bleach_0"] == 2
    assert updated["entities"][0]["cleared"] is True
    assert "clear_error" not in updated["entities"][0]


def test_clear_manifest_gives_up_after_the_retry_and_records_the_error():
    manifest = {"entities": [
        {"id": "cmd_bleach_0", "entity_name": "/World/Scenario/cmd_bleach_0", "ok": True,
         "asset_key": "ycb_021_bleach_cleanser", "where": "sofa"}], "skipped": []}
    client = _FlakyDeleteClient(flaky=5)
    updated = clear_manifest(manifest, client)
    assert client.attempts["/World/Scenario/cmd_bleach_0"] == 2
    assert updated["entities"][0]["cleared"] is False
    assert "timed out" in updated["entities"][0]["clear_error"]


def test_clear_manifest_drops_a_stale_clear_error_once_the_entity_is_cleared():
    # A re-run of `clear` on a manifest whose earlier attempt failed must not carry the
    # old error text next to "cleared": true.
    manifest = {"entities": [
        {"id": "cmd_bleach_0", "entity_name": "/World/Scenario/cmd_bleach_0", "ok": True,
         "asset_key": "ycb_021_bleach_cleanser", "where": "sofa", "cleared": False,
         "clear_error": "ServiceUnavailable('delete_entity timed out')"}], "skipped": []}
    updated = clear_manifest(manifest, FakeServiceClient())
    assert updated["entities"][0]["cleared"] is True
    assert "clear_error" not in updated["entities"][0]


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

def test_cli_runs_as_a_standalone_script_with_no_pythonpath(tmp_path):
    # Fix round 3: the bench invokes this file as a bare script from
    # another repo (`python3 /abs/path/tools/gpsr_spawn.py ...`), with no
    # PYTHONPATH help -- `from tools.gpsr_scene import ...` used to raise
    # ModuleNotFoundError in that mode (it only resolved under pytest,
    # which already has the repo root on sys.path). Exercise the actual
    # failure mode: a subprocess whose env has PATH only, no PYTHONPATH
    # and no cwd assumption tying it to the repo.
    out = tmp_path / "p.json"
    result = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "gpsr_spawn.py"),
         "plan", "--command", "bring me a mug", "--seed", "1", "--out", str(out)],
        env={"PATH": os.environ["PATH"]},
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert out.is_file()
