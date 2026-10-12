"""Referee handover (/sim/referee/hand_object): name -> entity resolution, the
backend ack payload, and the gateway's JSON ack publication.

The live teleport (USD / tensor-view write) is GPU-only; here the pure resolver
is pinned exhaustively and the orchestration around it is driven with the
stage/physics helpers stubbed on the instance.
"""
from __future__ import annotations

import json
import queue
import sys
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.backend import (  # noqa: E402
    IsaacWholeRobotBackend,
    normalize_handover_name,
    resolve_handover_entity,
)
from tinker_sim_isaac.ros_gateway import RosStandardGateway  # noqa: E402

CHILDREN = [
    "/World/Scenario/cmd_soup_0",
    "/World/Scenario/cmd_soup_can_0",
    "/World/Scenario/cmd_sugar_box_0",
    "/World/Scenario/cmd_sugar_box_1",
    "/World/Scenario/cmd_person_kitchen",
    "/World/Scenario/soup_can_decoy",
]


# ----------------------------------------------------------------- resolver

@pytest.mark.parametrize("raw, key", [
    ("soup", "soup"), ("Sugar Box", "sugar_box"), ("sugar-box", "sugar_box"),
    ("  sugar_box ", "sugar_box"), ("cheez it", "cheez_it"), ("PUDDING  BOX", "pudding_box"),
])
def test_normalize_handover_name(raw, key):
    assert normalize_handover_name(raw) == key


def test_full_prim_path_taken_verbatim():
    assert resolve_handover_entity("/World/Scenario/cmd_spam_0", [])["path"] == "/World/Scenario/cmd_spam_0"


def test_bare_entity_id_must_exist():
    assert resolve_handover_entity("cmd_soup_0", CHILDREN)["path"] == "/World/Scenario/cmd_soup_0"
    r = resolve_handover_entity("cmd_spam_0", CHILDREN)
    assert "error" in r and "no entity" in r["error"]


def test_semantic_name_matches_whole_key_not_prefix():
    # "soup" must pick cmd_soup_0 and NOT cmd_soup_can_0 / soup_can_decoy.
    assert resolve_handover_entity("soup", CHILDREN)["path"] == "/World/Scenario/cmd_soup_0"
    assert resolve_handover_entity("soup can", CHILDREN)["path"] == "/World/Scenario/cmd_soup_can_0"


def test_semantic_name_with_space_and_case():
    only = [c for c in CHILDREN if c != "/World/Scenario/cmd_sugar_box_1"]
    assert resolve_handover_entity("Sugar Box", only)["path"] == "/World/Scenario/cmd_sugar_box_0"


def test_ambiguous_never_picks_one():
    r = resolve_handover_entity("sugar box", CHILDREN)
    assert "error" in r
    assert r["error"].startswith("ambiguous: 2 entities")
    assert "cmd_sugar_box_0" in r["error"] and "cmd_sugar_box_1" in r["error"]


def test_no_match_and_empty_and_missing_scope():
    assert "no entity for 'mug'" in resolve_handover_entity("mug", CHILDREN)["error"]
    assert resolve_handover_entity("", CHILDREN) == {"error": "empty entity"}
    assert resolve_handover_entity("   ", CHILDREN) == {"error": "empty entity"}
    assert "no /World/Scenario scope" in resolve_handover_entity(
        "soup", [], scenario_present=False)["error"]


# ------------------------------------------------------------------ backend

class _Tensor:
    """Minimal stand-in for a torch tensor row: [0, i] -> detach().cpu().tolist()."""

    def __init__(self, rows):
        self._rows = rows

    def __getitem__(self, index):
        _, i = index
        return SimpleNamespace(detach=lambda: SimpleNamespace(cpu=lambda: SimpleNamespace(
            tolist=lambda: list(self._rows[i]))))


def _backend(*, body_names=("link_base", "link_tcp"), tcp=(0.5, 0.1, 0.9)):
    b = object.__new__(IsaacWholeRobotBackend)
    rows = [[0.0, 0.0, 0.0]] * len(body_names)
    if "link_tcp" in body_names:
        rows[body_names.index("link_tcp")] = list(tcp)
    b._robot = SimpleNamespace(data=SimpleNamespace(body_names=body_names, body_pos_w=_Tensor(rows)))
    b._torch_value = lambda value: value
    b._use_fabric = False
    return b


def test_tcp_world_position_reads_link_tcp_body():
    assert _backend()._tcp_world_position() == [0.5, 0.1, 0.9]
    assert _backend(body_names=("link_base",))._tcp_world_position() is None


def test_hand_object_ok_ack_has_entity_path_and_tcp_xyz(capsys):
    b = _backend(tcp=(0.51234567, -0.2, 0.875))
    b._resolve_handover_entity = lambda e: {"path": "/World/Scenario/cmd_soup_0"}
    writes = []
    b._teleport_rigid_body = lambda path, xyz: writes.append((path, list(xyz))) or "usd"
    ack = b.hand_object_to_gripper("soup")
    assert ack == {"ok": True, "entity": "/World/Scenario/cmd_soup_0", "xyz": [0.5123, -0.2, 0.875]}
    assert writes == [("/World/Scenario/cmd_soup_0", [0.51234567, -0.2, 0.875])]
    log = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    assert log["referee_hand_object"] == "/World/Scenario/cmd_soup_0" and log["method"] == "usd"
    json.dumps(ack)  # ack is plain JSON


def test_hand_object_nack_on_unresolved_keeps_request_as_entity():
    b = _backend()
    b._resolve_handover_entity = lambda e: {"error": "ambiguous: 2 entities match 'soup' ([...])"}
    ack = b.hand_object_to_gripper("soup")
    assert ack["ok"] is False and ack["entity"] == "soup"
    assert ack["error"].startswith("ambiguous")
    assert "xyz" not in ack


def test_hand_object_nack_without_tcp_body():
    b = _backend(body_names=("link_base",))
    b._resolve_handover_entity = lambda e: {"path": "/World/Scenario/cmd_soup_0"}
    ack = b.hand_object_to_gripper("soup")
    assert ack == {"ok": False, "entity": "/World/Scenario/cmd_soup_0",
                   "error": "robot has no link_tcp body"}


def test_hand_object_nack_when_pose_write_raises():
    b = _backend()
    b._resolve_handover_entity = lambda e: {"path": "/World/Scenario/cmd_soup_0"}

    def _boom(path, xyz):
        raise RuntimeError("no rigid-body view for entity")

    b._teleport_rigid_body = _boom
    ack = b.hand_object_to_gripper("soup")
    assert ack["ok"] is False and ack["error"] == "pose write failed: no rigid-body view for entity"


def test_hand_object_empty_request_is_nack_without_touching_stage():
    b = _backend()
    assert b.hand_object_to_gripper("") == {"ok": False, "entity": "", "error": "empty entity"}


# ------------------------------------------------------------------ gateway

class _String:
    def __init__(self):
        self.data = ""


class _Pub:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg.data)


class _Logger:
    def __init__(self):
        self.errors = []

    def error(self, text):
        self.errors.append(text)


def _gateway(backend):
    g = object.__new__(RosStandardGateway)
    g.backend = backend
    g._incoming_events = queue.SimpleQueue()
    g._last_command_error = None
    g._String = _String
    g.referee_result_pub = _Pub()
    g.node = SimpleNamespace(get_logger=lambda: _Logger())
    logger = g.node.get_logger()
    g.node.get_logger = lambda: logger
    stats = defaultdict(int)
    g._spin_stats = lambda: stats
    g._take_pending = lambda: 0
    g._enforce_safety_deadline = lambda: None
    g._enforce_command_deadline = lambda: None
    return g


def test_gateway_topics_match_decision_side():
    assert RosStandardGateway.REFEREE_HAND_OBJECT_TOPIC == "/sim/referee/hand_object"
    assert RosStandardGateway.REFEREE_HAND_OBJECT_RESULT_TOPIC == "/sim/referee/hand_object_result"


def test_gateway_queues_request_and_acks_json_on_spin():
    calls = []

    def _hand(entity):
        calls.append(entity)
        return {"ok": True, "entity": "/World/Scenario/cmd_soup_0", "xyz": [0.5, 0.1, 0.9]}

    g = _gateway(SimpleNamespace(hand_object_to_gripper=_hand))
    g._referee_hand_object(SimpleNamespace(data="  soup \n"))
    assert calls == []  # applied on the simulation thread, not in the callback
    g.spin_once()
    assert calls == ["soup"]
    assert json.loads(g.referee_result_pub.published[0]) == {
        "ok": True, "entity": "/World/Scenario/cmd_soup_0", "xyz": [0.5, 0.1, 0.9]}
    assert g.node.get_logger().errors == []


def test_gateway_acks_nack_and_logs():
    g = _gateway(SimpleNamespace(hand_object_to_gripper=lambda e: {
        "ok": False, "entity": e, "error": "no entity for 'mug' under /World/Scenario"}))
    g._referee_hand_object(SimpleNamespace(data="mug"))
    g.spin_once()
    ack = json.loads(g.referee_result_pub.published[0])
    assert ack["ok"] is False and ack["error"].startswith("no entity")
    assert g.node.get_logger().errors and "hand_object failed" in g.node.get_logger().errors[0]


def test_gateway_backend_exception_becomes_nack_not_crash():
    def _boom(entity):
        raise RuntimeError("stage exploded")

    g = _gateway(SimpleNamespace(hand_object_to_gripper=_boom))
    g._referee_hand_object(SimpleNamespace(data="soup"))
    g.spin_once()  # must not raise
    ack = json.loads(g.referee_result_pub.published[0])
    assert ack == {"ok": False, "entity": "soup", "error": "stage exploded"}


def test_gateway_empty_request_is_nack_without_backend_call():
    g = _gateway(SimpleNamespace(hand_object_to_gripper=lambda e: pytest.fail("must not be called")))
    g._referee_hand_object(SimpleNamespace(data="   "))
    g.spin_once()
    assert json.loads(g.referee_result_pub.published[0]) == {
        "ok": False, "entity": "", "error": "empty entity"}


def test_semantic_name_prefers_bench_spawned_entity_over_base_scenario_twin():
    # fix4b run 003: "soup" picked the base-scenario /World/Scenario/soup on the
    # kitchen table instead of the spawned cmd_soup_0 in front of the robot.
    children = ["/World/Scenario/soup", "/World/Scenario/cmd_soup_0"]
    assert resolve_handover_entity("soup", children)["path"] == "/World/Scenario/cmd_soup_0"
    assert resolve_handover_entity("soup", ["/World/Scenario/soup"])["path"] == "/World/Scenario/soup"


def test_candidate_list_hands_over_the_first_that_exists():
    # fix4b run 004: the tree guessed "pudding_box", which was not on the table.
    children = ["/World/Scenario/cmd_banana_0", "/World/Scenario/cmd_sugar_box_0"]
    r = resolve_handover_entity("pudding_box|banana|sugar_box", children)
    assert r == {"path": "/World/Scenario/cmd_banana_0", "name": "banana"}
    r = resolve_handover_entity("mug|bowl", children)
    assert r["error"].startswith("no candidate resolved")
