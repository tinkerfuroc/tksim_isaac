#!/usr/bin/env python3
"""Command-driven GPSR scene planner.

Reads a GPSR command's text and DEC's constants.rcw2026.json (rooms,
placement spots, object vocabulary) plus this repo's
simulation/scenarios/rcw2026-placements.json (measured surface/person
poses) and works out which objects (and person) that command's scene
actually needs, and where. Pure stdlib, no ROS -- tools/gpsr_spawn.py is
the only thing that talks to the simulator.

Never raises on an unparseable command: an unrecognised object/category,
location, or person mention degrades to an empty (or partial) ScenePlan
with an explanatory note in `.notes`, never an exception.
"""
from __future__ import annotations

import json
import random
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parent.parent

CATEGORY_MAP: dict[str, tuple[str, ...]] = {
    "food": ("banana", "spam", "pudding_box", "sugar_box", "cheez_it", "soup"),
    "drink": ("mug", "soup"),
    "kitchen item": ("bowl", "mug", "mustard", "bleach"),
}

NAME_TO_YCB_DIR: dict[str, str] = {
    "soup": "ycb_010_tomato_soup_can",
    "mug": "ycb_025_mug",
    "banana": "ycb_011_banana",
    "mustard": "ycb_006_mustard_bottle",
    "sugar_box": "ycb_002_sugar_box",
    "spam": "ycb_005_spam",
    "cheez_it": "ycb_001_cheez-it",
    "pudding_box": "ycb_008_pudding_box",
    "bowl": "ycb_024_bowl",
    "bleach": "ycb_021_bleach_cleanser",
}

# The bench scenario's own actor asset -- there is no per-person asset
# variety modelled yet, so every spawned person uses this same USD.
PERSON_ASSET_URI = (
    "artifacts/people/sobits/d29ee5ef3b71bcbf7013ec61785a584b162bfda83a322ecce0a6a481180e531a"
    "/person_standing/person.usd"
)

_PERSON_NAMES = ("alex", "sarah", "john", "emma", "liam", "olivia")
_PERSON_TRIGGER_VERBS = ("greet", "meet", "guide", "follow")


@dataclass(frozen=True)
class SpawnItem:
    id: str
    kind: str  # "object" | "person"
    name: str
    asset_uri: str
    room: str
    spot: str  # "" for persons (they use room, not a placement spot)
    xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class ScenePlan:
    items: tuple[SpawnItem, ...]
    notes: tuple[str, ...]


def _normalize_words(text: str) -> str:
    return re.sub(r"_+", " ", text).lower()


def _phrase_pattern(phrase: str) -> re.Pattern:
    words = _normalize_words(phrase).split()
    escaped = [re.escape(w) for w in words]
    if escaped:
        escaped[-1] = escaped[-1] + "s?"
    return re.compile(r"\b" + r"\s+".join(escaped) + r"\b")


def _text_contains(text: str, phrase: str) -> bool:
    return _phrase_pattern(phrase).search(_normalize_words(text)) is not None


def _is_counting_template(text: str) -> bool:
    normalized = text.lower()
    return "how many" in normalized or re.search(r"\bcount\b", normalized) is not None


def _object_names(knowledge: dict) -> list[str]:
    # Longest-first: a shorter name that is a text-prefix of a longer one
    # (there are none among the current 10, but this stays correct if that
    # ever changes) must not shadow the longer, more specific match.
    return sorted(knowledge.get("possible_objects", {}), key=lambda n: (-len(n), n))


def _match_object_name(text: str, knowledge: dict) -> Optional[str]:
    for name in _object_names(knowledge):
        if _text_contains(text, name):
            return name
    return None


def _match_category(text: str) -> Optional[str]:
    for category in sorted(CATEGORY_MAP, key=lambda c: (-len(c), c)):
        if _text_contains(text, category):
            return category
    return None


def _resolve_location(text: str, knowledge: dict) -> tuple[Optional[str], Optional[str]]:
    """Return (room, spot) from an explicit mention in `text`. An explicit
    placement spot wins over an explicit room name (its owning room is
    used); neither present returns (None, None). Longest-name-first so
    "side_table_02" is preferred over the "side_table" it starts with.
    """
    search_spots = knowledge["search_spots"]
    spot_to_room = {
        s: r for r, spots in search_spots.items() if not r.startswith("_") for s in spots
    }
    for spot in sorted(spot_to_room, key=lambda s: (-len(s), s)):
        if _text_contains(text, spot):
            return spot_to_room[spot], spot
    for room in sorted(search_spots, key=lambda r: (-len(r), r)):
        if room.startswith("_"):
            continue
        if _text_contains(text, room):
            return room, search_spots[room][0]
    return None, None


def _person_triggered(text: str) -> bool:
    normalized = text.lower()
    if re.search(r"\bpersons?\b", normalized):
        return True
    if re.search(r"\bsomeone\b", normalized):
        return True
    words = re.findall(r"[a-z]+", normalized)
    if any(name in words for name in _PERSON_NAMES):
        return True
    if any(re.search(r"\b" + verb + r"\w*\b", normalized) for verb in _PERSON_TRIGGER_VERBS):
        return True
    return False


def _resolve_person_room(text: str, knowledge: dict, first_object_spot: Optional[str]) -> str:
    room, _spot = _resolve_location(text, knowledge)
    if room is not None:
        return room
    if first_object_spot is not None:
        spot_to_room = {
            s: r
            for r, spots in knowledge["search_spots"].items()
            if not r.startswith("_")
            for s in spots
        }
        return spot_to_room.get(first_object_spot, "living_room")
    return "living_room"


def _load_object_asset_uris(asset_root: Path) -> dict[str, str]:
    manifest_path = Path(asset_root) / "artifacts" / "asset-manifest.json"
    try:
        data = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    by_name: dict[str, str] = {}
    for entry in data.get("generated_object_usds", []):
        path = entry.get("path", "")
        for name, ycb_dir in NAME_TO_YCB_DIR.items():
            if f"/{ycb_dir}/object.usd" in path:
                by_name[name] = path
    return by_name


def plan_scene(
    command_text: str,
    knowledge: dict,
    placements: dict,
    *,
    seed: int,
    asset_root: Optional[Path] = None,
) -> ScenePlan:
    """Plan the objects/person a GPSR command's scene needs. Pure Python,
    no ROS; never raises -- an unparseable or location-less command
    degrades to an empty or partial plan with an explanatory note.
    """
    notes: list[str] = []
    items: list[SpawnItem] = []
    rng = random.Random(seed)
    by_name = _load_object_asset_uris(asset_root or REPO_ROOT)

    object_name = _match_object_name(command_text, knowledge)
    category = None if object_name else _match_category(command_text)
    counting = _is_counting_template(command_text)

    object_specs: list[tuple[str, int]] = []
    if object_name:
        count = 3 if counting else 1
        object_specs = [(object_name, i) for i in range(count)]
        notes.append(
            f"explicit object '{object_name}' x{count}"
            + (" (counting template)" if counting else "")
        )
    elif category:
        members = sorted(CATEGORY_MAP[category])
        k = min(3, len(members))
        sampled = rng.sample(members, k)
        object_specs = [(name, 0) for name in sampled]
        notes.append(f"category '{category}' sampled {sampled} (seed={seed})")
    else:
        notes.append("no object or category named; no objects spawned")

    resolved_spot: Optional[str] = None
    if object_specs:
        _room, resolved_spot = _resolve_location(command_text, knowledge)
        if resolved_spot is None:
            first_name = object_specs[0][0]
            resolved_spot = knowledge.get("default_locations", {}).get(first_name)
            if resolved_spot is None:
                resolved_spot = knowledge["search_spots"]["kitchen"][0]
            notes.append(f"no location named; defaulted to '{resolved_spot}'")
        spot_info = placements["spots"].get(resolved_spot)
        if spot_info is None:
            notes.append(f"spot '{resolved_spot}' has no placement entry; no objects spawned")
        else:
            surface_xyz = spot_info["surface_xyz"]
            grid_dx = spot_info["grid_dx"]
            grid_dy = spot_info["grid_dy"]
            spot_to_room = {
                s: r
                for r, spots in knowledge["search_spots"].items()
                if not r.startswith("_")
                for s in spots
            }
            item_room = spot_to_room.get(resolved_spot, "kitchen")
            for i, (name, index) in enumerate(object_specs):
                xyz = (
                    surface_xyz[0] + (i % 3) * grid_dx,
                    surface_xyz[1] + (i // 3) * grid_dy,
                    surface_xyz[2],
                )
                asset_uri = by_name.get(name, "")
                if not asset_uri:
                    notes.append(f"asset uri not found for '{name}'")
                items.append(
                    SpawnItem(
                        id=f"cmd_{name}_{index}",
                        kind="object",
                        name=name,
                        asset_uri=asset_uri,
                        room=item_room,
                        spot=resolved_spot,
                        xyz=xyz,
                        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                    )
                )

    if _person_triggered(command_text):
        person_room = _resolve_person_room(command_text, knowledge, resolved_spot)
        person_pose = placements.get("persons", {}).get(person_room)
        if person_pose is None:
            notes.append(f"person triggered but room '{person_room}' has no known pose; skipped")
        else:
            items.append(
                SpawnItem(
                    id=f"cmd_person_{person_room}",
                    kind="person",
                    name="person",
                    asset_uri=PERSON_ASSET_URI,
                    room=person_room,
                    spot="",
                    xyz=tuple(person_pose["xyz"]),
                    quaternion_xyzw=tuple(person_pose["quaternion_xyzw"]),
                )
            )
            notes.append(f"person at '{person_room}'")

    return ScenePlan(items=tuple(items), notes=tuple(notes))


def scene_plan_to_json(plan: ScenePlan, *, command_text: str, seed: int) -> dict:
    return {
        "schema_version": 1,
        "command": command_text,
        "seed": seed,
        "notes": list(plan.notes),
        "items": [
            {
                "id": item.id,
                "kind": item.kind,
                "name": item.name,
                "asset_uri": item.asset_uri,
                "room": item.room,
                "spot": item.spot,
                "xyz": list(item.xyz),
                "quaternion_xyzw": list(item.quaternion_xyzw),
            }
            for item in plan.items
        ],
    }


def scene_plan_from_json(data: dict) -> ScenePlan:
    items = tuple(
        SpawnItem(
            id=it["id"],
            kind=it["kind"],
            name=it["name"],
            asset_uri=it["asset_uri"],
            room=it["room"],
            spot=it["spot"],
            xyz=tuple(it["xyz"]),
            quaternion_xyzw=tuple(it["quaternion_xyzw"]),
        )
        for it in data.get("items", [])
    )
    notes = tuple(data.get("notes", []))
    return ScenePlan(items=items, notes=notes)
