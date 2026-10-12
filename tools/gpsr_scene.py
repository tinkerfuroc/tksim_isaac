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

Person mentions pick a people-artifact asset that matches the command's
gesture/pose phrase ("waving person", "person raising their left arm",
"sitting person", "lying person", ...): `classify_person_phrase` labels
the mention, `PERSON_ASSET_FALLBACKS` maps the label to an ordered chain
of asset keys, and `choose_person_asset` takes the first one present in
the current people artifact manifest (else `person_standing`, noted).
"lying" has no asset: `lying_pose` lays the standing model on its back.
"""
from __future__ import annotations

import json
import math
import random
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Collection, Optional

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

# Object footprints (full x/y size, metres, identity yaw) measured 2026-09-27
# with pxr (UsdGeom.BBoxCache) from the YCB object artifact this repo
# spawns (artifacts/objects/ycb/b533a2e5.../<key>/object.usd). Keyed by the
# YCB asset key (`NAME_TO_YCB_DIR` values / gpsr_spawn's `asset_key`).
OBJECT_FOOTPRINT_XY: dict[str, tuple[float, float]] = {
    "ycb_001_cheez-it": (0.072, 0.164),
    "ycb_002_sugar_box": (0.049, 0.094),
    "ycb_005_spam": (0.102, 0.060),
    "ycb_006_mustard_bottle": (0.097, 0.067),
    "ycb_008_pudding_box": (0.138, 0.129),
    "ycb_010_tomato_soup_can": (0.068, 0.068),
    "ycb_011_banana": (0.109, 0.178),
    "ycb_021_bleach_cleanser": (0.102, 0.068),
    "ycb_024_bowl": (0.161, 0.161),
    "ycb_025_mug": (0.117, 0.093),
}
DEFAULT_FOOTPRINT_XY: tuple[float, float] = (0.10, 0.10)
# Placement rule (fix4 run 3: the old `surface_xyz + (i % 3) * grid_dx` grid
# marched cmd_soup_0 off side_table_02 onto the floor and left the sugar box
# hanging over the edge): every object's footprint must lie inside the spot's
# measured `surface_bbox_xy` shrunk by PLACEMENT_EDGE_MARGIN_M on every side,
# and at least PLACEMENT_GAP_M from every other footprint already on the
# surface (base-scenario objects, earlier spawns).
# The gap is sized for the gripper, not just for "not touching": at 0.02 m
# (~0.10-0.12 m centre-to-centre) the grasp stack's tier-0 footprint gate
# rejected nearly every grasp in the fix5 battery ("rejected: palm->env,
# knuckle->env") because the 0.14 m palm / knuckles hit the neighbour. 0.08 m
# edge-to-edge clears the palm overhang (~0.02 m per side beyond a ~0.10 m
# object) plus approach tolerance. Fewer objects fit; anything that no
# longer fits is refused loudly (`place`/`place_all` -> None -> not_attempted).
PLACEMENT_EDGE_MARGIN_M = 0.02
PLACEMENT_GAP_M = 0.06
PLACEMENT_STEP_M = 0.01


def object_footprint_xy(asset_key: str) -> tuple[float, float]:
    """Footprint of an object asset (YCB bbox if measured, else 0.10 x 0.10)."""
    return OBJECT_FOOTPRINT_XY.get(str(asset_key or ""), DEFAULT_FOOTPRINT_XY)


class SurfaceLayout:
    """Incremental packer for one placement spot.

    Candidate centres are a 1 cm lattice over the usable rectangle
    (`surface_bbox_xy` minus the edge margin), tried nearest-to-`surface_xyz`
    first with a preference for the same row (smaller |dy|), so N objects
    fill a row around the surface point and then an L / second row -- three
    objects on a 0.38 m side table form an L at ~0.17-0.18 m pitch. `place()`
    returns the first centre whose footprint is inside the usable rectangle
    and clear of every occupied footprint by the gap, or None (the caller
    refuses the spawn loudly instead of placing it off the edge).

    A spot without `surface_bbox_xy` (a placeholder/legacy entry) falls back
    to the historical `(k % 3, k // 3) * (grid_dx, grid_dy)` grid so nothing
    that worked before silently stops working -- but no extent check is
    possible there, and `describe()` says so.
    """

    def __init__(self, spot: str, spot_info: dict, *, margin: float = PLACEMENT_EDGE_MARGIN_M,
                 gap: float = PLACEMENT_GAP_M, step: float = PLACEMENT_STEP_M) -> None:
        self.spot = spot
        sx, sy, sz = (float(v) for v in spot_info["surface_xyz"])
        self.centre = (sx, sy)
        self.z = sz
        self.gap = float(gap)
        self.step = float(step)
        self.occupied: list[tuple[float, float, float, float]] = []  # x0, y0, x1, y1
        self._legacy_grid: Optional[tuple[float, float]] = None
        self.usable: Optional[tuple[float, float, float, float]] = None
        bbox = spot_info.get("surface_bbox_xy")
        if bbox:
            (x0, y0), (x1, y1) = bbox
            self.usable = (float(x0) + margin, float(y0) + margin, float(x1) - margin, float(y1) - margin)
        else:
            self._legacy_grid = (float(spot_info.get("grid_dx", 0.18)), float(spot_info.get("grid_dy", 0.15)))

    @staticmethod
    def _rect(xy: tuple[float, float], footprint: tuple[float, float]) -> tuple[float, float, float, float]:
        (x, y), (fx, fy) = xy, footprint
        return (x - fx / 2.0, y - fy / 2.0, x + fx / 2.0, y + fy / 2.0)

    def occupy(self, xy: tuple[float, float], footprint: tuple[float, float]) -> None:
        """Register something already on the surface (base object, earlier spawn)."""
        self.occupied.append(self._rect((float(xy[0]), float(xy[1])), footprint))

    def _clear_of_occupied(self, rect: tuple[float, float, float, float]) -> bool:
        g = self.gap
        x0, y0, x1, y1 = rect
        for ox0, oy0, ox1, oy1 in self.occupied:
            if x0 < ox1 + g and x1 > ox0 - g and y0 < oy1 + g and y1 > oy0 - g:
                return False
        return True

    def fits(self, xy: tuple[float, float], footprint: tuple[float, float]) -> bool:
        rect = self._rect(xy, footprint)
        if self.usable is not None:
            ux0, uy0, ux1, uy1 = self.usable
            eps = 1e-9
            if rect[0] < ux0 - eps or rect[1] < uy0 - eps or rect[2] > ux1 + eps or rect[3] > uy1 + eps:
                return False
        return self._clear_of_occupied(rect)

    def place(self, footprint: tuple[float, float]) -> Optional[tuple[float, float]]:
        """Reserve and return a centre for ONE `footprint` (greedy: the free
        lattice point nearest `surface_xyz`, same row preferred), or None.
        For a batch use `place_all`, which packs the whole set as a block."""
        fx, fy = (float(footprint[0]), float(footprint[1]))
        if self.usable is None:
            dx, dy = self._legacy_grid or (0.18, 0.15)
            k = len(self.occupied)
            xy = (round(self.centre[0] + (k % 3) * dx, 4), round(self.centre[1] + (k // 3) * dy, 4))
            self.occupy(xy, (fx, fy))
            return xy
        ux0, uy0, ux1, uy1 = self.usable
        cx_min, cx_max = ux0 + fx / 2.0, ux1 - fx / 2.0
        cy_min, cy_max = uy0 + fy / 2.0, uy1 - fy / 2.0
        if cx_min > cx_max + 1e-9 or cy_min > cy_max + 1e-9:
            return None  # footprint larger than the usable surface
        for xy in self._candidates(cx_min, cx_max, cy_min, cy_max, self.centre):
            if self._clear_of_occupied(self._rect(xy, (fx, fy))):
                self.occupy(xy, (fx, fy))
                return (round(xy[0], 4), round(xy[1], 4))
        return None

    def _candidates(self, x_lo: float, x_hi: float, y_lo: float, y_hi: float,
                    target: tuple[float, float]) -> list[tuple[float, float]]:
        """Lattice points in [x_lo, x_hi] x [y_lo, y_hi] ordered nearest-first
        to `target` (ties: same row first, then +x, then +y) -- deterministic."""
        step = self.step
        tx, ty = target

        def _axis(lo: float, hi: float, s: float) -> list[float]:
            values = [min(max(s, lo), hi)]
            n = int((hi - lo) / step) + 2
            for i in range(1, n):
                for v in (s + i * step, s - i * step):
                    if lo - 1e-9 <= v <= hi + 1e-9:
                        values.append(v)
            return values

        if x_lo > x_hi + 1e-9 or y_lo > y_hi + 1e-9:
            return []
        candidates = [(x, y) for x in _axis(x_lo, x_hi, tx) for y in _axis(y_lo, y_hi, ty)]
        candidates.sort(key=lambda c: (round(math.hypot(c[0] - tx, c[1] - ty), 6),
                                       round(abs(c[1] - ty), 6), round(c[0] - tx, 6), round(c[1] - ty, 6)))
        return candidates

    def place_all(self, footprints: "list[tuple[float, float]]") -> list[Optional[tuple[float, float]]]:
        """Pack a batch of footprints together and reserve their centres.

        The batch is laid out row-major as a `rows x cols` block whose column
        widths / row heights are the widest footprint in that column / row
        plus the gap, trying the widest layout first (one row of N, then
        N-1 columns + an L, ... down to a single column) so the pitch shrinks
        to whatever fits the surface; the block is centred on `surface_xyz`
        and, if a base object is in the way, shifted on the 1 cm lattice to
        the nearest position where every footprint is inside the usable
        rectangle and clear of everything already occupied. If no block
        layout fits, each footprint falls back to greedy `place()` (as many
        as fit; None for the rest, which the caller must refuse loudly).
        """
        fps = [(float(a), float(b)) for a, b in footprints]
        if not fps:
            return []
        if self.usable is None:
            return [self.place(fp) for fp in fps]
        n = len(fps)
        ux0, uy0, ux1, uy1 = self.usable
        sx, sy = self.centre
        for cols in range(n, 0, -1):
            rows = -(-n // cols)
            col_w = [max(fps[i][0] for i in range(n) if i % cols == c) for c in range(cols)]
            row_h = [max(fps[i][1] for i in range(n) if i // cols == r) for r in range(rows)]
            block_w = sum(col_w) + self.gap * (cols - 1)
            block_h = sum(row_h) + self.gap * (rows - 1)
            if block_w > ux1 - ux0 + 1e-9 or block_h > uy1 - uy0 + 1e-9:
                continue
            xs: list[float] = []
            acc = 0.0
            for c in range(cols):
                xs.append(acc + col_w[c] / 2.0)
                acc += col_w[c] + self.gap
            ys: list[float] = []
            acc = 0.0
            for r in range(rows):
                ys.append(acc + row_h[r] / 2.0)
                acc += row_h[r] + self.gap
            # Block origin (min corner) candidates: centred on the surface
            # point first, then the nearest shifts that keep it inside.
            origin_target = (sx - block_w / 2.0, sy - block_h / 2.0)
            for ox, oy in self._candidates(ux0, ux1 - block_w, uy0, uy1 - block_h, origin_target):
                centres = [(ox + xs[i % cols], oy + ys[i // cols]) for i in range(n)]
                if all(self._clear_of_occupied(self._rect(c, fps[i])) for i, c in enumerate(centres)):
                    for i, c in enumerate(centres):
                        self.occupy(c, fps[i])
                    return [(round(c[0], 4), round(c[1], 4)) for c in centres]
        return [self.place(fp) for fp in fps]

    def describe(self) -> str:
        if self.usable is None:
            return f"'{self.spot}' has no surface_bbox_xy (legacy grid, no extent check)"
        ux0, uy0, ux1, uy1 = self.usable
        return (f"'{self.spot}' usable {ux1 - ux0:.2f} x {uy1 - uy0:.2f} m after "
                f"{PLACEMENT_EDGE_MARGIN_M:.2f} m margin, {len(self.occupied)} object(s) already on it")

# The bench scenario's own actor asset, and the last-resort fallback when
# the people artifact manifest cannot be read at all (see
# `_load_person_asset_uris`): every person then uses this same USD, as
# the planner did before gesture/pose-aware asset selection existed.
PERSON_ASSET_URI = (
    "artifacts/people/sobits/d29ee5ef3b71bcbf7013ec61785a584b162bfda83a322ecce0a6a481180e531a"
    "/person_standing/person.usd"
)
PEOPLE_ARTIFACT_CURRENT = Path("artifacts") / "people" / "sobits" / "current.json"
PERSON_STANDING_KEY = "person_standing"

# Gesture/pose label of a person mention (`classify_person_phrase`), in
# the vocabulary of DEC's official command generator
# (_official_cmd_gen.py: gesture_person_list + pose_person_list). Each
# label maps to an ordered fallback chain of people-artifact asset keys
# (`<key>/person.usd` in the artifact manifest payload); the first key
# present in the manifest wins, and `person_standing` is always the last
# resort (`choose_person_asset`). "lying" has no lying asset at all: it
# uses the standing model rotated onto its back (`lying_pose`).
PERSON_POSE_DEFAULT = "standing"
PERSON_ASSET_FALLBACKS: dict[str, tuple[str, ...]] = {
    "waving": ("owada_right_hand_raising", PERSON_STANDING_KEY),
    "raising_left_arm": ("owada_left_hand_raising", PERSON_STANDING_KEY),
    "raising_right_arm": ("owada_right_hand_raising", PERSON_STANDING_KEY),
    "pointing_left": ("owada_left_hand_raising", PERSON_STANDING_KEY),
    "pointing_right": ("owada_right_hand_raising", PERSON_STANDING_KEY),
    "sitting": ("person_sitting", "sitting_woman", PERSON_STANDING_KEY),
    "standing": (PERSON_STANDING_KEY, "standing_man_casual"),
    "lying": (PERSON_STANDING_KEY,),
    "walking": ("person_walking", PERSON_STANDING_KEY),
}

# Ordered: the first pattern that matches the mention's phrase window wins,
# so the two-word gestures are tried before the single-word poses.
_PERSON_PHRASE_PATTERNS: tuple[tuple[str, re.Pattern], ...] = (
    ("raising_left_arm", re.compile(r"\brais(?:ing|es?|ed)\s+(?:their|his|her|the|a)?\s*left\s+(?:arm|hand)s?\b")),
    ("raising_right_arm", re.compile(r"\brais(?:ing|es?|ed)\s+(?:their|his|her|the|a)?\s*right\s+(?:arm|hand)s?\b")),
    ("pointing_left", re.compile(r"\bpoint(?:ing|s|ed)?\s+(?:to\s+the\s+|to\s+|at\s+the\s+)?left\b")),
    ("pointing_right", re.compile(r"\bpoint(?:ing|s|ed)?\s+(?:to\s+the\s+|to\s+|at\s+the\s+)?right\b")),
    ("waving", re.compile(r"\bwav(?:ing|es?|ed)\b")),
    ("lying", re.compile(r"\b(?:lying|laying|lies|lays|lie|lay)\b")),
    ("sitting", re.compile(r"\b(?:sitting|seated|sits?)\b")),
    ("walking", re.compile(r"\bwalk(?:ing|s|ed)?\b")),
    ("standing", re.compile(r"\bstand(?:ing|s)?\b")),
)

# `lying_pose`: the person_standing model faces its own -y (eyes/teeth
# meshes sit at local y~-0.12, toes reach y=-0.25; shoulders span x) and
# stands on z=0 (z-up, 1.91m tall). Rotating -90deg about the model's own
# x axis (quaternion xyzw (-sin45, 0, 0, cos45)) turns local -y (face)
# into world +z, i.e. flat on its back, head pointing along the local +y
# of the model -- so, after the room pose's yaw is composed on top, the
# head points to the yaw-rotated +y ("left" of where the standing person
# would have faced). The back (local +y face of the collision box,
# config/person-import.json collision_box_m.person_standing max y =
# 0.12) then hangs 0.12m below the prim origin, so the origin is lifted
# by that much above whatever surface the body rests on.
LYING_QUATERNION_XYZW: tuple[float, float, float, float] = (-0.7071067811865476, 0.0, 0.0, 0.7071067811865476)
LYING_BACK_OFFSET_M = 0.12
# Feet (prim origin) to head of the lying body: collision_box_m.person_standing max z.
LYING_BODY_LENGTH_M = 1.91
# A placement spot whose name contains one of these is a surface a person
# can lie ON (seat/mattress height = the spot's `surface_xyz[2]`);
# anywhere else the lying person is on the floor (z = 0 + back offset).
LYING_SURFACE_SPOT_WORDS = ("sofa", "couch", "bed")

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
    # Optional (schema_version stays 1): the manifest key of the chosen
    # asset ("person_sitting", "ycb_025_mug", ...) and, for persons, the
    # gesture/pose label the command asked for (a `PERSON_ASSET_FALLBACKS`
    # key; "" for objects). `asset_key` is the same value
    # tools/gpsr_spawn.py derives from `asset_uri` for its dedupe keys.
    asset_key: str = ""
    pose: str = ""


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


def _match_object_names(text: str, knowledge: dict) -> list[str]:
    """Every known object name explicitly mentioned in `text`, in order of
    first appearance in the text (not name length or alphabetical order):
    the controller spawns one of each mentioned object in mention order,
    e.g. "grasp the mug and place it next to the bowl" -> ["mug", "bowl"].
    Per-name matching is still longest-name-first via `_object_names` so a
    name that text-prefixes another can't shadow the more specific one;
    that only affects *whether* a name matches, not the returned order.
    """
    normalized = _normalize_words(text)
    hits: list[tuple[int, str]] = []
    seen: set[str] = set()
    for name in _object_names(knowledge):
        if name in seen:
            continue
        match = _phrase_pattern(name).search(normalized)
        if match:
            hits.append((match.start(), name))
            seen.add(name)
    hits.sort(key=lambda pair: pair[0])
    return [name for _pos, name in hits]


def _match_category(text: str) -> Optional[str]:
    for category in sorted(CATEGORY_MAP, key=lambda c: (-len(c), c)):
        if _text_contains(text, category):
            return category
    return None


def _resolve_location(
    text: str, knowledge: dict, *, after: int = 0
) -> tuple[Optional[str], Optional[str]]:
    """Return (room, spot) from the FIRST-MENTIONED location in `text`,
    by text order (`match.start()`) across every known SPOT and ROOM name
    together -- not spot-vs-room priority. A spot resolves to itself; a
    room resolves to its first `search_spots` entry. Within a tie at the
    same start position, longest-name-first so "side_table_02" is
    preferred over the "side_table" it starts with. Neither present
    returns (None, None).

    `after` (an offset into the normalized text) prefers the first
    location mentioned at or after that offset -- the person's room is the
    location named right after the person mention ("deliver it to Emma in
    the bedroom") -- falling back to the first location overall when
    nothing follows.

    A category phrase (e.g. "kitchen item") is masked out of the text
    before location matching: the room "kitchen" is a substring word of
    that category name and must never be mistaken for a location mention
    of the room -- e.g. "locate a kitchen item in the living_room" must
    not resolve to the kitchen via the word "kitchen" inside "kitchen
    item".
    """
    search_spots = knowledge["search_spots"]
    spot_to_room = {
        s: r for r, spots in search_spots.items() if not r.startswith("_") for s in spots
    }
    normalized = _normalize_words(text)
    masked = normalized
    for category in CATEGORY_MAP:
        match = _phrase_pattern(category).search(masked)
        if match:
            start, end = match.span()
            masked = masked[:start] + " " * (end - start) + masked[end:]

    candidates: list[tuple[int, int, str, bool]] = []
    for spot in spot_to_room:
        match = _phrase_pattern(spot).search(masked)
        if match:
            candidates.append((match.start(), -len(spot), spot, True))
    for room in search_spots:
        if room.startswith("_"):
            continue
        match = _phrase_pattern(room).search(masked)
        if match:
            candidates.append((match.start(), -len(room), room, False))

    if not candidates:
        return None, None
    candidates.sort()
    following = [c for c in candidates if c[0] >= after]
    _start, _neg_len, name, is_spot = (following or candidates)[0]
    if is_spot:
        return spot_to_room[name], name
    return name, search_spots[name][0]


_OPERATOR_DELIVERY_RE = re.compile(r"\b(bring|give|deliver|hand|take)\b.*\bme\b")


def _operator_delivery(text: str) -> bool:
    """A command whose object goes to the operator ("bring me ...", "give
    ... to me"). Only these spawn the operator: a "tell me how many persons
    ..." count must not gain an extra person at the command point."""
    return bool(_OPERATOR_DELIVERY_RE.search(text.lower()))


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


def _person_mention_positions(text: str) -> list[int]:
    """Offsets (in `_normalize_words(text)`) of every person mention -- a
    known name, "person(s)", or "someone" -- in text order; `[0]` when only
    a trigger verb ("meet", "greet", ...) implied the person.
    """
    normalized = _normalize_words(text)
    positions: list[int] = []
    for token in (*_PERSON_NAMES, "persons", "person", "someone"):
        for match in re.finditer(r"\b" + re.escape(token) + r"\b", normalized):
            positions.append(match.start())
    return sorted(set(positions)) or [0]


def _person_phrase_window(text: str, pos: int, *, before: int = 2, after: int = 6) -> str:
    """The words around the person mention at offset `pos` (in
    `_normalize_words(text)`): up to `before` words preceding it and
    `after` words following it, clipped so the window never reaches into
    a neighbouring person mention's own descriptor words. The official
    generator always puts the gesture/pose right next to "person(s)":
    one word before ("waving person", "sitting person") or up to four
    after ("person raising their left arm", "person pointing to the
    left"), so this window is what `classify_person_phrase` looks at.
    """
    normalized = _normalize_words(text)
    tokens = [(m.start(), m.group(0)) for m in re.finditer(r"[a-z0-9]+", normalized)]
    if not tokens:
        return ""
    mention_starts = set(_person_mention_positions(text))
    idx = next((i for i, (start, _w) in enumerate(tokens) if start >= pos), len(tokens) - 1)
    other_mentions = [
        i for i, (start, _w) in enumerate(tokens) if start in mention_starts and i != idx
    ]
    lo = max(0, idx - before)
    hi = min(len(tokens), idx + 1 + after)
    for other in other_mentions:
        if other < idx:
            lo = max(lo, other + 1)
        else:
            hi = min(hi, max(idx + 1, other - before))
    return " ".join(word for _start, word in tokens[lo:hi])


def classify_person_phrase(phrase: str) -> str:
    """Pure: the gesture/pose label (a `PERSON_ASSET_FALLBACKS` key) named
    in `phrase`, or `PERSON_POSE_DEFAULT` ("standing") when it names none.
    Matching is word-bounded on the underscore-normalised, lower-cased
    text, first pattern in `_PERSON_PHRASE_PATTERNS` wins:

        "waving person"                    -> waving
        "person raising their left arm"    -> raising_left_arm
        "person raising their right arm"   -> raising_right_arm
        "person pointing to the left"      -> pointing_left
        "person pointing to the right"     -> pointing_right
        "sitting person"                   -> sitting
        "standing person"                  -> standing
        "lying person"                     -> lying
        "walking person"                   -> walking
        anything else (names, clothing)    -> standing
    """
    normalized = _normalize_words(phrase)
    for label, pattern in _PERSON_PHRASE_PATTERNS:
        if pattern.search(normalized):
            return label
    return PERSON_POSE_DEFAULT


def person_asset_candidates(pose: str) -> tuple[str, ...]:
    """Pure: the ordered asset-key fallback chain for a gesture/pose label,
    always ending in `person_standing`; an unknown label gets the default
    ("standing") chain.
    """
    chain = PERSON_ASSET_FALLBACKS.get(pose, PERSON_ASSET_FALLBACKS[PERSON_POSE_DEFAULT])
    if PERSON_STANDING_KEY not in chain:
        chain = (*chain, PERSON_STANDING_KEY)
    return chain


def choose_person_asset(pose: str, available: Collection[str]) -> tuple[str, Optional[str]]:
    """Pure: (asset_key, note). The first key of `person_asset_candidates(pose)`
    that is in `available` (the people-artifact manifest's model keys).
    When none is -- not even `person_standing` -- the key is still
    `person_standing` (the legacy constant URI covers it); whenever the
    chain's first choice was not available the note says what was
    substituted, else the note is None.
    """
    chain = person_asset_candidates(pose)
    preferred = chain[0]
    for key in chain:
        if key in available:
            if key == preferred:
                return key, None
            return key, f"person asset '{preferred}' for pose '{pose}' not in people manifest; using '{key}'"
    return (
        PERSON_STANDING_KEY,
        f"person asset '{preferred}' for pose '{pose}' not in people manifest; using '{PERSON_STANDING_KEY}'",
    )


def _load_person_asset_uris(asset_root: Path) -> dict[str, str]:
    """{asset_key: asset_uri} for every `<key>/person.usd` in the current
    people artifact's manifest payload (artifacts/people/sobits/current.json
    -> manifest.json), URIs relative to the repo root like
    `PERSON_ASSET_URI`. Empty when the pointer or manifest is unreadable.
    """
    root = Path(asset_root)
    try:
        current = json.loads((root / PEOPLE_ARTIFACT_CURRENT).read_text(encoding="utf-8"))
        manifest_rel = Path(current["manifest"])
        manifest = json.loads((root / manifest_rel).read_text(encoding="utf-8"))
    except (OSError, ValueError, KeyError, TypeError):
        return {}
    payload = manifest.get("payload", {})
    if not isinstance(payload, dict):
        return {}
    uris: dict[str, str] = {}
    for entry in payload:
        match = re.fullmatch(r"([^/]+)/person\.usd", str(entry))
        if match:
            uris[match.group(1)] = (manifest_rel.parent / match.group(1) / "person.usd").as_posix()
    return uris


def _quat_multiply(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, float, float, float]:
    """Hamilton product a*b of xyzw quaternions: rotate by b first, then a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_rotate(q: tuple[float, ...], v: tuple[float, float, float]) -> tuple[float, float, float]:
    """`v` rotated by the xyzw quaternion `q`."""
    conj = (-q[0], -q[1], -q[2], q[3])
    x, y, z, _w = _quat_multiply(_quat_multiply(q, (v[0], v[1], v[2], 0.0)), conj)
    return (x, y, z)


def _is_lying_surface(spot: Optional[str]) -> bool:
    return bool(spot) and any(word in spot.lower() for word in LYING_SURFACE_SPOT_WORDS)


def lying_pose(
    room_pose: dict,
    *,
    spot: Optional[str],
    placements: dict,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float], str]:
    """Pure: (xyz, quaternion_xyzw, note) for a person lying on their back.

    Orientation is the room's actor yaw composed with
    `LYING_QUATERNION_XYZW` (see its comment for the axis choice). If
    `spot` is a lie-able surface (`LYING_SURFACE_SPOT_WORDS`) with a
    placement entry, the body lies ON it: along the long axis of its
    `surface_bbox_xy` and centred on `surface_xyz` (xy of `surface_xyz`
    itself when there is no bbox), z = its surface height +
    `LYING_BACK_OFFSET_M`. Otherwise the body lies on the floor at the
    room's precomputed `lying` pose (a free-floor rectangle for the whole
    body, `persons.<room>.lying` in the placements), else at the room's
    actor xy/yaw; z = `LYING_BACK_OFFSET_M`.
    """
    base_xyz = tuple(float(v) for v in room_pose["xyz"])
    base_quat = tuple(float(v) for v in room_pose["quaternion_xyzw"])
    quat = _quat_multiply(base_quat, LYING_QUATERNION_XYZW)
    spot_info = placements.get("spots", {}).get(spot) if _is_lying_surface(spot) else None
    if spot_info is not None:
        sx, sy, sz = (float(v) for v in spot_info["surface_xyz"])
        z = round(sz + LYING_BACK_OFFSET_M, 4)
        bbox = spot_info.get("surface_bbox_xy")
        if bbox:
            # Body along the surface's long axis, CENTRED on surface_xyz (the
            # prim origin is at the feet, the head LYING_BODY_LENGTH_M along
            # the yaw-rotated +y): with the room yaw alone and the feet at the
            # centre, the body hung ~0.9 m past the sofa's end.
            (x0, y0), (x1, y1) = bbox
            yaw = math.pi / 2.0 if (x1 - x0) >= (y1 - y0) else 0.0  # head -> -x / +y
            yaw_quat = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
            quat = _quat_multiply(yaw_quat, LYING_QUATERNION_XYZW)
            head = (-math.sin(yaw), math.cos(yaw))
            half = LYING_BODY_LENGTH_M / 2.0
            xyz = (round(sx - head[0] * half, 4), round(sy - head[1] * half, 4), z)
            return xyz, quat, f"lying on '{spot}' (surface z={sz}, along its long axis)"
        return (sx, sy, z), quat, f"lying on '{spot}' (surface z={sz})"
    floor = room_pose.get("lying")
    if isinstance(floor, dict) and floor.get("xyz") and floor.get("quaternion_xyzw"):
        # Precomputed free-floor pose for the whole 1.91 m body (see
        # rcw2026-placements.json `_persons_comment`): the upright pose's xy
        # with the body laid out behind it would run into walls/furniture.
        fx, fy = (float(v) for v in floor["xyz"][:2])
        fquat = tuple(float(v) for v in floor["quaternion_xyzw"])
        return ((fx, fy, LYING_BACK_OFFSET_M), _quat_multiply(fquat, LYING_QUATERNION_XYZW),
                "lying on the floor (room lying pose)")
    return (base_xyz[0], base_xyz[1], LYING_BACK_OFFSET_M), quat, "lying on the floor"


@dataclass(frozen=True)
class PersonMention:
    room: str
    spot: Optional[str]  # the location resolved for this mention (None when none named)
    pose: str  # `classify_person_phrase` label


def _resolve_person_mentions(text: str, knowledge: dict) -> tuple[list[PersonMention], list[str]]:
    """One `PersonMention` per person mention, in text order, deduplicated
    by room (see `_resolve_person_rooms` for the room rule); `pose` is the
    gesture/pose classified from the words around that mention.
    """
    mentions: list[PersonMention] = []
    notes: list[str] = []
    for pos in _person_mention_positions(text):
        room, spot = _resolve_location(text, knowledge, after=pos)
        room = room or "living_room"
        if any(m.room == room for m in mentions):
            notes.append(f"one person per room: second person mention also resolves to '{room}'; collapsed")
            continue
        pose = classify_person_phrase(_person_phrase_window(text, pos))
        mentions.append(PersonMention(room=room, spot=spot, pose=pose))
    return mentions, notes


def _resolve_person_rooms(text: str, knowledge: dict) -> tuple[list[str], list[str]]:
    """One room per person mention, in text order, deduplicated: the room
    of the first location (spot or room) NAMED IN `text` at or after that
    mention -- "deliver it to Emma in the bedroom" puts Emma in the
    bedroom even though the kitchen was named earlier for the object; "the
    person at the shelf_02 ... the person at the sofa" yields
    [kitchen, living_room] -- else the first location overall, else
    "living_room". Returns (rooms, notes). A spot that was only
    *defaulted* for an object (via `default_locations`, never mentioned in
    the text) never leaks in; `_resolve_location` only matches present
    text. The arena has one actor pose per room, so two mentions that
    resolve to the same room collapse to one person (noted).
    """
    mentions, notes = _resolve_person_mentions(text, knowledge)
    return [m.room for m in mentions], notes


def _resolve_person_room(text: str, knowledge: dict) -> str:
    """Room of the first person mention (see `_resolve_person_rooms`)."""
    return _resolve_person_rooms(text, knowledge)[0][0]


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
    person_assets: Optional[dict[str, str]] = None,
) -> ScenePlan:
    """Plan the objects/person a GPSR command's scene needs. Pure Python,
    no ROS; never raises -- an unparseable or location-less command
    degrades to an empty or partial plan with an explanatory note.

    `person_assets` ({asset_key: asset_uri}) overrides the people-artifact
    manifest lookup under `asset_root` (see `_load_person_asset_uris`);
    an empty/unreadable manifest falls back to `PERSON_ASSET_URI` for
    every person, with a note.
    """
    notes: list[str] = []
    items: list[SpawnItem] = []
    rng = random.Random(seed)
    by_name = _load_object_asset_uris(asset_root or REPO_ROOT)
    if person_assets is None:
        person_assets = _load_person_asset_uris(asset_root or REPO_ROOT)

    object_names = _match_object_names(command_text, knowledge)
    category = None if object_names else _match_category(command_text)
    counting = _is_counting_template(command_text)

    object_specs: list[tuple[str, int]] = []
    if object_names:
        # Every explicitly named object gets 1 instance each, in order of
        # first appearance; the counting template's x3 applies only to the
        # first-mentioned object.
        first_name, *rest_names = object_names
        first_count = 3 if counting else 1
        object_specs = [(first_name, i) for i in range(first_count)]
        object_specs.extend((name, 0) for name in rest_names)
        notes.append(
            f"explicit objects {object_names} (first '{first_name}' x{first_count})"
            + (" (counting template)" if counting else "")
        )
    elif category:
        members = sorted(CATEGORY_MAP[category])
        k = min(3, len(members))
        sampled = rng.sample(members, k)
        object_specs = [(name, 0) for name in sampled]
        notes.append(f"category '{category}' sampled {sampled} (seed={seed})")
    elif re.search(r"\bobjects?\b", _normalize_words(command_text)):
        # tellObjPropOnPlcmt-style commands ("the thinnest object on the
        # side_table") name no object; an empty table makes them meaningless.
        members = sorted(knowledge.get("possible_objects", {}))
        k = min(3, len(members))
        sampled = rng.sample(members, k)
        object_specs = [(name, 0) for name in sampled]
        notes.append(f"generic 'object' phrase sampled {sampled} (seed={seed})")
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
        spot_info = placements.get("spots", {}).get(resolved_spot)
        if spot_info is None:
            notes.append(f"spot '{resolved_spot}' has no placement entry; no objects spawned")
        else:
            layout = SurfaceLayout(resolved_spot, spot_info)
            spot_to_room = {
                s: r
                for r, spots in knowledge["search_spots"].items()
                if not r.startswith("_")
                for s in spots
            }
            item_room = spot_to_room.get(resolved_spot, "kitchen")
            # Provisional slots (the planner does not know the base
            # scenario); gpsr_spawn re-packs against what is really on the
            # surface and refuses what cannot fit.
            placed = layout.place_all(
                [object_footprint_xy(NAME_TO_YCB_DIR.get(name, "")) for name, _ in object_specs]
            )
            for (name, index), xy in zip(object_specs, placed):
                if xy is None:
                    notes.append(f"'{name}' does not fit on {layout.describe()}; spawn will refuse it")
                    xy = layout.centre
                xyz = (xy[0], xy[1], layout.z)
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
                        asset_key=NAME_TO_YCB_DIR.get(name, ""),
                        pose="",
                    )
                )

    if _person_triggered(command_text):
        mentions, person_notes = _resolve_person_mentions(command_text, knowledge)
        notes.extend(person_notes)
        if not person_assets:
            notes.append(
                f"people artifact manifest unreadable; every person uses '{PERSON_STANDING_KEY}'"
            )
        for mention in mentions:
            person_room = mention.room
            person_pose = placements.get("persons", {}).get(person_room)
            if person_pose is None:
                notes.append(f"person triggered but room '{person_room}' has no known pose; skipped")
                continue
            asset_key, asset_note = choose_person_asset(mention.pose, person_assets)
            if asset_note:
                notes.append(asset_note)
            asset_uri = person_assets.get(asset_key) or PERSON_ASSET_URI
            xyz = tuple(float(v) for v in person_pose["xyz"])
            quat = tuple(float(v) for v in person_pose["quaternion_xyzw"])
            pose_note = ""
            if mention.pose == "lying":
                xyz, quat, pose_note = lying_pose(person_pose, spot=mention.spot, placements=placements)
            items.append(
                SpawnItem(
                    id=f"cmd_person_{person_room}",
                    kind="person",
                    name="person",
                    asset_uri=asset_uri,
                    room=person_room,
                    spot="",
                    xyz=xyz,
                    quaternion_xyzw=quat,
                    asset_key=asset_key,
                    pose=mention.pose,
                )
            )
            notes.append(
                f"person at '{person_room}': pose '{mention.pose}' -> asset '{asset_key}'"
                + (f"; {pose_note}" if pose_note else "")
            )

    # The operator who gave the command stands at the command point (sim
    # battery 2026-10-01 fix8 run s70718-000: "bring me a soup" had no one
    # there, the robot went for the living-room person 2.9 m away and the
    # approach aborted with the soup still in the gripper).
    operator_pose = placements.get("operator")
    if _operator_delivery(command_text):
        if operator_pose is None:
            notes.append("command addressed to the operator but placements have no 'operator' pose; skipped")
        else:
            asset_key, asset_note = choose_person_asset("standing", person_assets)
            if asset_note:
                notes.append(asset_note)
            items.append(
                SpawnItem(
                    id="cmd_person_operator",
                    kind="person",
                    name="person",
                    asset_uri=person_assets.get(asset_key) or PERSON_ASSET_URI,
                    room="command_point",
                    spot="",
                    xyz=tuple(float(v) for v in operator_pose["xyz"]),
                    quaternion_xyzw=tuple(float(v) for v in operator_pose["quaternion_xyzw"]),
                    asset_key=asset_key,
                    pose="standing",
                )
            )
            notes.append(f"operator at the command point -> asset '{asset_key}'")

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
                # Optional keys (absent in plans written before gesture-aware
                # person assets); readers default them to "".
                "asset_key": item.asset_key,
                "pose": item.pose,
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
            asset_key=it.get("asset_key", ""),
            pose=it.get("pose", ""),
        )
        for it in data.get("items", [])
    )
    notes = tuple(data.get("notes", []))
    return ScenePlan(items=items, notes=notes)
