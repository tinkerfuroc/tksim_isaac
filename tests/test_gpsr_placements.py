import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

PLACEMENTS_PATH = ROOT / "simulation" / "scenarios" / "rcw2026-placements.json"
CONSTANTS_PATH = Path(
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json"
)
BENCH_SCENARIO_PATH = ROOT / "simulation" / "scenarios" / "gpsr-rcw2026-bench.json"


def _load_placements() -> dict:
    return json.loads(PLACEMENTS_PATH.read_text())


def _load_constants() -> dict:
    return json.loads(CONSTANTS_PATH.read_text())


def test_placements_file_is_valid_json_with_schema_version():
    data = _load_placements()
    assert data["schema_version"] == 1
    assert "spots" in data and "persons" in data


def test_every_dec_search_spot_has_a_placements_surface():
    constants = _load_constants()
    placements = _load_placements()
    search_spots = constants["search_spots"]
    all_spots = {spot for room, spots in search_spots.items() if not room.startswith("_") for spot in spots}
    missing = all_spots - set(placements["spots"])
    assert missing == set(), f"placements.spots is missing: {missing}"


def test_every_dec_room_has_a_placements_person_pose():
    constants = _load_constants()
    placements = _load_placements()
    rooms = {r for r in constants["search_spots"] if not r.startswith("_")}
    missing = rooms - set(placements["persons"])
    assert missing == set(), f"placements.persons is missing: {missing}"


def test_every_spot_entry_has_a_3d_surface_xyz_and_grid_spacing():
    placements = _load_placements()
    for spot, info in placements["spots"].items():
        assert len(info["surface_xyz"]) == 3, spot
        assert all(isinstance(v, (int, float)) for v in info["surface_xyz"]), spot
        assert info["grid_dx"] > 0, spot
        assert info["grid_dy"] > 0, spot


def test_every_person_entry_has_a_3d_xyz_and_a_unit_quaternion():
    placements = _load_placements()
    for room, info in placements["persons"].items():
        assert len(info["xyz"]) == 3, room
        xyzw = info["quaternion_xyzw"]
        assert len(xyzw) == 4, room
        magnitude = sum(v * v for v in xyzw) ** 0.5
        assert abs(magnitude - 1.0) < 1e-3, room


def test_known_surfaces_match_the_bench_scenario_object_poses():
    # kitchen_table/side_table_02/shelf_02 are the three surfaces the bench
    # scenario already measures via its spawned soup/banana/bowl poses --
    # the placement table must not silently drift from what is actually
    # spawned there today.
    placements = _load_placements()
    bench = json.loads(BENCH_SCENARIO_PATH.read_text())
    by_id = {obj["id"]: obj for obj in bench["objects"]}
    assert placements["spots"]["kitchen_table"]["surface_xyz"] == by_id["soup"]["pose"]["xyz"]
    assert placements["spots"]["side_table_02"]["surface_xyz"] == by_id["banana"]["pose"]["xyz"]
    assert placements["spots"]["shelf_02"]["surface_xyz"] == by_id["bowl"]["pose"]["xyz"]


def test_known_person_poses_match_the_bench_scenario_actor_poses():
    placements = _load_placements()
    bench = json.loads(BENCH_SCENARIO_PATH.read_text())
    by_id = {actor["id"]: actor for actor in bench["actors"]}
    assert placements["persons"]["kitchen"]["xyz"] == by_id["kitchen_person"]["pose"]["xyz"]
    assert (
        placements["persons"]["kitchen"]["quaternion_xyzw"]
        == by_id["kitchen_person"]["pose"]["quaternion_xyzw"]
    )
    assert placements["persons"]["living_room"]["xyz"] == by_id["livingroom_person"]["pose"]["xyz"]


def test_measured_spots_are_within_the_arena_bounds():
    # Sanity bound on the four newly measured spots -- the rcw2026 arena is
    # roughly a 10m x 10m room; a wildly wrong measurement (e.g. an
    # un-transformed local-frame coordinate) would fall far outside this.
    placements = _load_placements()
    for spot in ("side_table", "laundry_desk", "shelf", "sofa"):
        x, y, z = placements["spots"][spot]["surface_xyz"]
        assert -8.0 < x < 8.0, spot
        assert -8.0 < y < 8.0, spot
        assert 0.0 < z < 2.5, spot


def test_measured_person_poses_are_within_the_arena_bounds():
    placements = _load_placements()
    for room in ("bedroom", "laundry_room"):
        x, y, z = placements["persons"][room]["xyz"]
        assert -8.0 < x < 8.0, room
        assert -8.0 < y < 8.0, room
        assert z == 0.0, room
