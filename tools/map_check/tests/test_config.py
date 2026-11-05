from pathlib import Path

from map_check.config import Settings


def test_defaults():
    s = Settings.from_env({})
    assert s.host == "127.0.0.1"
    assert s.port == 9081
    assert s.overlap_margin == 0.02
    assert (s.default_wall_height, s.default_shape_height, s.default_fill_area) == (0.6, 0.6, 4.0)
    p = str(s.arm_poses_json)
    assert p.endswith("behavior_tree/HRI/constants.json")
    assert not p.endswith("HRI/HRI/constants.json")
    assert (s.repo_root / "tools" / "map_check").is_dir()
    assert s.arena_root == s.repo_root / "artifacts" / "arena"
    assert s.robot_pointer == s.repo_root / "artifacts/robot/tinker2/current.json"
    assert s.state_dir == s.repo_root / "outputs/map_check"
    assert s.isaac_python == s.repo_root / ".venv/bin/python"


def test_env_override():
    s = Settings.from_env({"MAP_CHECK_PORT": "9100", "MAP_CHECK_MAPS_DIR": "/tmp/m"})
    assert s.port == 9100
    assert s.maps_dir == Path("/tmp/m")
