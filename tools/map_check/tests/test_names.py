import pytest

from map_check.names import check_map_name, check_pose_name


@pytest.mark.parametrize("n", ["1104", "0406.map", "robocup_home_2026_measured_5cm"])
def test_map_names_ok(n):
    assert check_map_name(n) == n


@pytest.mark.parametrize("n", ["../x", "a b", ".hidden", "x.poses", "x..y", "a" * 101, ""])
def test_map_names_bad(n):
    with pytest.raises(ValueError, match="map name"):
        check_map_name(n)


@pytest.mark.parametrize("n", ["POS_REGISTER", "kitchen_table_0_0"])
def test_pose_names_ok(n):
    assert check_pose_name(n) == n


@pytest.mark.parametrize("n", ["a-b", "", "a" * 65])
def test_pose_names_bad(n):
    with pytest.raises(ValueError, match="pose name"):
        check_pose_name(n)
