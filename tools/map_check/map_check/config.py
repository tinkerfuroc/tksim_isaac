"""Settings, overridable through MAP_CHECK_<FIELD_UPPER> environment variables."""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

# tools/map_check/map_check/config.py -> the directory containing tools/
_REPO_ROOT = Path(__file__).resolve().parents[3]

_NAV_MAPS = Path('/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/maps')
_ARM_POSES = Path('/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/'
                  'behavior_tree/HRI/constants.json')


@dataclass(frozen=True)
class Settings:
    host: str = '127.0.0.1'
    port: int = 9081
    maps_dir: Path = _NAV_MAPS
    arena_root: Path = _REPO_ROOT / 'artifacts' / 'arena'
    robot_pointer: Path = _REPO_ROOT / 'artifacts' / 'robot' / 'tinker2' / 'current.json'
    arm_poses_json: Path = _ARM_POSES
    state_dir: Path = _REPO_ROOT / 'outputs' / 'map_check'
    repo_root: Path = _REPO_ROOT
    isaac_python: Path = _REPO_ROOT / '.venv' / 'bin' / 'python'
    overlap_margin: float = 0.02
    default_wall_height: float = 0.6
    default_shape_height: float = 0.6
    default_fill_area: float = 4.0

    @classmethod
    def from_env(cls, environ: Mapping[str, str]) -> 'Settings':
        # Path defaults derived from repo_root follow a repo_root override.
        repo = Path(environ['MAP_CHECK_REPO_ROOT']) if 'MAP_CHECK_REPO_ROOT' in environ \
            else _REPO_ROOT
        derived = {
            'arena_root': repo / 'artifacts' / 'arena',
            'robot_pointer': repo / 'artifacts' / 'robot' / 'tinker2' / 'current.json',
            'state_dir': repo / 'outputs' / 'map_check',
            'isaac_python': repo / '.venv' / 'bin' / 'python',
            'repo_root': repo,
        }
        kwargs = {}
        for f in dataclasses.fields(cls):
            raw = environ.get('MAP_CHECK_' + f.name.upper())
            default = derived.get(f.name, f.default)
            if raw is None:
                kwargs[f.name] = default
            else:
                kwargs[f.name] = type(f.default)(raw)
        return cls(**kwargs)
