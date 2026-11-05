"""Name rules, copied from Map Studio's names.py so both tools agree.

Map names are nav2 map file stems (`1104`, `0406.map`). Pose names become
YAML keys and Python constant names (`POS_<NAME>`), so they are identifier-like.
"""
import re

_MAP_NAME = re.compile(r'[A-Za-z0-9_][A-Za-z0-9_.-]{0,99}')
_POSE_NAME = re.compile(r'[A-Za-z0-9_]{1,64}')


def check_map_name(name):
    if (not isinstance(name, str) or not _MAP_NAME.fullmatch(name)
            or '..' in name or name.endswith('.poses')):
        raise ValueError(
            f'invalid map name {name!r} (1-100 chars of letters, digits, _ . -; '
            'not starting with . or -, no "..", not ending in ".poses")')
    return name


def check_pose_name(name):
    if not isinstance(name, str) or not _POSE_NAME.fullmatch(name):
        raise ValueError(
            f'invalid pose name {name!r} (1-64 chars: letters, digits and _ only)')
    return name
