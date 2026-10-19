"""URDF forward kinematics -> per-link transforms relative to base_link."""
import math
import xml.etree.ElementTree as ET

import numpy as np


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, q):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * k @ k


def link_transforms(urdf_path, q=None, root="base_link"):
    q = q or {}
    tree = ET.parse(urdf_path).getroot()
    by_child = {}
    for j in tree.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        ax = j.find("axis")
        axis = [float(v) for v in (ax.get("xyz", "1 0 0") if ax is not None else "1 0 0").split()]
        mimic = j.find("mimic")
        by_child[j.find("child").get("link")] = dict(
            name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
            xyz=xyz, rpy=rpy, axis=axis,
            mimic=None if mimic is None else (mimic.get("joint"), float(mimic.get("multiplier", 1)), float(mimic.get("offset", 0))))
    cache = {root: np.eye(4)}

    def angle(j):
        if j["mimic"]:
            src, m, off = j["mimic"]
            return q.get(src, 0.0) * m + off
        return q.get(j["name"], 0.0)

    def T(link):
        if link in cache:
            return cache[link]
        j = by_child[link]
        o = np.eye(4); o[:3, :3] = _rpy(*j["rpy"]); o[:3, 3] = j["xyz"]
        m = np.eye(4)
        if j["type"] in ("revolute", "continuous"):
            m[:3, :3] = _axis_angle(j["axis"], angle(j))
        elif j["type"] == "prismatic":
            m[:3, 3] = np.asarray(j["axis"], float) * angle(j)
        cache[link] = T(j["parent"]) @ o @ m
        return cache[link]

    return {link: T(link) for link in list(by_child) + [root]}
