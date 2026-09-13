"""Read a URDF with the stdlib and expose links, joints and rigid transforms."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]
ZERO: Vec3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Origin:
    xyz: Vec3 = ZERO
    rpy: Vec3 = ZERO


@dataclass(frozen=True)
class Geometry:
    kind: str
    size: Vec3 | None = None
    radius: float | None = None
    length: float | None = None
    filename: str | None = None


@dataclass(frozen=True)
class Inertial:
    mass: float
    origin: Origin
    ixx: float
    iyy: float
    izz: float
    ixy: float
    ixz: float
    iyz: float


@dataclass(frozen=True)
class Link:
    name: str
    inertial: Inertial | None
    collisions: tuple[tuple[Origin, Geometry], ...]


@dataclass(frozen=True)
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: Origin
    axis: Vec3
    lower: float | None
    upper: float | None
    mimic: str | None


def parse_urdf(data: bytes) -> ET.Element:
    root = ET.fromstring(data)
    if root.tag != "robot":
        raise ValueError("URDF root element must be <robot>")
    return root


def _vec3(text: str | None, default: Vec3 = ZERO) -> Vec3:
    if text is None:
        return default
    parts = [float(item) for item in text.split()]
    if len(parts) != 3:
        raise ValueError(f"expected three numbers, got {text!r}")
    return (parts[0], parts[1], parts[2])


def _origin(element: ET.Element | None) -> Origin:
    if element is None:
        return Origin()
    return Origin(_vec3(element.get("xyz")), _vec3(element.get("rpy")))


def _geometry(element: ET.Element | None) -> Geometry:
    if element is None:
        return Geometry("none")
    box = element.find("box")
    if box is not None:
        return Geometry("box", size=_vec3(box.get("size")))
    cylinder = element.find("cylinder")
    if cylinder is not None:
        return Geometry("cylinder", radius=float(cylinder.get("radius", "0")), length=float(cylinder.get("length", "0")))
    sphere = element.find("sphere")
    if sphere is not None:
        return Geometry("sphere", radius=float(sphere.get("radius", "0")))
    mesh = element.find("mesh")
    if mesh is not None:
        return Geometry("mesh", filename=mesh.get("filename"))
    return Geometry("none")


def links(root: ET.Element) -> dict[str, Link]:
    result: dict[str, Link] = {}
    for element in root.findall("link"):
        name = element.get("name") or ""
        inertial_element = element.find("inertial")
        inertial = None
        if inertial_element is not None:
            mass_element = inertial_element.find("mass")
            inertia = inertial_element.find("inertia")
            get = (lambda key: float(inertia.get(key, "0"))) if inertia is not None else (lambda key: 0.0)
            inertial = Inertial(
                mass=float(mass_element.get("value", "0")) if mass_element is not None else 0.0,
                origin=_origin(inertial_element.find("origin")),
                ixx=get("ixx"), iyy=get("iyy"), izz=get("izz"), ixy=get("ixy"), ixz=get("ixz"), iyz=get("iyz"),
            )
        collisions = tuple(
            (_origin(collision.find("origin")), _geometry(collision.find("geometry")))
            for collision in element.findall("collision")
        )
        result[name] = Link(name, inertial, collisions)
    return result


def joints(root: ET.Element) -> dict[str, Joint]:
    result: dict[str, Joint] = {}
    for element in root.findall("joint"):
        parent = element.find("parent")
        child = element.find("child")
        limit = element.find("limit")
        axis = element.find("axis")
        mimic = element.find("mimic")
        result[element.get("name") or ""] = Joint(
            name=element.get("name") or "",
            type=element.get("type") or "",
            parent=parent.get("link") if parent is not None else "",
            child=child.get("link") if child is not None else "",
            origin=_origin(element.find("origin")),
            axis=_vec3(axis.get("xyz") if axis is not None else None, (1.0, 0.0, 0.0)),
            lower=float(limit.get("lower")) if limit is not None and limit.get("lower") is not None else None,
            upper=float(limit.get("upper")) if limit is not None and limit.get("upper") is not None else None,
            mimic=mimic.get("joint") if mimic is not None else None,
        )
    return result


def parent_joint(root: ET.Element) -> dict[str, Joint]:
    return {joint.child: joint for joint in joints(root).values()}


def root_links(root: ET.Element) -> list[str]:
    children = {joint.child for joint in joints(root).values()}
    return [name for name in links(root) if name not in children]


def movable_joints(root: ET.Element) -> list[str]:
    return [name for name, joint in joints(root).items() if joint.type != "fixed"]


def fixed_subtree(root: ET.Element, link: str) -> set[str]:
    by_parent: dict[str, list[Joint]] = {}
    for joint in joints(root).values():
        by_parent.setdefault(joint.parent, []).append(joint)
    seen = {link}
    stack = [link]
    while stack:
        current = stack.pop()
        for joint in by_parent.get(current, []):
            if joint.type == "fixed" and joint.child not in seen:
                seen.add(joint.child)
                stack.append(joint.child)
    return seen


def rotation(rpy: Vec3) -> Mat3:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # URDF: R = Rz(yaw) · Ry(pitch) · Rx(roll)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def rotate_axis_angle(axis: Vec3, angle: float) -> Mat3:
    norm = math.sqrt(sum(component * component for component in axis)) or 1.0
    x, y, z = (component / norm for component in axis)
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return (
        (t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    )


def _matmul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)) for row in range(3))  # type: ignore[return-value]


def _matvec(m: Mat3, v: Vec3) -> Vec3:
    return tuple(sum(m[row][col] * v[col] for col in range(3)) for row in range(3))  # type: ignore[return-value]


def _rpy_from_matrix(m: Mat3) -> Vec3:
    pitch = math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0]))
    if abs(math.cos(pitch)) < 1e-9:
        return (math.atan2(-m[1][2], m[1][1]), pitch, 0.0)
    return (math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0]))


def apply(origin: Origin, point: Vec3) -> Vec3:
    rotated = _matvec(rotation(origin.rpy), point)
    return (rotated[0] + origin.xyz[0], rotated[1] + origin.xyz[1], rotated[2] + origin.xyz[2])


def compose(a: Origin, b: Origin) -> Origin:
    """Transform that applies b first, then a (T_a · T_b)."""
    return Origin(apply(a, b.xyz), _rpy_from_matrix(_matmul(rotation(a.rpy), rotation(b.rpy))))


def compose_rotation(a: Origin, r: Mat3) -> Origin:
    """T_a followed by a pure rotation r about a's origin (used for joint angles)."""
    return Origin(a.xyz, _rpy_from_matrix(_matmul(rotation(a.rpy), r)))
