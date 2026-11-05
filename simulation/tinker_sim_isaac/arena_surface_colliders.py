"""Which support-surface furniture colliders get the boundingCube override.

Pure pxr (no Kit), so it is testable on an in-memory stage. Furniture carries
two kinds of mesh colliders: legacy whole-visual-mesh hulls anywhere in the
model, and CoACD convex hulls under the model's ``Colliders`` scope. Only the
legacy ones are replaced by a box; the CoACD hulls are the intended shape.
"""
from __future__ import annotations

from typing import Iterable

COLLIDERS_SCOPE = "Colliders"


def support_surface_box_targets(stage, furniture_paths: Iterable[str]) -> list:
    """Prims under each furniture path with ``MeshCollisionAPI`` that are not
    inside a ``Colliders`` scope. Missing furniture paths are skipped."""
    from pxr import Usd, UsdPhysics

    targets = []
    for furniture_path in furniture_paths:
        furniture = stage.GetPrimAtPath(furniture_path)
        if not furniture.IsValid():
            continue
        for prim in Usd.PrimRange(furniture):
            if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                continue
            if any(
                p.GetName() == COLLIDERS_SCOPE
                for p in _ancestors_within(prim, furniture)
            ):
                continue
            targets.append(prim)
    return targets


def _ancestors_within(prim, root):
    """``prim`` and its parents up to (excluding) ``root``."""
    root_path = root.GetPath()
    while prim.IsValid() and prim.GetPath() != root_path:
        yield prim
        prim = prim.GetParent()
