"""Nav2 occupancy maps (yaml + image) and the closed outlines drawn in them."""
import io
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Mapping

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage


class MapFileError(ValueError):
    """A map yaml/image that cannot be used; the message names the problem."""


@dataclass(frozen=True)
class OccupancyMap:
    occupied: np.ndarray  # bool (H, W); row 0 is the top (max y)
    resolution: float
    origin: tuple
    meta: dict
    image_name: str
    yaml_bytes: bytes
    image_bytes: bytes


@dataclass(frozen=True)
class Outline:
    id: int  # 1-based
    area_m2: float
    cx: float
    cy: float


def parse_map(yaml_bytes: bytes, files: Mapping[str, bytes]) -> OccupancyMap:
    try:
        meta = yaml.safe_load(yaml_bytes)
    except yaml.YAMLError as e:
        raise MapFileError(f"map yaml is not valid YAML: {e}") from e
    if not isinstance(meta, dict):
        raise MapFileError("map yaml must be a mapping")
    for key in ("image", "resolution", "origin"):
        if key not in meta:
            raise MapFileError(f"map yaml has no '{key}'")
    try:
        res = float(meta["resolution"])
        origin = (float(meta["origin"][0]), float(meta["origin"][1]))
        thresh = float(meta.get("occupied_thresh", 0.65))
    except (TypeError, ValueError, IndexError, KeyError) as e:
        raise MapFileError(f"map yaml has a bad resolution/origin/occupied_thresh: {e}") from e
    if res <= 0:
        raise MapFileError(f"map yaml resolution must be positive, got {res}")
    name = PurePosixPath(str(meta["image"]).replace("\\", "/")).name
    if name not in files:
        raise MapFileError(f"map image '{name}' (from yaml image: {meta['image']}) was not provided")
    image_bytes = files[name]
    try:
        img = np.array(Image.open(io.BytesIO(image_bytes)).convert("L"), dtype=float)
    except Exception as e:
        raise MapFileError(f"map image '{name}' cannot be read: {e}") from e
    p = img / 255.0 if meta.get("negate", 0) else (255.0 - img) / 255.0
    return OccupancyMap(occupied=p > thresh, resolution=res, origin=origin, meta=meta,
                        image_name=name, yaml_bytes=yaml_bytes, image_bytes=image_bytes)


def load_map_dir(yaml_path: Path) -> OccupancyMap:
    yaml_path = Path(yaml_path)
    yaml_bytes = yaml_path.read_bytes()
    try:
        meta = yaml.safe_load(yaml_bytes)
    except yaml.YAMLError as e:
        raise MapFileError(f"map yaml is not valid YAML: {e}") from e
    files = {}
    if isinstance(meta, dict) and "image" in meta:
        name = PurePosixPath(str(meta["image"]).replace("\\", "/")).name
        image_path = yaml_path.parent / name
        if image_path.is_file():
            files[name] = image_path.read_bytes()
    return parse_map(yaml_bytes, files)


def cell_center(m: OccupancyMap, row, col) -> tuple:
    h = m.occupied.shape[0]
    return (m.origin[0] + (col + 0.5) * m.resolution,
            m.origin[1] + (h - row - 0.5) * m.resolution)


def closed_outlines(m: OccupancyMap):
    """Label free regions fully enclosed by occupied cells (0 = not enclosed)."""
    holes = ndimage.binary_fill_holes(m.occupied) & ~m.occupied
    labels, n = ndimage.label(holes)
    labels = labels.astype(np.int32)
    outlines = []
    for k in range(1, n + 1):
        rows, cols = np.nonzero(labels == k)
        xs = m.origin[0] + (cols + 0.5) * m.resolution
        ys = m.origin[1] + (m.occupied.shape[0] - rows - 0.5) * m.resolution
        outlines.append(Outline(id=k, area_m2=float(len(rows) * m.resolution ** 2),
                                cx=float(xs.mean()), cy=float(ys.mean())))
    return labels, outlines
