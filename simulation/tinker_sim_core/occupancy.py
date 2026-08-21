from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class OccupancyMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    occupied: tuple[tuple[bool, ...], ...]

    @classmethod
    def from_pgm(cls, path: Path, *, resolution: float, origin_x: float, origin_y: float, occupied_threshold: int = 100) -> "OccupancyMap":
        data = path.read_bytes()
        if not data.startswith(b"P5"):
            raise ValueError("only binary P5 PGM maps are supported")
        cursor = 2
        tokens: list[bytes] = []
        while len(tokens) < 3:
            while cursor < len(data) and chr(data[cursor]).isspace():
                cursor += 1
            if cursor < len(data) and data[cursor] == ord("#"):
                while cursor < len(data) and data[cursor] not in (10, 13):
                    cursor += 1
                continue
            start = cursor
            while cursor < len(data) and not chr(data[cursor]).isspace():
                cursor += 1
            tokens.append(data[start:cursor])
        width, height, maximum = (int(token) for token in tokens)
        if maximum != 255:
            raise ValueError("PGM maximum value must be 255")
        while cursor < len(data) and chr(data[cursor]).isspace():
            cursor += 1
        pixels = data[cursor:cursor + width * height]
        if len(pixels) != width * height:
            raise ValueError("PGM pixel data is truncated")
        rows = []
        for world_y in range(height):
            source_y = height - 1 - world_y
            start = source_y * width
            rows.append(tuple(value <= occupied_threshold for value in pixels[start:start + width]))
        return cls(width, height, resolution, origin_x, origin_y, tuple(rows))

    def occupied_at_world(self, x: float, y: float) -> bool:
        gx = int((x - self.origin_x) // self.resolution)
        gy = int((y - self.origin_y) // self.resolution)
        if gx < 0 or gy < 0 or gx >= self.width or gy >= self.height:
            return True
        return self.occupied[gy][gx]

    def rectangles(self) -> tuple[tuple[float, float, float, float], ...]:
        active: dict[tuple[int, int], tuple[int, int]] = {}
        finished: list[tuple[int, int, int, int]] = []
        for y, row in enumerate(self.occupied):
            runs: list[tuple[int, int]] = []
            x = 0
            while x < self.width:
                if not row[x]:
                    x += 1
                    continue
                start = x
                while x < self.width and row[x]:
                    x += 1
                runs.append((start, x))
            current = set(runs)
            for run, (start_y, last_y) in tuple(active.items()):
                if run not in current:
                    finished.append((run[0], run[1], start_y, last_y + 1))
                    del active[run]
            for run in runs:
                active[run] = (active[run][0], y) if run in active else (y, y)
        for run, (start_y, last_y) in active.items():
            finished.append((run[0], run[1], start_y, last_y + 1))
        result = []
        for x0, x1, y0, y1 in sorted(finished, key=lambda item: (item[2], item[0])):
            sx = (x1 - x0) * self.resolution
            sy = (y1 - y0) * self.resolution
            cx = self.origin_x + (x0 + x1) * self.resolution / 2.0
            cy = self.origin_y + (y0 + y1) * self.resolution / 2.0
            result.append((cx, cy, sx, sy))
        return tuple(result)

    def raycast(self, x: float, y: float, angle: float, minimum: float = 0.3, maximum: float = 40.0) -> float:
        import math
        distance = minimum
        step = self.resolution / 2.0
        while distance <= maximum:
            if self.occupied_at_world(x + math.cos(angle) * distance, y + math.sin(angle) * distance):
                return distance
            distance += step
        return float("inf")

    def _distance_ladder(self, minimum: float, maximum: float) -> "list[float]":
        """The exact sample distances `raycast` visits, built the same way.

        `raycast` accumulates ``distance += step`` in Python floats; the ladder
        is produced by that very loop (not ``minimum + k * step``) so every
        vectorised sample lands on a bit-identical distance.
        """
        step = self.resolution / 2.0
        key = (minimum, maximum, step)
        cache = self.__dict__.get("_ladder_cache")
        if cache is None:
            cache = {}
            object.__setattr__(self, "_ladder_cache", cache)
        ladder = cache.get(key)
        if ladder is None:
            ladder = []
            distance = minimum
            while distance <= maximum:
                ladder.append(distance)
                distance += step
            cache[key] = ladder
        return ladder

    def _grid_array(self):
        import numpy as np

        grid = self.__dict__.get("_grid_array_cache")
        if grid is None:
            grid = np.array(self.occupied, dtype=bool).reshape(self.height, self.width)
            object.__setattr__(self, "_grid_array_cache", grid)
        return grid

    def raycast_many(
        self,
        x: float,
        y: float,
        angles: "list[float]",
        minimum: float = 0.3,
        maximum: float = 40.0,
        chunk: int = 64,
    ) -> "list[float]":
        """Vectorised `raycast` for many angles from one origin; bit-identical.

        Same sample ladder, same per-sample arithmetic (``x + cos(angle) *
        distance`` with ``math.cos``), the same Python floor-division cell
        lookup and the same out-of-bounds-is-occupied rule as `raycast`, so the
        returned distances are exactly what the scalar loop returns. The
        ladder is marched in chunks across all still-unresolved rays and stops
        as soon as every ray has hit, keeping the scalar loop's early exit
        (indoors most rays resolve within the first few chunks).
        """
        import math

        import numpy as np

        angles = list(angles)
        if not angles:
            return []
        ladder = self._distance_ladder(minimum, maximum)
        if not ladder:
            return [float("inf")] * len(angles)
        distances = self.__dict__.get("_ladder_array_cache", {}).get((minimum, maximum))
        if distances is None:
            distances = np.array(ladder, dtype=np.float64)
            cache = self.__dict__.setdefault("_ladder_array_cache", {})
            object.__setattr__(self, "_ladder_array_cache", cache)
            cache[(minimum, maximum)] = distances
        cos = np.array([math.cos(a) for a in angles], dtype=np.float64)
        sin = np.array([math.sin(a) for a in angles], dtype=np.float64)
        grid = self._grid_array()
        width = self.width
        height = self.height
        pending = np.arange(len(angles))
        result = np.full(len(angles), np.inf, dtype=np.float64)
        for offset in range(0, len(ladder), chunk):
            seg = distances[offset : offset + chunk]
            px = x + cos[pending, None] * seg[None, :]
            py = y + sin[pending, None] * seg[None, :]
            gx = np.floor_divide(px - self.origin_x, self.resolution)
            gy = np.floor_divide(py - self.origin_y, self.resolution)
            outside = (gx < 0) | (gy < 0) | (gx >= width) | (gy >= height)
            ix = np.clip(gx, 0, width - 1).astype(np.int64)
            iy = np.clip(gy, 0, height - 1).astype(np.int64)
            hit = outside | grid[iy, ix]
            first = hit.argmax(axis=1)
            resolved = hit[np.arange(len(pending)), first]
            if resolved.any():
                rays = pending[resolved]
                result[rays] = seg[first[resolved]]
                pending = pending[~resolved]
                if len(pending) == 0:
                    break
        # Distances come straight from the ladder array, so they are the very
        # floats the scalar loop would have returned.
        return [float(value) for value in result]

    def free_with_clearance(self, x: float, y: float, clearance_m: float) -> bool:
        if clearance_m < 0.0:
            raise ValueError("clearance must be non-negative")
        step = self.resolution
        steps = int(clearance_m / step) + 1
        for gx in range(-steps, steps + 1):
            for gy in range(-steps, steps + 1):
                dx = gx * step
                dy = gy * step
                if dx * dx + dy * dy > clearance_m * clearance_m:
                    continue
                if self.occupied_at_world(x + dx, y + dy):
                    return False
        return True

    def nearest_free_world(
        self, x: float, y: float, clearance_m: float, max_radius_m: float = 5.0
    ) -> "tuple[float, float] | None":
        if self.free_with_clearance(x, y, clearance_m):
            return (x, y)
        step = self.resolution
        ring = 1
        while ring * step <= max_radius_m:
            candidates = []
            for gx in range(-ring, ring + 1):
                for gy in range(-ring, ring + 1):
                    if max(abs(gx), abs(gy)) != ring:
                        continue
                    cx = x + gx * step
                    cy = y + gy * step
                    if self.free_with_clearance(cx, cy, clearance_m):
                        candidates.append((gx * gx + gy * gy, cx, cy))
            if candidates:
                _, cx, cy = min(candidates)
                return (round(cx, 3), round(cy, 3))
            ring += 1
        return None
