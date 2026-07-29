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
