from __future__ import annotations

#: Saturated, evenly spaced hues for deterministic arena-wall coloring.
#: Sixty degrees of separation keeps hue classification unambiguous under
#: ray-traced lighting.  Entries are (name, diffuse rgb 0-1, hue degrees).
WALL_PALETTE = (
    ("red", (0.80, 0.02, 0.02), 0.0),
    ("yellow", (0.80, 0.80, 0.02), 60.0),
    ("green", (0.02, 0.80, 0.02), 120.0),
    ("cyan", (0.02, 0.80, 0.80), 180.0),
    ("blue", (0.02, 0.02, 0.80), 240.0),
    ("magenta", (0.80, 0.02, 0.80), 300.0),
)


def wall_color(index: int) -> tuple[str, tuple[float, float, float], float]:
    """Palette entry for wall ``index`` (modular, so adjacent walls differ)."""
    if index < 0:
        raise ValueError("wall index must not be negative")
    return WALL_PALETTE[index % len(WALL_PALETTE)]


def expected_wall_colors(count: int) -> dict[str, int]:
    """How many walls each palette color receives for ``count`` walls."""
    if count < 0:
        raise ValueError("wall count must not be negative")
    tally = {name: 0 for name, _rgb, _hue in WALL_PALETTE}
    for index in range(count):
        tally[wall_color(index)[0]] += 1
    return tally
