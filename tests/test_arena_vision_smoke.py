from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

from arena_vision_smoke import (
    MIN_SATURATION,
    RESOLUTION,
    WALL_PALETTE,
    expected_wall_colors,
    hue_presence,
    persist_png,
    rgb_image,
    wall_color,
)


def _palette_stripes(height: int = 120, width: int = 240, brightness: float = 1.0):
    """Render one vertical stripe per palette color at a given brightness."""
    image = np.zeros((height, width, 3), dtype=np.uint8)
    span = width // len(WALL_PALETTE)
    for index, (_name, rgb, _hue) in enumerate(WALL_PALETTE):
        start = index * span
        stop = width if index == len(WALL_PALETTE) - 1 else (index + 1) * span
        image[:, start:stop, :] = (np.asarray(rgb) * 255.0 * brightness).astype(np.uint8)
    return image


class WallColorTest(unittest.TestCase):
    def test_assignment_alternates_between_adjacent_walls(self) -> None:
        names = [wall_color(index)[0] for index in range(len(WALL_PALETTE) + 2)]
        self.assertEqual(names[0], "red")
        self.assertEqual(names[: len(WALL_PALETTE)], [name for name, _rgb, _hue in WALL_PALETTE])
        # Wrapping keeps the cycle short so neighbouring walls differ.
        self.assertEqual(names[len(WALL_PALETTE)], names[0])
        for first, second in zip(names, names[1:]):
            self.assertNotEqual(first, second)

    def test_assignment_is_deterministic(self) -> None:
        self.assertEqual(
            [wall_color(index) for index in range(50)],
            [wall_color(index) for index in range(50)],
        )

    def test_negative_index_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "must not be negative"):
            wall_color(-1)

    def test_every_wall_is_assigned_exactly_once(self) -> None:
        tally = expected_wall_colors(707)
        self.assertEqual(sum(tally.values()), 707)
        self.assertEqual(set(tally), {name for name, _rgb, _hue in WALL_PALETTE})
        # 707 walls over six colors: no color may be starved.
        self.assertTrue(all(count >= 117 for count in tally.values()))


class HuePresenceTest(unittest.TestCase):
    def test_detects_every_palette_hue(self) -> None:
        report = hue_presence(_palette_stripes())
        for name, _rgb, _hue in WALL_PALETTE:
            self.assertTrue(report["colors"][name]["present"], name)
        self.assertAlmostEqual(report["chromatic_pixel_ratio"], 1.0, places=3)

    def test_survives_diffuse_shading_falloff(self) -> None:
        # Ray-traced diffuse shading darkens authored colors; hue must survive.
        report = hue_presence(_palette_stripes(brightness=0.35))
        for name, _rgb, _hue in WALL_PALETTE:
            self.assertTrue(report["colors"][name]["present"], name)

    def test_achromatic_frame_matches_no_color(self) -> None:
        for fill in (0, 128, 255):
            report = hue_presence(np.full((64, 64, 3), fill, dtype=np.uint8))
            self.assertEqual(report["chromatic_pixel_ratio"], 0.0)
            for name, _rgb, _hue in WALL_PALETTE:
                self.assertFalse(report["colors"][name]["present"], f"{name}@{fill}")

    def test_desaturated_frame_is_not_credited(self) -> None:
        # A barely-tinted gray must not count as a rendered wall color.
        nearly_gray = np.full((64, 64, 3), 128, dtype=np.uint8)
        nearly_gray[:, :, 0] = 134
        report = hue_presence(nearly_gray)
        self.assertLess(
            float(np.max([stats["ratio"] for stats in report["colors"].values()])),
            1.0,
        )
        self.assertEqual(report["chromatic_pixel_ratio"], 0.0)

    def test_single_color_frame_credits_only_that_color(self) -> None:
        red = np.zeros((64, 64, 3), dtype=np.uint8)
        red[:, :, 0] = 204
        report = hue_presence(red)
        self.assertTrue(report["colors"]["red"]["present"])
        for name in ("green", "blue", "cyan", "magenta", "yellow"):
            self.assertFalse(report["colors"][name]["present"], name)

    def test_reports_thresholds_used(self) -> None:
        report = hue_presence(_palette_stripes())
        self.assertEqual(report["min_saturation"], MIN_SATURATION)
        self.assertIn("hue_tolerance_deg", report)

    def test_rejects_non_rgb_input(self) -> None:
        with self.assertRaisesRegex(ValueError, "expected an RGB image"):
            hue_presence(np.zeros((8, 8), dtype=np.uint8))


class RgbImageTest(unittest.TestCase):
    def test_accepts_uint8_rgb(self) -> None:
        buffer = np.zeros((*RESOLUTION, 3), dtype=np.uint8)
        self.assertEqual(rgb_image(buffer, RESOLUTION).size, (RESOLUTION[1], RESOLUTION[0]))

    def test_drops_alpha_channel(self) -> None:
        buffer = np.zeros((*RESOLUTION, 4), dtype=np.uint8)
        self.assertEqual(rgb_image(buffer, RESOLUTION).mode, "RGB")

    def test_squeezes_leading_batch_dimension(self) -> None:
        buffer = np.zeros((1, *RESOLUTION, 3), dtype=np.uint8)
        self.assertEqual(rgb_image(buffer, RESOLUTION).size, (RESOLUTION[1], RESOLUTION[0]))

    def test_scales_unit_range_floats(self) -> None:
        buffer = np.ones((*RESOLUTION, 3), dtype=np.float32)
        self.assertEqual(np.asarray(rgb_image(buffer, RESOLUTION)).max(), 255)

    def test_rejects_unexpected_resolution(self) -> None:
        with self.assertRaisesRegex(ValueError, "unexpected RGB frame shape"):
            rgb_image(np.zeros((16, 16, 3), dtype=np.uint8), RESOLUTION)

    def test_rejects_non_finite_floats(self) -> None:
        buffer = np.full((*RESOLUTION, 3), np.nan, dtype=np.float32)
        with self.assertRaisesRegex(ValueError, "non-finite"):
            rgb_image(buffer, RESOLUTION)


class PersistPngTest(unittest.TestCase):
    def test_writes_readable_png_and_leaves_no_temp_files(self) -> None:
        image = Image.fromarray(_palette_stripes(height=8, width=24), mode="RGB")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "nested" / "frame.png"
            persist_png(image, output)
            self.assertTrue(output.is_file())
            with Image.open(output) as written:
                self.assertEqual(written.size, (24, 8))
            self.assertEqual(sorted(p.name for p in output.parent.iterdir()), ["frame.png"])


class PaletteSourceTest(unittest.TestCase):
    def test_palette_is_shared_from_core(self) -> None:
        sys.path.insert(0, str(ROOT / "simulation"))
        from tinker_sim_core import arena_palette
        import arena_vision_smoke

        self.assertIs(arena_vision_smoke.WALL_PALETTE, arena_palette.WALL_PALETTE)
        self.assertIs(arena_vision_smoke.wall_color, arena_palette.wall_color)


if __name__ == "__main__":
    unittest.main()
