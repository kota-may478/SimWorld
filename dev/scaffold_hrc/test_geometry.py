#!/usr/bin/env python3
"""Unit tests for Stage-1 scaffold extents (no UE)."""

from __future__ import annotations

import unittest

from scene.geometry import STAGE1_GEOM


class ScaffoldGeomTest(unittest.TestCase):
    def test_stage1_deck_and_stair_footprint(self) -> None:
        g = STAGE1_GEOM
        self.assertAlmostEqual(g.deck_width_m, 2.4)
        self.assertAlmostEqual(g.deck_length_m, 10.0)
        self.assertAlmostEqual(g.lift_m, 1.8)
        self.assertEqual(g.n_floors, 3)
        self.assertAlmostEqual(g.stair_bay_m, 1.8)
        self.assertAlmostEqual(g.total_length_m, 11.8)
        self.assertAlmostEqual(g.floor_z_m(1), 0.0)
        self.assertAlmostEqual(g.floor_z_m(3), 3.6)

    def test_working_deck_excludes_stair_bay(self) -> None:
        g = STAGE1_GEOM
        x0, x1, y0, y1 = g.deck_xy_bounds()
        self.assertAlmostEqual(x0, 0.0)
        self.assertAlmostEqual(x1, 10.0)
        self.assertAlmostEqual(y0, 0.0)
        self.assertAlmostEqual(y1, 2.4)
        sx0, sx1, sy0, sy1 = g.stair_xy_bounds()
        self.assertAlmostEqual(sx1, 0.0)
        self.assertAlmostEqual(sx0, -1.8)
        self.assertAlmostEqual(sy1 - sy0, 2.4)

    def test_floor_index_rejects_out_of_range(self) -> None:
        with self.assertRaises(ValueError):
            STAGE1_GEOM.floor_z_m(0)
        with self.assertRaises(ValueError):
            STAGE1_GEOM.floor_z_m(4)

    def test_geom_is_frozen(self) -> None:
        with self.assertRaises(Exception):
            STAGE1_GEOM.deck_width_m = 3.0  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()
