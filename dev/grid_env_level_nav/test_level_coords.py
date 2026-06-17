#!/usr/bin/env python3
"""Unit tests for level_coords (no UE)."""

from __future__ import annotations

import unittest

import level_coords as lc


class LevelCoordsTest(unittest.TestCase):
    def test_corner_a_roundtrip(self) -> None:
        wx, wy = lc.local_xy_to_world(0.0, 0.0)
        self.assertAlmostEqual(wx, -1000.0)
        self.assertAlmostEqual(wy, -2200.0)
        lx, ly = lc.world_xy_to_local(wx, wy)
        self.assertAlmostEqual(lx, 0.0)
        self.assertAlmostEqual(ly, 0.0)

    def test_ue_y_increase_maps_to_local_x(self) -> None:
        ox, oy = lc.REGION_ORIGIN_WORLD_XY
        lx, ly = lc.world_xy_to_local(ox, oy + 500.0)
        self.assertAlmostEqual(lx, 500.0)
        self.assertAlmostEqual(ly, 0.0)

    def test_ue_x_increase_maps_to_local_y(self) -> None:
        ox, oy = lc.REGION_ORIGIN_WORLD_XY
        lx, ly = lc.world_xy_to_local(ox + 500.0, oy)
        self.assertAlmostEqual(lx, 0.0)
        self.assertAlmostEqual(ly, 500.0)

    def test_local_to_world_swap(self) -> None:
        wx, wy = lc.local_xy_to_world(1300.0, 1700.0)
        self.assertAlmostEqual(wx, -1000.0 + 1700.0)
        self.assertAlmostEqual(wy, -2200.0 + 1300.0)

    def test_foot_z_uses_floor_ref(self) -> None:
        _, _, wz = lc.foot_world_xyz_from_local_xy(0.0, 0.0, foot_offset_cm=50.0)
        self.assertAlmostEqual(wz, lc.FLOOR_REF_Z_CM + 50.0)


if __name__ == "__main__":
    unittest.main()
