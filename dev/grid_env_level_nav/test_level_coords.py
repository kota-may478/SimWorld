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

    def test_foot_z_uses_floor_ref(self) -> None:
        _, _, wz = lc.foot_world_xyz_from_local_xy(0.0, 0.0, foot_offset_cm=50.0)
        self.assertAlmostEqual(wz, lc.FLOOR_REF_Z_CM + 50.0)


if __name__ == "__main__":
    unittest.main()
