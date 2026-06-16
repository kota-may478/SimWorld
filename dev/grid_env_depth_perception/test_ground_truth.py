#!/usr/bin/env python3
"""Unit tests for ground-truth geometry (no UE)."""

from __future__ import annotations

import math
import unittest

from ground_truth import (
    bearing_relative_to_forward_deg,
    ground_truth_for_prop,
    horizontal_distance_m,
    normalize_angle_deg,
    rmse,
)
from prop_placement import PropPlacement


class GroundTruthTests(unittest.TestCase):
    def test_normalize_angle(self) -> None:
        self.assertAlmostEqual(normalize_angle_deg(190.0), -170.0)
        self.assertAlmostEqual(normalize_angle_deg(-190.0), 170.0)

    def test_bearing_ahead(self) -> None:
        b = bearing_relative_to_forward_deg((0.0, 0.0), 0.0, (100.0, 0.0))
        self.assertAlmostEqual(b, 0.0)

    def test_bearing_right(self) -> None:
        b = bearing_relative_to_forward_deg((0.0, 0.0), 0.0, (0.0, 100.0))
        self.assertAlmostEqual(b, 90.0)

    def test_horizontal_distance(self) -> None:
        d = horizontal_distance_m((0.0, 0.0), (300.0, 400.0))
        self.assertAlmostEqual(d, 5.0)

    def test_ground_truth_for_prop(self) -> None:
        prop = PropPlacement(
            slot_id="depth_test_prop_001",
            catalog_index=0,
            prop_type_id="barrel_01",
            bp_name="BP_Barrel_01",
            bp_path="/Game/x.BP_x",
            mask_color_rgb=(40, 50, 60),
            local_xy_cm=(500.0, 0.0),
            world_xyz_cm=(500.0, 0.0, 100.0),
            visit_order=1,
        )
        gt = ground_truth_for_prop((0.0, 0.0), 0.0, prop, fov_deg=90.0)
        self.assertAlmostEqual(gt.distance_m, 5.0)
        self.assertAlmostEqual(gt.bearing_deg, 0.0)
        self.assertTrue(gt.in_fov)

    def test_rmse(self) -> None:
        self.assertAlmostEqual(rmse([1.0, 2.0], [1.0, 4.0]), math.sqrt(2.0))


if __name__ == "__main__":
    unittest.main()
