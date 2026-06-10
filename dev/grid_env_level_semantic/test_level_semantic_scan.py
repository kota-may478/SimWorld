#!/usr/bin/env python3
"""Unit tests for nadir depth surface extraction (no UE)."""

from __future__ import annotations

import unittest

import numpy as np

from level_semantic_scan import (
    DEPTH_SKY_THRESHOLD_CM,
    _surface_z_calibration_fallback,
    _surface_z_from_nadir_center,
    classify_semantic_from_center_tiers,
    cube_center_z_cm,
    cube_inscribed_probe_radius_cm,
    depth_band_hits,
)


class TestNadirDepthSurface(unittest.TestCase):
    def _depth_frame(self, center: float, peripheral: float) -> np.ndarray:
        depth = np.full((480, 640), peripheral, dtype=np.float32)
        depth[240, 320] = center
        return depth

    def test_center_hit_returns_surface_below_camera(self) -> None:
        cam_z = 7430.0
        depth = self._depth_frame(602.0, DEPTH_SKY_THRESHOLD_CM)
        surface = _surface_z_from_nadir_center(depth, cam_z_cm=cam_z)
        self.assertIsNotNone(surface)
        self.assertAlmostEqual(surface, cam_z - 602.0, places=1)

    def test_center_sky_without_frame_fallback_returns_none(self) -> None:
        cam_z = 7430.0
        depth = self._depth_frame(DEPTH_SKY_THRESHOLD_CM, 601.5)
        surface = _surface_z_from_nadir_center(depth, cam_z_cm=cam_z)
        self.assertIsNone(surface)

    def test_calibration_fallback_uses_frame_min_when_center_sky(self) -> None:
        cam_z = 7430.0
        depth = self._depth_frame(DEPTH_SKY_THRESHOLD_CM, 601.5)
        surface = _surface_z_calibration_fallback(depth, cam_z_cm=cam_z)
        self.assertIsNotNone(surface)
        self.assertAlmostEqual(surface, cam_z - 601.5, places=1)

    def test_depth_band_hits_none_is_false(self) -> None:
        self.assertFalse(depth_band_hits(None, 6873.5, 30.0))


class TestCubeCollisionLabelRule(unittest.TestCase):
    def test_inscribed_radius_and_center_z(self) -> None:
        self.assertAlmostEqual(cube_inscribed_probe_radius_cm(30.0), 15.0)
        self.assertAlmostEqual(cube_center_z_cm(6500.0, 30.0), 6515.0)
        self.assertAlmostEqual(cube_center_z_cm(6470.0, 30.0), 6485.0)

    def test_classify_semantic_from_center_tiers(self) -> None:
        self.assertEqual(
            classify_semantic_from_center_tiers(
                hit_at_initial_center=True, hit_at_lower_center=True,
            ),
            "wall",
        )
        self.assertEqual(
            classify_semantic_from_center_tiers(
                hit_at_initial_center=False, hit_at_lower_center=True,
            ),
            "floor",
        )
        self.assertEqual(
            classify_semantic_from_center_tiers(
                hit_at_initial_center=False, hit_at_lower_center=False,
            ),
            "air",
        )


if __name__ == "__main__":
    unittest.main()
