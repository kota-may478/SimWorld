#!/usr/bin/env python3
"""Unit tests for surface-distance proximity metrics."""

from __future__ import annotations

import unittest

from surface_distance import (
    SurfaceObstacle,
    center_to_aabb_surface_distance_cm,
    nearest_surface_distance_cm,
)


class TestSurfaceDistance(unittest.TestCase):
    def test_center_outside_aabb(self) -> None:
        obstacle = SurfaceObstacle(
            obstacle_id="box",
            cx=0.0,
            cy=0.0,
            half_x=50.0,
            half_y=50.0,
        )
        dist = center_to_aabb_surface_distance_cm((200.0, 0.0), obstacle)
        self.assertAlmostEqual(dist, 150.0)

    def test_center_inside_aabb_reports_zero_surface_gap(self) -> None:
        obstacle = SurfaceObstacle(
            obstacle_id="box",
            cx=0.0,
            cy=0.0,
            half_x=50.0,
            half_y=50.0,
        )
        dist = center_to_aabb_surface_distance_cm((10.0, 10.0), obstacle)
        self.assertAlmostEqual(dist, 0.0)

    def test_nearest_among_multiple(self) -> None:
        obstacles = (
            SurfaceObstacle("a", 0.0, 0.0, 40.0, 40.0),
            SurfaceObstacle("b", 300.0, 0.0, 40.0, 40.0),
        )
        dist, obstacle_id = nearest_surface_distance_cm((120.0, 0.0), obstacles)
        self.assertEqual(obstacle_id, "a")
        self.assertAlmostEqual(dist, 80.0)


if __name__ == "__main__":
    unittest.main()
