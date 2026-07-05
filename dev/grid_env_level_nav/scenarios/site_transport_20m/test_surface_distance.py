#!/usr/bin/env python3
"""Unit tests for surface-distance proximity metrics."""

from __future__ import annotations

import unittest

from surface_distance import (
    SurfaceObstacle,
    center_to_aabb_surface_distance_cm,
    densify_waypoints_for_chord_clearance,
    min_clearance_on_segment_cm,
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

    def test_chord_clearance_inserts_midpoint(self) -> None:
        obstacles = (
            SurfaceObstacle("box", 0.0, 100.0, 50.0, 50.0),
        )
        start = (-200.0, 0.0)
        end = (200.0, 0.0)
        clearance = min_clearance_on_segment_cm(start, end, obstacles)
        self.assertIsNotNone(clearance)
        assert clearance is not None
        self.assertLess(clearance, 100.0)
        dense = densify_waypoints_for_chord_clearance(
            [start, end],
            obstacles,
            min_clearance_cm=100.0,
        )
        self.assertGreater(len(dense), 2)


if __name__ == "__main__":
    unittest.main()
