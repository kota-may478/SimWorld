#!/usr/bin/env python3
"""Unit tests for surface-distance proximity metrics."""

from __future__ import annotations

import unittest

from surface_distance import (
    SurfaceObstacle,
    adjust_xy_for_planning_clearance,
    build_path_clearance_obstacles,
    center_to_aabb_surface_distance_cm,
    densify_waypoints_for_chord_clearance,
    min_clearance_on_segment_cm,
    nearest_surface_distance_cm,
    validate_path_center_clearance,
    validate_path_corridor_clearance,
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

    def test_chord_densify_inserts_midpoint(self) -> None:
        obstacles = (
            SurfaceObstacle("wall", 100.0, 0.0, 10.0, 200.0),
        )
        points = ((0.0, 0.0), (200.0, 0.0))
        dense = densify_waypoints_for_chord_clearance(
            points,
            obstacles,
            min_clearance_cm=120.0,
            sample_spacing_cm=20.0,
            max_insertions=8,
        )
        self.assertGreater(len(dense), len(points))
        clearance = min_clearance_on_segment_cm(
            dense[0],
            dense[1],
            obstacles,
            sample_spacing_cm=10.0,
        )
        self.assertIsNotNone(clearance)
        if clearance is not None:
            self.assertGreater(clearance, 50.0)

    def test_validate_path_center_clearance_rejects_tight_segment(self) -> None:
        obstacles = (
            SurfaceObstacle("wall", 100.0, 0.0, 10.0, 200.0),
        )
        points = ((0.0, 0.0), (200.0, 0.0))
        report = validate_path_center_clearance(
            points,
            obstacles,
            min_center_clearance_cm=170.0,
            body_radius_cm=70.0,
            sample_spacing_cm=16.0,
            validate_segments=True,
        )
        self.assertFalse(report.ok)
        self.assertGreater(report.violating_segment_count, 0)


    def test_adjust_xy_for_planning_clearance(self) -> None:
        obstacles = (
            SurfaceObstacle("wall", 100.0, 0.0, 10.0, 200.0),
        )
        tight = (115.0, 0.0)
        before, _ = nearest_surface_distance_cm(tight, obstacles)
        adjusted, ok = adjust_xy_for_planning_clearance(
            tight,
            obstacles,
            min_center_clearance_cm=170.0,
        )
        self.assertTrue(ok)
        after, _ = nearest_surface_distance_cm(adjusted, obstacles)
        self.assertIsNotNone(before)
        self.assertIsNotNone(after)
        if after is not None:
            self.assertGreaterEqual(after, 170.0)
        self.assertGreater(adjusted[0], tight[0])


    def test_build_path_clearance_obstacles_exempts_material_and_humanoid(self) -> None:
        from navmesh_types import ActorBounds

        cache = {
            "site20_prop_001": ActorBounds("site20_prop_001", 0, 0, 0, 10, 10, 10),
            "site20_material": ActorBounds("site20_material", 100, 0, 0, 20, 20, 10),
            "site20_humanoid": ActorBounds("site20_humanoid", 200, 0, 0, 30, 30, 10),
        }
        obstacles = build_path_clearance_obstacles(
            cache,
            exempt_actor_names=("site20_material", "site20_humanoid"),
        )
        ids = {o.obstacle_id for o in obstacles}
        self.assertEqual(ids, {"site20_prop_001"})


    def test_corridor_excludes_last_waypoint(self) -> None:
        obstacles = (
            SurfaceObstacle("wall", 100.0, 0.0, 10.0, 10.0),
        )
        points = ((0.0, 200.0), (200.0, 200.0), (115.0, 0.0))
        full = validate_path_center_clearance(
            points,
            obstacles,
            min_center_clearance_cm=170.0,
        )
        corridor = validate_path_corridor_clearance(
            points,
            obstacles,
            min_center_clearance_cm=170.0,
        )
        self.assertFalse(full.ok)
        self.assertTrue(corridor.ok)

    def test_corridor_excludes_first_and_last_waypoint(self) -> None:
        obstacles = (
            SurfaceObstacle("wall", 100.0, 0.0, 10.0, 10.0),
        )
        points = ((115.0, 0.0), (200.0, 200.0), (0.0, 200.0))
        full = validate_path_center_clearance(
            points,
            obstacles,
            min_center_clearance_cm=170.0,
        )
        corridor = validate_path_corridor_clearance(
            points,
            obstacles,
            min_center_clearance_cm=170.0,
        )
        self.assertFalse(full.ok)
        self.assertTrue(corridor.ok)


if __name__ == "__main__":
    unittest.main()
