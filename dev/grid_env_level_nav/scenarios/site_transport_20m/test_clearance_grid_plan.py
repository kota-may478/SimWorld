#!/usr/bin/env python3
"""Unit tests for clearance grid A* planner."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

from clearance_grid_plan import (  # noqa: E402
    clearance_hug_penalty_cm,
    mean_path_clearance_cm,
    min_center_clearance_at,
    plan_clearance_grid_waypoints,
)
from level_coords import local_xy_to_world  # noqa: E402
from region import ROBOT_START_LOCAL_CM  # noqa: E402
from surface_distance import (  # noqa: E402
    SurfaceObstacle,
    densify_waypoints_for_chord_clearance,
    validate_path_center_clearance,
)


class TestClearanceGridPlan(unittest.TestCase):
    def test_routes_around_blocking_obstacle(self) -> None:
        start = local_xy_to_world(*ROBOT_START_LOCAL_CM)
        goal = local_xy_to_world(1850.0, 1850.0)
        mid_x, mid_y = local_xy_to_world(1000.0, 1000.0)
        obstacles = (
            SurfaceObstacle(
                obstacle_id="block",
                cx=mid_x,
                cy=mid_y,
                half_x=250.0,
                half_y=250.0,
            ),
        )
        path = plan_clearance_grid_waypoints(
            start,
            goal,
            obstacles,
            center_clearance_cm=170.0,
            resolution_cm=40.0,
            block_margin_cm=2.0,
        )
        self.assertGreaterEqual(len(path), 2)
        report = validate_path_center_clearance(
            densify_waypoints_for_chord_clearance(
                path,
                obstacles,
                min_clearance_cm=170.0,
                sample_spacing_cm=16.0,
            ),
            obstacles,
            min_center_clearance_cm=170.0,
            sample_spacing_cm=16.0,
        )
        self.assertTrue(report.ok, msg=f"min={report.min_center_clearance_cm}")
        for wx, wy in path:
            for obs in obstacles:
                dx = abs(wx - obs.cx) - obs.half_x
                dy = abs(wy - obs.cy) - obs.half_y
                dx = max(0.0, dx)
                dy = max(0.0, dy)
                dist = (dx * dx + dy * dy) ** 0.5
                self.assertGreaterEqual(dist, 170.0 - 1.0)

    def test_prefers_open_corridor_over_prop_perimeter(self) -> None:
        """Wide southern detour beats shorter northern skim along obstacle edge."""
        start = local_xy_to_world(100.0, 1000.0)
        goal = local_xy_to_world(1900.0, 1000.0)
        mid_x, mid_y = local_xy_to_world(1000.0, 1000.0)
        obstacles = (
            SurfaceObstacle(
                obstacle_id="block",
                cx=mid_x,
                cy=mid_y,
                half_x=300.0,
                half_y=300.0,
            ),
        )
        short_hug = plan_clearance_grid_waypoints(
            start,
            goal,
            obstacles,
            center_clearance_cm=170.0,
            resolution_cm=40.0,
            block_margin_cm=2.0,
            hug_penalty_weight=0.0,
        )
        open_corridor = plan_clearance_grid_waypoints(
            start,
            goal,
            obstacles,
            center_clearance_cm=170.0,
            resolution_cm=40.0,
            block_margin_cm=2.0,
            hug_penalty_weight=2000.0,
            hug_open_margin_cm=250.0,
        )
        self.assertGreaterEqual(len(short_hug), 2)
        self.assertGreaterEqual(len(open_corridor), 2)
        hug_mean = mean_path_clearance_cm(short_hug, obstacles)
        open_mean = mean_path_clearance_cm(open_corridor, obstacles)
        hug_min = min(
            min_center_clearance_at(wx, wy, obstacles) for wx, wy in short_hug
        )
        open_min = min(
            min_center_clearance_at(wx, wy, obstacles) for wx, wy in open_corridor
        )
        self.assertGreater(
            open_mean,
            hug_mean,
            msg=f"open_mean={open_mean:.1f} hug_mean={hug_mean:.1f}",
        )
        self.assertGreater(
            open_min,
            hug_min + 5.0,
            msg=f"open_min={open_min:.1f} hug_min={hug_min:.1f}",
        )

    def test_hug_penalty_increases_near_minimum_clearance(self) -> None:
        low = clearance_hug_penalty_cm(
            190.0,
            min_clearance_cm=170.0,
            hug_penalty_weight=600.0,
            open_margin_cm=250.0,
        )
        high = clearance_hug_penalty_cm(
            500.0,
            min_clearance_cm=170.0,
            hug_penalty_weight=600.0,
            open_margin_cm=250.0,
        )
        self.assertGreater(low, high)
        self.assertEqual(high, 0.0)


if __name__ == "__main__":
    unittest.main()
