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

from clearance_grid_plan import plan_clearance_grid_waypoints  # noqa: E402
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


if __name__ == "__main__":
    unittest.main()
