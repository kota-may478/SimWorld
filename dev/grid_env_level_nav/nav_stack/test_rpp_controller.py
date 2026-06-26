#!/usr/bin/env python3
"""Unit tests for Regulated Pure Pursuit controller."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PKG = Path(__file__).resolve().parents[1]
_REPO = _PKG.parent.parent
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))
if str(_REPO / "dev" / "grid_env_level_nav") not in sys.path:
    sys.path.insert(0, str(_REPO / "dev" / "grid_env_level_nav"))

from bootstrap import setup_paths

setup_paths()

from nav_stack.controllers.rpp import (  # noqa: E402
    RppConfig,
    compute_rpp_command,
    find_lookahead_point,
)


class RppControllerTest(unittest.TestCase):
    def test_find_lookahead_on_straight_path(self) -> None:
        pos = (0.0, 0.0)
        waypoints = [(100.0, 0.0), (200.0, 0.0)]
        lookahead = find_lookahead_point(pos, waypoints, 0, 80.0)
        self.assertAlmostEqual(lookahead[0], 80.0, places=1)
        self.assertAlmostEqual(lookahead[1], 0.0, places=1)

    def test_find_lookahead_past_final_waypoint(self) -> None:
        pos = (0.0, 0.0)
        waypoints = [(30.0, 0.0)]
        lookahead = find_lookahead_point(pos, waypoints, 0, 80.0)
        self.assertAlmostEqual(lookahead[0], 30.0, places=1)

    def test_rotate_only_when_heading_error_large(self) -> None:
        pos = (0.0, 0.0)
        yaw = 0.0
        waypoints = [(100.0, 100.0)]
        cfg = RppConfig(rotate_to_heading_threshold_deg=35.0)
        cmd = compute_rpp_command(
            pos,
            yaw,
            waypoints,
            0,
            config=cfg,
            max_move_cm=120.0,
        )
        self.assertIsNotNone(cmd)
        assert cmd is not None
        self.assertGreater(cmd.turn_deg, 35.0)
        self.assertAlmostEqual(cmd.move_cm, 0.0)

    def test_forward_move_on_aligned_path(self) -> None:
        pos = (0.0, 0.0)
        yaw = 0.0
        waypoints = [(200.0, 0.0)]
        cfg = RppConfig(lookahead_cm=80.0)
        cmd = compute_rpp_command(
            pos,
            yaw,
            waypoints,
            0,
            config=cfg,
            max_move_cm=120.0,
        )
        self.assertIsNotNone(cmd)
        assert cmd is not None
        self.assertAlmostEqual(cmd.turn_deg, 0.0)
        self.assertGreater(cmd.move_cm, 50.0)
        self.assertLessEqual(cmd.move_cm, 120.0)

    def test_curvature_regulation_reduces_speed(self) -> None:
        pos = (0.0, 0.0)
        yaw = 0.0
        waypoints = [(40.0, 80.0)]
        cfg = RppConfig(
            lookahead_cm=60.0,
            regulated_linear_scaling_min_radius_cm=200.0,
            regulated_linear_scaling_min_speed_frac=0.4,
            rotate_to_heading_threshold_deg=90.0,
        )
        straight_cmd = compute_rpp_command(
            pos,
            yaw,
            [(200.0, 0.0)],
            0,
            config=cfg,
            max_move_cm=100.0,
        )
        curve_cmd = compute_rpp_command(
            pos,
            yaw,
            waypoints,
            0,
            config=cfg,
            max_move_cm=100.0,
        )
        self.assertIsNotNone(straight_cmd)
        self.assertIsNotNone(curve_cmd)
        assert straight_cmd is not None and curve_cmd is not None
        self.assertGreater(straight_cmd.move_cm, curve_cmd.move_cm)

    def test_reach_waypoint_returns_none_at_goal(self) -> None:
        pos = (100.0, 0.0)
        yaw = 0.0
        waypoints = [(100.0, 0.0)]
        cmd = compute_rpp_command(
            pos,
            yaw,
            waypoints,
            0,
            config=RppConfig(),
            max_move_cm=120.0,
        )
        self.assertIsNone(cmd)


if __name__ == "__main__":
    unittest.main()
