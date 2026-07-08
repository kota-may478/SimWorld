#!/usr/bin/env python3
"""Unit tests for dynamic NavMesh obstacle tracking helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_SCENARIO_DIR = Path(__file__).resolve().parent
_NAV_PKG = _SCENARIO_DIR.parents[1]
if str(_NAV_PKG) not in sys.path:
    sys.path.insert(0, str(_NAV_PKG))
if str(_SCENARIO_DIR) not in sys.path:
    sys.path.insert(0, str(_SCENARIO_DIR))

from dynamic_nav_obstacles import DynamicNavObstacleTracker, union_nav_aabbs


class TestUnionNavAabbs(unittest.TestCase):
    def test_union_expands_bounds(self) -> None:
        a = (0.0, 0.0, 0.0, 10.0, 10.0, 5.0)
        b = (5.0, 5.0, -2.0, 20.0, 15.0, 8.0)
        self.assertEqual(union_nav_aabbs(a, b), (0.0, 0.0, -2.0, 20.0, 15.0, 8.0))

    def test_tracker_with_pose_immutable(self) -> None:
        t0 = DynamicNavObstacleTracker(
            actor_name="site20_humanoid",
            obstacle_id="site20_humanoid_nav_obs",
        )
        t1 = t0.with_pose((100.0, 200.0), (90.0, 190.0, 0.0, 110.0, 210.0, 10.0))
        self.assertIsNone(t0.last_xy)
        self.assertEqual(t1.last_xy, (100.0, 200.0))


if __name__ == "__main__":
    unittest.main()
