#!/usr/bin/env python3
"""Unit tests for layered costmap + Room D closure detour."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from costmap_layers import LayeredCostmap  # noqa: E402
from l0_nav_mask import COSTMAP_LETHAL_COST, synthetic_l0_corridor  # noqa: E402
from level_coords import local_xy_to_world  # noqa: E402
from zone_registry import ZoneRegistry  # noqa: E402


class TestLayeredCostmap(unittest.TestCase):
  def test_room_d_closure_changes_path(self) -> None:
    l0 = synthetic_l0_corridor(resolution_cm=100.0, corridor_local_y=(0.0, 7900.0))
    layers = LayeredCostmap.from_l0_array(l0, resolution_cm=100.0)
    reg = ZoneRegistry(resolution_cm=100.0)
    reg.add_rect_zone("RoomD", 2500.0, 3500.0, 3200.0, 4200.0, closed_cost=COSTMAP_LETHAL_COST)

    start = local_xy_to_world(500.0, 500.0)
    goal = local_xy_to_world(5000.0, 6000.0)
    open_plan = layers.plan_astar(start, goal)
    self.assertTrue(open_plan.waypoints_xy)

    layers.close_zone("RoomD", reg)
    closed_plan = layers.plan_astar(start, goal)
    self.assertTrue(closed_plan.waypoints_xy)

    def crosses_room_d(waypoints):
      for wx, wy in waypoints:
        lx = wx - (-1000.0)
        ly = wy - (-2200.0)
        if 2500 <= lx <= 3200 and 3500 <= ly <= 4200:
          return True
      return False

    self.assertTrue(crosses_room_d(open_plan.waypoints_xy) or open_plan.total_cost <= closed_plan.total_cost)
    self.assertFalse(crosses_room_d(closed_plan.waypoints_xy))

  def test_l2_merge(self) -> None:
    l0 = np.ones((10, 10), dtype=np.float32)
    layers = LayeredCostmap.from_l0_array(l0, resolution_cm=30.0)
    layers.set_l2_cell(5, 5, 80.0)
    merged = layers.merged_costs()
    self.assertEqual(float(merged[5, 5]), 80.0)


if __name__ == "__main__":
  unittest.main()
