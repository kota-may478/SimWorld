#!/usr/bin/env python3
"""Integration tests for nav_stack costmaps and mission BT."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths()

from costmap_layers import LayeredCostmap  # noqa: E402
from nav_stack.behavior_server import BehaviorServer, RecoveryActionSpec, RecoveryResult  # noqa: E402
from nav_stack.global_costmap import build_planning_costmap  # noqa: E402
from nav_stack.local_costmap import build_local_costmap  # noqa: E402
from nav_stack.mission_bt import MissionRunner, NodeStatus  # noqa: E402
from nav_stack.nav_context import NavContext  # noqa: E402
from nav_stack.planner_server import replan_on_merged_layers  # noqa: E402


def _layers_with_obstacle() -> LayeredCostmap:
    l0 = np.zeros((40, 40), dtype=np.float32)
    layers = LayeredCostmap(
        l0=l0,
        origin_xy=(-1000.0, -2200.0),
        resolution_cm=50.0,
    )
    layers.l2[20, 20] = 255.0
    return layers


class NavStackIntegrationTest(unittest.TestCase):
    def test_global_costmap_adds_clearance(self) -> None:
        layers = _layers_with_obstacle()
        costmap = build_planning_costmap(
            layers,
            planning_clearance_cm=100.0,
            planning_clearance_cost=300.0,
        )
        self.assertGreater(float(costmap.costs.max()), 0.0)

    def test_local_costmap_crop(self) -> None:
        layers = _layers_with_obstacle()
        robot_xy = (-250.0, -1200.0)
        local = build_local_costmap(
            layers,
            robot_xy,
            size_cm=600.0,
            resolution_cm=50.0,
        )
        self.assertEqual(local.width_cells, local.height_cells)
        self.assertAlmostEqual(local.size_cm, 600.0)

    def test_planner_replan_returns_path_or_none(self) -> None:
        layers = _layers_with_obstacle()
        start = (-500.0, -1700.0)
        goal = (-300.0, -1500.0)
        waypoints = replan_on_merged_layers(
            layers,
            start,
            goal,
            planning_clearance_cm=100.0,
            planning_clearance_cost=300.0,
        )
        self.assertTrue(waypoints.waypoints is None or len(waypoints.waypoints) >= 1)

    def test_behavior_server_chain(self) -> None:
        calls: list[str] = []

        def _ok(_ctx: NavContext) -> RecoveryResult:
            calls.append("backup")
            return RecoveryResult(success=True)

        server = BehaviorServer([RecoveryActionSpec("backup", _ok)])
        ctx = NavContext(ucv=None, layers=None, local_costmap=None)
        result = server.run_chain(ctx)
        self.assertIsNotNone(result)
        assert result is not None
        self.assertTrue(result.success)
        self.assertEqual(calls, ["backup"])

    def test_mission_runner_sequence(self) -> None:
        state = {"leg1": False, "carry": False, "leg2": False}

        def leg1() -> bool:
            state["leg1"] = True
            return True

        def carry() -> bool:
            state["carry"] = True
            return True

        def leg2() -> bool:
            state["leg2"] = True
            return True

        runner = MissionRunner(leg1_fn=leg1, carry_fn=carry, leg2_fn=leg2)
        self.assertTrue(runner.run_to_completion())
        self.assertTrue(all(state.values()))

    def test_mission_runner_failure_stops(self) -> None:
        runner = MissionRunner(
            leg1_fn=lambda: False,
            carry_fn=lambda: True,
            leg2_fn=lambda: True,
        )
        self.assertEqual(runner.tick(), NodeStatus.FAILURE)


if __name__ == "__main__":
    unittest.main()
