#!/usr/bin/env python3
"""Unit tests for stuck_recovery policy."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import List, Tuple
from unittest.mock import MagicMock

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from nav_stack.stuck_recovery import (  # noqa: E402
    StuckRecoveryCallbacks,
    StuckRecoverySession,
    run_site_stuck_recovery,
)

WorldXY = Tuple[float, float]


class _FakePlan:
    def __init__(self, waypoints: List[WorldXY]) -> None:
        self.waypoints_xy = waypoints
        self.stage = "tight_merged"


class StuckRecoveryTest(unittest.TestCase):
    def _callbacks(
        self,
        *,
        replan_waypoints: List[WorldXY],
        spin_calls: List[str],
        clear_calls: List[str],
    ) -> StuckRecoveryCallbacks:
        return StuckRecoveryCallbacks(
            mark_stuck_cells=lambda _s: (1, 0, 0),
            unstuck_backup=lambda _s: None,
            safe_get_pos2d=lambda s: ((s.stuck_xy[0], s.stuck_xy[1] + 2.0), s.ucv),
            execute_escape_step=lambda _s, _xy: None,
            world_to_local=lambda xy: (xy[0], xy[1]),
            record_plan=lambda _s, _wps, _reason: None,
            spin_backup=lambda _s: spin_calls.append("spin"),
            clear_local_l2=lambda _s: clear_calls.append("clear") or 3,
            wait_settle=lambda _s: None,
        )

    def test_low_displacement_triggers_tiered_spin(self) -> None:
        spin_calls: List[str] = []
        clear_calls: List[str] = []
        layers = MagicMock()
        session = StuckRecoverySession(
            ucv=MagicMock(),
            layers=layers,
            robot_name="robot",
            goal_xy=(100.0, 0.0),
            stuck_xy=(0.0, 0.0),
            waypoints=[(50.0, 0.0)],
            wp_index=0,
            waypoint_xy=(50.0, 0.0),
            unstuck_attempts=1,
            max_unstuck_attempts=16,
        )

        def _replan(_layers, _pos, _goal, **kwargs):
            del kwargs
            return _FakePlan([(60.0, 0.0)])

        import nav_stack.stuck_recovery as sr

        original = sr.replan_on_merged_layers
        sr.replan_on_merged_layers = lambda *a, **k: MagicMock(
            stage="tight_merged", waypoints=[(60.0, 0.0)]
        )
        try:
            outcome = run_site_stuck_recovery(
                session,
                callbacks=self._callbacks(
                    replan_waypoints=[(60.0, 0.0)],
                    spin_calls=spin_calls,
                    clear_calls=clear_calls,
                ),
            )
        finally:
            sr.replan_on_merged_layers = original

        self.assertIn("spin", spin_calls)
        self.assertFalse(outcome.mission_failed)
        self.assertEqual(len(outcome.waypoints), 1)

    def test_mission_failed_at_max_attempts(self) -> None:
        layers = MagicMock()
        session = StuckRecoverySession(
            ucv=MagicMock(),
            layers=layers,
            robot_name="robot",
            goal_xy=(100.0, 0.0),
            stuck_xy=(0.0, 0.0),
            waypoints=[(50.0, 0.0)],
            wp_index=0,
            waypoint_xy=(50.0, 0.0),
            unstuck_attempts=15,
            max_unstuck_attempts=16,
        )

        import nav_stack.stuck_recovery as sr

        original = sr.replan_on_merged_layers
        sr.replan_on_merged_layers = lambda *a, **k: MagicMock(stage="failed", waypoints=None)
        try:
            outcome = run_site_stuck_recovery(
                session,
                callbacks=StuckRecoveryCallbacks(
                    mark_stuck_cells=lambda _s: (1, 0, 0),
                    unstuck_backup=lambda _s: None,
                    safe_get_pos2d=lambda s: (s.stuck_xy, s.ucv),
                    execute_escape_step=lambda _s, _xy: None,
                    world_to_local=lambda xy: xy,
                    record_plan=lambda _s, _wps, _reason: None,
                ),
            )
        finally:
            sr.replan_on_merged_layers = original

        self.assertTrue(outcome.mission_failed)


if __name__ == "__main__":
    unittest.main()
