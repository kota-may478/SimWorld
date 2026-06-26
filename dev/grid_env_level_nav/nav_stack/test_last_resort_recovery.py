#!/usr/bin/env python3
"""Unit tests for last_resort_recovery."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from nav_stack.last_resort_recovery import (  # noqa: E402
    MAX_L2_FLUSH_COUNT,
    try_last_resort_recovery,
)


class _Plan:
    waypoints_xy = [(10.0, 0.0), (20.0, 0.0)]


class LastResortRecoveryTest(unittest.TestCase):
    def test_returns_none_when_budget_exhausted(self) -> None:
        layers = MagicMock()
        outcome = try_last_resort_recovery(
            layers=layers,
            pos_xy=(0.0, 0.0),
            goal_xy=(100.0, 0.0),
            l2_seen_cells=set(),
            l2_flush_count=MAX_L2_FLUSH_COUNT,
            stuck_xy=(0.0, 0.0),
            soft_reset_fn=None,
            replan_fn=lambda _a, _b: _Plan(),
            nearest_wp_index_fn=lambda _pos, _wps, _idx: 0,
        )
        self.assertIsNone(outcome)

    def test_aggressive_flush_and_replan(self) -> None:
        layers = MagicMock()
        seen: set = {(1, 1)}
        reset_calls: list = []

        def _soft_reset(cells, stuck_xy, *, aggressive=False):
            reset_calls.append(aggressive)
            cells.clear()

        outcome = try_last_resort_recovery(
            layers=layers,
            pos_xy=(0.0, 0.0),
            goal_xy=(100.0, 0.0),
            l2_seen_cells=seen,
            l2_flush_count=0,
            stuck_xy=(0.0, 0.0),
            soft_reset_fn=_soft_reset,
            replan_fn=lambda _a, _b: _Plan(),
            nearest_wp_index_fn=lambda _pos, _wps, _idx: 0,
        )
        self.assertIsNotNone(outcome)
        assert outcome is not None
        self.assertTrue(outcome.success)
        self.assertEqual(outcome.l2_flush_count, 1)
        self.assertTrue(reset_calls and reset_calls[0] is True)
        self.assertEqual(len(seen), 0)


if __name__ == "__main__":
    unittest.main()
