#!/usr/bin/env python3
"""Unit tests for nav_kpi."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from nav_stack.nav_kpi import NavKpiTracker, cross_track_error_cm  # noqa: E402


class NavKpiTest(unittest.TestCase):
    def test_cross_track_on_segment(self) -> None:
        err = cross_track_error_cm((50.0, 50.0), (0.0, 0.0), (100.0, 0.0))
        self.assertAlmostEqual(err, 50.0, places=1)

    def test_replan_success_rate(self) -> None:
        kpi = NavKpiTracker()
        kpi.record_replan(success=True)
        kpi.record_replan(success=False)
        self.assertAlmostEqual(kpi.replan_success_rate, 0.5)

    def test_mean_cross_track(self) -> None:
        kpi = NavKpiTracker()
        kpi.record_cross_track(10.0)
        kpi.record_cross_track(30.0)
        self.assertAlmostEqual(kpi.mean_cross_track_error_cm, 20.0)

    def test_open_loop_scale_ema(self) -> None:
        kpi = NavKpiTracker()
        kpi.update_open_loop_scale(100.0, 80.0)
        self.assertTrue(math.isfinite(kpi.open_loop_scale_ema))
        self.assertLess(kpi.open_loop_scale_ema, 1.0)

    def test_to_dict_keys(self) -> None:
        kpi = NavKpiTracker()
        kpi.record_stuck()
        payload = kpi.to_dict()
        self.assertIn("stuck_events", payload)
        self.assertIn("replan_success_rate", payload)
        self.assertEqual(payload["stuck_events"], 1)


if __name__ == "__main__":
    unittest.main()
