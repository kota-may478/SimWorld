#!/usr/bin/env python3
"""Unit tests for L2 cell-delta replan threshold (cells_removed propagation)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

import layered_nav as ln  # noqa: E402


class L2ReplanThresholdTest(unittest.TestCase):
    def test_cells_removed_can_trigger_replan(self) -> None:
        ln.L2_REPLAN_CELL_DELTA_THRESHOLD = 3
        self.assertFalse(ln._l2_cell_delta_warrants_replan(0, 2))
        self.assertTrue(ln._l2_cell_delta_warrants_replan(0, 3))
        self.assertFalse(ln._l2_cell_delta_warrants_replan(1, 2))
        self.assertTrue(ln._l2_cell_delta_warrants_replan(2, 4))

    def test_peak_and_net_delta(self) -> None:
        ln.L2_REPLAN_CELL_DELTA_THRESHOLD = 5
        self.assertFalse(ln._l2_cell_delta_warrants_replan(2, 3))
        self.assertFalse(ln._l2_cell_delta_warrants_replan(2, 2))
        self.assertTrue(ln._l2_cell_delta_warrants_replan(0, 5))
        self.assertTrue(ln._l2_cell_delta_warrants_replan(10, 3))


if __name__ == "__main__":
    unittest.main()
