#!/usr/bin/env python3
"""Unit tests for L2 replan cell-delta policy."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from nav_stack.l2_replan_policy import l2_cell_delta_warrants_replan  # noqa: E402


class L2ReplanPolicyTest(unittest.TestCase):
    def test_large_clear_triggers(self) -> None:
        self.assertTrue(l2_cell_delta_warrants_replan(0, 54, threshold=5))
        self.assertTrue(l2_cell_delta_warrants_replan(11, 101, threshold=5))

    def test_balanced_flicker_skipped(self) -> None:
        self.assertFalse(l2_cell_delta_warrants_replan(2, 2, threshold=5))
        self.assertFalse(l2_cell_delta_warrants_replan(3, 3, threshold=5))

    def test_peak_threshold(self) -> None:
        self.assertTrue(l2_cell_delta_warrants_replan(0, 5, threshold=5))
        self.assertFalse(l2_cell_delta_warrants_replan(0, 4, threshold=5))

    def test_net_threshold(self) -> None:
        self.assertTrue(l2_cell_delta_warrants_replan(10, 3, threshold=5))
        self.assertTrue(l2_cell_delta_warrants_replan(6, 3, threshold=5))
        self.assertFalse(l2_cell_delta_warrants_replan(4, 3, threshold=5))


if __name__ == "__main__":
    unittest.main()
