#!/usr/bin/env python3
"""Unit tests for the frozen ISO-front index (no UE)."""

from __future__ import annotations

import unittest

from fronts.ab_map import LEVELS
from fronts.frozen import load_frozen, named_cells, theta_table


class FrozenThetaTableTest(unittest.TestCase):
    def test_twelve_distinct_theta_on_five_by_five(self) -> None:
        payload = load_frozen()
        self.assertEqual(payload["nsga2_iso_points"], 356)
        self.assertEqual(payload["grid_cells"], 25)
        self.assertEqual(payload["distinct_theta"], 12)
        table = theta_table(payload)
        self.assertEqual(len(table), 25)
        keys = {
            (round(row.theta.vmax_mps, 4), round(row.theta.dmin_m, 4))
            for row in table.values()
        }
        self.assertEqual(len(keys), 12)

    def test_alpha_zero_row_collapses(self) -> None:
        table = theta_table()
        first = table[(0.0, 0.0)]
        for beta in LEVELS:
            row = table[(0.0, beta)]
            self.assertAlmostEqual(row.theta.vmax_mps, first.theta.vmax_mps, places=4)
            self.assertAlmostEqual(row.theta.dmin_m, first.theta.dmin_m, places=4)

    def test_seven_named_absolute_cells(self) -> None:
        named = named_cells()
        self.assertEqual(len(named), 7)
        labels = [item["cell"] for item in named]
        self.assertEqual(
            labels,
            [
                "efficient",
                "normal_distance",
                "normal",
                "normal_slow",
                "safe_distance",
                "safe",
                "safe_slow",
            ],
        )
        self.assertGreater(named[-1]["tt_s"], named[0]["tt_s"])
        self.assertLess(named[-1]["vmax_mps"], named[0]["vmax_mps"])
        for item in named:
            self.assertGreaterEqual(item["si_min"], 1.0)
