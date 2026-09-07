#!/usr/bin/env python3
"""Unit tests for (α, β) weighted-sum projection (no UE, no LLM)."""

from __future__ import annotations

import unittest

from constraints.pareto import EvaluatedTheta, Theta
from fronts.ab_map import (
    LEVELS,
    enumerate_table,
    mix_weights,
    pick_weighted,
    snap_level,
    step_preference,
)


def _row(
    vmax: float,
    dmin: float,
    tt: float,
    min_sep: float,
) -> EvaluatedTheta:
    return EvaluatedTheta(
        Theta(vmax_mps=vmax, dmin_m=dmin),
        tt=tt,
        t_ssm=vmax,
        si_min=1.2,
        completed=True,
        min_sep_m=min_sep,
    )


FRONT = (
    _row(1.0, 0.35, tt=50.0, min_sep=0.40),
    _row(0.20, 1.60, tt=200.0, min_sep=1.60),
    _row(1.0, 1.60, tt=55.0, min_sep=1.50),
)


class AbMapTest(unittest.TestCase):
    def test_snap_uses_five_levels(self) -> None:
        self.assertEqual(LEVELS, (0.0, 0.25, 0.5, 0.75, 1.0))
        self.assertAlmostEqual(snap_level(0.26), 0.25)
        self.assertAlmostEqual(snap_level(0.9), 1.0)

    def test_alpha_zero_ignores_beta(self) -> None:
        self.assertEqual(mix_weights(0.0, 0.0), mix_weights(0.0, 1.0))
        self.assertEqual(mix_weights(0.0, 0.0), (1.0, 0.0, 0.0))

    def test_alpha_one_splits_speed_and_distance(self) -> None:
        self.assertEqual(mix_weights(1.0, 1.0), (0.0, 1.0, 0.0))
        self.assertEqual(mix_weights(1.0, 0.0), (0.0, 0.0, 1.0))
        self.assertEqual(mix_weights(1.0, 0.5), (0.0, 0.5, 0.5))

    def test_pick_efficiency_is_lowest_tt(self) -> None:
        picked = pick_weighted(FRONT, 0.0, 0.5)
        self.assertAlmostEqual(picked.tt, 50.0)

    def test_pick_slow_end_minimizes_vmax(self) -> None:
        picked = pick_weighted(FRONT, 1.0, 1.0)
        self.assertAlmostEqual(picked.theta.vmax_mps, 0.20)

    def test_pick_distance_end_maximizes_min_sep(self) -> None:
        picked = pick_weighted(FRONT, 1.0, 0.0)
        self.assertAlmostEqual(picked.min_sep_m, 1.60)

    def test_table_has_twenty_five_cells_and_collapses_alpha_zero(self) -> None:
        table = enumerate_table(FRONT)
        self.assertEqual(len(table), 25)
        zeros = {id(table[(0.0, b)]) for b in LEVELS}
        self.assertEqual(len({table[(0.0, b)].theta for b in LEVELS}), 1)
        self.assertEqual(len(zeros), 1)

    def test_step_from_efficiency_opens_beta(self) -> None:
        alpha, beta = step_preference(0.0, 0.5, d_alpha=0, d_beta=1)
        self.assertAlmostEqual(alpha, 0.25)
        self.assertAlmostEqual(beta, 0.75)

    def test_step_clamps_at_the_rail(self) -> None:
        alpha, beta = step_preference(1.0, 1.0, d_alpha=1, d_beta=1)
        self.assertAlmostEqual(alpha, 1.0)
        self.assertAlmostEqual(beta, 1.0)

    def test_distinct_theta_skips_shared_cells(self) -> None:
        shared = _row(0.43, 1.60, tt=1171.0, min_sep=1.58)
        nxt = _row(0.42, 1.60, tt=1188.0, min_sep=1.57)
        far = _row(0.27, 1.36, tt=1702.0, min_sep=1.34)
        table = {(a, b): shared for a in LEVELS for b in LEVELS}
        table[(1.0, 0.0)] = shared
        table[(1.0, 0.25)] = shared
        table[(1.0, 0.5)] = nxt
        table[(1.0, 0.75)] = far
        alpha, beta = step_preference(
            1.0, 0.0, d_alpha=0, d_beta=1, theta_table=table
        )
        self.assertAlmostEqual(alpha, 1.0)
        self.assertAlmostEqual(beta, 0.5)
        alpha, beta = step_preference(
            1.0, 0.0, d_alpha=0, d_beta=2, theta_table=table
        )
        self.assertAlmostEqual(alpha, 1.0)
        self.assertAlmostEqual(beta, 0.75)

    def test_distinct_theta_keeps_neighbor_when_already_unique(self) -> None:
        here = _row(0.62, 1.60, tt=886.0, min_sep=1.56)
        nxt = _row(0.56, 1.60, tt=953.0, min_sep=1.56)
        table = {(a, b): here for a in LEVELS for b in LEVELS}
        table[(0.5, 0.5)] = here
        table[(0.5, 0.75)] = nxt
        alpha, beta = step_preference(
            0.5, 0.5, d_alpha=0, d_beta=1, theta_table=table
        )
        self.assertAlmostEqual(alpha, 0.5)
        self.assertAlmostEqual(beta, 0.75)

    def test_collapsed_alpha_row_opens_then_steps_beta(self) -> None:
        shared = _row(1.0, 0.78, tt=640.0, min_sep=0.76)
        opened = _row(0.85, 0.78, tt=700.0, min_sep=0.80)
        slower = _row(0.60, 0.78, tt=800.0, min_sep=0.80)
        table = {(a, b): shared for a in LEVELS for b in LEVELS}
        for b in LEVELS:
            table[(0.0, b)] = shared
        table[(0.25, 0.5)] = opened
        table[(0.25, 0.75)] = opened
        table[(0.25, 1.0)] = slower
        alpha, beta = step_preference(
            0.0, 0.5, d_alpha=0, d_beta=1, theta_table=table
        )
        self.assertAlmostEqual(alpha, 0.25)
        self.assertAlmostEqual(beta, 0.5)
        alpha, beta = step_preference(
            0.0, 0.5, d_alpha=0, d_beta=2, theta_table=table
        )
        self.assertAlmostEqual(alpha, 0.25)
        self.assertAlmostEqual(beta, 1.0)

    def test_empty_front_raises(self) -> None:
        with self.assertRaises(ValueError):
            pick_weighted((), 0.5, 0.5)


if __name__ == "__main__":
    unittest.main()
