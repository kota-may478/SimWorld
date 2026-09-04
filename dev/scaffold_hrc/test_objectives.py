#!/usr/bin/env python3
"""Unit tests for dimensionless Jeff / Jsafe scoring."""

from __future__ import annotations

import unittest

from oracle.objectives import W_SAFE, W_TCR, W_TT, score
from oracle.simulate import OracleResult


def _result(
    *,
    filled: int,
    sockets: int,
    makespan_s: float,
    violation_s: float,
    timeout_s: float = 100.0,
    completed: bool = True,
) -> OracleResult:
    return OracleResult(
        completed=completed,
        makespan_s=makespan_s,
        path_length_m=0.0,
        corridor_time_s=0.0,
        min_separation_m=1.0,
        wait_s=0.0,
        violation_s=violation_s,
        n_filled=filled,
        n_sockets=sockets,
        floors_completed=0,
        timeout_s=timeout_s,
        trace=(),
    )


class ObjectiveScoreTest(unittest.TestCase):
    def test_tcr_and_tt_are_dimensionless(self) -> None:
        out = score(_result(filled=15, sockets=30, makespan_s=40.0, violation_s=10.0))
        self.assertAlmostEqual(out.tcr, 0.5)
        self.assertAlmostEqual(out.tt, 0.4)
        self.assertAlmostEqual(out.jsafe, 0.1)
        self.assertGreaterEqual(out.tcr, 0.0)
        self.assertLessEqual(out.tcr, 1.0)
        self.assertGreaterEqual(out.tt, 0.0)
        self.assertGreaterEqual(out.jsafe, 0.0)

    def test_jeff_matches_weighted_tcr_minus_tt(self) -> None:
        out = score(_result(filled=30, sockets=30, makespan_s=25.0, violation_s=0.0))
        self.assertAlmostEqual(out.jeff, W_TCR * 1.0 - W_TT * 0.25)
        self.assertAlmostEqual(out.j, out.jeff - W_SAFE * out.jsafe)

    def test_jsafe_is_penalty_not_a_hard_constraint(self) -> None:
        clean = score(_result(filled=30, sockets=30, makespan_s=50.0, violation_s=0.0))
        dirty = score(_result(filled=30, sockets=30, makespan_s=50.0, violation_s=20.0))
        self.assertAlmostEqual(clean.jeff, dirty.jeff)
        self.assertLess(dirty.j, clean.j)
        self.assertGreater(dirty.jsafe, clean.jsafe)

    def test_tt_and_jsafe_may_exceed_one_versus_t_ref(self) -> None:
        out = score(
            _result(filled=2, sockets=2, makespan_s=250.0, violation_s=150.0, timeout_s=100.0),
            t_ref_s=100.0,
        )
        self.assertAlmostEqual(out.tt, 2.5)
        self.assertAlmostEqual(out.jsafe, 1.5)


if __name__ == "__main__":
    unittest.main()
