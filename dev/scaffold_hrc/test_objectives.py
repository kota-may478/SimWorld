#!/usr/bin/env python3
"""Unit tests for TT scoring (full mission time in seconds)."""

from __future__ import annotations

import unittest

from oracle.objectives import score
from oracle.simulate import OracleResult


def _result(
    *,
    filled: int,
    sockets: int,
    makespan_s: float,
    scaffold_time_s: float,
    timeout_s: float = 100.0,
    completed: bool = True,
    si_min: float = 1.2,
    violation_s: float = 0.0,
    corridor_time_s: float = 0.0,
) -> OracleResult:
    return OracleResult(
        completed=completed,
        makespan_s=makespan_s,
        path_length_m=0.0,
        corridor_time_s=corridor_time_s,
        min_separation_m=1.0,
        wait_s=0.0,
        violation_s=violation_s,
        n_filled=filled,
        n_sockets=sockets,
        floors_completed=0,
        timeout_s=timeout_s,
        trace=(),
        ssm_s=0.0,
        si_min=si_min,
        scaffold_safe_s=scaffold_time_s,
        scaffold_unsafe_s=0.0,
        scaffold_time_s=scaffold_time_s,
    )


class ObjectiveScoreTest(unittest.TestCase):
    def test_tt_is_full_mission_time_including_corridor(self) -> None:
        out = score(
            _result(
                filled=15,
                sockets=30,
                makespan_s=80.0,
                scaffold_time_s=44.0,
                corridor_time_s=36.0,
            )
        )
        self.assertAlmostEqual(out.tcr, 0.5)
        self.assertAlmostEqual(out.tt, 80.0)
        self.assertAlmostEqual(out.mission_s, 80.0)
        self.assertAlmostEqual(out.scaffold_time_s, 44.0)
        self.assertTrue(out.iso_feasible)

    def test_tt_does_not_use_a_reference_ratio(self) -> None:
        out = score(
            _result(
                filled=30,
                sockets=30,
                makespan_s=50.0,
                scaffold_time_s=12.5,
            ),
            t_ref_s=40.0,
        )
        self.assertAlmostEqual(out.tt, 50.0)

    def test_iso_infeasible_when_si_min_below_one(self) -> None:
        out = score(
            _result(
                filled=30,
                sockets=30,
                makespan_s=50.0,
                scaffold_time_s=50.0,
                si_min=0.7,
            )
        )
        self.assertFalse(out.iso_feasible)

    def test_incomplete_run_is_not_iso_feasible(self) -> None:
        out = score(
            _result(
                filled=2,
                sockets=6,
                makespan_s=100.0,
                scaffold_time_s=20.0,
                completed=False,
                si_min=1.5,
            )
        )
        self.assertFalse(out.iso_feasible)


if __name__ == "__main__":
    unittest.main()
