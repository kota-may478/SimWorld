#!/usr/bin/env python3
"""Unit tests for the 3F kinematic erection oracle (no UE)."""

from __future__ import annotations

import unittest

from constraints.pareto import Theta
from oracle.objectives import score
from oracle.simulate import OracleConfig, run_erection
from scene.geometry import STAGE1_GEOM

FAST = OracleConfig(
    dt_s=0.25,
    timeout_s=480.0,
    erect_s=0.25,
    sockets_per_floor=2,
    handoff_spot_m=0.50,
    handoff_human_m=0.50,
    d_safe_m=1.00,
)


class ErectionOracleTest(unittest.TestCase):
    def test_both_agents_start_on_ground_floor(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(dmin_m=0.8, vmax_mps=0.8),
            config=FAST,
        )
        self.assertGreaterEqual(len(result.trace), 2)
        first = result.trace[0]
        self.assertAlmostEqual(first.human[2], 0.0, places=1)
        self.assertAlmostEqual(first.spot[2], 0.0, places=1)
        self.assertLess(first.spot[0], -STAGE1_GEOM.stair_bay_m)

    def test_spot_cannot_climb_until_floor_one_is_built(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(dmin_m=0.7, vmax_mps=1.0),
            config=FAST,
        )
        n_f1 = FAST.sockets_per_floor
        for sample in result.trace:
            if sample.n_filled < n_f1:
                self.assertLess(sample.spot[2], 0.45)

    def test_completes_three_floors_in_order(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(dmin_m=0.7, vmax_mps=1.0),
            config=FAST,
        )
        self.assertTrue(result.completed)
        self.assertEqual(result.floors_completed, 3)
        self.assertEqual(result.n_filled, result.n_sockets)
        self.assertEqual(result.n_sockets, 6)
        zs_h = [s.human[2] for s in result.trace]
        zs_s = [s.spot[2] for s in result.trace]
        self.assertGreater(max(zs_h), 3.4)
        self.assertGreater(max(zs_s), 3.4)
        first_f2 = next(s.t_s for s in result.trace if s.human[2] > 1.6)
        first_spot_f2 = next(s.t_s for s in result.trace if s.spot[2] > 1.6)
        self.assertGreaterEqual(first_spot_f2, first_f2 - FAST.dt_s)

    def test_spot_waits_on_deck_when_human_is_near_drop(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(dmin_m=1.35, vmax_mps=0.6),
            config=FAST,
            constraint_active=True,
        )
        self.assertGreater(result.wait_s, 0.0)
        self.assertTrue(any(s.blocked for s in result.trace))

    def test_jsafe_is_dmin_violation_fraction_not_always_zero(self) -> None:
        aggressive = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(dmin_m=0.45, vmax_mps=1.0),
            config=FAST,
            constraint_active=True,
        )
        cautious = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(dmin_m=1.4, vmax_mps=0.45),
            config=FAST,
            constraint_active=True,
        )
        free = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(dmin_m=1.4, vmax_mps=0.45),
            config=FAST,
            constraint_active=False,
        )
        scored_ag = score(aggressive)
        scored_free = score(free)
        self.assertGreater(aggressive.violation_s, cautious.violation_s)
        self.assertGreater(scored_ag.jsafe, 0.0)
        self.assertGreater(scored_free.jsafe, 0.0)

    def test_loose_theta_finishes_faster_than_tight(self) -> None:
        loose = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(dmin_m=0.45, vmax_mps=1.0),
            config=FAST,
        )
        tight = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(dmin_m=1.4, vmax_mps=0.35),
            config=FAST,
        )
        self.assertTrue(loose.completed)
        self.assertTrue(tight.completed)
        self.assertLess(loose.makespan_s, tight.makespan_s)

    def test_corridor_ignores_scaffold_theta(self) -> None:
        a = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(dmin_m=0.45, vmax_mps=1.0),
            config=FAST,
        )
        b = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(dmin_m=1.4, vmax_mps=0.35),
            config=FAST,
        )
        self.assertTrue(a.completed)
        self.assertTrue(b.completed)
        self.assertGreater(a.corridor_time_s, 1.0)
        self.assertAlmostEqual(a.corridor_time_s, b.corridor_time_s, delta=40.0)

    def test_trace_records_spot_and_human_each_tick(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(dmin_m=0.8, vmax_mps=0.8),
            config=FAST,
        )
        self.assertGreaterEqual(len(result.trace), 2)
        self.assertAlmostEqual(result.trace[0].t_s, FAST.dt_s, places=5)
        self.assertEqual(len(result.trace), len({s.t_s for s in result.trace}))


if __name__ == "__main__":
    unittest.main()
