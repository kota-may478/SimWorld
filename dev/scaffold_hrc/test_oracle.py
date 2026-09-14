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
    timeout_s=1200.0,
    erect_s=0.25,
    truck_load_s=0.25,
    drop_place_s=0.25,
    assembler_pickup_s=0.25,
    sockets_per_floor=2,
    handoff_spot_m=0.50,
    handoff_human_m=0.50,
    d_safe_m=1.00,
)


class ErectionOracleTest(unittest.TestCase):
    def test_both_agents_start_on_ground_floor(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=0.8, dmin_m=0.80),
            config=FAST,
        )
        self.assertGreaterEqual(len(result.trace), 2)
        first = result.trace[0]
        self.assertAlmostEqual(first.human[2], 0.0, places=1)
        self.assertAlmostEqual(first.spot[2], 0.0, places=1)
        self.assertLess(first.spot[0], STAGE1_GEOM.stair_xy_bounds()[0])

    def test_spot_cannot_climb_until_floor_one_is_built(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=0.35),
            config=FAST,
        )
        n_f1 = FAST.sockets_per_floor
        for sample in result.trace:
            if sample.n_filled < n_f1:
                self.assertLess(sample.spot[2], 0.45)

    def test_completes_three_floors_in_order(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=0.35),
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
            theta=Theta(vmax_mps=0.6, dmin_m=1.35),
            config=FAST,
            constraint_active=True,
        )
        self.assertGreater(result.wait_s, 0.0)
        self.assertTrue(any(s.blocked for s in result.trace))

    def test_ssm_intervention_is_recorded(self) -> None:
        aggressive = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=0.35),
            config=FAST,
            constraint_active=True,
        )
        cautious = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=0.45, dmin_m=1.40),
            config=FAST,
            constraint_active=True,
        )
        scored_ag = score(aggressive)
        self.assertGreaterEqual(aggressive.ssm_s, 0.0)
        self.assertGreaterEqual(cautious.ssm_s, 0.0)
        self.assertGreater(scored_ag.tt, 0.0)
        self.assertTrue(hasattr(aggressive.trace[0], "si"))
        self.assertTrue(aggressive.completed)
        self.assertTrue(cautious.completed)
        self.assertGreaterEqual(aggressive.si_min, 1.0)
        self.assertGreaterEqual(cautious.si_min, 1.0)

    def test_loose_theta_finishes_faster_than_tight(self) -> None:
        loose = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=0.35),
            config=FAST,
        )
        tight = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=0.35, dmin_m=1.40),
            config=FAST,
        )
        self.assertTrue(loose.completed)
        self.assertTrue(tight.completed)
        self.assertLess(loose.makespan_s, tight.makespan_s)

    def test_corridor_ignores_scaffold_theta(self) -> None:
        a = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=0.35),
            config=FAST,
        )
        b = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=0.35, dmin_m=1.40),
            config=FAST,
        )
        self.assertTrue(a.completed)
        self.assertTrue(b.completed)
        self.assertGreater(a.corridor_time_s, 1.0)
        self.assertAlmostEqual(a.corridor_time_s, b.corridor_time_s, delta=40.0)

    def test_trace_records_spot_and_human_each_tick(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=0.8, dmin_m=0.80),
            config=FAST,
        )
        self.assertGreaterEqual(len(result.trace), 2)
        self.assertAlmostEqual(result.trace[0].t_s, FAST.dt_s, places=5)
        self.assertEqual(len(result.trace), len({s.t_s for s in result.trace}))

    def test_ssm_never_uses_a_reduce_band(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=0.35),
            config=FAST,
        )
        modes = {s.ssm_mode for s in result.trace}
        self.assertNotIn("reduce", modes)
        self.assertTrue(modes <= {"free", "stop"})

    def test_larger_dmin_waits_more_than_iso_only_keepout(self) -> None:
        loose = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=0.35),
            config=FAST,
        )
        tight = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=1.50),
            config=FAST,
        )
        self.assertTrue(loose.completed)
        self.assertTrue(tight.completed)
        self.assertGreater(tight.makespan_s, loose.makespan_s)

    def test_scaffold_clocks_and_corridor_partition_the_mission(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=0.35),
            config=FAST,
        )
        self.assertTrue(result.completed)
        self.assertGreater(result.corridor_time_s, 1.0)
        self.assertGreater(result.scaffold_time_s, 1.0)
        self.assertLess(result.scaffold_time_s, result.makespan_s - 1.0)
        self.assertAlmostEqual(
            result.scaffold_time_s + result.corridor_time_s,
            result.makespan_s,
            delta=FAST.dt_s,
        )
        breakdown = score(result)
        self.assertAlmostEqual(breakdown.tt, result.makespan_s, places=5)
        self.assertAlmostEqual(breakdown.mission_s, result.makespan_s)

    def test_arm_load_and_place_add_real_time(self) -> None:
        quick = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=0.35),
            config=FAST,
        )
        slow_arm = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=0.35),
            config=OracleConfig(
                dt_s=0.25,
                timeout_s=480.0,
                erect_s=0.25,
                truck_load_s=2.0,
                drop_place_s=2.0,
                assembler_pickup_s=0.25,
                sockets_per_floor=2,
            ),
        )
        self.assertTrue(quick.completed)
        self.assertTrue(slow_arm.completed)
        self.assertGreater(slow_arm.arm_load_s, quick.arm_load_s + 8.0)
        self.assertGreater(slow_arm.arm_place_s, quick.arm_place_s + 8.0)

    def test_stair_hops_respect_vmax(self) -> None:
        theta = Theta(vmax_mps=1.0, dmin_m=0.35)
        result = run_erection(geom=STAGE1_GEOM, theta=theta, config=FAST)
        self.assertTrue(result.completed)
        cap = theta.vmax_mps + 1e-6
        climbed = False
        prev = result.trace[0]
        for sample in result.trace[1:]:
            dz = sample.spot[2] - prev.spot[2]
            if dz > 1e-6 and not sample.blocked:
                climbed = True
                speed = _dist3(sample.spot, prev.spot) / FAST.dt_s
                self.assertLessEqual(speed, cap + 0.05)
            prev = sample
        self.assertTrue(climbed)

    def test_human_evacuates_to_refuge_then_spot_may_move(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=1.50),
            config=FAST,
        )
        self.assertTrue(result.completed)
        refuge_x = FAST.refuge_x_m
        self.assertTrue(any(abs(s.human[0] - refuge_x) < 0.6 for s in result.trace))
        for sample in result.trace:
            if sample.blocked:
                self.assertAlmostEqual(sample.spot_speed_mps, 0.0, places=5)

    def test_enforced_min_sep_rises_when_dmin_exceeds_iso(self) -> None:
        iso_only = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=0.35),
            config=FAST,
        )
        extra = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=1.50),
            config=FAST,
        )
        self.assertTrue(iso_only.completed)
        self.assertTrue(extra.completed)
        self.assertGreater(extra.min_separation_m, iso_only.min_separation_m + 0.20)

    def test_min_sep_uses_moving_ticks_only(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=1.0, dmin_m=1.50),
            config=FAST,
        )
        self.assertTrue(result.completed)
        moving = [
            sample.sep_m
            for sample in result.trace
            if (not sample.in_corridor) and sample.spot_speed_mps > 0.05
        ]
        self.assertGreater(len(moving), 0)
        self.assertAlmostEqual(result.min_separation_m, min(moving), places=5)


def _dist3(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


if __name__ == "__main__":
    unittest.main()

