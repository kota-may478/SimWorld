#!/usr/bin/env python3
"""Unit tests for Pareto projection Π (no UE)."""

from __future__ import annotations

import unittest

from constraints.pareto import (
    EvaluatedTheta,
    Theta,
    iso_pareto,
    nondominated,
    project,
    synthetic_front,
)


def _row(
    vmax: float,
    dmin: float,
    tt: float,
    t_ssm: float,
    si_min: float = 1.2,
    min_sep_m: float = 0.0,
) -> EvaluatedTheta:
    return EvaluatedTheta(
        Theta(vmax_mps=vmax, dmin_m=dmin),
        tt=tt,
        t_ssm=t_ssm,
        si_min=si_min,
        completed=True,
        min_sep_m=min_sep_m,
    )


class ParetoProjectionTest(unittest.TestCase):
    def test_alpha_one_picks_safest_end(self) -> None:
        front = synthetic_front()
        theta = project(Theta(vmax_mps=1.0, dmin_m=0.2), alpha=1.0, front=front)
        self.assertGreaterEqual(theta.dmin_m, front[-1].dmin_m - 1e-9)
        self.assertLessEqual(theta.vmax_mps, front[-1].vmax_mps + 1e-9)

    def test_alpha_zero_picks_efficient_end(self) -> None:
        front = synthetic_front()
        theta = project(Theta(vmax_mps=0.1, dmin_m=2.0), alpha=0.0, front=front)
        self.assertLessEqual(theta.dmin_m, front[0].dmin_m + 1e-9)
        self.assertGreaterEqual(theta.vmax_mps, front[0].vmax_mps - 1e-9)

    def test_hallucinated_theta_is_snapped_onto_front(self) -> None:
        front = synthetic_front()
        theta = project(Theta(vmax_mps=3.0, dmin_m=8.0), alpha=0.7, front=front)
        dmins = [p.dmin_m for p in front]
        vmaxs = [p.vmax_mps for p in front]
        self.assertGreaterEqual(theta.dmin_m, min(dmins) - 1e-9)
        self.assertLessEqual(theta.dmin_m, max(dmins) + 1e-9)
        self.assertGreaterEqual(theta.vmax_mps, min(vmaxs) - 1e-9)
        self.assertLessEqual(theta.vmax_mps, max(vmaxs) + 1e-9)
        self.assertLessEqual(theta.vmax_mps, 1.0)

    def test_empty_front_raises(self) -> None:
        with self.assertRaises(ValueError):
            project(Theta(1.0, 0.5), alpha=0.5, front=())

    def test_nondominated_keeps_tt_vmax_tradeoff(self) -> None:
        rows = (
            _row(1.0, 0.35, tt=50.0, t_ssm=1.0),
            _row(0.4, 1.40, tt=80.0, t_ssm=0.4),
            _row(0.9, 0.50, tt=90.0, t_ssm=0.9),
        )
        front = nondominated(rows)
        thetas = {(p.theta.vmax_mps, p.theta.dmin_m) for p in front}
        self.assertIn((1.0, 0.35), thetas)
        self.assertIn((0.4, 1.40), thetas)
        self.assertNotIn((0.9, 0.50), thetas)

    def test_nondominated_keeps_better_min_sep_at_same_vmax(self) -> None:
        rows = (
            _row(1.0, 0.35, tt=50.0, t_ssm=1.0, min_sep_m=0.40),
            _row(1.0, 1.40, tt=55.0, t_ssm=1.0, min_sep_m=1.40),
            _row(0.4, 1.40, tt=80.0, t_ssm=0.4, min_sep_m=1.50),
            _row(0.9, 0.50, tt=90.0, t_ssm=0.9, min_sep_m=0.40),
        )
        front = nondominated(rows)
        thetas = {(p.theta.vmax_mps, p.theta.dmin_m) for p in front}
        self.assertIn((1.0, 0.35), thetas)
        self.assertIn((1.0, 1.40), thetas)
        self.assertIn((0.4, 1.40), thetas)
        self.assertNotIn((0.9, 0.50), thetas)

    def test_iso_pareto_drops_si_violators(self) -> None:
        rows = (
            _row(1.0, 0.35, tt=0.50, t_ssm=0.20, si_min=0.4),
            _row(0.4, 1.40, tt=0.80, t_ssm=0.05, si_min=1.2),
        )
        front = iso_pareto(rows)
        self.assertEqual(len(front), 1)
        self.assertAlmostEqual(front[0].theta.dmin_m, 1.40)


if __name__ == "__main__":
    unittest.main()
