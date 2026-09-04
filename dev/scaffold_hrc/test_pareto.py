#!/usr/bin/env python3
"""Unit tests for Pareto projection Π (no UE)."""

from __future__ import annotations

import unittest

from constraints.pareto import EvaluatedTheta, Theta, nondominated, project, synthetic_front


class ParetoProjectionTest(unittest.TestCase):
    def test_alpha_one_picks_safest_end(self) -> None:
        front = synthetic_front()
        theta = project(Theta(dmin_m=0.2, vmax_mps=1.0), alpha=1.0, front=front)
        self.assertGreaterEqual(theta.dmin_m, front[-1].dmin_m - 1e-9)
        self.assertLessEqual(theta.vmax_mps, front[-1].vmax_mps + 1e-9)

    def test_alpha_zero_picks_efficient_end(self) -> None:
        front = synthetic_front()
        theta = project(Theta(dmin_m=2.0, vmax_mps=0.1), alpha=0.0, front=front)
        self.assertLessEqual(theta.dmin_m, front[0].dmin_m + 1e-9)
        self.assertGreaterEqual(theta.vmax_mps, front[0].vmax_mps - 1e-9)

    def test_hallucinated_dmin_is_snapped_onto_front(self) -> None:
        front = synthetic_front()
        theta = project(Theta(dmin_m=8.0, vmax_mps=3.0), alpha=0.7, front=front)
        dmins = [p.dmin_m for p in front]
        vmaxs = [p.vmax_mps for p in front]
        self.assertGreaterEqual(theta.dmin_m, min(dmins) - 1e-9)
        self.assertLessEqual(theta.dmin_m, max(dmins) + 1e-9)
        self.assertGreaterEqual(theta.vmax_mps, min(vmaxs) - 1e-9)
        self.assertLessEqual(theta.vmax_mps, max(vmaxs) + 1e-9)
        self.assertLessEqual(theta.vmax_mps, 1.0)

    def test_empty_front_raises(self) -> None:
        with self.assertRaises(ValueError):
            project(Theta(0.5, 1.0), alpha=0.5, front=())

    def test_nondominated_keeps_jeff_and_jsafe_tradeoff(self) -> None:
        rows = (
            EvaluatedTheta(Theta(0.4, 1.0), jeff=0.9, jsafe=0.2, completed=True),
            EvaluatedTheta(Theta(1.0, 0.4), jeff=0.7, jsafe=0.0, completed=True),
            EvaluatedTheta(Theta(0.5, 0.9), jeff=0.6, jsafe=0.2, completed=True),
        )
        front = nondominated(rows)
        thetas = {(p.theta.dmin_m, p.theta.vmax_mps) for p in front}
        self.assertIn((0.4, 1.0), thetas)
        self.assertIn((1.0, 0.4), thetas)
        self.assertNotIn((0.5, 0.9), thetas)


if __name__ == "__main__":
    unittest.main()
