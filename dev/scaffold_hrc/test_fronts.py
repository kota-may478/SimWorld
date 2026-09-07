#!/usr/bin/env python3
"""Unit tests for front-discovery methods (no UE)."""

from __future__ import annotations

import unittest

from constraints.pareto import EvaluatedTheta, Theta, nondominated
from fronts.evaluate import OracleEvaluator, measure_t_ref, opt_config
from fronts.grid_sweep import run_grid
from fronts.lhs_sample import run_lhs
from fronts.nsga2 import run_nsga2
from fronts.safe_bo import run_safe_bo
from fronts.space import REF_THETA, ThetaBox
from oracle.simulate import OracleConfig

FAST = opt_config(
    OracleConfig(
        dt_s=0.25,
        timeout_s=480.0,
        erect_s=0.25,
        truck_load_s=0.25,
        drop_place_s=0.25,
        sockets_per_floor=2,
        record_trace=False,
    )
)


class FrontMethodsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.t_ref = measure_t_ref(FAST, REF_THETA)
        cls.evaluator = OracleEvaluator(config=FAST, t_ref_s=cls.t_ref)

    def test_grid_covers_box_corners(self) -> None:
        rows = run_grid(self.evaluator, n_vmax=3, n_dmin=3)
        self.assertEqual(len(rows), 9)
        vmaxs = {round(r.theta.vmax_mps, 5) for r in rows}
        dmins = {round(r.theta.dmin_m, 5) for r in rows}
        box = ThetaBox()
        self.assertIn(round(box.vmax_lo, 5), vmaxs)
        self.assertIn(round(box.vmax_hi, 5), vmaxs)
        self.assertIn(round(box.dmin_lo, 5), dmins)
        self.assertIn(round(box.dmin_hi, 5), dmins)

    def test_lhs_and_nsga_return_evaluated_rows(self) -> None:
        lhs = run_lhs(self.evaluator, n_samples=4, seed=1)
        nsga = run_nsga2(self.evaluator, pop_size=4, n_gen=1, seed=2)
        self.assertEqual(len(lhs), 4)
        self.assertGreaterEqual(len(nsga), 4)
        nd = nondominated(tuple(lhs + nsga))
        self.assertGreaterEqual(len(nd), 1)

    def test_lhs_fills_the_theta_box(self) -> None:
        lhs = run_lhs(self.evaluator, n_samples=12, seed=1)
        box = ThetaBox()
        vmaxs = [r.theta.vmax_mps for r in lhs]
        dmins = [r.theta.dmin_m for r in lhs]
        self.assertLess(min(vmaxs), box.vmax_lo + 0.12)
        self.assertGreater(max(vmaxs), box.vmax_hi - 0.12)
        self.assertLess(min(dmins), box.dmin_lo + 0.15)
        self.assertGreater(max(dmins), box.dmin_hi - 0.15)

    def test_safe_bo_starts_from_a_conservative_seed(self) -> None:
        rows = run_safe_bo(
            self.evaluator,
            n_iter=3,
            n_vmax=4,
            n_dmin=4,
            d_lim=0.55,
            densify=False,
        )
        self.assertGreaterEqual(len(rows), 1)
        self.assertGreaterEqual(rows[0].theta.dmin_m, 1.2)
        self.assertLessEqual(rows[0].theta.vmax_mps, 0.50)


@unittest.skipUnless(__import__("importlib").util.find_spec("matplotlib"), "matplotlib required")
class FrontVizTest(unittest.TestCase):
    def test_writes_per_method_pngs(self) -> None:
        from pathlib import Path
        from tempfile import TemporaryDirectory

        from fronts.viz_fronts import write_method_plots

        rows = (
            EvaluatedTheta(
                Theta(vmax_mps=1.0, dmin_m=0.35),
                tt=0.8,
                t_ssm=0.1,
                si_min=1.2,
                completed=True,
                min_sep_m=0.45,
            ),
            EvaluatedTheta(
                Theta(vmax_mps=0.4, dmin_m=1.40),
                tt=1.1,
                t_ssm=0.0,
                si_min=1.4,
                completed=True,
                min_sep_m=1.35,
            ),
        )
        with TemporaryDirectory() as tmp:
            paths = write_method_plots(Path(tmp) / "grid", "grid", rows)
            for path in paths:
                self.assertTrue(path.is_file(), msg=str(path))
                self.assertGreater(path.stat().st_size, 100)


if __name__ == "__main__":
    unittest.main()
