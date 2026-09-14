#!/usr/bin/env python3
"""Unit tests for oracle plots (no UE)."""

from __future__ import annotations

import os
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

os.environ["MPLBACKEND"] = "Agg"

try:
    import matplotlib

    matplotlib.use("Agg")
except ImportError:
    matplotlib = None

from constraints.pareto import EvaluatedTheta, Theta, synthetic_front
from oracle.simulate import OracleConfig, run_erection
from scene.geometry import STAGE1_GEOM

if matplotlib is not None:
    from viz import write_pareto_plots, write_trajectory_plots


@unittest.skipUnless(matplotlib, "matplotlib required (conda env simworld)")
class VizTest(unittest.TestCase):
    def test_writes_pngs(self) -> None:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=Theta(vmax_mps=0.8, dmin_m=0.80),
            config=OracleConfig(
                dt_s=0.25,
                timeout_s=240.0,
                erect_s=0.25,
                truck_load_s=0.25,
                drop_place_s=0.25,
                assembler_pickup_s=0.25,
                sockets_per_floor=2,
            ),
        )
        front = synthetic_front()
        rows = tuple(
            EvaluatedTheta(t, tt=0.8, t_ssm=0.0, si_min=1.2, completed=True) for t in front
        )
        with TemporaryDirectory() as tmp:
            out = Path(tmp)
            pareto = write_pareto_plots(out, rows=rows, front=front, chosen=front[4])
            traj = write_trajectory_plots(out, geom=STAGE1_GEOM, result=result)
            for path in (*pareto, *traj):
                self.assertTrue(path.is_file(), msg=str(path))
                self.assertGreater(path.stat().st_size, 100)


if __name__ == "__main__":
    unittest.main()
