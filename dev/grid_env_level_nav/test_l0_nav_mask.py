#!/usr/bin/env python3
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from l0_nav_mask import (  # noqa: E402
    build_l0_mask_from_project_fn,
    load_l0_mask_npz,
    save_l0_mask_npz,
    upsample_strided_l0,
)


class TestL0NavMask(unittest.TestCase):
    def test_xy_tolerance_rejects_far_snap(self) -> None:
        from l0_nav_mask import project_cell_to_cost

        result = {"ok": True, "x": -400.0, "y": -2100.0, "z": 6450.0}
        cost = project_cell_to_cost(result, wx=-685.0, wy=-1585.0, wz=6490.0, xy_tolerance_cm=4.5)
        self.assertGreaterEqual(cost, 1e8)

    def test_xy_tolerance_accepts_near_snap(self) -> None:
        from l0_nav_mask import project_cell_to_cost

        result = {"ok": True, "x": -684.0, "y": -1585.5, "z": 6450.0}
        cost = project_cell_to_cost(result, wx=-685.0, wy=-1585.0, wz=6490.0, xy_tolerance_cm=4.5)
        self.assertLess(cost, 10.0)

    def test_project_fn_mock(self) -> None:
        def project(wx, wy, wz):
            if wx <= -500:
                return {"ok": False}
            return {"ok": True, "x": wx, "y": wy, "z": 6450.0}

        costs = build_l0_mask_from_project_fn(
            project,
            resolution_cm=100.0,
            stride=2,
        )
        self.assertEqual(costs.ndim, 2)
        self.assertGreater(costs.shape[0], 0)

    def test_npz_roundtrip(self) -> None:
        arr = np.ones((5, 6), dtype=np.float32)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "l0.npz"
            save_l0_mask_npz(path, arr, resolution_cm=30.0)
            loaded, res, origin, lethal = load_l0_mask_npz(path)
        self.assertEqual(loaded.shape, (5, 6))
        self.assertEqual(res, 30.0)

    def test_upsample_stride(self) -> None:
        base = np.arange(16, dtype=np.float32).reshape(4, 4)
        up = upsample_strided_l0(base, stride=2)
        self.assertEqual(up.shape, base.shape)


if __name__ == "__main__":
    unittest.main()
