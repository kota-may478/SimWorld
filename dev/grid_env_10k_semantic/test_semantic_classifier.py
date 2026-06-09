"""Unit tests for semantic classification (no UE required)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from block_semantic_scan import classify_semantic  # noqa: E402
import grid_env_10k_semantic as sem  # noqa: E402


class TestClassifySemantic(unittest.TestCase):
    def test_wall_priority(self) -> None:
        self.assertEqual(
            classify_semantic(hit_at_z0=True, hit_at_z_low=True),
            "wall",
        )
        self.assertEqual(
            classify_semantic(hit_at_z0=True, hit_at_z_low=False),
            "wall",
        )

    def test_floor(self) -> None:
        self.assertEqual(
            classify_semantic(hit_at_z0=False, hit_at_z_low=True),
            "floor",
        )

    def test_air(self) -> None:
        self.assertEqual(
            classify_semantic(hit_at_z0=False, hit_at_z_low=False),
            "air",
        )


class TestLayerGeometry(unittest.TestCase):
    def test_auto_elevation(self) -> None:
        geom = sem.compute_layer_geometry(floor_top_z_cm=100.0)
        self.assertAlmostEqual(geom.existing_block_top_z_cm, 130.5, places=1)
        self.assertAlmostEqual(geom.temp_floor_top_z_cm, 330.5, places=1)
        self.assertAlmostEqual(geom.block_bottom_z_cm, 345.5, places=1)

    def test_block_bottom_pivot_bottom(self) -> None:
        z = sem.block_bottom_to_actor_z(345.0)
        self.assertAlmostEqual(z, 345.0, places=3)

    def test_cell_on_temp_floor(self) -> None:
        self.assertTrue(sem.cell_on_temp_floor(1, 1))
        self.assertTrue(sem.cell_on_temp_floor(6, 6))
        self.assertFalse(sem.cell_on_temp_floor(7, 1))
        self.assertFalse(sem.cell_on_temp_floor(10, 10))

    def test_fill_region_size(self) -> None:
        cells = list(sem.iter_rectangle_indices(1, 1, 10, 10))
        self.assertEqual(len(cells), 100)


if __name__ == "__main__":
    unittest.main()
