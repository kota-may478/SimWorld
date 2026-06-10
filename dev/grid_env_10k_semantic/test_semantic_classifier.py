"""Unit tests for semantic classification (no UE required)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from block_semantic_scan import ObstacleBox, classify_cell_at_heights, classify_semantic  # noqa: E402
import grid_env_10k_semantic as sem  # noqa: E402


class TestClassifySemantic(unittest.TestCase):
    def test_wall_priority(self) -> None:
        self.assertEqual(
            classify_semantic(hit_at_z_initial=True, hit_at_z_lower=True),
            "wall",
        )
        self.assertEqual(
            classify_semantic(hit_at_z_initial=True, hit_at_z_lower=False),
            "wall",
        )

    def test_floor(self) -> None:
        self.assertEqual(
            classify_semantic(hit_at_z_initial=False, hit_at_z_lower=True),
            "floor",
        )

    def test_air(self) -> None:
        self.assertEqual(
            classify_semantic(hit_at_z_initial=False, hit_at_z_lower=False),
            "air",
        )


class TestGeometricScan(unittest.TestCase):
    def test_floor_and_air_on_demo_layout(self) -> None:
        geom = sem.compute_layer_geometry(floor_top_z_cm=100.0)
        obstacles = [sem.build_temp_floor_obstacle(geom)]
        x_air, y_air = sem.cell_center_world_xy_cm(5, 5)
        x_floor, y_floor = sem.cell_center_world_xy_cm(2, 2)
        self.assertEqual(
            classify_cell_at_heights(
                x_air,
                y_air,
                z_initial_bottom_cm=geom.block_bottom_z_cm,
                block_height_cm=30.0,
                obstacles=obstacles,
            ),
            "air",
        )
        self.assertEqual(
            classify_cell_at_heights(
                x_floor,
                y_floor,
                z_initial_bottom_cm=geom.block_bottom_z_cm,
                block_height_cm=30.0,
                obstacles=obstacles,
            ),
            "floor",
        )

    def test_temp_floor_does_not_wall_at_initial_height(self) -> None:
        geom = sem.compute_layer_geometry(floor_top_z_cm=100.0)
        obstacles = [sem.build_temp_floor_obstacle(geom)]
        x, y = sem.cell_center_world_xy_cm(2, 2)
        sem_at_initial = classify_cell_at_heights(
            x,
            y,
            z_initial_bottom_cm=geom.block_bottom_z_cm,
            block_height_cm=30.0,
            obstacles=obstacles,
        )
        self.assertNotEqual(sem_at_initial, "wall")


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
        self.assertTrue(sem.cell_on_temp_floor(3, 3))
        self.assertFalse(sem.cell_on_temp_floor(4, 1))
        self.assertFalse(sem.cell_on_temp_floor(5, 5))

    def test_fill_region_size(self) -> None:
        cells = list(sem.iter_rectangle_indices(1, 1, 5, 5))
        self.assertEqual(len(cells), 25)

    def test_visual_modes(self) -> None:
        self.assertEqual(sem.mode_for_semantic("floor"), "F")
        self.assertEqual(sem.mode_for_semantic("air"), "T")
        self.assertEqual(sem.mode_for_semantic("wall"), "T")


if __name__ == "__main__":
    unittest.main()
