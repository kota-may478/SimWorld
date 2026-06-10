#!/usr/bin/env python3
"""Unit tests for level_region (no UE)."""

from __future__ import annotations

import unittest

from level_region import (
    BLOCK_BOTTOM_Z_CM,
    CORNER_A_XY_CM,
    CORNER_B_XY_CM,
    CELL_SIZE_CM,
    HEIGHT_STEP_CM,
    MAX_HEIGHT_TRIES,
    MAX_HEIGHT_TRIES_ALL_AIR,
    OUTWARD_MARGIN_CM,
    all_air_height_scan_action,
    default_level_region,
    height_scan_attempt_limit,
    initial_block_bottom_z_cm,
    initial_bottom_on_wall_detected,
    semantic_count_adjustment,
    subgrid_around_cell,
    world_xy_to_cell_index,
)


class TestLevelRegion(unittest.TestCase):
    def test_default_corners_and_margin(self) -> None:
        region = default_level_region()
        self.assertEqual(region.corner_a_xy_cm, CORNER_A_XY_CM)
        self.assertEqual(region.corner_b_xy_cm, CORNER_B_XY_CM)
        self.assertAlmostEqual(region.block_bottom_z_cm, BLOCK_BOTTOM_Z_CM)
        self.assertAlmostEqual(region.core_x_min_cm, CORNER_A_XY_CM[0])
        self.assertAlmostEqual(region.core_x_max_cm, CORNER_B_XY_CM[0])
        self.assertAlmostEqual(
            region.expanded_x_min_cm,
            CORNER_A_XY_CM[0] - OUTWARD_MARGIN_CM,
        )
        self.assertAlmostEqual(
            region.expanded_x_max_cm,
            CORNER_B_XY_CM[0] + OUTWARD_MARGIN_CM,
        )

    def test_grid_cell_count_positive(self) -> None:
        region = default_level_region()
        self.assertGreater(region.grid_nx, 0)
        self.assertGreater(region.grid_ny, 0)
        self.assertEqual(region.cell_count, region.grid_nx * region.grid_ny)
        cells = list(region.iter_indices())
        self.assertEqual(len(cells), region.cell_count)
        self.assertEqual(cells[0], (1, 1))
        self.assertEqual(cells[-1], (region.grid_nx, region.grid_ny))

    def test_cell_centers_on_grid(self) -> None:
        region = default_level_region()
        x1, y1 = region.cell_center_xy_cm(1, 1)
        x2, y2 = region.cell_center_xy_cm(2, 1)
        self.assertAlmostEqual(x2 - x1, -30.0)
        ox, oy = region.grid_origin_xy_cm
        self.assertAlmostEqual(
            x1,
            ox + (region.grid_nx - 1) * CELL_SIZE_CM + CELL_SIZE_CM / 2.0,
        )
        self.assertAlmostEqual(
            y1,
            oy + (region.grid_ny - 1) * CELL_SIZE_CM + CELL_SIZE_CM / 2.0,
        )

    def test_world_xy_cell_roundtrip(self) -> None:
        region = default_level_region()
        gx, gy = world_xy_to_cell_index(region, 6300.0, 1170.0)
        x, y = region.cell_center_xy_cm(gx, gy)
        gx2, gy2 = world_xy_to_cell_index(region, x, y)
        self.assertEqual((gx2, gy2), (gx, gy))

    def test_all_air_height_scan_action(self) -> None:
        n = 25
        self.assertEqual(
            all_air_height_scan_action({"wall": 0, "air": 25, "floor": 0}, n),
            "lower",
        )
        self.assertEqual(
            all_air_height_scan_action({"wall": 3, "air": 22, "floor": 0}, n),
            "lock_wall",
        )
        self.assertEqual(
            all_air_height_scan_action({"wall": 25, "air": 0, "floor": 0}, n),
            "lock_wall",
        )
        self.assertEqual(
            all_air_height_scan_action({"wall": 0, "air": 10, "floor": 15}, n),
            "lower",
        )
        self.assertEqual(
            all_air_height_scan_action({"wall": 0, "air": 0, "floor": 0}, n),
            "stop",
        )

    def test_initial_bottom_on_wall_detected(self) -> None:
        self.assertAlmostEqual(
            initial_bottom_on_wall_detected(6400.0),
            6400.0 + HEIGHT_STEP_CM,
        )

    def test_semantic_count_adjustment(self) -> None:
        self.assertIsNone(semantic_count_adjustment({"wall": 1, "air": 1, "floor": 2}))
        self.assertEqual(semantic_count_adjustment({"wall": 5, "air": 2, "floor": 0}), "raise")
        self.assertEqual(semantic_count_adjustment({"wall": 2, "air": 5, "floor": 0}), "lower")
        self.assertIsNone(semantic_count_adjustment({"wall": 3, "air": 3, "floor": 0}))
        self.assertEqual(semantic_count_adjustment({"wall": 25, "air": 0, "floor": 0}), "raise")
        self.assertEqual(semantic_count_adjustment({"wall": 0, "air": 25, "floor": 0}), "lower")
        self.assertIsNone(semantic_count_adjustment({"wall": 0, "air": 0, "floor": 0}))

    def test_initial_block_bottom_z_one_step_to_floor(self) -> None:
        surface = 6328.5
        z0 = initial_block_bottom_z_cm(surface)
        self.assertAlmostEqual(z0, surface + CELL_SIZE_CM + HEIGHT_STEP_CM)

    def test_world_xy_to_cell_and_subgrid(self) -> None:
        region = default_level_region()
        gx, gy = world_xy_to_cell_index(region, 6300.0, 1170.0)
        self.assertGreaterEqual(gx, 1)
        self.assertGreaterEqual(gy, 1)
        self.assertLessEqual(gx, region.grid_nx)
        self.assertLessEqual(gy, region.grid_ny)
        sg = subgrid_around_cell(gx, gy, half=2)
        self.assertEqual(sg[2] - sg[0] + 1, 5)
        self.assertEqual(sg[3] - sg[1] + 1, 5)

    def test_height_scan_attempt_limit(self) -> None:
        self.assertEqual(
            height_scan_attempt_limit(
                {"wall": 0, "air": 25, "floor": 0}, 25, nadir_surface_hits=5,
            ),
            MAX_HEIGHT_TRIES_ALL_AIR,
        )
        self.assertEqual(
            height_scan_attempt_limit(
                {"wall": 0, "air": 25, "floor": 0}, 25, nadir_surface_hits=0,
            ),
            1,
        )
        self.assertEqual(
            height_scan_attempt_limit({"wall": 5, "air": 20, "floor": 0}, 25),
            MAX_HEIGHT_TRIES,
        )
        self.assertEqual(
            height_scan_attempt_limit({"wall": 0, "air": 10, "floor": 15}, 25),
            MAX_HEIGHT_TRIES,
        )


if __name__ == "__main__":
    unittest.main()
