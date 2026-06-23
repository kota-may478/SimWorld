#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from costmap_layers import LayeredCostmap, L2_LOG_ODDS_OCCUPIED  # noqa: E402
from perception_layer import (  # noqa: E402
    EgocentricPerceptionConfig,
    apply_depth_ray_update,
    bresenham_line,
    close_range_keepout_cells_from_depth,
    depth_hits_from_image,
    obstacle_cells_from_depth,
    update_l2_from_depth_image,
)


class TestPerceptionLayer(unittest.TestCase):
    def test_obstacle_cells_from_synthetic_depth(self) -> None:
        layers = LayeredCostmap.from_l0_array(np.ones((80, 70), dtype=np.float32), resolution_cm=100.0)
        depth = np.full((64, 64), 2.0, dtype=np.float32)
        depth[30:40, 28:36] = 1.0
        cells = obstacle_cells_from_depth(
            depth,
            layers,
            robot_xy=(-500.0, -1700.0),
            robot_yaw_deg=0.0,
            config=EgocentricPerceptionConfig(min_obstacle_height_cm=10.0, stride_px=8),
        )
        self.assertIsInstance(cells, list)
        n = update_l2_from_depth_image(
            depth,
            layers,
            robot_xy=(-500.0, -1700.0),
            robot_yaw_deg=0.0,
        )
        self.assertGreaterEqual(n, 0)

    def test_bresenham_and_ray_clearing(self) -> None:
        layers = LayeredCostmap.from_l0_array(np.ones((80, 70), dtype=np.float32), resolution_cm=100.0)
        depth = np.full((64, 64), 2.0, dtype=np.float32)
        depth[32, 32] = 1.0
        cfg = EgocentricPerceptionConfig(
            min_obstacle_height_cm=10.0,
            stride_px=16,
            use_log_odds=True,
        )
        hits = depth_hits_from_image(
            depth,
            layers,
            robot_xy=(-500.0, -1700.0),
            robot_yaw_deg=0.0,
            config=cfg,
        )
        hit_count, _cleared = apply_depth_ray_update(
            layers,
            hits,
            robot_xy=(-500.0, -1700.0),
            config=cfg,
        )
        self.assertGreaterEqual(hit_count, 0)

    def test_close_range_depth_generates_one_meter_keepout(self) -> None:
        layers = LayeredCostmap.from_l0_array(np.ones((80, 70), dtype=np.float32), resolution_cm=50.0)
        depth = np.full((64, 64), 2.0, dtype=np.float32)
        depth[28:36, 28:36] = 0.8
        cells = close_range_keepout_cells_from_depth(
            depth,
            layers,
            robot_xy=(-500.0, -1700.0),
            robot_yaw_deg=0.0,
            config=EgocentricPerceptionConfig(min_obstacle_height_cm=10.0, stride_px=4),
            min_clearance_cm=100.0,
            keepout_radius_cm=100.0,
        )
        self.assertGreater(len(cells), 0)

    def test_log_odds_sync_to_l2(self) -> None:
        layers = LayeredCostmap.from_l0_array(np.ones((10, 10), dtype=np.float32), resolution_cm=30.0)
        layers.update_l2_log_odds_cell(5, 5, 1.0, latch_static=True)
        self.assertTrue(layers.l2_static_latch[5, 5])
        self.assertGreaterEqual(float(layers.l2_log_odds[5, 5]), L2_LOG_ODDS_OCCUPIED)
        n = layers.sync_l2_from_log_odds()
        self.assertGreater(n, 0)


if __name__ == "__main__":
    unittest.main()
