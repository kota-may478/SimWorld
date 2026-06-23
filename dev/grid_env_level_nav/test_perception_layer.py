#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from costmap_layers import LayeredCostmap  # noqa: E402
from perception_layer import (  # noqa: E402
    EgocentricPerceptionConfig,
    close_range_keepout_cells_from_depth,
    obstacle_cells_from_depth,
    obstacle_cells_from_depth_gated_by_detections,
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

    def test_depth_cells_are_gated_by_ai_detection_sector(self) -> None:
        layers = LayeredCostmap.from_l0_array(np.ones((80, 70), dtype=np.float32), resolution_cm=100.0)
        depth = np.full((64, 64), 2.0, dtype=np.float32)
        cfg = EgocentricPerceptionConfig(min_obstacle_height_cm=10.0, stride_px=8)
        robot_xy = (-500.0, -1700.0)

        hit = SimpleNamespace(bearing_deg=0.0, distance_m=2.0)
        cells = obstacle_cells_from_depth_gated_by_detections(
            depth,
            layers,
            robot_xy=robot_xy,
            robot_yaw_deg=0.0,
            detections=[hit],
            config=cfg,
            bearing_margin_deg=8.0,
        )
        self.assertGreater(len(cells), 0)

        miss = SimpleNamespace(bearing_deg=45.0, distance_m=2.0)
        off_axis_cells = obstacle_cells_from_depth_gated_by_detections(
            depth,
            layers,
            robot_xy=robot_xy,
            robot_yaw_deg=0.0,
            detections=[miss],
            config=cfg,
            bearing_margin_deg=4.0,
        )
        self.assertEqual(off_axis_cells, [])

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


if __name__ == "__main__":
    unittest.main()
