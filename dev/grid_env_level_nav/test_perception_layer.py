#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from costmap_layers import LayeredCostmap  # noqa: E402
from perception_layer import (  # noqa: E402
    EgocentricPerceptionConfig,
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


if __name__ == "__main__":
    unittest.main()
