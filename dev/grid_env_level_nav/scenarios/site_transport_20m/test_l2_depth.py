#!/usr/bin/env python3
"""Unit tests for L2_depth pipeline (offline synthetic depth replay)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from costmap_layers import LayeredCostmap, L2_LOG_ODDS_OCCUPIED  # noqa: E402
from l2_depth import DepthCellTracker, soft_l2_depth_reset, update_l2_depth  # noqa: E402
from perception_layer import (  # noqa: E402
    EgocentricPerceptionConfig,
    apply_depth_ray_update,
    bresenham_line,
    depth_hits_from_image,
    update_l2_from_depth_image,
)


def _tiny_layers() -> LayeredCostmap:
    costs = np.zeros((40, 40), dtype=np.float32)
    return LayeredCostmap(l0=costs, origin_xy=(-1000.0, -2200.0), resolution_cm=30.0)


class L2DepthTest(unittest.TestCase):
    def test_bresenham_line(self) -> None:
        cells = bresenham_line(0, 0, 3, 1)
        self.assertIn((0, 0), cells)
        self.assertIn((3, 1), cells)
        self.assertGreater(len(cells), 2)

    def test_depth_hits_from_synthetic_npy(self) -> None:
        layers = _tiny_layers()
        depth = np.full((64, 64), 2.0, dtype=np.float32)
        depth[30:40, 28:36] = 1.0
        cfg = EgocentricPerceptionConfig(
            min_obstacle_height_cm=10.0,
            stride_px=8,
            use_log_odds=True,
            latch_static=True,
        )
        hits = depth_hits_from_image(
            depth,
            layers,
            robot_xy=(-500.0, -1700.0),
            robot_yaw_deg=0.0,
            config=cfg,
        )
        self.assertGreater(len(hits), 0)

    def test_ray_clearing_writes_log_odds(self) -> None:
        layers = _tiny_layers()
        depth = np.full((64, 64), 2.0, dtype=np.float32)
        depth[32, 32] = 1.0
        cfg = EgocentricPerceptionConfig(
            min_obstacle_height_cm=10.0,
            stride_px=16,
            use_log_odds=True,
            latch_static=True,
        )
        hits = depth_hits_from_image(
            depth,
            layers,
            robot_xy=(-500.0, -1700.0),
            robot_yaw_deg=0.0,
            config=cfg,
        )
        hit_count, cleared = apply_depth_ray_update(
            layers,
            hits,
            robot_xy=(-500.0, -1700.0),
            config=cfg,
        )
        self.assertGreater(hit_count, 0)
        self.assertGreaterEqual(cleared, 0)
        if hits:
            gx, gy = hits[0].cell
            self.assertGreaterEqual(float(layers.l2_log_odds[gy, gx]), L2_LOG_ODDS_OCCUPIED)

    def test_carry_forward_mask_preserves_cells(self) -> None:
        layers = _tiny_layers()
        layers.set_l2_cell(10, 10, 1.0e9)
        layers.l2_static_latch[10, 10] = True
        tracker = DepthCellTracker()
        tracker.snapshot_occupied(layers)
        self.assertIn((10, 10), tracker.carry_forward_mask)

        depth = np.full((64, 64), 3.0, dtype=np.float32)
        cfg = EgocentricPerceptionConfig(
            min_obstacle_height_cm=10.0,
            stride_px=16,
            use_log_odds=True,
            latch_static=True,
        )
        update_l2_depth(
            depth,
            layers,
            robot_xy=(-500.0, -1700.0),
            robot_yaw_deg=0.0,
            config=cfg,
            tracker=tracker,
        )
        self.assertGreater(float(layers.l2[10, 10]), 0)

    def test_aggressive_soft_reset_evicts_carry_forward_near_stuck(self) -> None:
        layers = _tiny_layers()
        layers.update_l2_log_odds_cell(12, 12, 0.85, latch_static=True)
        layers.update_l2_log_odds_cell(12, 12, 0.85, latch_static=True)
        tracker = DepthCellTracker()
        tracker.carry_forward_mask = {(12, 12)}
        seen: set = {(12, 12)}
        stuck_xy = (
            layers.origin_xy[0] + (12 + 0.5) * layers.resolution_cm,
            layers.origin_xy[1] + (12 + 0.5) * layers.resolution_cm,
        )
        removed = soft_l2_depth_reset(
            layers,
            tracker,
            seen,
            stuck_world_xy=stuck_xy,
            aggressive=True,
        )
        self.assertGreater(removed, 0)
        self.assertEqual(float(layers.l2[12, 12]), 0.0)

    def test_aggressive_soft_reset_clears_latched_near_stuck(self) -> None:
        layers = _tiny_layers()
        layers.update_l2_log_odds_cell(12, 12, 0.85, latch_static=True)
        layers.update_l2_log_odds_cell(12, 12, 0.85, latch_static=True)
        self.assertTrue(layers.l2_static_latch[12, 12])
        tracker = DepthCellTracker()
        tracker.active_cells.add((12, 12))
        seen: set = {(12, 12)}
        stuck_xy = (
            layers.origin_xy[0] + (12 + 0.5) * layers.resolution_cm,
            layers.origin_xy[1] + (12 + 0.5) * layers.resolution_cm,
        )
        removed = soft_l2_depth_reset(
            layers,
            tracker,
            seen,
            stuck_world_xy=stuck_xy,
            aggressive=True,
        )
        self.assertGreater(removed, 0)
        self.assertEqual(float(layers.l2[12, 12]), 0.0)
        self.assertFalse(layers.l2_static_latch[12, 12])

    def test_keepout_skipped_when_near_fraction_high(self) -> None:
        layers = _tiny_layers()
        depth = np.full((64, 64), 2.0, dtype=np.float32)
        depth[int(64 * 0.82) :, :] = 0.25
        cfg = EgocentricPerceptionConfig(
            min_obstacle_height_cm=10.0,
            stride_px=16,
            use_log_odds=True,
            latch_static=True,
        )
        result = update_l2_depth(
            depth,
            layers,
            robot_xy=(-500.0, -1700.0),
            robot_yaw_deg=0.0,
            config=cfg,
            close_range_clearance_cm=125.0,
        )
        self.assertEqual(result.keepout_cells, 0)

    def test_keepout_applied_for_close_forward_wall(self) -> None:
        layers = _tiny_layers()
        depth = np.full((64, 64), 2.5, dtype=np.float32)
        depth[18:34, 26:38] = 0.75
        cfg = EgocentricPerceptionConfig(
            min_obstacle_height_cm=10.0,
            stride_px=8,
            use_log_odds=True,
            latch_static=True,
        )
        result = update_l2_depth(
            depth,
            layers,
            robot_xy=(-500.0, -1700.0),
            robot_yaw_deg=0.0,
            config=cfg,
            close_range_clearance_cm=100.0,
            close_range_keepout_cm=100.0,
        )
        self.assertGreater(result.keepout_cells, 0)

    def test_update_l2_from_depth_image_offline(self) -> None:
        layers = _tiny_layers()
        depth = np.full((64, 64), 2.0, dtype=np.float32)
        depth[28:36, 28:36] = 0.8
        n = update_l2_from_depth_image(
            depth,
            layers,
            robot_xy=(-500.0, -1700.0),
            robot_yaw_deg=0.0,
            config=EgocentricPerceptionConfig(
                min_obstacle_height_cm=10.0,
                stride_px=8,
                use_log_odds=True,
            ),
        )
        self.assertGreaterEqual(n, 0)


if __name__ == "__main__":
    unittest.main()
