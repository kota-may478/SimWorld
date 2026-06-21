#!/usr/bin/env python3
"""Unit tests for geometric L2 FOV perception (no UE)."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

import level_coords as lc  # noqa: E402
from grid_env_10k_pie_patrol import yaw_to_target  # noqa: E402
from l2_geom import GeomPerceptionConfig, geom_detections, visible_props_from_geom  # noqa: E402
from placement import build_registry, to_placement_registry  # noqa: E402
from region import ROBOT_START_LOCAL_CM  # noqa: E402


def _robot_yaw_toward_local(target_local: tuple[float, float]) -> float:
    start_world = lc.local_xy_to_world(*ROBOT_START_LOCAL_CM)
    target_world = lc.local_xy_to_world(*target_local)
    return yaw_to_target(start_world, target_world)


class GeomPerceptionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.registry = to_placement_registry(build_registry())
        self.robot_xy = lc.local_xy_to_world(*ROBOT_START_LOCAL_CM)
        self.cfg = GeomPerceptionConfig(fov_deg=90.0, max_range_cm=650.0)

    def test_in_fov_center(self) -> None:
        yaw = _robot_yaw_toward_local((400.0, 450.0))
        visible = visible_props_from_geom(
            self.robot_xy, yaw, self.registry, config=self.cfg
        )
        types = {item[0].prop_type_id for item in visible}
        self.assertIn("dumpster", types)

    def test_outside_fov(self) -> None:
        yaw = _robot_yaw_toward_local((400.0, 450.0))
        side_yaw = yaw + 90.0
        visible = visible_props_from_geom(
            self.robot_xy, side_yaw, self.registry, config=self.cfg
        )
        types = {item[0].prop_type_id for item in visible}
        self.assertNotIn("dumpster", types)

    def test_beyond_max_range(self) -> None:
        yaw = _robot_yaw_toward_local((1850.0, 1850.0))
        short_cfg = GeomPerceptionConfig(fov_deg=90.0, max_range_cm=200.0)
        visible = visible_props_from_geom(
            self.robot_xy, yaw, self.registry, config=short_cfg
        )
        types = {item[0].prop_type_id for item in visible}
        self.assertNotIn("shipping_crate", types)

    def test_start_near_sw_prop(self) -> None:
        yaw = _robot_yaw_toward_local((400.0, 450.0))
        detections = geom_detections(self.robot_xy, yaw, self.registry, config=self.cfg)
        dumpster = [d for d in detections if d.prop_type_id == "dumpster"]
        self.assertEqual(len(dumpster), 1)
        self.assertLess(dumpster[0].distance_m, 5.0)

    def test_geom_detections_shape(self) -> None:
        yaw = _robot_yaw_toward_local((900.0, 700.0))
        detections = geom_detections(self.robot_xy, yaw, self.registry, config=self.cfg)
        self.assertGreater(len(detections), 0)
        for det in detections:
            self.assertTrue(det.prop_type_id)
            self.assertGreater(det.distance_m, 0.0)
            self.assertLessEqual(abs(det.bearing_deg), self.cfg.fov_deg * 0.5 + 1.0)
            self.assertGreaterEqual(det.confidence, self.cfg.min_confidence)

    def test_roadblocks_excluded(self) -> None:
        yaw = _robot_yaw_toward_local((1450.0, 1090.0))
        detections = geom_detections(self.robot_xy, yaw, self.registry, config=self.cfg)
        roadblocks = [
            d for d in detections if d.prop_type_id.startswith("roadblock")
        ]
        self.assertEqual(roadblocks, [])


if __name__ == "__main__":
    unittest.main()
