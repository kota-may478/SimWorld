#!/usr/bin/env python3
"""Unit tests for velocity_scaler."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths()

from nav_stack.controllers.velocity_scaler import (  # noqa: E402
    VelocityScaleConfig,
    dynamic_max_move_cm,
    velocity_scale_factor,
)


class VelocityScalerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = VelocityScaleConfig(
            max_move_cm=120.0,
            near_obstacle_slow_cm=220.0,
            perception_standoff_cm=50.0,
        )

    def test_far_obstacle_full_speed(self) -> None:
        self.assertEqual(dynamic_max_move_cm(300.0, None, config=self.cfg), 120.0)
        self.assertAlmostEqual(velocity_scale_factor(300.0, None, config=self.cfg), 1.0)

    def test_standoff_band(self) -> None:
        self.assertEqual(dynamic_max_move_cm(60.0, None, config=self.cfg), 70.0)
        self.assertAlmostEqual(velocity_scale_factor(60.0, None, config=self.cfg), 0.5)

    def test_close_band(self) -> None:
        self.assertEqual(dynamic_max_move_cm(30.0, None, config=self.cfg), 35.0)
        self.assertAlmostEqual(velocity_scale_factor(30.0, None, config=self.cfg), 0.25)

    def test_forward_depth_limits_when_closer(self) -> None:
        move = dynamic_max_move_cm(300.0, 40.0, config=self.cfg)
        self.assertEqual(move, 35.0)


if __name__ == "__main__":
    unittest.main()
