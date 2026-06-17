#!/usr/bin/env python3
"""Unit tests for compact 30m nav placement (no UE)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="compact_nav")

from placement import PROP_COUNT, build_compact_nav_registry, to_placement_registry  # noqa: E402
from region import GOAL_LOCAL_CM, REGION_SIZE_CM, ROBOT_START_LOCAL_CM  # noqa: E402


class CompactNavPlacementTest(unittest.TestCase):
    def test_registry_geometry(self) -> None:
        reg = build_compact_nav_registry()
        self.assertEqual(reg.robot_start_local_cm, ROBOT_START_LOCAL_CM)
        self.assertEqual(reg.goal_local_cm, GOAL_LOCAL_CM)
        self.assertEqual(reg.region_size_cm, REGION_SIZE_CM)
        self.assertEqual(len(reg.props), PROP_COUNT)

    def test_props_unique_types(self) -> None:
        reg = build_compact_nav_registry()
        bp_names = [p.bp_name for p in reg.props]
        self.assertEqual(len(bp_names), len(set(bp_names)))

    def test_placement_adapter(self) -> None:
        reg = build_compact_nav_registry()
        placement = to_placement_registry(reg)
        self.assertEqual(len(placement.props), PROP_COUNT)


if __name__ == "__main__":
    unittest.main()
