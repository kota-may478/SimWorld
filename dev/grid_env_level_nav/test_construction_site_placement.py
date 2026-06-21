#!/usr/bin/env python3
"""Unit tests for construction site placement registry."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from construction_site_placement import (  # noqa: E402
    EXCLUDED_BP_NAMES,
    MIN_CORRIDOR_WIDTH_CM,
    assert_corridor_clear,
    build_construction_site_registry,
    corridor_clearance_cm,
)


class ConstructionSitePlacementTests(unittest.TestCase):
    def test_registry_has_twenty_unique_types(self) -> None:
        registry = build_construction_site_registry()
        unique = {p.bp_name for p in registry.props}
        self.assertEqual(len(unique), 20)
        for name in unique:
            self.assertNotIn(name, EXCLUDED_BP_NAMES)

    def test_transport_target_is_crate(self) -> None:
        registry = build_construction_site_registry()
        target = registry.transport_slot()
        self.assertIsNotNone(target)
        assert target is not None
        self.assertEqual(target.bp_name, "BP_Crate_01a")
        self.assertTrue(target.is_transport_target)

    def test_corridor_clearance_for_all_slots(self) -> None:
        registry = build_construction_site_registry()
        assert_corridor_clear(registry.props, min_width_cm=MIN_CORRIDOR_WIDTH_CM)
        obstacles = [p for p in registry.props if not p.is_transport_target]
        worst = min(corridor_clearance_cm(p.local_xy_cm) for p in obstacles)
        self.assertGreaterEqual(worst, MIN_CORRIDOR_WIDTH_CM * 0.5)

    def test_cinder_wall_has_three_instances(self) -> None:
        registry = build_construction_site_registry()
        cinder = [p for p in registry.props if p.cluster_id == "cinder_wall"]
        self.assertEqual(len(cinder), 3)
        self.assertTrue(all(p.bp_name == "BP_CinderStack_01a" for p in cinder))


if __name__ == "__main__":
    unittest.main()
