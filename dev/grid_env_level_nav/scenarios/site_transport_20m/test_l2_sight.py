#!/usr/bin/env python3
"""Unit tests for L2 sight memory (static persist / dynamic evict)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

import numpy as np  # noqa: E402
from costmap_layers import LayeredCostmap  # noqa: E402
from l2_sight import (  # noqa: E402
    L2SlotCellTracker,
    SightConfig,
    SightMemory,
    VisibleTarget,
    _apply_slot_cells,
    _parse_sight_payload,
    _remove_slot_cells,
    build_actor_maps,
    is_dynamic_slot,
)
from perception_layer import L2_LETHAL_COST  # noqa: E402
from placement import build_registry, to_placement_registry  # noqa: E402


def _tiny_layers() -> LayeredCostmap:
    costs = np.zeros((40, 40), dtype=np.float32)
    return LayeredCostmap(l0=costs, origin_xy=(-1000.0, -2200.0), resolution_cm=30.0)


class SightMemoryTest(unittest.TestCase):
    def test_dynamic_slot_classification(self) -> None:
        reg = build_registry()
        placement = to_placement_registry(reg)
        _actor_map, _slot_map, dynamic = build_actor_maps(
            placement,
            humanoid_actor_name=reg.humanoid_actor_name,
            material_actor_name=reg.material_actor_name,
        )
        self.assertTrue(is_dynamic_slot(reg.humanoid_actor_name, dynamic_slots=dynamic, prop_type_id="human_worker"))
        self.assertFalse(is_dynamic_slot("site20_prop_000", dynamic_slots=dynamic, prop_type_id="dumpster"))

    def test_apply_and_remove_slot_cells(self) -> None:
        layers = _tiny_layers()
        tracker = L2SlotCellTracker()
        seen: set = set()
        cfg = SightConfig(prop_radius_cm=60.0)
        center = (-900.0, -2100.0)
        added, removed = _apply_slot_cells(
            layers,
            "site20_prop_000",
            center,
            prop_type_id="dumpster",
            config=cfg,
            tracker=tracker,
            l2_seen_cells=seen,
        )
        self.assertGreater(added, 0)
        self.assertEqual(removed, 0)
        self.assertGreater(len(seen), 0)

        n_removed = _remove_slot_cells(
            layers,
            "site20_prop_000",
            tracker=tracker,
            l2_seen_cells=seen,
        )
        self.assertEqual(n_removed, added)
        self.assertEqual(len(seen), 0)
        self.assertEqual(np.count_nonzero(layers.l2), 0)

    def test_apply_slot_cells_uses_lethal_obstacle_footprint(self) -> None:
        layers = _tiny_layers()
        tracker = L2SlotCellTracker()
        seen: set = set()
        center = (-900.0, -2100.0)
        added, _removed = _apply_slot_cells(
            layers,
            "site20_prop_000",
            center,
            prop_type_id="dumpster",
            config=SightConfig(prop_radius_cm=60.0),
            tracker=tracker,
            l2_seen_cells=seen,
        )

        self.assertGreaterEqual(added, 80)
        self.assertTrue(np.all(layers.l2[layers.l2 > 0] == L2_LETHAL_COST))

    def test_static_memory_persists_xy(self) -> None:
        memory = SightMemory()
        memory.static_last_seen_xy["site20_prop_001"] = (-800.0, -2000.0)
        self.assertIn("site20_prop_001", memory.static_last_seen_xy)
        memory.dynamic_last_seen_xy["site20_humanoid"] = (-750.0, -1950.0)
        memory.dynamic_last_seen_xy.pop("site20_humanoid")
        self.assertEqual(len(memory.dynamic_last_seen_xy), 0)

    def test_parse_vbp_return_value_wrapper(self) -> None:
        raw = '{"ReturnValue": "{\\"targets\\":[{\\"actor\\":\\"site20_prop_003\\"}]}"}'
        parsed = _parse_sight_payload(raw)
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(len(parsed), 1)
        self.assertEqual(parsed[0].actor_name, "site20_prop_003")


if __name__ == "__main__":
    unittest.main()
