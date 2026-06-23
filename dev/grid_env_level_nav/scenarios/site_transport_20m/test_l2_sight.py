#!/usr/bin/env python3
"""Unit tests for ObjectRegistry (AI Sight semantic tracking, no L2 painting)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from object_registry import (  # noqa: E402
    ObjectRegistry,
    VisibleTarget,
    _parse_sight_payload,
    build_actor_maps,
    is_dynamic_slot,
)
from placement import build_registry, to_placement_registry  # noqa: E402


class ObjectRegistryTest(unittest.TestCase):
    def test_dynamic_slot_classification(self) -> None:
        reg = build_registry()
        placement = to_placement_registry(reg)
        _actor_map, _slot_map, dynamic = build_actor_maps(
            placement,
            humanoid_actor_name=reg.humanoid_actor_name,
            material_actor_name=reg.material_actor_name,
        )
        self.assertTrue(
            is_dynamic_slot(reg.humanoid_actor_name, dynamic_slots=dynamic, prop_type_id="human_worker")
        )
        self.assertFalse(
            is_dynamic_slot("site20_prop_000", dynamic_slots=dynamic, prop_type_id="dumpster")
        )

    def test_registry_upsert_and_goal(self) -> None:
        registry = ObjectRegistry()
        registry.upsert(
            slot_id="site20_prop_000",
            prop_type_id="dumpster",
            world_xy=(-900.0, -2100.0),
            is_dynamic=False,
        )
        self.assertEqual(registry.goal_xy("site20_prop_000"), (-900.0, -2100.0))
        self.assertIsNotNone(registry.goal_local("site20_prop_000"))

    def test_clear_dynamic_preserves_static(self) -> None:
        registry = ObjectRegistry()
        registry.upsert(
            slot_id="static_prop",
            prop_type_id="dumpster",
            world_xy=(-800.0, -2000.0),
            is_dynamic=False,
        )
        registry.upsert(
            slot_id="human",
            prop_type_id="human_worker",
            world_xy=(-750.0, -1950.0),
            is_dynamic=True,
        )
        registry.clear_dynamic()
        self.assertIn("static_prop", registry.entries)
        self.assertNotIn("human", registry.entries)

    def test_parse_vbp_return_value_wrapper(self) -> None:
        raw = '{"ReturnValue": "{\\"targets\\":[{\\"actor\\":\\"site20_prop_003\\"}]}"}'
        parsed = _parse_sight_payload(raw)
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(len(parsed), 1)
        self.assertEqual(parsed[0].actor_name, "site20_prop_003")


if __name__ == "__main__":
    unittest.main()
