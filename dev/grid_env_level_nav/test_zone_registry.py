#!/usr/bin/env python3
import json
import sys
import tempfile
import unittest
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from zone_registry import ZoneRegistry  # noqa: E402


class TestZoneRegistry(unittest.TestCase):
    def test_rect_and_roundtrip(self) -> None:
        reg = ZoneRegistry(resolution_cm=100.0)
        reg.add_rect_zone("RoomD", 0, 0, 200, 200)
        self.assertGreater(len(reg.zones["RoomD"].cells), 0)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "zones.json"
            reg.save(path)
            loaded = ZoneRegistry.load(path)
        self.assertIn("RoomD", loaded.zones)
        self.assertEqual(len(loaded.zones["RoomD"].cells), len(reg.zones["RoomD"].cells))

    def test_flat_json_format(self) -> None:
        payload = {
            "resolution_cm": 30.0,
            "RoomD": {"cells": [[1, 2], [3, 4]], "default_cost": 1.0, "closed_cost": 1e9},
        }
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "flat.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            loaded = ZoneRegistry.load(path)
        self.assertEqual(len(loaded.zones["RoomD"].cells), 2)


if __name__ == "__main__":
    unittest.main()
