#!/usr/bin/env python3
import sys
import tempfile
import unittest
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from zone_catalog import ZoneCatalog, catalog_to_zone_registry  # noqa: E402


class TestZoneCatalog(unittest.TestCase):
    def test_rect_local_resolves_at_resolution(self) -> None:
        cat = ZoneCatalog()
        cat.add_rect_local("RoomD", 100.0, 200.0, 400.0, 500.0)
        cells_30 = cat.entries["RoomD"].cells_at(30.0)
        cells_100 = cat.entries["RoomD"].cells_at(100.0)
        self.assertGreater(len(cells_30), len(cells_100))
        reg = catalog_to_zone_registry(cat, 30.0)
        self.assertEqual(len(reg.zones["RoomD"].cells), len(cells_30))

    def test_template_roundtrip(self) -> None:
        path = _THIS_DIR / "cache" / "zone_catalog.template.json"
        cat = ZoneCatalog.load(path)
        self.assertIn("RoomA", cat.entries)
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "cat.json"
            cat.save(out)
            loaded = ZoneCatalog.load(out)
        self.assertEqual(loaded.list_zones(), cat.list_zones())


if __name__ == "__main__":
    unittest.main()
