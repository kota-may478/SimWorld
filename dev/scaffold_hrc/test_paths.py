#!/usr/bin/env python3
"""Unit tests for timestamped run directories."""

from __future__ import annotations

import unittest
from datetime import datetime
from pathlib import Path
from tempfile import TemporaryDirectory

from paths import make_run_dir


class MakeRunDirTest(unittest.TestCase):
    def test_layout_is_compact_timestamp(self) -> None:
        when = datetime(2026, 9, 3, 16, 48, 12)
        with TemporaryDirectory() as tmp:
            path = make_run_dir(Path(tmp), when=when)
            self.assertEqual(path, Path(tmp) / "20260903164812")
            self.assertTrue(path.is_dir())

    def test_collision_gets_numeric_suffix(self) -> None:
        when = datetime(2026, 9, 3, 16, 48, 12)
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            a = make_run_dir(root, when=when)
            b = make_run_dir(root, when=when)
            self.assertEqual(a.name, "20260903164812")
            self.assertEqual(b.name, "20260903164812_2")
            self.assertTrue(b.is_dir())


if __name__ == "__main__":
    unittest.main()
