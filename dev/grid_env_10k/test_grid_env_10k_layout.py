"""レイアウト API のユニットテスト（UE 不要）。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

import grid_env_10k as g10k  # noqa: E402


class TestLayoutIndices(unittest.TestCase):
    def test_perimeter_count_396(self) -> None:
        cells = list(g10k.iter_perimeter_indices(100))
        self.assertEqual(len(cells), 396)

    def test_rectangle_inclusive_11x11(self) -> None:
        cells = list(g10k.iter_rectangle_indices(10, 10, 20, 20))
        self.assertEqual(len(cells), 11 * 11)
        self.assertIn((10, 10), cells)
        self.assertIn((20, 20), cells)

    def test_mode_parsing(self) -> None:
        self.assertEqual(g10k.parse_block_mode("t"), "T")
        self.assertTrue(g10k.mode_to_set_blocking("T"))
        self.assertFalse(g10k.mode_to_set_blocking("F"))


if __name__ == "__main__":
    unittest.main()
