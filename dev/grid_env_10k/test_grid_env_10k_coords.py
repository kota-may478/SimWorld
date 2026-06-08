"""1-indexed ブロック座標のユニットテスト（UE 不要）。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

import grid_env_10k as g10k  # noqa: E402


class TestBlockCoords(unittest.TestCase):
    def test_corner_mapping(self) -> None:
        self.assertEqual(g10k.block_index_to_row_col(1, 1), (0, 0))
        self.assertEqual(g10k.block_index_to_row_col(100, 1), (0, 99))
        self.assertEqual(g10k.block_index_to_row_col(1, 100), (99, 0))

    def test_actor_name_roundtrip(self) -> None:
        name = g10k.block_actor_name(50, 50)
        self.assertEqual(name, "block_050_050")
        self.assertEqual(g10k.parse_block_actor_name(name), (50, 50))

    def test_rectangle_iter_inclusive(self) -> None:
        cells = list(g10k.iter_rectangle_indices(50, 50, 53, 53))
        self.assertEqual(len(cells), 16)
        self.assertEqual(cells[0], (50, 50))
        self.assertEqual(cells[-1], (53, 53))


if __name__ == "__main__":
    unittest.main()
