#!/usr/bin/env python3
"""Unit tests for parametric scaffold sockets and modules (no UE)."""

from __future__ import annotations

import unittest

from scene.geometry import STAGE1_GEOM
from scene.scaffold_grammar import build_scaffold, module_kind


class ScaffoldGrammarTest(unittest.TestCase):
    def test_five_bays_along_ten_metre_deck(self) -> None:
        spec = build_scaffold(STAGE1_GEOM)
        self.assertEqual(spec.n_bays, 5)
        self.assertAlmostEqual(spec.bay_m, 2.0)
        frames = [m for m in spec.modules if m.kind == "frame"]
        self.assertEqual(len(frames), 6 * STAGE1_GEOM.n_floors)

    def test_each_floor_has_board_sockets_on_the_deck(self) -> None:
        spec = build_scaffold(STAGE1_GEOM)
        for floor in (1, 2, 3):
            sockets = spec.sockets_on_floor(floor)
            self.assertGreaterEqual(len(sockets), 4)
            for s in sockets:
                self.assertGreaterEqual(s.x_m, 0.0)
                self.assertLessEqual(s.x_m, 10.0)
                self.assertGreaterEqual(s.y_m, 0.0)
                self.assertLessEqual(s.y_m, 2.4)
                self.assertAlmostEqual(s.z_m, STAGE1_GEOM.floor_z_m(floor))

    def test_stair_treads_connect_lifts(self) -> None:
        spec = build_scaffold(STAGE1_GEOM)
        treads = [m for m in spec.modules if m.kind == "stair_tread"]
        self.assertGreaterEqual(len(treads), 2 * (STAGE1_GEOM.n_floors - 1))
        xs = [m.x_m for m in treads]
        self.assertTrue(all(x < 0.0 for x in xs))

    def test_place_fills_next_empty_socket(self) -> None:
        spec = build_scaffold(STAGE1_GEOM)
        first = spec.next_empty_socket(floor=1)
        self.assertIsNotNone(first)
        filled = spec.with_placed(first.socket_id)
        self.assertIsNone(next((s for s in filled.sockets_on_floor(1) if s.socket_id == first.socket_id and not s.filled), None))
        self.assertNotEqual(filled.next_empty_socket(1).socket_id, first.socket_id)

        with self.assertRaises(ValueError):
            spec.with_placed("no_such_socket")

    def test_module_kind_helper(self) -> None:
        self.assertEqual(module_kind("frame"), "frame")


if __name__ == "__main__":
    unittest.main()
