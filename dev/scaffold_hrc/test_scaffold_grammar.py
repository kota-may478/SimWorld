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
        frames = [m for m in spec.modules if m.kind == "post"]
        n_lifts = STAGE1_GEOM.n_floors - 1
        deck_posts = [m for m in frames if not m.module_id.startswith("post_ramp_")]
        ramp_posts = [m for m in frames if m.module_id.startswith("post_ramp_")]
        self.assertEqual(len(deck_posts), 14 * n_lifts)
        self.assertEqual(len(ramp_posts), 3 * 2 * 2 * n_lifts)
        self.assertTrue(all(m.y_m in (0.0, STAGE1_GEOM.deck_width_m) for m in deck_posts))
        self.assertTrue(all(abs(m.sz_m - STAGE1_GEOM.lift_m) < 1e-9 for m in frames))
        self.assertEqual({round(m.z_m, 6) for m in frames}, {0.0, STAGE1_GEOM.lift_m})
        walls = [
            m
            for m in spec.modules
            if m.sy_m >= STAGE1_GEOM.deck_width_m * 0.9 and m.sz_m >= 1.0
        ]
        self.assertEqual(walls, [])

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

    def test_stair_treads_match_recast_zigzag(self) -> None:
        from scene.field import RAMP_STEPS, stair_lane_y_m, stair_tread_poses

        spec = build_scaffold(STAGE1_GEOM)
        treads = [m for m in spec.modules if m.kind == "stair_tread"]
        self.assertEqual(len(treads), 2 * RAMP_STEPS)
        poses = stair_tread_poses(
            lift_m=STAGE1_GEOM.lift_m,
            deck_width_m=STAGE1_GEOM.deck_width_m,
            n_lifts=STAGE1_GEOM.n_floors - 1,
        )
        self.assertEqual(len(treads), len(poses))
        for module, pose in zip(treads, poses):
            self.assertEqual(module.module_id, f"tread_L{pose.lift}_{pose.step}")
            self.assertAlmostEqual(module.x_m, pose.x_m)
            self.assertAlmostEqual(module.y_m, pose.y_m)
            self.assertLess(module.x_m, 0.0)
        l0 = [m for m in treads if m.module_id.startswith("tread_L0_")]
        l1 = [m for m in treads if m.module_id.startswith("tread_L1_")]
        self.assertAlmostEqual(l0[0].y_m, stair_lane_y_m(0, STAGE1_GEOM.deck_width_m))
        self.assertAlmostEqual(l1[0].y_m, stair_lane_y_m(1, STAGE1_GEOM.deck_width_m))
        self.assertGreater(abs(l0[0].x_m - l1[0].x_m), 0.5)

    def test_stair_tower_posts_stand_on_ramp_stringers(self) -> None:
        from scene.field import (
            RAMP_WIDTH_M,
            ramp_yard_x_m,
            stair_lane_y_m,
            stair_post_poses,
        )

        spec = build_scaffold(STAGE1_GEOM)
        ramp_posts = [m for m in spec.modules if m.module_id.startswith("post_ramp_")]
        poses = stair_post_poses(
            lift_m=STAGE1_GEOM.lift_m,
            deck_width_m=STAGE1_GEOM.deck_width_m,
            n_lifts=STAGE1_GEOM.n_floors - 1,
        )
        self.assertEqual(len(ramp_posts), len(poses))
        self.assertEqual(len(ramp_posts), 24)
        xs = sorted({round(m.x_m, 6) for m in ramp_posts})
        self.assertEqual(len(xs), 3)
        self.assertAlmostEqual(xs[0], ramp_yard_x_m())
        for pose in poses:
            y_c = stair_lane_y_m(pose.lane, STAGE1_GEOM.deck_width_m)
            self.assertAlmostEqual(abs(pose.y_m - y_c), 0.5 * RAMP_WIDTH_M)

    def test_place_fills_next_empty_socket(self) -> None:
        spec = build_scaffold(STAGE1_GEOM)
        first = spec.next_empty_socket(floor=1)
        self.assertIsNotNone(first)
        filled = spec.with_placed(first.socket_id)
        self.assertIsNone(next((s for s in filled.sockets_on_floor(1) if s.socket_id == first.socket_id and not s.filled), None))
        self.assertNotEqual(filled.next_empty_socket(1).socket_id, first.socket_id)

        with self.assertRaises(ValueError):
            spec.with_placed("no_such_socket")

    def test_no_mid_brace_rail_or_stair_mouth_transom(self) -> None:
        spec = build_scaffold(STAGE1_GEOM)
        rails = [m for m in spec.modules if m.kind == "rail"]
        self.assertEqual(rails, [])
        transoms = [m for m in spec.modules if m.kind == "transom"]
        self.assertTrue(all(m.x_m > 0.5 for m in transoms))

    def test_ledgers_are_one_bay(self) -> None:
        spec = build_scaffold(STAGE1_GEOM)
        ledgers = [m for m in spec.modules if m.kind == "ledger"]
        self.assertEqual(len(ledgers), 3 * 2 * spec.n_bays)
        self.assertTrue(all(abs(m.sx_m - spec.bay_m) < 1e-9 for m in ledgers))
        ids = {m.module_id for m in ledgers}
        self.assertIn("ledger_f1_0_b0", ids)
        self.assertNotIn("ledger_f1_0", ids)

    def test_module_kind_helper(self) -> None:
        self.assertEqual(module_kind("post"), "post")
        self.assertEqual(module_kind("brace"), "brace")
        self.assertEqual(module_kind("frame"), "frame")


if __name__ == "__main__":
    unittest.main()
