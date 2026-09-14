#!/usr/bin/env python3
"""Minimal bare-erect regression: small and mid sequences must complete."""

from __future__ import annotations

import unittest

from constraints.pareto import Theta
from oracle.erect_bare import BareErectConfig, _install_pose, run_bare_erection
from scene.erect_plan import (
    build_erect_sequence,
    keepout_sample_item_count,
    limit_erect_sequence,
    prefix_enables_keepout_sampling,
    select_erect_sequence,
    ue_protocol_limits,
)
from scene.geometry import STAGE1_GEOM
from scene.mission_protocol import STAGE1_PROTOCOL


class BareErectTests(unittest.TestCase):
    def test_six_staging_slots_overlap_truck_trips(self) -> None:
        kwargs = dict(max_items=15, record_trace=False, timeout_s=8000, dt_s=0.25)
        serial = run_bare_erection(
            theta=Theta(1.0, 0.35),
            config=BareErectConfig(staging_slots=1, **kwargs),
        )
        pipelined = run_bare_erection(
            theta=Theta(1.0, 0.35),
            config=BareErectConfig(staging_slots=6, **kwargs),
        )
        self.assertTrue(serial.completed)
        self.assertTrue(pipelined.completed)
        self.assertEqual(serial.n_filled, 15)
        self.assertEqual(pipelined.n_filled, 15)
        self.assertLessEqual(pipelined.makespan_s, serial.makespan_s)

    def test_posts_then_brace_complete(self) -> None:
        r = run_bare_erection(
            theta=Theta(1.0, 0.35),
            config=BareErectConfig(
                max_items=15, record_trace=False, timeout_s=5000, dt_s=0.25
            ),
        )
        self.assertTrue(r.completed)
        self.assertEqual(r.n_filled, 15)

    def test_protocol_dwells_and_early_keepout(self) -> None:
        self.assertEqual(STAGE1_PROTOCOL.truck_load_s, 10.0)
        self.assertEqual(STAGE1_PROTOCOL.drop_place_s, 10.0)
        self.assertEqual(STAGE1_PROTOCOL.board_erect_s, 10.0)
        self.assertEqual(STAGE1_PROTOCOL.structural_erect_s, 10.0)
        self.assertEqual(STAGE1_PROTOCOL.erect_s, 10.0)
        self.assertEqual(STAGE1_PROTOCOL.staging_slots, 6)
        self.assertEqual(STAGE1_PROTOCOL.assembler_pickup_s, 5.0)
        self.assertAlmostEqual(STAGE1_PROTOCOL.human_retreat_mps, STAGE1_PROTOCOL.human_speed_mps)
        self.assertTrue(STAGE1_PROTOCOL.assembler_keepout_on_scaffold)
        self.assertFalse(STAGE1_PROTOCOL.yard_keepout)

    def test_floor1_thinned_boards_complete(self) -> None:
        r = run_bare_erection(
            theta=Theta(1.0, 0.35),
            config=BareErectConfig(
                max_floors=1,
                boards_per_floor=2,
                record_trace=False,
                timeout_s=20000,
                dt_s=0.25,
            ),
        )
        self.assertTrue(r.completed)
        self.assertGreaterEqual(r.n_filled, 50)

    def test_lift_order_lays_2f_floor_before_2f_posts(self) -> None:
        seq = build_erect_sequence(STAGE1_GEOM)
        ids = [item.item_id for item in seq]
        last_f1_board = max(i for i, mid in enumerate(ids) if mid.startswith("boardslot_f1_"))
        first_f0_brace = min(i for i, mid in enumerate(ids) if mid.startswith("brace_f0_"))
        last_f2_board = max(i for i, mid in enumerate(ids) if mid.startswith("boardslot_f2_"))
        last_l0_stair = max(i for i, mid in enumerate(ids) if mid.startswith("tread_L0_"))
        first_l1_post = min(
            i
            for i, it in enumerate(seq)
            if it.item_id.endswith("_L1")
            and it.kind == "post"
            and not it.item_id.startswith("post_ramp_")
        )
        first_ramp_l0 = min(i for i, mid in enumerate(ids) if mid.startswith("post_ramp_") and mid.endswith("_L0"))
        first_l0_tread = min(i for i, mid in enumerate(ids) if mid.startswith("tread_L0_"))
        self.assertLess(last_f1_board, first_f0_brace)
        self.assertLess(first_f0_brace, last_f2_board)
        self.assertLess(last_f2_board, first_ramp_l0)
        self.assertLess(first_ramp_l0, first_l0_tread)
        self.assertLess(last_l0_stair, first_l1_post)
        self.assertEqual(seq[first_l1_post].floor, 2)

    def test_full_sequence_length(self) -> None:
        self.assertEqual(len(build_erect_sequence(STAGE1_GEOM)), 174)

    def test_smoke_prefix_cannot_sample_smin(self) -> None:
        smoke = select_erect_sequence("smoke")
        self.assertEqual(len(smoke), 3)
        self.assertFalse(prefix_enables_keepout_sampling(smoke))

    def test_smin_prefix_unlocks_deck_then_one_more(self) -> None:
        seq = select_erect_sequence("smin")
        self.assertTrue(prefix_enables_keepout_sampling(seq))
        self.assertEqual(len(seq), keepout_sample_item_count(build_erect_sequence(STAGE1_GEOM)))
        self.assertTrue(any(item.unlocks_deck for item in seq))
        self.assertTrue(any(item.kind == "board" for item in seq))

    def test_full_protocol_includes_all_member_kinds(self) -> None:
        seq = select_erect_sequence("full")
        self.assertEqual(len(seq), 174)
        kinds = {item.kind for item in seq}
        for kind in ("post", "brace", "ledger", "transom", "board", "stair_tread"):
            self.assertIn(kind, kinds)
        limits = ue_protocol_limits("full")
        self.assertIsNone(limits.max_items)
        self.assertIsNone(limits.max_floors)
        self.assertIsNone(limits.boards_per_floor)

    def test_limit_zero_means_unlimited(self) -> None:
        full = build_erect_sequence(STAGE1_GEOM)
        limited = limit_erect_sequence(
            full, max_items=None, max_floors=None, boards_per_floor=None
        )
        self.assertEqual(len(limited), len(full))

    def test_install_pose_is_offset_from_the_member(self) -> None:
        post = build_erect_sequence(STAGE1_GEOM)[0]
        pose = _install_pose(post, STAGE1_GEOM)
        dist = ((pose[0] - post.module.x_m) ** 2 + (pose[1] - post.module.y_m) ** 2) ** 0.5
        self.assertGreater(dist, 0.5)
        self.assertAlmostEqual(pose[2], STAGE1_GEOM.floor_z_m(post.floor), places=4)


if __name__ == "__main__":
    unittest.main()
