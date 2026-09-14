#!/usr/bin/env python3
"""Unit tests for Level-map scaffold placement (no UE)."""

from __future__ import annotations

import math
import unittest

from scene.geometry import STAGE1_GEOM
from ue.layout import (
    ENTRANCE_LOCAL_XY_CM,
    STAGING_LOCAL_XY_CM,
    cube_scale,
    cube_scale_level_world,
    footprint_inside_level_work,
    footprint_local_cm,
    scaffold_xyz_to_local_cm,
)
from ue.placement import (
    ACTOR_PREFIX,
    BOARD_SEAM_OVERLAP_M,
    CARRY_MAX_M,
    DECK_NAV_OVERLAP_M,
    DECK_THICKNESS_M,
    LANDING_SILL_DEPTH_M,
    LANDING_SOUTH_WIDTH_M,
    TRUCK_BP_CANDIDATES,
    TRUCK_LENGTH_CM,
    TRUCK_YAW_DEG,
    SPOT_YARD_LOCAL_XY_CM,
    YARD_HUMAN_ALONG_CM,
    YARD_HUMAN_SIDE_CM,
    RAMP_STEP_DEPTH_M,
    RAMP_STEP_OVERLAP_M,
    RAMP_STEPS,
    STRINGER_THICK_M,
    all_spawn_boxes,
    boxes_from_modules,
    floor_walk_z_m,
    ramp_pitch_deg,
    ramp_step_rise_m,
    ramp_yard_x_m,
    skip_board_module,
    module_to_spawn_box,
    spot_ramp_cheeks,
    spot_ramps,
)
from scene.scaffold_grammar import build_scaffold

NATIVE_M = 0.30


def _scaffold_x_m(box) -> float:
    return (box.local_cm[1] - ENTRANCE_LOCAL_XY_CM[1]) / 100.0


def _scaffold_x0_m(box) -> float:
    return _scaffold_x_m(box) - 0.5 * box.scale[0] * NATIVE_M


def _scaffold_x1_m(box) -> float:
    return _scaffold_x_m(box) + 0.5 * box.scale[0] * NATIVE_M


def _scaffold_y_m(box) -> float:
    return (box.local_cm[0] - ENTRANCE_LOCAL_XY_CM[0]) / 100.0


def _scaffold_y0_m(box) -> float:
    return _scaffold_y_m(box) - 0.5 * box.scale[1] * NATIVE_M


def _scaffold_y1_m(box) -> float:
    return _scaffold_y_m(box) + 0.5 * box.scale[1] * NATIVE_M


def _scaffold_z_top_m(box) -> float:
    return box.local_cm[2] / 100.0 + box.scale[2] * NATIVE_M


def _named(boxes, suffix: str):
    prefix = f"{ACTOR_PREFIX}_{suffix}"
    hit = [
        box
        for box in boxes
        if box.actor == prefix or box.actor.startswith(prefix + "_")
    ]
    hit.sort(key=lambda box: box.actor)
    return hit


def _span_x_m(boxes) -> tuple[float, float]:
    return (
        min(_scaffold_x0_m(box) for box in boxes),
        max(_scaffold_x1_m(box) for box in boxes),
    )


def _span_y_m(boxes) -> tuple[float, float]:
    return (
        min(_scaffold_y0_m(box) for box in boxes),
        max(_scaffold_y1_m(box) for box in boxes),
    )


def _overlap_1d(a0: float, a1: float, b0: float, b1: float) -> float:
    return min(a1, b1) - max(a0, b0)


class UeLayoutTest(unittest.TestCase):
    def test_entrance_is_along_local_y(self) -> None:
        self.assertTrue(footprint_inside_level_work())
        lx, ly, _lz = scaffold_xyz_to_local_cm(0.0, 0.0, 0.0)
        self.assertAlmostEqual(lx, ENTRANCE_LOCAL_XY_CM[0])
        self.assertAlmostEqual(ly, ENTRANCE_LOCAL_XY_CM[1])
        self.assertAlmostEqual(ly, 1400.0)
        x0, y0, x1, y1 = footprint_local_cm()
        self.assertGreaterEqual(x0, 0.0)
        self.assertGreaterEqual(y0, 0.0)
        self.assertLessEqual(x1, 7000.0)
        self.assertLessEqual(y1, 7900.0)
        self.assertLess(lx, 50.0)

    def test_deck_runs_along_local_y(self) -> None:
        lx0, ly0, _ = scaffold_xyz_to_local_cm(0.0, 1.2, 0.0)
        lx1, ly1, _ = scaffold_xyz_to_local_cm(10.0, 1.2, 0.0)
        self.assertAlmostEqual(lx0, lx1)
        self.assertAlmostEqual(ly1 - ly0, 1000.0)

    def test_staging_is_near_local_origin(self) -> None:
        self.assertLess(STAGING_LOCAL_XY_CM[0], 300.0)
        self.assertGreater(STAGING_LOCAL_XY_CM[1], 200.0)
        self.assertLess(STAGING_LOCAL_XY_CM[1], 500.0)
        self.assertLess(STAGING_LOCAL_XY_CM[1], ENTRANCE_LOCAL_XY_CM[1] - 800.0)

    def test_spot_yard_pose_is_beside_the_kei_truck(self) -> None:
        dx = SPOT_YARD_LOCAL_XY_CM[0] - STAGING_LOCAL_XY_CM[0]
        dy = SPOT_YARD_LOCAL_XY_CM[1] - STAGING_LOCAL_XY_CM[1]
        dist_m = math.hypot(dx, dy) / 100.0
        self.assertGreater(dist_m, 0.5)
        self.assertLess(dist_m, 3.5)
        self.assertGreater(abs(dx), 120.0)
        self.assertGreater(dy, 0.5 * TRUCK_LENGTH_CM)

    def test_ground_boards_are_not_spawned(self) -> None:
        spec = build_scaffold(STAGE1_GEOM)
        ground_boards = [
            m for m in spec.modules if m.kind == "board" and skip_board_module(m)
        ]
        self.assertEqual(len(ground_boards), 30)
        spawned = boxes_from_modules()
        kinds = {box.kind for box in spawned}
        self.assertIn("post", kinds)
        self.assertIn("brace", kinds)
        self.assertNotIn("board", kinds)
        self.assertNotIn("stair_tread", kinds)

    def test_brace_install_pose_is_diagonal(self) -> None:
        spec = build_scaffold(STAGE1_GEOM)
        brace = next(m for m in spec.modules if m.kind == "brace")
        expected = math.degrees(
            math.atan2(STAGE1_GEOM.lift_m, STAGE1_GEOM.deck_length_m / 5.0)
        )
        box = module_to_spawn_box(brace)
        self.assertGreater(abs(box.pitch_deg), 20.0)
        self.assertAlmostEqual(abs(box.pitch_deg), expected, places=4)
        self.assertAlmostEqual(box.roll_deg, 0.0)
        finished = {b.actor: b for b in boxes_from_modules()}
        ref = finished[box.actor]
        self.assertAlmostEqual(box.pitch_deg, ref.pitch_deg)
        self.assertAlmostEqual(box.yaw_deg, ref.yaw_deg)
        self.assertAlmostEqual(box.roll_deg, ref.roll_deg)
        self.assertEqual(box.local_cm, ref.local_cm)
        self.assertEqual(box.scale, ref.scale)

    def test_posts_stay_on_the_long_edges(self) -> None:
        posts = [box for box in boxes_from_modules() if box.kind == "post"]
        n_lifts = STAGE1_GEOM.n_floors - 1
        deck = [box for box in posts if "_post_ramp_" not in box.actor]
        ramp = [box for box in posts if "_post_ramp_" in box.actor]
        self.assertEqual(len(deck), 14 * n_lifts)
        self.assertEqual(len(ramp), 24)
        names = {box.actor for box in posts}
        self.assertIn(f"{ACTOR_PREFIX}_post_0_0_L0", names)
        self.assertIn(f"{ACTOR_PREFIX}_post_0_0_L1", names)
        self.assertIn(f"{ACTOR_PREFIX}_post_ramp_l0s0_L0", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_post_0_0", names)
        native_m = 0.30
        for box in posts:
            self.assertAlmostEqual(box.scale[2] * native_m, STAGE1_GEOM.lift_m)
        l0 = [box for box in posts if box.actor.endswith("_L0")]
        self.assertTrue(l0)
        self.assertAlmostEqual(l0[0].local_cm[2], 0.0)

    def test_spot_stairs_are_stair_tower_zigzag(self) -> None:
        ramps = spot_ramps()
        self.assertEqual(len(ramps), 2 * RAMP_STEPS)
        l0 = [box for box in ramps if "_L0_" in box.actor]
        l1 = [box for box in ramps if "_L1_" in box.actor]
        self.assertLess(l0[0].local_cm[1], l0[-1].local_cm[1])
        self.assertGreater(l1[0].local_cm[1], l1[-1].local_cm[1])
        self.assertLess(l0[0].local_cm[2], l0[-1].local_cm[2])
        self.assertLess(l1[0].local_cm[2], l1[-1].local_cm[2])
        self.assertGreater(l1[0].local_cm[2], l0[-1].local_cm[2] - 5.0)
        self.assertGreater(abs(l0[0].local_cm[0] - l1[0].local_cm[0]), 50.0)
        self.assertAlmostEqual(l0[0].scale[0] * NATIVE_M, RAMP_STEP_DEPTH_M)
        self.assertAlmostEqual(l0[0].scale[2] * NATIVE_M, DECK_THICKNESS_M)
        names = {box.actor for box in all_spawn_boxes()}
        self.assertTrue(
            any(n.startswith(f"{ACTOR_PREFIX}_landing_f2_sill_") for n in names)
        )
        self.assertIn(f"{ACTOR_PREFIX}_landing_f3_exit", names)
        self.assertTrue(
            any(n.startswith(f"{ACTOR_PREFIX}_landing_f3_south_") for n in names)
        )
        self.assertNotIn(f"{ACTOR_PREFIX}_landing_f3_north", names)
        self.assertIn(f"{ACTOR_PREFIX}_landing_f3_sill", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_landing_f2", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_landing_f3", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_run_L0", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_tread_L0_0", names)

    def test_landings_leave_stair_shafts_open(self) -> None:
        boxes = {box.actor: box for box in all_spawn_boxes()}
        spawned = all_spawn_boxes()
        native_m = 0.30
        sills = _named(spawned, "landing_f2_sill")
        self.assertEqual(len(sills), 2)
        self.assertAlmostEqual(
            sills[0].scale[0] * native_m,
            LANDING_SILL_DEPTH_M + DECK_NAV_OVERLAP_M,
        )
        y0, y1 = _span_y_m(sills)
        self.assertAlmostEqual(y1 - y0, STAGE1_GEOM.deck_width_m + BOARD_SEAM_OVERLAP_M)
        self.assertLess(sills[0].scale[0] * native_m, STAGE1_GEOM.stair_bay_m)
        souths = _named(spawned, "landing_f3_south")
        self.assertGreaterEqual(len(souths), 2)
        sx0, sx1 = _span_x_m(souths)
        want_span = DECK_NAV_OVERLAP_M - (ramp_yard_x_m() - 0.50)
        self.assertGreaterEqual(sx1 - sx0, want_span - 1e-6)
        self.assertLessEqual(sx1 - sx0, want_span + BOARD_SEAM_OVERLAP_M + 1e-6)
        self.assertAlmostEqual(souths[0].scale[1] * native_m, LANDING_SOUTH_WIDTH_M)
        self.assertLess(souths[0].scale[1] * native_m, STAGE1_GEOM.deck_width_m)
        exit3 = boxes[f"{ACTOR_PREFIX}_landing_f3_exit"]
        sill3 = boxes[f"{ACTOR_PREFIX}_landing_f3_sill"]
        self.assertAlmostEqual(exit3.scale[1] * native_m, LANDING_SOUTH_WIDTH_M)
        self.assertAlmostEqual(sill3.scale[1] * native_m, LANDING_SOUTH_WIDTH_M)
        self.assertGreaterEqual(_scaffold_x1_m(exit3), sx0)
        self.assertGreaterEqual(sx1, _scaffold_x0_m(sill3))
        self.assertGreaterEqual(sx1, DECK_NAV_OVERLAP_M - 1e-6)
        self.assertLessEqual(sx0, -STAGE1_GEOM.stair_bay_m + 1e-6)
        names = {box.actor for box in spawned}
        self.assertNotIn(f"{ACTOR_PREFIX}_rail_f2_0", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_transom_f2_0", names)
        last_l1 = boxes[f"{ACTOR_PREFIX}_ramp_L1_{RAMP_STEPS - 1}"]
        yards = _named(spawned, "landing_f3_yard")
        yx0, yx1 = _span_x_m(yards)
        self.assertGreater(
            _overlap_1d(
                _scaffold_x0_m(last_l1),
                _scaffold_x1_m(last_l1),
                yx0,
                yx1,
            ),
            0.20,
        )
        self.assertLess(
            _overlap_1d(
                0.0,
                LANDING_SOUTH_WIDTH_M,
                _scaffold_y0_m(last_l1),
                _scaffold_y1_m(last_l1),
            ),
            1e-6,
        )

    def test_last_treads_meet_destination_floors(self) -> None:
        boxes = {box.actor: box for box in all_spawn_boxes()}
        last_l0 = boxes[f"{ACTOR_PREFIX}_ramp_L0_{RAMP_STEPS - 1}"]
        last_l1 = boxes[f"{ACTOR_PREFIX}_ramp_L1_{RAMP_STEPS - 1}"]
        deck2 = boxes[f"{ACTOR_PREFIX}_deck_f2_b0_r0"]
        deck3 = boxes[f"{ACTOR_PREFIX}_deck_f3_b0_r0"]
        souths = _named(all_spawn_boxes(), "landing_f3_south")
        sx0, sx1 = _span_x_m(souths)
        self.assertAlmostEqual(_scaffold_z_top_m(last_l0), _scaffold_z_top_m(deck2))
        self.assertAlmostEqual(_scaffold_z_top_m(last_l1), _scaffold_z_top_m(deck3))
        self.assertAlmostEqual(_scaffold_z_top_m(last_l0), floor_walk_z_m(2))
        self.assertAlmostEqual(_scaffold_z_top_m(last_l1), floor_walk_z_m(3))
        overlap_3f = _overlap_1d(
            sx0,
            sx1,
            0.0,
            STAGE1_GEOM.deck_length_m,
        )
        self.assertGreaterEqual(overlap_3f, 0.70)

    def test_lower_treads_do_not_go_under_the_next_floor(self) -> None:
        boxes = {box.actor: box for box in all_spawn_boxes()}
        sill2 = _named(all_spawn_boxes(), "landing_f2_sill")[0]
        last = boxes[f"{ACTOR_PREFIX}_ramp_L0_{RAMP_STEPS - 1}"]
        self.assertGreaterEqual(_scaffold_x1_m(last), _scaffold_x0_m(sill2) - 1e-6)
        first = boxes[f"{ACTOR_PREFIX}_ramp_L0_0"]
        self.assertLess(_scaffold_x1_m(first), 0.0)

    def test_ramps_stay_under_recast_max_slope(self) -> None:
        self.assertAlmostEqual(ramp_pitch_deg(), 0.0)
        self.assertLessEqual(ramp_step_rise_m(), 0.35)
        ramps = spot_ramps()
        self.assertEqual(len(ramps), 2 * RAMP_STEPS)
        names = {box.actor: box for box in ramps}
        low = names[f"{ACTOR_PREFIX}_ramp_L0_0"]
        north = names[f"{ACTOR_PREFIX}_ramp_L1_0"]
        self.assertAlmostEqual(low.pitch_deg, 0.0)
        self.assertAlmostEqual(low.scale[0] * NATIVE_M, RAMP_STEP_DEPTH_M)
        nxt = names[f"{ACTOR_PREFIX}_ramp_L0_1"]
        self.assertGreaterEqual(
            _overlap_1d(
                _scaffold_x0_m(low),
                _scaffold_x1_m(low),
                _scaffold_x0_m(nxt),
                _scaffold_x1_m(nxt),
            ),
            RAMP_STEP_OVERLAP_M - 1e-6,
        )
        self.assertGreater(
            _scaffold_y0_m(north),
            _scaffold_y1_m(low) - 1e-6,
        )
        cheeks = spot_ramp_cheeks()
        self.assertEqual(len(cheeks), 4 * (RAMP_STEPS - 1))
        self.assertAlmostEqual(cheeks[0].scale[0] * NATIVE_M, RAMP_STEP_OVERLAP_M)
        self.assertAlmostEqual(cheeks[0].scale[1] * NATIVE_M, STRINGER_THICK_M)
        self.assertAlmostEqual(
            cheeks[0].scale[2] * NATIVE_M,
            ramp_step_rise_m() - DECK_THICKNESS_M,
        )
        names = {box.actor for box in all_spawn_boxes()}
        self.assertIn(f"{ACTOR_PREFIX}_cheek_L0_s_0", names)
        self.assertIn(f"{ACTOR_PREFIX}_cheek_L0_n_0", names)
        self.assertIn(f"{ACTOR_PREFIX}_cheek_L1_s_0", names)
        self.assertIn(f"{ACTOR_PREFIX}_cheek_L1_n_0", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_cheek_L0_0", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_stringer_L0_outer", names)
        l0_s = [box for box in cheeks if "_L0_s_" in box.actor]
        l0_n = [box for box in cheeks if "_L0_n_" in box.actor]
        self.assertEqual(len(l0_s), RAMP_STEPS - 1)
        self.assertLess(_scaffold_y_m(l0_s[0]), _scaffold_y_m(l0_n[0]))

    def test_spawn_has_no_mid_brace_header(self) -> None:
        boxes = all_spawn_boxes()
        names = {box.actor for box in boxes}
        self.assertFalse(any("rail_" in name for name in names))
        transoms = [box for box in boxes if box.kind == "transom"]
        self.assertTrue(all(_scaffold_x_m(box) > 0.5 for box in transoms))

    def test_elevated_decks_and_truck_are_spawned(self) -> None:
        names = {box.actor for box in all_spawn_boxes()}
        self.assertIn(f"{ACTOR_PREFIX}_deck_f1_b0_r0", names)
        self.assertIn(f"{ACTOR_PREFIX}_deck_f2_b4_r1", names)
        self.assertIn(f"{ACTOR_PREFIX}_deck_f3_b2_r0", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_deck_f1", names)
        self.assertTrue(any(n.startswith(f"{ACTOR_PREFIX}_landing_f1_") for n in names))
        self.assertTrue(
            any(n.startswith(f"{ACTOR_PREFIX}_landing_f2_sill_") for n in names)
        )
        self.assertNotIn(f"{ACTOR_PREFIX}_truck_cab", names)
        self.assertIn(f"{ACTOR_PREFIX}_ramp_L0_0", names)
        self.assertIn(f"{ACTOR_PREFIX}_ramp_L1_{RAMP_STEPS - 1}", names)
        self.assertIn(f"{ACTOR_PREFIX}_cheek_L0_s_0", names)
        self.assertIn(f"{ACTOR_PREFIX}_cheek_L0_n_0", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_cheek_L0_0", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_stringer_L0_outer", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_tread_L0_0", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_ramp_L0", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_ramp_L0_a", names)
        self.assertNotIn(f"{ACTOR_PREFIX}_landing_mid_L0", names)
        self.assertIn(
            "/Game/SimWorld/Props/KeiTruck/BP_KeiTruck.BP_KeiTruck_C",
            TRUCK_BP_CANDIDATES,
        )

    def test_parts_are_spot_carryable(self) -> None:
        brace_max = math.hypot(STAGE1_GEOM.deck_length_m / 5.0, STAGE1_GEOM.lift_m)
        decks = [box for box in all_spawn_boxes() if box.kind == "deck"]
        self.assertEqual(len(decks), 30)
        for box in all_spawn_boxes():
            if box.kind == "marker":
                continue
            extent = tuple(axis * NATIVE_M for axis in box.scale)
            longest = max(extent)
            if box.kind == "brace":
                self.assertLessEqual(longest, brace_max + 1e-6, box.actor)
            else:
                self.assertLessEqual(longest, CARRY_MAX_M + 1e-6, box.actor)

    def test_cube_scale_matches_native_thirty_cm(self) -> None:
        sx, sy, sz = cube_scale(0.30, 0.60, 0.90)
        self.assertAlmostEqual(sx, 1.0)
        self.assertAlmostEqual(sy, 2.0)
        self.assertAlmostEqual(sz, 3.0)

    def test_level_world_scale_follows_scaffold_xy(self) -> None:
        sx, sy, sz = cube_scale_level_world(10.0, 2.4, 0.15)
        native_m = 0.30
        self.assertAlmostEqual(sx * native_m, 10.0)
        self.assertAlmostEqual(sy * native_m, 2.4)
        self.assertAlmostEqual(sz * native_m, 0.15)

    def test_spawn_list_includes_landmarks(self) -> None:
        names = {box.actor for box in all_spawn_boxes()}
        self.assertIn(f"{ACTOR_PREFIX}_storage", names)
        self.assertIn(f"{ACTOR_PREFIX}_drop_f1", names)
        self.assertIn(f"{ACTOR_PREFIX}_refuge_f1", names)
        for box in all_spawn_boxes():
            self.assertGreater(min(box.scale), 0.0)

    def test_yard_truck_bed_faces_scaffold(self) -> None:
        self.assertAlmostEqual(TRUCK_YAW_DEG, 180.0)
        self.assertGreater(YARD_HUMAN_SIDE_CM, 100.0)
        self.assertGreater(YARD_HUMAN_ALONG_CM, 80.0)
        self.assertLess(YARD_HUMAN_SIDE_CM, 250.0)
