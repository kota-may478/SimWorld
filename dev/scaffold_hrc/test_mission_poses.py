#!/usr/bin/env python3
"""Unit tests for UE drop/socket split (no PIE)."""

from __future__ import annotations

import unittest

from scene.erect_plan import build_erect_sequence
from scene.field import DECK_THICKNESS_M
from scene.geometry import STAGE1_GEOM
from ue.mission_poses import (
    ASSEMBLER_FOOT_Z_CM,
    ASSEMBLER_SCALE,
    extra_humanoid_actor_names,
    HUMANOID_NAME_MARKERS,
    humanoid_fit_scale,
    humanoid_root_z_cm,
    snap_hover_local_cm,
    SPOT_STAND_Z_CM,
    story_clearance_cm,
    walk_surface_z_m,
    next_surface_hop_xyz,
    ASSEMBLER_INSTALL_STANDOFF_M,
    assembler_hip_local_cm,
    assembler_install_local_cm,
    assembler_install_xy_m,
    assembler_stand_local_cm,
    first_free_drop_slot,
    floor_drop_local_cm,
    floor_drop_slot_local_cm,
    floor_refuge_local_cm,
    lift_sunk_local_cm,
    next_hop_local_cm,
    next_hop_standing_local_cm,
    reachable_floor,
    socket_local_cm,
    standing_local_cm,
)
from ue.placement import DROP_SLOT_COUNT, DROP_SLOT_PITCH_M, DROP_XY_M, REFUGE_XY_M
from ue.layout import local_cm_to_scaffold_xyz, scaffold_xyz_to_local_cm


class MissionPoseTests(unittest.TestCase):
    def test_drop_is_floor_staging_not_the_socket(self) -> None:
        post = build_erect_sequence(STAGE1_GEOM)[0]
        drop = floor_drop_local_cm(1)
        sock = socket_local_cm(post)
        expected = scaffold_xyz_to_local_cm(DROP_XY_M[0], DROP_XY_M[1], 0.0)
        self.assertAlmostEqual(drop[0], expected[0], places=4)
        self.assertAlmostEqual(drop[1], expected[1], places=4)
        self.assertNotAlmostEqual(drop[0], sock[0], places=1)
        self.assertNotAlmostEqual(drop[1], sock[1], places=1)

    def test_drop_slots_form_a_line_along_the_deck(self) -> None:
        a = floor_drop_slot_local_cm(1, 0)
        b = floor_drop_slot_local_cm(1, 1)
        self.assertEqual(a, floor_drop_local_cm(1))
        self.assertAlmostEqual(a[0], b[0], places=4)
        self.assertAlmostEqual(abs(b[1] - a[1]), DROP_SLOT_PITCH_M * 100.0, places=4)

    def test_first_free_slot_skips_occupied_and_fills_up(self) -> None:
        self.assertEqual(first_free_drop_slot({0, 2}), 1)
        self.assertIsNone(first_free_drop_slot(set(range(DROP_SLOT_COUNT))))

    def test_refuge_is_far_end_of_deck(self) -> None:
        refuge = floor_refuge_local_cm(1)
        expected = scaffold_xyz_to_local_cm(REFUGE_XY_M[0], REFUGE_XY_M[1], 0.0)
        self.assertAlmostEqual(refuge[0], expected[0], places=4)
        self.assertAlmostEqual(refuge[1], expected[1], places=4)

    def test_assembler_stand_is_on_the_walk_plane(self) -> None:
        drop = floor_drop_local_cm(1)
        stand = assembler_stand_local_cm(drop)
        walk = DECK_THICKNESS_M * 100.0
        self.assertAlmostEqual(stand[2], walk + ASSEMBLER_FOOT_Z_CM, places=4)

    def test_humanoid_scale_fits_under_storey_clearance(self) -> None:
        clear = story_clearance_cm(STAGE1_GEOM.lift_m)
        scale = humanoid_fit_scale(lift_m=STAGE1_GEOM.lift_m)
        self.assertLess(scale, 1.0)
        scaled_height = 180.0 * scale
        self.assertLessEqual(scaled_height, clear + 1e-6)
        self.assertAlmostEqual(ASSEMBLER_SCALE, scale, places=6)
        self.assertAlmostEqual(
            ASSEMBLER_FOOT_Z_CM,
            humanoid_root_z_cm(scale=scale),
            places=6,
        )
        # Head (≈ scaled native height) must clear the underside of the next deck.
        self.assertLess(
            scaled_height,
            STAGE1_GEOM.lift_m * 100.0 - DECK_THICKNESS_M * 100.0,
        )

    def test_lift_sunk_local_puts_a_fallen_pawn_back_on_the_floor(self) -> None:
        drop = floor_drop_local_cm(1)
        sunk = (drop[0], drop[1], drop[2] - 140.0)
        lifted = lift_sunk_local_cm(sunk, 50.0)
        self.assertAlmostEqual(lifted[2], DECK_THICKNESS_M * 100.0 + 50.0, places=4)
        ok = standing_local_cm(drop, 50.0)
        self.assertEqual(lift_sunk_local_cm(ok, 50.0), ok)

    def test_assembler_hip_raises_z(self) -> None:
        drop = floor_drop_local_cm(1)
        hip = assembler_hip_local_cm(drop)
        self.assertGreater(hip[2], drop[2] + 50.0)

    def test_reach_starts_on_floor_1(self) -> None:
        self.assertEqual(reachable_floor(set()), 1)
        self.assertEqual(reachable_floor({0}), 2)
        self.assertEqual(reachable_floor({0, 1}), 3)

    def test_upper_deck_work_stays_on_reachable_floor(self) -> None:
        seq = build_erect_sequence(STAGE1_GEOM)
        board_f2 = next(it for it in seq if it.item_id.startswith("boardslot_f2_"))
        post_l1 = next(it for it in seq if it.item_id.endswith("_L1") and it.kind == "post")
        z1 = STAGE1_GEOM.floor_z_m(1) * 100.0
        z2 = STAGE1_GEOM.floor_z_m(2) * 100.0
        clamped = socket_local_cm(board_f2, walk_floor=1)
        self.assertAlmostEqual(clamped[2], z1, places=4)
        opened = socket_local_cm(post_l1, walk_floor=2)
        self.assertAlmostEqual(opened[2], z2, places=4)

    def test_local_cm_roundtrip_scaffold_metres(self) -> None:
        local = scaffold_xyz_to_local_cm(2.0, 1.2, 1.8)
        x_m, y_m, z_m = local_cm_to_scaffold_xyz(*local)
        self.assertAlmostEqual(x_m, 2.0, places=6)
        self.assertAlmostEqual(y_m, 1.2, places=6)
        self.assertAlmostEqual(z_m, 1.8, places=6)

    def test_assembler_install_is_in_front_of_the_member(self) -> None:
        post = build_erect_sequence(STAGE1_GEOM)[0]
        ix, iy = assembler_install_xy_m(post)
        dist = ((ix - post.module.x_m) ** 2 + (iy - post.module.y_m) ** 2) ** 0.5
        self.assertAlmostEqual(dist, ASSEMBLER_INSTALL_STANDOFF_M, places=4)
        sock = socket_local_cm(post)
        inst = assembler_install_local_cm(post)
        self.assertNotAlmostEqual(sock[0], inst[0], places=1)
        # Install stands on the deck top; socket uses the floor reference plane.
        self.assertAlmostEqual(
            inst[2] - sock[2], DECK_THICKNESS_M * 100.0, places=4
        )

    def test_stair_post_install_stays_on_walkable_deck(self) -> None:
        """Regression: off-deck stair XY + standing snapped to tread tops (~2F)."""
        stair = next(
            i for i in build_erect_sequence(STAGE1_GEOM) if i.item_id == "post_stair_0_L0"
        )
        ix, iy = assembler_install_xy_m(stair)
        self.assertGreaterEqual(ix, 0.20)
        self.assertLessEqual(ix, STAGE1_GEOM.deck_length_m - 0.20)
        stand = assembler_stand_local_cm(
            assembler_install_local_cm(stair, walk_floor=1)
        )
        drop_stand = assembler_stand_local_cm(floor_drop_local_cm(1))
        self.assertAlmostEqual(stand[2], drop_stand[2], places=3)
        refuge = assembler_stand_local_cm(floor_refuge_local_cm(1))
        hop = next_hop_standing_local_cm(refuge, stand, ASSEMBLER_FOOT_Z_CM)
        hop_remain = ((hop[0] - refuge[0]) ** 2 + (hop[1] - refuge[1]) ** 2) ** 0.5
        goal_remain = (
            (stand[0] - refuge[0]) ** 2 + (stand[1] - refuge[1]) ** 2
        ) ** 0.5
        # Must walk toward the install on this deck, not a 15m ramp/stair detour.
        self.assertLess(hop_remain, 1200.0)
        self.assertLessEqual(hop_remain, goal_remain + 1.0)

    def test_stair_hop_from_1f_drop_to_2f_drop_climbs(self) -> None:
        src = floor_drop_local_cm(1)
        dest = floor_drop_local_cm(2)
        hop = next_hop_local_cm(src, dest)
        sx, sy, sz = local_cm_to_scaffold_xyz(*src)
        hx, hy, hz = local_cm_to_scaffold_xyz(*hop)
        self.assertLess(hz, STAGE1_GEOM.floor_z_m(2) - 0.05)
        self.assertNotAlmostEqual(hx, sx, places=1)
        self.assertGreater(abs(hy - sy), 0.05)

    def test_standing_hop_keeps_spot_above_the_floor(self) -> None:
        src = standing_local_cm(floor_drop_local_cm(1), SPOT_STAND_Z_CM)
        dest = standing_local_cm(floor_drop_local_cm(2), SPOT_STAND_Z_CM)
        lifted = next_hop_standing_local_cm(src, dest, SPOT_STAND_Z_CM)
        two = standing_local_cm(floor_drop_local_cm(2), SPOT_STAND_Z_CM)
        self.assertGreater(lifted[2], DECK_THICKNESS_M * 100.0)
        self.assertLess(lifted[2], two[2])
        self.assertAlmostEqual(
            two[2],
            STAGE1_GEOM.floor_z_m(2) * 100.0 + DECK_THICKNESS_M * 100.0 + SPOT_STAND_Z_CM,
            places=4,
        )

    def test_assembler_root_on_1f_is_not_treated_as_2f(self) -> None:
        drop = floor_drop_local_cm(1)
        stand = assembler_stand_local_cm(drop)
        walk = DECK_THICKNESS_M * 100.0
        self.assertAlmostEqual(stand[2], walk + ASSEMBLER_FOOT_Z_CM, places=4)
        hop = next_hop_standing_local_cm(stand, stand, ASSEMBLER_FOOT_Z_CM)
        self.assertAlmostEqual(hop[2], stand[2], places=4)
        refuge = assembler_stand_local_cm(floor_refuge_local_cm(1))
        hopped = next_hop_standing_local_cm(stand, refuge, ASSEMBLER_FOOT_Z_CM)
        self.assertAlmostEqual(hopped[2], walk + ASSEMBLER_FOOT_Z_CM, places=4)

    def test_spot_on_the_yard_sits_near_the_ground(self) -> None:
        self.assertAlmostEqual(walk_surface_z_m(-11.2, 1.5, 0.0), 0.0, places=4)
        yard = standing_local_cm((150.0, 280.0, 0.0), SPOT_STAND_Z_CM)
        self.assertAlmostEqual(yard[2], SPOT_STAND_Z_CM, places=4)

    def test_stair_surface_hop_lands_on_a_tread_top(self) -> None:
        src = (2.0, 1.2, DECK_THICKNESS_M)
        dest = (2.0, 1.2, STAGE1_GEOM.floor_z_m(2) + DECK_THICKNESS_M)
        hop = next_surface_hop_xyz(src, dest)
        self.assertLess(hop[2], STAGE1_GEOM.floor_z_m(2))
        self.assertGreater(hop[2], DECK_THICKNESS_M - 1e-6)
        self.assertAlmostEqual(
            hop[2],
            walk_surface_z_m(hop[0], hop[1], hop[2]),
            places=4,
        )

    def test_hovering_assembler_snaps_back_to_the_goal_plane(self) -> None:
        dest = assembler_stand_local_cm(floor_drop_local_cm(1))
        floated = (dest[0], dest[1], dest[2] + 240.0)
        snapped = snap_hover_local_cm(floated, dest)
        self.assertAlmostEqual(snapped[2], dest[2], places=4)

    def test_extra_humanoid_names_keep_named_pawns(self) -> None:
        extras = extra_humanoid_actor_names(
            [
                "ScaffoldHrc_assembler",
                "ScaffoldHrc_yard_human",
                "BP_Pedestrian_C_2",
                "User_Agent",
                "GridEnv_SpotRobot",
                "ScaffoldHrc_truck",
            ],
            ("ScaffoldHrc_assembler", "ScaffoldHrc_yard_human"),
        )
        self.assertEqual(extras, ["BP_Pedestrian_C_2", "User_Agent"])
        self.assertTrue(any("Pedestrian" in m for m in HUMANOID_NAME_MARKERS))


if __name__ == "__main__":
    unittest.main()
