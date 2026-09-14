#!/usr/bin/env python3
"""Pipeline intent tests (no PIE)."""

from __future__ import annotations

import unittest

from ue.hrc_pipeline import (
    assembler_holds_refuge,
    assembler_is_airborne,
    assembler_is_lost,
    assembler_should_yield_to_locked_spot,
    assist_step_cm,
    drive_duration_s,
    evac_cargo_action,
    gait_closing_on_goal,
    gait_command_remain_cm,
    grounded_root_z_cm,
    nav_projection_is_usable,
    next_asm_mode,
    next_spot_mode,
    pawn_arrived,
    playback_sim_dt_s,
    playback_step_cm,
    should_reissue_drive,
    spot_blocks_staging_handoff,
    spot_on_scaffold_local_x,
    wall_from_sim,
    wall_tick_remainder_s,
    yaw_error_deg,
)


class HrcPipelineTests(unittest.TestCase):
    def test_spot_goes_to_truck_right_after_place(self) -> None:
        self.assertEqual(
            next_spot_mode(
                pending=True,
                loaded=False,
                has_free_slot=True,
                at_yard=False,
                at_drop=False,
            ),
            "to_yard",
        )

    def test_spot_loads_while_assembler_still_has_cargo(self) -> None:
        self.assertEqual(
            next_spot_mode(
                pending=True,
                loaded=False,
                has_free_slot=False,
                at_yard=True,
                at_drop=False,
            ),
            "loading",
        )

    def test_spot_delivers_to_a_free_slot_when_some_are_occupied(self) -> None:
        self.assertEqual(
            next_spot_mode(
                pending=True,
                loaded=True,
                has_free_slot=True,
                at_yard=False,
                at_drop=False,
            ),
            "to_drop",
        )

    def test_spot_waits_only_when_every_slot_is_full(self) -> None:
        self.assertEqual(
            next_spot_mode(
                pending=True,
                loaded=True,
                has_free_slot=False,
                at_yard=False,
                at_drop=False,
            ),
            "wait_drop",
        )

    def test_spot_waits_at_the_truck_until_staging_is_empty(self) -> None:
        self.assertEqual(
            next_spot_mode(
                pending=True,
                loaded=False,
                has_free_slot=False,
                at_yard=True,
                at_drop=False,
                drain_hold=True,
            ),
            "idle",
        )
        self.assertEqual(
            next_spot_mode(
                pending=True,
                loaded=True,
                has_free_slot=False,
                at_yard=False,
                at_drop=False,
                drain_hold=True,
            ),
            "to_yard",
        )

    def test_assembler_picks_when_piece_is_waiting(self) -> None:
        self.assertEqual(
            next_asm_mode(cargo_at_drop=True, holding=False, erecting=False),
            "to_drop",
        )

    def test_assembler_keeps_walking_after_claiming_a_slot(self) -> None:
        self.assertEqual(
            next_asm_mode(
                cargo_at_drop=False,
                holding=False,
                erecting=False,
                assigned=True,
            ),
            "to_drop",
        )

    def test_assembler_lifts_before_carrying_to_socket(self) -> None:
        self.assertEqual(
            next_asm_mode(
                cargo_at_drop=True,
                holding=False,
                erecting=False,
                picking=True,
            ),
            "pickup",
        )
        self.assertEqual(
            next_asm_mode(cargo_at_drop=False, holding=True, erecting=False),
            "to_socket",
        )

    def test_keepout_during_lift_keeps_the_member_attached(self) -> None:
        self.assertEqual(
            evac_cargo_action(picking=True, holding=False, erecting=False),
            "carry",
        )
        self.assertEqual(
            evac_cargo_action(picking=False, holding=True, erecting=False),
            "carry",
        )
        self.assertEqual(
            evac_cargo_action(picking=False, holding=False, erecting=True),
            "carry",
        )
        self.assertEqual(
            evac_cargo_action(picking=False, holding=False, erecting=False),
            "none",
        )

    def test_refuge_hold_waits_for_spot_to_leave_scaffold(self) -> None:
        self.assertTrue(
            assembler_holds_refuge(
                refuge_waived=True, spot_on_scaffold=True, spot_delivering=True
            )
        )
        self.assertFalse(
            assembler_holds_refuge(
                refuge_waived=True, spot_on_scaffold=False, spot_delivering=True
            )
        )
        self.assertFalse(
            assembler_holds_refuge(
                refuge_waived=True, spot_on_scaffold=True, spot_delivering=False
            )
        )
        self.assertEqual(
            next_asm_mode(
                cargo_at_drop=True,
                holding=False,
                erecting=False,
                refuge_hold=True,
            ),
            "to_refuge",
        )

    def test_assembler_waits_while_spot_is_on_the_staging_slot(self) -> None:
        self.assertTrue(
            spot_blocks_staging_handoff(
                spot_at_drop=True,
                spot_placing=True,
                spot_loaded_en_route=False,
            )
        )
        self.assertEqual(
            next_asm_mode(
                cargo_at_drop=False,
                holding=False,
                erecting=False,
            ),
            "idle",
        )

    def test_spot_on_scaffold_past_yard_ramp(self) -> None:
        self.assertFalse(spot_on_scaffold_local_x(local_x_m=0.0, ramp_yard_x_m=2.0))
        self.assertTrue(spot_on_scaffold_local_x(local_x_m=2.0, ramp_yard_x_m=2.0))

    def test_spot_waits_at_yard_while_assembler_owns_a_member(self) -> None:
        self.assertEqual(
            next_spot_mode(
                pending=True,
                loaded=True,
                has_free_slot=True,
                at_yard=True,
                at_drop=False,
                assembler_busy=True,
            ),
            "idle",
        )
        self.assertEqual(
            next_spot_mode(
                pending=True,
                loaded=False,
                has_free_slot=True,
                at_yard=True,
                at_drop=False,
                assembler_busy=True,
            ),
            "idle",
        )
        self.assertEqual(
            next_spot_mode(
                pending=True,
                loaded=True,
                has_free_slot=True,
                at_yard=False,
                at_drop=False,
                assembler_busy=True,
            ),
            "to_yard",
        )

    def test_holding_yields_to_locked_spot_but_dwells_do_not(self) -> None:
        self.assertTrue(
            assembler_should_yield_to_locked_spot(
                spot_locked=True, erecting=False, picking=False
            )
        )
        self.assertFalse(
            assembler_should_yield_to_locked_spot(
                spot_locked=True, erecting=True, picking=False
            )
        )
        self.assertEqual(
            next_asm_mode(
                cargo_at_drop=False,
                holding=True,
                erecting=False,
                evac=True,
            ),
            "to_refuge",
        )

    def test_assembler_finishes_erect_before_retreat(self) -> None:
        self.assertEqual(
            next_asm_mode(
                cargo_at_drop=True,
                holding=True,
                erecting=True,
                assigned=True,
                evac=True,
            ),
            "erect",
        )
        self.assertEqual(
            next_asm_mode(
                cargo_at_drop=True,
                holding=False,
                erecting=False,
                assigned=True,
                evac=True,
                picking=True,
            ),
            "pickup",
        )
        self.assertEqual(
            next_asm_mode(
                cargo_at_drop=False,
                holding=False,
                erecting=False,
                evac=False,
            ),
            "idle",
        )

    def test_yaw_error_wraps_across_180(self) -> None:
        self.assertAlmostEqual(yaw_error_deg(170.0, -170.0), 20.0, places=4)
        self.assertAlmostEqual(yaw_error_deg(0.0, 0.0), 0.0, places=4)

    def test_gait_slice_covers_a_straight_leg(self) -> None:
        self.assertGreaterEqual(drive_duration_s(8.0, 1.0, cap_s=2.4, min_s=0.40), 2.0)
        self.assertLessEqual(drive_duration_s(8.0, 1.0, cap_s=2.4, min_s=0.40), 2.4)

    def test_gait_command_uses_goal_range_when_the_hop_is_aligned(self) -> None:
        self.assertGreater(
            gait_command_remain_cm(
                hop_remain_cm=40.0, goal_remain_cm=800.0, hop_yaw_err_deg=5.0
            ),
            700.0,
        )
        self.assertEqual(
            gait_command_remain_cm(
                hop_remain_cm=40.0, goal_remain_cm=800.0, hop_yaw_err_deg=90.0
            ),
            40.0,
        )

    def test_move_speed_until_is_wall_clock_under_slomo(self) -> None:
        self.assertAlmostEqual(wall_from_sim(0.45, 4.0), 0.1125, places=4)

    def test_assist_step_when_gait_does_not_translate(self) -> None:
        self.assertGreater(
            assist_step_cm(progressed_cm=1.0, expected_cm=40.0, remain_cm=200.0),
            20.0,
        )
        self.assertEqual(
            assist_step_cm(progressed_cm=30.0, expected_cm=40.0, remain_cm=200.0),
            0.0,
        )

    def test_hold_in_engine_gait_until_it_expires(self) -> None:
        self.assertFalse(should_reissue_drive(now=1.0, cmd_until=2.0))
        self.assertTrue(should_reissue_drive(now=2.0, cmd_until=2.0))
        self.assertTrue(should_reissue_drive(now=2.1, cmd_until=2.0))

    def test_gait_must_close_on_the_goal_not_just_translate(self) -> None:
        self.assertFalse(
            gait_closing_on_goal(remain_cm=500.0, remain0_cm=400.0, moved_cm=80.0)
        )
        self.assertTrue(
            gait_closing_on_goal(remain_cm=300.0, remain0_cm=400.0, moved_cm=80.0)
        )
        self.assertFalse(
            gait_closing_on_goal(remain_cm=390.0, remain0_cm=400.0, moved_cm=10.0)
        )

    def test_assembler_far_from_goal_is_lost(self) -> None:
        self.assertTrue(assembler_is_lost(remain_cm=4000.0))
        self.assertFalse(assembler_is_lost(remain_cm=400.0))
        self.assertTrue(assembler_is_lost(remain_cm=float("nan")))

    def test_assembler_high_above_the_work_plane_is_airborne(self) -> None:
        self.assertTrue(assembler_is_airborne(z_cm=800.0, goal_z_cm=90.0))
        self.assertFalse(assembler_is_airborne(z_cm=95.0, goal_z_cm=90.0))
        self.assertTrue(assembler_is_airborne(z_cm=float("nan"), goal_z_cm=90.0))

    def test_assembler_arrives_on_xy_even_if_hip_and_feet_differ(self) -> None:
        # Old Z gate (80 cm) blocked pickup/install when the pawn origin is feet
        # and the goal was hip (+90 cm). Staging then filled and Spot idled.
        self.assertTrue(
            pawn_arrived(horiz_m=0.20, same_floor=True, horiz_tol_m=0.50)
        )
        self.assertFalse(
            pawn_arrived(horiz_m=0.20, same_floor=False, horiz_tol_m=0.50)
        )
        self.assertFalse(
            pawn_arrived(horiz_m=0.80, same_floor=True, horiz_tol_m=0.50)
        )

    def test_four_x_playback_keeps_sim_seconds_and_shortens_wall(self) -> None:
        self.assertAlmostEqual(wall_from_sim(10.0, 4.0), 2.5)
        self.assertAlmostEqual(wall_from_sim(0.16, 4.0), 0.04)
        self.assertAlmostEqual(wall_from_sim(10.0, 1.0), 10.0)
        self.assertAlmostEqual(
            wall_tick_remainder_s(spent_s=0.01, sim_dt=0.16, sim_rate=4.0), 0.03
        )
        self.assertEqual(
            wall_tick_remainder_s(spent_s=0.05, sim_dt=0.16, sim_rate=4.0), 0.0
        )
        self.assertAlmostEqual(playback_sim_dt_s(0.04, 4.0), 0.16)
        self.assertAlmostEqual(playback_sim_dt_s(0.16, 4.0), 0.64)
        self.assertAlmostEqual(playback_sim_dt_s(0.16, 1.0), 0.16)
        outbound = playback_step_cm(
            speed_cm_s=100.0, wall_dt_s=0.04, sim_rate=4.0, remain_cm=1000.0
        )
        returning = playback_step_cm(
            speed_cm_s=100.0, wall_dt_s=0.04, sim_rate=4.0, remain_cm=800.0
        )
        self.assertAlmostEqual(outbound, 16.0)
        self.assertAlmostEqual(returning, 16.0)

    def test_grounded_root_keeps_height_when_nav_misses(self) -> None:
        z = grounded_root_z_cm(
            surface_z_cm=None, clearance_cm=50.0, prev_root_z_cm=6490.0
        )
        self.assertAlmostEqual(z, 6490.0)

    def test_grounded_root_sits_on_the_measured_surface(self) -> None:
        z = grounded_root_z_cm(
            surface_z_cm=6440.0, clearance_cm=50.0, prev_root_z_cm=None
        )
        self.assertAlmostEqual(z, 6490.0)

    def test_grounded_root_does_not_bob_to_a_far_floor(self) -> None:
        z = grounded_root_z_cm(
            surface_z_cm=6620.0,
            clearance_cm=50.0,
            prev_root_z_cm=6490.0,
            max_step_cm=22.0,
        )
        self.assertAlmostEqual(z, 6512.0)

    def test_nav_projection_rejects_sideways_snaps(self) -> None:
        self.assertTrue(
            nav_projection_is_usable(query_xy=(0.0, 0.0), projected_xy=(10.0, 10.0))
        )
        self.assertFalse(
            nav_projection_is_usable(query_xy=(0.0, 0.0), projected_xy=(200.0, 0.0))
        )


if __name__ == "__main__":
    unittest.main()
