"""Spot / Assembler pipeline intents (no UE).

Spot fills floor staging (up to staging_slots). When the line is full it
waits at the kei truck until the Assembler has emptied every slot, then
starts the next batch. The Assembler picks FIFO and installs with a dwell.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple


def next_spot_mode(
    *,
    pending: bool,
    loaded: bool,
    has_free_slot: bool,
    at_yard: bool,
    at_drop: bool,
    drain_hold: bool = False,
    assembler_busy: bool = False,
) -> str:
    """Return one of: to_yard, loading, to_drop, wait_drop, placing, idle.

    ``drain_hold``: staging is (or was) full; wait at the truck until empty.
    ``wait_drop`` is also at the truck, not on the scaffold standoff.
    ``assembler_busy``: Assembler still owns a member (pickup/carry/erect). Spot
    must not drive onto the scaffold — keep-out would bounce the Assembler into
    refuge forever while Spot fills every staging slot.
    """
    if drain_hold:
        if loaded and at_drop and has_free_slot:
            return "placing"
        if at_yard:
            return "idle"
        return "to_yard"
    if assembler_busy:
        # Do not fetch/deliver while Assembler still owns a member.
        if at_yard:
            return "idle"
        return "to_yard"
    if loaded:
        if at_drop and has_free_slot:
            return "placing"
        if not has_free_slot:
            return "wait_drop"
        return "to_drop"
    if at_yard and pending:
        return "loading"
    if pending or loaded:
        return "to_yard"
    return "idle"


def spot_on_scaffold_local_x(*, local_x_m: float, ramp_yard_x_m: float, margin_m: float = 0.5) -> bool:
    """True when the pawn is on the scaffold deck (past the yard ramp threshold)."""
    return float(local_x_m) >= float(ramp_yard_x_m) - float(margin_m)


def assembler_holds_refuge(
    *,
    refuge_waived: bool,
    spot_on_scaffold: bool,
    spot_delivering: bool = True,
) -> bool:
    """Stay at refuge only while Spot is still delivering on the scaffold.

    Once Spot returns to the yard / waits for staging drain, the Assembler must
    leave refuge and empty the slots — otherwise both deadlock.
    """
    return bool(refuge_waived and spot_on_scaffold and spot_delivering)


def spot_is_delivering(intent: str) -> bool:
    return intent in ("to_drop", "placing")


def spot_blocks_staging_handoff(
    *,
    spot_at_drop: bool,
    spot_placing: bool,
    spot_loaded_en_route: bool,
) -> bool:
    """Do not walk the Assembler onto the slot while Spot is placing or standing on it."""
    return bool(spot_at_drop and (spot_placing or spot_loaded_en_route))


def assembler_should_yield_to_locked_spot(
    *,
    spot_locked: bool,
    erecting: bool,
    picking: bool,
) -> bool:
    """When Spot is SI/dmin-stopped, Assembler yields unless finishing a dwell."""
    return bool(spot_locked and not erecting and not picking)


def next_asm_mode(
    *,
    cargo_at_drop: bool,
    holding: bool,
    erecting: bool,
    assigned: bool = False,
    evac: bool = False,
    picking: bool = False,
    refuge_hold: bool = False,
) -> Optional[str]:
    """Return to_refuge / pickup / to_drop / to_socket / erect / idle.

    ``evac`` requests refuge only while the Assembler is not in a dwell task.
    ``assigned`` stays true while walking to a claimed slot even after the
    Boolean cargo flag is cleared.
    """
    if erecting:
        return "erect"
    if picking:
        return "pickup"
    if refuge_hold or evac:
        return "to_refuge"
    if holding:
        return "to_socket"
    if assigned or cargo_at_drop:
        return "to_drop"
    return "idle"


def evac_cargo_action(
    *,
    picking: bool,
    holding: bool,
    erecting: bool,
) -> str:
    """Keep-out must not leave a lifted member hovering in the walk path.

    ``restage``: lift dwell already raised the cube; put it back on the slot.
    ``carry``: the Assembler already has it in hand; keep it attached.
    ``none``: no claimed member.
    """
    if erecting or holding:
        return "carry"
    if picking:
        return "carry"
    return "none"


def wall_from_sim(sim_s: float, sim_rate: float) -> float:
    """Wall seconds for a simulation interval at playback rate N."""
    return float(sim_s) / max(float(sim_rate), 1e-6)


def wall_tick_remainder_s(
    *, spent_s: float, sim_dt: float, sim_rate: float
) -> float:
    """Sleep leftover so one sim tick occupies sim_dt / sim_rate wall seconds."""
    return max(0.0, wall_from_sim(sim_dt, sim_rate) - float(spent_s))


MAX_PLAYBACK_WALL_DT_S = 0.25


def playback_sim_dt_s(
    wall_dt_s: float,
    sim_rate: float,
    *,
    cap_s: float = MAX_PLAYBACK_WALL_DT_S,
) -> float:
    """Simulation seconds advanced during a wall interval at playback rate N."""
    return min(max(0.0, float(wall_dt_s)), float(cap_s)) * max(float(sim_rate), 1e-6)


def playback_step_cm(
    *,
    speed_cm_s: float,
    wall_dt_s: float,
    sim_rate: float,
    remain_cm: float,
) -> float:
    """1x speed * wall * sim_rate: same outbound/return, 4x is fast-forward of 1x."""
    return min(
        max(0.0, float(speed_cm_s)) * playback_sim_dt_s(wall_dt_s, sim_rate),
        max(0.0, float(remain_cm)),
    )


def yaw_error_deg(current_deg: float, desired_deg: float) -> float:
    return (desired_deg - current_deg + 180.0) % 360.0 - 180.0


def drive_duration_s(
    dist_m: float,
    speed_mps: float,
    *,
    cap_s: float = 1.2,
    min_s: float = 0.30,
) -> float:
    if speed_mps <= 1e-6 or dist_m <= 0.0:
        return 0.0
    return min(cap_s, max(min_s, dist_m / speed_mps))


def should_reissue_drive(*, now: float, cmd_until: float) -> bool:
    """True when the in-engine Move_Speed timer finished.

    Do not reissue for heading error while a command is live: a new Move_Speed
    or set_orientation cancels the BP timer and Spot walks in place.
    ``cmd_until`` must be wall-clock (use ``wall_from_sim`` on the BP duration).
    """
    return now >= cmd_until


def gait_command_remain_cm(
    *,
    hop_remain_cm: float,
    goal_remain_cm: float,
    hop_yaw_err_deg: float,
    align_deg: float = 25.0,
) -> float:
    """Long Move_Speed along a straight leg; short only when the hop turns."""
    if abs(float(hop_yaw_err_deg)) <= float(align_deg):
        return max(float(hop_remain_cm), float(goal_remain_cm))
    return float(hop_remain_cm)


def assist_step_cm(
    *,
    progressed_cm: float,
    expected_cm: float,
    remain_cm: float,
    min_frac: float = 0.25,
) -> float:
    """XY assist when BP Move_Speed plays the gait but does not translate."""
    if expected_cm <= 2.0 or remain_cm <= 0.5:
        return 0.0
    if progressed_cm >= min_frac * expected_cm:
        return 0.0
    return min(remain_cm, expected_cm)


def gait_closing_on_goal(
    *,
    remain_cm: float,
    remain0_cm: float,
    moved_cm: float,
    min_moved_cm: float = 25.0,
) -> bool:
    """True only if engine gait both translates and shortens remaining range."""
    if moved_cm < min_moved_cm:
        return False
    return remain_cm <= remain0_cm - 0.25 * moved_cm


def assembler_is_lost(*, remain_cm: float, max_cm: float = 2500.0) -> bool:
    """Off the work site (fallen / wandered) — snap back."""
    return remain_cm != remain_cm or remain_cm > max_cm


def pawn_arrived(
    *,
    horiz_m: float,
    same_floor: bool,
    horiz_tol_m: float = 0.50,
) -> bool:
    """Arrive on XY + floor. Hip-vs-foot Z must not block the pipeline."""
    return float(horiz_m) < float(horiz_tol_m) and bool(same_floor)


def assembler_is_airborne(
    *, z_cm: float, goal_z_cm: float, max_cm: float = 150.0
) -> bool:
    """Launched by CharacterMovement overlap — snap hip back to the work plane."""
    return z_cm != z_cm or abs(z_cm - goal_z_cm) > max_cm


def nav_projection_is_usable(
    *,
    query_xy: Tuple[float, float],
    projected_xy: Tuple[float, float],
    max_drift_cm: float = 80.0,
) -> bool:
    """Reject NavProject snaps that pulled the pawn onto another polygon."""
    return math.hypot(
        float(projected_xy[0]) - float(query_xy[0]),
        float(projected_xy[1]) - float(query_xy[1]),
    ) <= float(max_drift_cm)


def grounded_root_z_cm(
    *,
    surface_z_cm: Optional[float],
    clearance_cm: float,
    prev_root_z_cm: Optional[float],
    max_step_cm: float = 22.0,
) -> float:
    """Sit on a measured walk surface. Do not snap to a kinematic floor band.

    ``surface_z_cm`` is NavMesh / ground. Missing samples keep the last root
    height so the pawn does not bob when a stair model is guessed off the mesh.
    """
    if surface_z_cm is None:
        if prev_root_z_cm is not None:
            return float(prev_root_z_cm)
        return float(clearance_cm)
    target = float(surface_z_cm) + float(clearance_cm)
    if prev_root_z_cm is None:
        return target
    prev = float(prev_root_z_cm)
    delta = target - prev
    cap = max(0.0, float(max_step_cm))
    if abs(delta) <= cap:
        return target
    return prev + (cap if delta > 0.0 else -cap)
