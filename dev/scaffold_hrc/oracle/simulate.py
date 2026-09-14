"""Headless 3F erection oracle. Corridor unconstrained; scaffold uses θ.

Story: a worker hands cargo to Spot's arm at the truck (timed), Spot places it
on the deck (timed), then the assembler Humanoid attaches it by hand (timed).

ISO/TS 15066 SSM and d_min apply only in scaffolding space, until the
human reaches that floor's refuge. After that, Spot may ignore keep-out
until it leaves the scaffold. Stair hops use v_max * stair_speed_factor.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from constraints.pareto import Theta
from oracle.ssm import SSM_STOPPED_MPS, apply_ssm, protective_separation_m, safety_index
from scene.mission_protocol import STAGE1_PROTOCOL  # noqa: E402
from scene.field import (
    RAMP_DECK_X_M,
    scaffold_edge_x_m,
    stair_lane_y_m,
    stair_run_ends,
)
from scene.geometry import ScaffoldGeom
from scene.scaffold_grammar import ScaffoldSpec, Socket, build_scaffold

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class TraceSample:
    t_s: float
    spot: Vec3
    human: Vec3
    sep_m: float
    spot_speed_mps: float
    blocked: bool
    in_corridor: bool
    n_filled: int
    current_floor: int
    violating: bool
    si: float = 1.0
    sp_m: float = 0.0
    ssm_mode: str = "free"
    unsafe: bool = False
    retreating: bool = False


@dataclass(frozen=True)
class OracleConfig:
    dt_s: float = 0.1
    human_speed_mps: float = STAGE1_PROTOCOL.human_speed_mps
    human_retreat_mps: float = STAGE1_PROTOCOL.human_retreat_mps
    timeout_s: float = 7200.0
    handoff_spot_m: float = 0.50
    handoff_human_m: float = 0.50
    collision_m: float = 0.40
    d_safe_m: float = 1.00
    erect_s: float = STAGE1_PROTOCOL.erect_s
    truck_load_s: float = STAGE1_PROTOCOL.truck_load_s
    drop_place_s: float = STAGE1_PROTOCOL.drop_place_s
    assembler_pickup_s: float = STAGE1_PROTOCOL.assembler_pickup_s
    stair_speed_factor: float = 0.5
    sockets_per_floor: Optional[int] = None
    drop_x_m: float = 2.0
    standoff_x_m: float = 6.0
    refuge_x_m: float = 9.0
    record_trace: bool = True
    # Yard handoff is timed and unconstrained; only the deck Assembler
    # (this `human` pose) is under Sp / d_min on scaffold.
    yard_keepout: bool = STAGE1_PROTOCOL.yard_keepout


@dataclass(frozen=True)
class OracleResult:
    completed: bool
    makespan_s: float
    path_length_m: float
    corridor_time_s: float
    min_separation_m: float  # closest S while keep-out is on and Spot is moving
    wait_s: float
    violation_s: float
    n_filled: int
    n_sockets: int
    floors_completed: int
    timeout_s: float
    trace: Tuple[TraceSample, ...] = ()
    ssm_s: float = 0.0
    si_min: float = 1.0
    scaffold_safe_s: float = 0.0
    scaffold_unsafe_s: float = 0.0
    arm_load_s: float = 0.0
    arm_place_s: float = 0.0
    scaffold_time_s: float = 0.0


def _horiz(a: Vec3, b: Vec3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _dist3(a: Vec3, b: Vec3) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _on_scaffold(x_m: float, geom: ScaffoldGeom) -> bool:
    _ = geom
    return x_m >= scaffold_edge_x_m() - 1e-6


def _same_level(a: Vec3, b: Vec3, *, tol_m: float = 0.40) -> bool:
    return abs(a[2] - b[2]) < tol_m


def _floor_of_z(geom: ScaffoldGeom, z_m: float) -> int:
    # Bias upward so mid-stair agents count as the floor they are entering.
    return max(
        1,
        min(
            geom.n_floors,
            1 + int(math.floor((z_m + 0.55 * geom.lift_m) / geom.lift_m)),
        ),
    )


def _move_toward(src: Vec3, dst: Vec3, speed: float, dt: float) -> Tuple[Vec3, float]:
    dx, dy, dz = dst[0] - src[0], dst[1] - src[1], dst[2] - src[2]
    span = math.sqrt(dx * dx + dy * dy + dz * dz)
    step = speed * dt
    if span <= 1e-9:
        return dst, 0.0
    if step >= span:
        return dst, span
    t = step / span
    nxt: Vec3 = (src[0] + t * dx, src[1] + t * dy, src[2] + t * dz)
    return nxt, step


def _pose(x_m: float, geom: ScaffoldGeom, floor: int) -> Vec3:
    return (x_m, geom.deck_width_m * 0.5, geom.floor_z_m(floor))


def _limit_sockets(spec: ScaffoldSpec, per_floor: Optional[int]) -> ScaffoldSpec:
    if per_floor is None:
        return spec
    limited = spec
    for floor in range(1, 1 + max((s.floor for s in spec.sockets), default=0)):
        kept = 0
        for socket in spec.sockets_on_floor(floor):
            if kept < per_floor:
                kept += 1
            else:
                limited = limited.with_placed(socket.socket_id)
    return limited


def _next_hop(geom: ScaffoldGeom, src: Vec3, dest: Vec3) -> Vec3:
    """Waypoints mirror UE zigzag Recast runs (L0 south, L1 north)."""
    arrive = 0.45
    mid_y = geom.deck_width_m * 0.5
    edge = scaffold_edge_x_m()
    src_f = _floor_of_z(geom, src[2])
    dest_f = _floor_of_z(geom, dest[2])
    if src_f != dest_f:
        going_up = dest_f > src_f
        lift = (src_f - 1) if going_up else (src_f - 2)
        lift = max(0, min(geom.n_floors - 2, lift))
        y_lane = stair_lane_y_m(lift, geom.deck_width_m)
        start_x, end_x = stair_run_ends(lift)
        if not going_up:
            start_x, end_x = end_x, start_x
        z_lo = geom.floor_z_m(src_f)
        z_hi = geom.floor_z_m(src_f + (1 if going_up else -1))
        run = end_x - start_x
        if abs(run) < 1e-9:
            progress = 1.0 if abs(src[2] - z_hi) < 0.25 else 0.0
        else:
            progress = (src[0] - start_x) / run
        on_lane = abs(src[1] - y_lane) <= arrive
        if (not on_lane) or progress < -0.05:
            return (start_x, y_lane, z_lo)
        if progress < 0.98 or abs(src[2] - z_hi) > 0.30:
            return (end_x, y_lane, z_hi)
        return dest
    if _on_scaffold(src[0], geom) and not _on_scaffold(dest[0], geom):
        gate: Vec3 = (edge - 0.05, mid_y, src[2])
        if src[0] > gate[0] + 0.05:
            return gate
        return dest
    if (not _on_scaffold(src[0], geom)) and _on_scaffold(dest[0], geom):
        if src_f == 1 and abs(src[2] - geom.floor_z_m(1)) < 0.40:
            gate = (edge, stair_lane_y_m(0, geom.deck_width_m), geom.floor_z_m(1))
        else:
            gate = (RAMP_DECK_X_M, mid_y, src[2])
        if src[0] < edge - 0.05:
            return gate
        return dest
    return dest


def _count_filled(spec: ScaffoldSpec) -> int:
    return sum(1 for s in spec.sockets if s.filled)


def _floor_done(spec: ScaffoldSpec, floor: int) -> bool:
    sockets = spec.sockets_on_floor(floor)
    return bool(sockets) and all(s.filled for s in sockets)


def _cmd_speed(
    *,
    in_corridor: bool,
    constraint_active: bool,
    vmax_mps: float,
    stair_hop: bool,
    stair_speed_factor: float,
) -> float:
    _ = (in_corridor, constraint_active, stair_hop, stair_speed_factor)
    return float(vmax_mps)


def _keepout_violated(sep_m: float, cmd_speed: float, dmin_m: float) -> bool:
    return sep_m < protective_separation_m(cmd_speed) or sep_m < dmin_m


def run_erection(
    *,
    geom: ScaffoldGeom,
    theta: Theta,
    config: OracleConfig,
    constraint_active: bool = True,
) -> OracleResult:
    spec = _limit_sockets(build_scaffold(geom), config.sockets_per_floor)
    n_sockets = sum(1 for s in spec.sockets if not s.filled) + _count_filled(spec)
    n_work = sum(1 for s in spec.sockets if not s.filled)
    store: Vec3 = (*geom.storage_xy(), 0.0)
    spot = store
    human = _pose(config.standoff_x_m, geom, 1)
    current_floor = 1
    spot_max_floor = 1
    spot_loaded = False
    human_loaded = False
    cargo_at_drop = False
    reserved: Optional[Socket] = None
    erect_timer = 0.0
    load_timer = 0.0
    place_timer = 0.0
    pickup_timer = 0.0
    t = 0.0
    wait_s = 0.0
    path_m = 0.0
    corridor_time_s = 0.0
    scaffold_safe_s = 0.0
    scaffold_unsafe_s = 0.0
    arm_load_s = 0.0
    arm_place_s = 0.0
    min_sep_enforced = 1e9
    min_sep_onsite = 1e9
    violation_s = 0.0
    si_min = 1e9
    floors_completed = 0
    evac_requested = False
    keepout_waived = False
    trace: List[TraceSample] = []
    scaffold_time_s = 0.0

    while t < config.timeout_s:
        drop = _pose(config.drop_x_m, geom, current_floor)
        refuge = _pose(config.refuge_x_m, geom, current_floor)
        unfilled = sum(1 for s in spec.sockets_on_floor(current_floor) if not s.filled)
        pipeline = int(spot_loaded) + int(cargo_at_drop) + int(human_loaded)
        need_fetch = unfilled > pipeline
        floor_clear = unfilled == 0 and not human_loaded and not cargo_at_drop
        next_sock = spec.next_empty_socket(current_floor)
        spot_on_floor_scaffold = (
            _on_scaffold(spot[0], geom) and _same_level(spot, drop)
        )
        if not _on_scaffold(spot[0], geom):
            keepout_waived = False
            evac_requested = False

        if evac_requested or keepout_waived:
            human_dest = refuge
        elif human_loaded:
            if reserved is None:
                reserved = next_sock
            human_dest = (
                (reserved.x_m, reserved.y_m, reserved.z_m) if reserved is not None else refuge
            )
        elif cargo_at_drop and not spot_on_floor_scaffold:
            human_dest = drop
        elif floor_clear and current_floor < geom.n_floors:
            human_dest = _pose(config.refuge_x_m, geom, current_floor + 1)
        elif next_sock is not None:
            human_dest = (next_sock.x_m, next_sock.y_m, next_sock.z_m)
        else:
            human_dest = refuge

        if spot_loaded and current_floor <= spot_max_floor and not cargo_at_drop:
            spot_dest = drop
        elif need_fetch and not spot_loaded:
            spot_dest = store
        else:
            spot_dest = store

        in_corridor = not _on_scaffold(spot[0], geom)
        hop = _next_hop(geom, spot, spot_dest)
        stair_hop = abs(hop[2] - spot[2]) > 1e-6
        cmd_speed = _cmd_speed(
            in_corridor=in_corridor,
            constraint_active=constraint_active,
            vmax_mps=theta.vmax_mps,
            stair_hop=stair_hop,
            stair_speed_factor=config.stair_speed_factor,
        )
        sep = _horiz(spot, human)
        on_site = (
            _on_scaffold(spot[0], geom)
            and _on_scaffold(human[0], geom)
            and _same_level(spot, human)
        )
        keepout_enforced = constraint_active and not keepout_waived
        if keepout_enforced:
            speed, ssm_mode = apply_ssm(sep, cmd_speed, on_site=on_site)
        else:
            speed, ssm_mode = cmd_speed, "free"
        trial, _ = _move_toward(spot, hop, speed, config.dt_s)
        dmin_block = keepout_enforced and on_site and sep < theta.dmin_m
        ssm_stop = ssm_mode == "stop"
        blocked = dmin_block or ssm_stop
        if blocked:
            evac_requested = True
            wait_s += config.dt_s
            nxt, moved = spot, 0.0
            speed = 0.0
        else:
            nxt, moved = trial, _dist3(spot, trial)
        if evac_requested or keepout_waived:
            human_dest = refuge
        if keepout_enforced and on_site and speed > SSM_STOPPED_MPS:
            si_cmd = safety_index(sep, protective_separation_m(speed))
            si_min = min(si_min, si_cmd)

        retreating = evac_requested and not keepout_waived
        human_hop = _next_hop(geom, human, human_dest)
        human, _ = _move_toward(human, human_hop, config.human_speed_mps, config.dt_s)
        if evac_requested and _horiz(human, refuge) < 0.50 and _same_level(human, refuge):
            keepout_waived = True
            evac_requested = False

        if (not spot_loaded) and need_fetch and _dist3(nxt, store) < 0.40:
            load_timer += config.dt_s
            arm_load_s += config.dt_s
            if load_timer >= config.truck_load_s:
                spot_loaded = True
                load_timer = 0.0
        else:
            load_timer = 0.0

        at_drop = _horiz(nxt, drop) < config.handoff_spot_m and _same_level(nxt, drop)
        human_at_drop = _horiz(human, drop) < theta.dmin_m and _same_level(human, drop)
        can_place = (not keepout_enforced) or not (human_at_drop or blocked)
        if spot_loaded and at_drop and not cargo_at_drop and can_place:
            place_timer += config.dt_s
            arm_place_s += config.dt_s
            if place_timer >= config.drop_place_s:
                spot_loaded = False
                cargo_at_drop = True
                place_timer = 0.0
        elif not (spot_loaded and at_drop and not cargo_at_drop):
            place_timer = 0.0

        if human_loaded and reserved is not None:
            goal = (reserved.x_m, reserved.y_m, reserved.z_m)
            if _dist3(human, goal) < 0.35:
                erect_timer += config.dt_s
                if erect_timer >= config.erect_s:
                    spec = spec.with_placed(reserved.socket_id)
                    human_loaded = False
                    reserved = None
                    erect_timer = 0.0
            else:
                erect_timer = 0.0
        elif cargo_at_drop and _horiz(human, drop) < config.handoff_human_m and _same_level(human, drop):
            pickup_timer += config.dt_s
            if pickup_timer >= config.assembler_pickup_s:
                human_loaded = True
                cargo_at_drop = False
                reserved = spec.next_empty_socket(current_floor)
                erect_timer = 0.0
                pickup_timer = 0.0
        else:
            pickup_timer = 0.0

        if _floor_done(spec, current_floor):
            floors_completed = max(floors_completed, current_floor)
        if (
            current_floor < geom.n_floors
            and _floor_done(spec, current_floor)
            and not human_loaded
            and not cargo_at_drop
            and abs(human[2] - geom.floor_z_m(current_floor + 1)) < 0.25
            and _on_scaffold(human[0], geom)
        ):
            current_floor += 1
            spot_max_floor = current_floor

        spot = nxt
        path_m += moved
        sep = _horiz(spot, human)
        on_site = (
            _on_scaffold(spot[0], geom)
            and _on_scaffold(human[0], geom)
            and _same_level(spot, human)
        )
        realized_speed = 0.0 if blocked else speed
        sp_m = protective_separation_m(cmd_speed)
        unsafe = keepout_enforced and on_site and _keepout_violated(
            sep, cmd_speed, theta.dmin_m
        )
        in_corridor_now = not _on_scaffold(spot[0], geom)
        if in_corridor_now:
            corridor_time_s += config.dt_s
        else:
            scaffold_time_s += config.dt_s
            if unsafe:
                scaffold_unsafe_s += config.dt_s
            else:
                scaffold_safe_s += config.dt_s
        si = safety_index(sep, protective_separation_m(realized_speed)) if on_site else float("inf")
        violating = keepout_enforced and on_site and si < 1.0 and realized_speed > SSM_STOPPED_MPS
        if on_site and realized_speed > SSM_STOPPED_MPS:
            min_sep_onsite = min(min_sep_onsite, sep)
        if keepout_enforced and on_site and realized_speed > SSM_STOPPED_MPS:
            min_sep_enforced = min(min_sep_enforced, sep)
        if violating:
            violation_s += config.dt_s
        t += config.dt_s
        filled_now = sum(1 for s in spec.sockets if s.filled) - (n_sockets - n_work)
        if config.record_trace:
            trace.append(
                TraceSample(
                    t_s=t,
                    spot=spot,
                    human=human,
                    sep_m=sep,
                    spot_speed_mps=realized_speed,
                    blocked=blocked,
                    in_corridor=in_corridor_now,
                    n_filled=max(0, filled_now),
                    current_floor=current_floor,
                    violating=violating,
                    si=1.0 if si == float("inf") else si,
                    sp_m=sp_m,
                    ssm_mode=ssm_mode,
                    unsafe=unsafe,
                    retreating=retreating,
                )
            )
        if floors_completed >= geom.n_floors and not human_loaded:
            break

    if min_sep_enforced < 1e8:
        min_sep = min_sep_enforced
    elif min_sep_onsite < 1e8:
        min_sep = min_sep_onsite
    else:
        min_sep = _horiz(spot, human)
    if si_min > 1e8:
        si_min = 1.0
    n_filled = max(0, sum(1 for s in spec.sockets if s.filled) - (n_sockets - n_work))
    return OracleResult(
        completed=floors_completed >= geom.n_floors,
        makespan_s=t,
        path_length_m=path_m,
        corridor_time_s=corridor_time_s,
        min_separation_m=min_sep,
        wait_s=wait_s,
        violation_s=violation_s,
        n_filled=n_filled,
        n_sockets=n_work,
        floors_completed=floors_completed,
        timeout_s=config.timeout_s,
        trace=tuple(trace),
        ssm_s=scaffold_unsafe_s,
        si_min=si_min,
        scaffold_safe_s=scaffold_safe_s,
        scaffold_unsafe_s=scaffold_unsafe_s,
        arm_load_s=arm_load_s,
        arm_place_s=arm_place_s,
        scaffold_time_s=scaffold_time_s,
    )
