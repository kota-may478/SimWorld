"""Bare-ground erection oracle: Spot + Assembler place every member in order.

Starts with an empty scaffold. Install order comes from scene.erect_plan.
Corridor / yard handoff is unconstrained. SSM and d_min apply only to the
Assembler while Spot is on scaffold.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

from constraints.pareto import Theta
from oracle.ssm import SSM_STOPPED_MPS, apply_ssm, protective_separation_m, safety_index
from oracle.simulate import (
    OracleConfig,
    OracleResult,
    TraceSample,
    _cmd_speed,
    _dist3,
    _horiz,
    _move_toward,
    _next_hop,
    _on_scaffold,
    _pose,
    _same_level,
)
from scene.erect_plan import ErectItem, build_erect_sequence, limit_erect_sequence
from scene.geometry import ScaffoldGeom, STAGE1_GEOM
from scene.mission_protocol import STAGE1_PROTOCOL
from ue.hrc_pipeline import evac_cargo_action, next_asm_mode, next_spot_mode
from ue.mission_poses import assembler_install_xy_m

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class BareErectConfig:
    protocol: object = STAGE1_PROTOCOL
    max_items: Optional[int] = None
    max_floors: Optional[int] = None
    boards_per_floor: Optional[int] = None
    dt_s: float = 0.1
    timeout_s: float = 36000.0
    record_trace: bool = True
    stair_speed_factor: float = 0.5
    drop_standoff_m: float = 0.80
    staging_slots: Optional[int] = None


def _install_pose(item: ErectItem, geom: ScaffoldGeom, walk_floor: int | None = None) -> Vec3:
    """Work pose in front of the member (not on the spawned pipe AABB)."""
    x_m, y_m = assembler_install_xy_m(item, geom)
    floor = item.floor if walk_floor is None else min(item.floor, walk_floor)
    floor = max(1, min(floor, geom.n_floors))
    return (x_m, y_m, geom.floor_z_m(floor))


def _reachable_floor(completed_stair_lifts: set[int]) -> int:
    floor = 1
    lift = 0
    while lift in completed_stair_lifts:
        floor += 1
        lift += 1
    return floor


def run_bare_erection(
    *,
    theta: Theta,
    geom: ScaffoldGeom = STAGE1_GEOM,
    config: BareErectConfig = BareErectConfig(),
    oracle: OracleConfig | None = None,
) -> OracleResult:
    oc = oracle or OracleConfig(
        truck_load_s=STAGE1_PROTOCOL.truck_load_s,
        drop_place_s=STAGE1_PROTOCOL.drop_place_s,
        erect_s=STAGE1_PROTOCOL.erect_s,
        human_speed_mps=STAGE1_PROTOCOL.human_speed_mps,
        human_retreat_mps=STAGE1_PROTOCOL.human_retreat_mps,
        sockets_per_floor=None,
        record_trace=config.record_trace,
        timeout_s=config.timeout_s,
        dt_s=config.dt_s,
        stair_speed_factor=config.stair_speed_factor,
    )
    sequence = limit_erect_sequence(
        build_erect_sequence(geom),
        max_items=config.max_items,
        max_floors=config.max_floors,
        boards_per_floor=config.boards_per_floor,
    )
    n_work = len(sequence)
    slots = int(
        config.staging_slots
        if config.staging_slots is not None
        else STAGE1_PROTOCOL.staging_slots
    )
    store: Vec3 = (*geom.storage_xy(), 0.0)
    spot = store
    human = _pose(oc.refuge_x_m, geom, 1)
    completed_stairs: set[int] = set()
    unlocked_decks: set[int] = set()
    pending: deque[ErectItem] = deque(sequence)
    staged: deque[ErectItem] = deque()
    spot_item: Optional[ErectItem] = None
    asm_item: Optional[ErectItem] = None
    spot_loaded = False
    holding = False
    erecting = False
    picking = False
    drain_hold = False
    load_timer = 0.0
    place_timer = 0.0
    pickup_timer = 0.0
    erect_timer = 0.0
    t = 0.0
    wait_s = 0.0
    path_m = 0.0
    corridor_time_s = 0.0
    scaffold_safe_s = 0.0
    scaffold_unsafe_s = 0.0
    min_sep = float("inf")
    si_min = float("inf")
    n_filled = 0
    trace: list[TraceSample] = []
    evac_requested = False
    keepout_waived = False

    def _floor_drop(item: Optional[ErectItem], reach: int) -> Vec3:
        floor = 1 if item is None else min(item.floor, reach)
        drop = _pose(oc.drop_x_m, geom, floor)
        return (drop[0] - config.drop_standoff_m, drop[1], drop[2])

    while t < oc.timeout_s and (
        pending or staged or spot_item is not None or asm_item is not None or spot_loaded
    ):
        reach = _reachable_floor(completed_stairs)
        if len(staged) >= slots:
            drain_hold = True
        elif not staged:
            drain_hold = False
        drop_spot = _floor_drop(spot_item, reach)
        at_yard = _horiz(spot, store) < oc.handoff_spot_m
        at_drop = _horiz(spot, drop_spot) < max(oc.handoff_spot_m, config.drop_standoff_m + 0.15)
        has_free_slot = len(staged) < slots
        intent = next_spot_mode(
            pending=bool(pending),
            loaded=spot_loaded,
            has_free_slot=has_free_slot if spot_loaded else True,
            at_yard=at_yard,
            at_drop=at_drop,
            drain_hold=drain_hold,
        )
        if intent == "loading" and at_yard:
            load_timer += oc.dt_s
            if load_timer >= oc.truck_load_s and pending and not spot_loaded:
                spot_item = pending.popleft()
                spot_loaded = True
                load_timer = 0.0
                intent = "to_drop" if has_free_slot else "wait_drop"
        else:
            load_timer = 0.0
        if intent == "placing" and spot_item is not None:
            place_timer += oc.dt_s
            if place_timer >= oc.drop_place_s:
                staged.append(spot_item)
                spot_item = None
                spot_loaded = False
                place_timer = 0.0
                keepout_waived = False
        else:
            place_timer = 0.0

        if intent in ("to_yard", "loading", "wait_drop", "idle"):
            spot_dest = store
        elif intent in ("to_drop", "placing") and spot_item is not None:
            spot_dest = drop_spot
        else:
            spot_dest = store
        delivering = intent in ("to_drop", "placing")
        if not delivering:
            keepout_waived = False
            evac_requested = False

        cargo_act = evac_cargo_action(
            picking=picking, holding=holding, erecting=erecting
        )
        evac = bool(evac_requested)
        assigned = asm_item is not None and not holding and not erecting and not picking
        asm_intent = next_asm_mode(
            cargo_at_drop=bool(staged),
            holding=holding,
            erecting=erecting,
            assigned=assigned,
            evac=evac,
            picking=picking,
        )
        if asm_intent == "to_drop" and staged and asm_item is None:
            asm_item = staged.popleft()
        work_item = asm_item if asm_item is not None else spot_item
        work_floor = min(work_item.floor, reach) if work_item is not None else 1
        drop_asm = _floor_drop(asm_item, reach)
        refuge = _pose(oc.refuge_x_m, geom, work_floor)
        socket = (
            _install_pose(asm_item, geom, walk_floor=reach)
            if asm_item is not None
            else drop_asm
        )
        if asm_intent == "to_refuge":
            human_dest = refuge
        elif asm_intent in ("to_drop", "pickup"):
            human_dest = drop_asm
        elif asm_intent in ("to_socket", "erect"):
            human_dest = socket
        else:
            human_dest = refuge

        on_site = _on_scaffold(spot[0], geom)
        in_corridor = not on_site
        sep = _horiz(spot, human)
        keepout_enforced = (
            delivering
            and on_site
            and _on_scaffold(human[0], geom)
            and _same_level(spot, human)
            and not keepout_waived
            and STAGE1_PROTOCOL.assembler_keepout_on_scaffold
        )
        spot_hop_probe = _next_hop(geom, spot, spot_dest)
        stair_hop = abs(spot_hop_probe[2] - spot[2]) > 0.05
        cmd = _cmd_speed(
            in_corridor=in_corridor,
            constraint_active=True,
            vmax_mps=theta.vmax_mps,
            stair_hop=stair_hop and on_site,
            stair_speed_factor=oc.stair_speed_factor,
        )
        if keepout_enforced:
            speed, ssm_mode = apply_ssm(sep, cmd, on_site=True)
            if sep < theta.dmin_m:
                speed, ssm_mode = 0.0, "dmin_stop"
        else:
            speed, ssm_mode = cmd, "free"
        blocked = speed <= SSM_STOPPED_MPS and keepout_enforced
        if blocked:
            wait_s += oc.dt_s
            if not (erecting or picking):
                evac_requested = True
        if keepout_enforced and on_site and speed > SSM_STOPPED_MPS:
            min_sep = min(min_sep, sep)
            sp = protective_separation_m(speed)
            si_min = min(si_min, safety_index(sep, sp))

        if evac_requested:
            human_hop = _next_hop(geom, human, refuge)
            human, _ = _move_toward(human, human_hop, oc.human_retreat_mps, oc.dt_s)
            if _horiz(human, refuge) < 0.50 and _same_level(human, refuge):
                keepout_waived = True
                evac_requested = False
        else:
            human_hop = _next_hop(geom, human, human_dest)
            human, _ = _move_toward(human, human_hop, oc.human_speed_mps, oc.dt_s)

        if intent in ("loading", "placing", "idle", "wait_drop") or blocked:
            step = 0.0
        else:
            spot_hop = _next_hop(geom, spot, spot_dest)
            spot, step = _move_toward(spot, spot_hop, speed, oc.dt_s)
            path_m += step
            if not _on_scaffold(spot[0], geom):
                corridor_time_s += oc.dt_s
            else:
                scaffold_safe_s += oc.dt_s
        if blocked:
            step = 0.0

        if asm_intent == "to_drop" and asm_item is not None:
            if _horiz(human, drop_asm) < oc.handoff_human_m and _same_level(human, drop_asm):
                picking = True
                pickup_timer = 0.0
        if picking and asm_item is not None and not evac:
            if _horiz(human, drop_asm) < oc.handoff_human_m and _same_level(human, drop_asm):
                pickup_timer += oc.dt_s
                if pickup_timer >= STAGE1_PROTOCOL.assembler_pickup_s:
                    picking = False
                    holding = True
                    pickup_timer = 0.0
            else:
                pickup_timer = 0.0
        elif holding and asm_item is not None and not erecting and not evac:
            if _dist3(human, socket) < 0.40:
                holding = False
                erecting = True
                erect_timer = 0.0
        elif erecting and asm_item is not None and not evac:
            erect_timer += oc.dt_s
            need = (
                STAGE1_PROTOCOL.board_erect_s
                if asm_item.kind == "board"
                else STAGE1_PROTOCOL.structural_erect_s
            )
            if erect_timer >= need:
                if asm_item.unlocks_deck:
                    unlocked_decks.add(asm_item.floor)
                if asm_item.unlocks_stair_lift is not None:
                    completed_stairs.add(asm_item.unlocks_stair_lift)
                n_filled += 1
                asm_item = None
                holding = False
                erecting = False
                erect_timer = 0.0
        else:
            pickup_timer = 0.0

        t += oc.dt_s
        if oc.record_trace:
            realized = step / oc.dt_s if oc.dt_s > 0 else 0.0
            sp = protective_separation_m(max(realized, theta.vmax_mps * 0.01))
            si = safety_index(sep, sp) if keepout_enforced else 1.0
            trace.append(
                TraceSample(
                    t_s=t,
                    spot=spot,
                    human=human,
                    sep_m=sep,
                    spot_speed_mps=realized,
                    blocked=blocked,
                    in_corridor=not _on_scaffold(spot[0], geom),
                    n_filled=n_filled,
                    current_floor=work_floor,
                    violating=keepout_enforced and si < 1.0 and realized > SSM_STOPPED_MPS,
                    si=si,
                    sp_m=sp,
                    ssm_mode=ssm_mode,
                    unsafe=keepout_enforced and sep < protective_separation_m(realized),
                    retreating=evac_requested,
                )
            )

    completed = (
        n_filled >= n_work
        and not pending
        and not staged
        and spot_item is None
        and asm_item is None
        and not spot_loaded
    )
    return OracleResult(
        completed=completed,
        makespan_s=t,
        path_length_m=path_m,
        corridor_time_s=corridor_time_s,
        min_separation_m=0.0 if min_sep is float("inf") else min_sep,
        wait_s=wait_s,
        violation_s=scaffold_unsafe_s,
        n_filled=n_filled,
        n_sockets=n_work,
        floors_completed=max(unlocked_decks) if unlocked_decks else 0,
        timeout_s=oc.timeout_s,
        trace=tuple(trace) if oc.record_trace else tuple(),
        ssm_s=scaffold_unsafe_s,
        si_min=1.0 if si_min is float("inf") else si_min,
        scaffold_safe_s=scaffold_safe_s,
        scaffold_unsafe_s=scaffold_unsafe_s,
        scaffold_time_s=scaffold_safe_s + wait_s,
    )

    while t < oc.timeout_s and (queue or reserved is not None or spot_loaded or cargo_at_drop or human_loaded):
        reach = _reachable_floor(completed_stairs)
        if reserved is None and queue:
            reserved = queue.pop(0)

        work_floor = reserved.floor if reserved is not None else 1
        work_floor = min(work_floor, reach)
        drop = _install_pose(reserved, geom) if reserved is not None else _pose(oc.drop_x_m, geom, 1)
        if reserved is not None and reserved.floor > reach:
            drop = _pose(oc.drop_x_m, geom, reach)

        refuge = _pose(oc.refuge_x_m, geom, min(work_floor, reach))
        human_at_refuge = _horiz(human, refuge) < 0.80 and _same_level(human, refuge)
        # Do not start the next fetch until Assembler is clear of the work point.
        need_fetch = (
            reserved is not None
            and not spot_loaded
            and not cargo_at_drop
            and not human_loaded
            and (n_filled == 0 or human_at_refuge or keepout_waived)
        )

        if evac_requested or keepout_waived:
            human_dest = refuge
        elif human_loaded and reserved is not None:
            human_dest = _install_pose(reserved, geom, walk_floor=reach)
        elif cargo_at_drop:
            human_dest = drop
        else:
            human_dest = refuge

        if need_fetch:
            spot_dest = store
        elif spot_loaded and reserved is not None:
            spot_dest = (
                drop[0] - config.drop_standoff_m,
                drop[1],
                drop[2],
            )
        else:
            spot_dest = store

        on_site = _on_scaffold(spot[0], geom)
        in_corridor = not on_site
        sep = _horiz(spot, human)
        # Waive keep-out while Assembler is picking up / installing the piece Spot
        # just staged — otherwise Spot parked at the drop blocks the human forever.
        work_handoff = cargo_at_drop or human_loaded
        keepout_enforced = (
            on_site
            and _on_scaffold(human[0], geom)
            and _same_level(spot, human)
            and not keepout_waived
            and not work_handoff
            and STAGE1_PROTOCOL.assembler_keepout_on_scaffold
        )
        spot_hop_probe = _next_hop(geom, spot, spot_dest)
        stair_hop = abs(spot_hop_probe[2] - spot[2]) > 0.05
        cmd = _cmd_speed(
            in_corridor=in_corridor,
            constraint_active=True,
            vmax_mps=theta.vmax_mps,
            stair_hop=stair_hop and on_site,
            stair_speed_factor=oc.stair_speed_factor,
        )

        if keepout_enforced:
            speed, ssm_mode = apply_ssm(sep, cmd, on_site=True)
            if sep < theta.dmin_m:
                speed, ssm_mode = 0.0, "dmin_stop"
        else:
            speed, ssm_mode = cmd, "free"

        blocked = speed <= SSM_STOPPED_MPS and keepout_enforced
        if blocked:
            wait_s += oc.dt_s
            if not (erecting or picking):
                evac_requested = True

        if keepout_enforced and on_site and speed > SSM_STOPPED_MPS:
            min_sep = min(min_sep, sep)
            sp = protective_separation_m(speed)
            si_min = min(si_min, safety_index(sep, sp))

        # Move human
        if evac_requested:
            human_hop = _next_hop(geom, human, refuge)
            human, _ = _move_toward(human, human_hop, oc.human_retreat_mps, oc.dt_s)
            if _horiz(human, refuge) < 0.50 and _same_level(human, refuge):
                keepout_waived = True
                evac_requested = False
        else:
            human_hop = _next_hop(geom, human, human_dest)
            human, _ = _move_toward(human, human_hop, oc.human_speed_mps, oc.dt_s)

        # Move spot
        if not blocked:
            prev = spot
            spot_hop = _next_hop(geom, spot, spot_dest)
            spot, step = _move_toward(spot, spot_hop, speed, oc.dt_s)
            path_m += step
            if not _on_scaffold(spot[0], geom):
                corridor_time_s += oc.dt_s
            else:
                scaffold_safe_s += oc.dt_s
        else:
            step = 0.0

        # Timers / state machine
        if need_fetch and _horiz(spot, store) < oc.handoff_spot_m:
            load_timer += oc.dt_s
            if load_timer >= oc.truck_load_s:
                spot_loaded = True
                load_timer = 0.0
        else:
            load_timer = 0.0

        if spot_loaded and reserved is not None:
            # Arrived at standoff or at drop: either is enough to start place dwell.
            near_drop = _horiz(spot, drop) < max(oc.handoff_spot_m, config.drop_standoff_m + 0.15)
            if near_drop and _same_level(spot, drop):
                place_timer += oc.dt_s
                if place_timer >= oc.drop_place_s:
                    spot_loaded = False
                    cargo_at_drop = True
                    place_timer = 0.0
                    keepout_waived = False
            else:
                place_timer = 0.0
        else:
            place_timer = 0.0

        if cargo_at_drop and _horiz(human, drop) < oc.handoff_human_m and _same_level(human, drop):
            pickup_timer += oc.dt_s
            if pickup_timer >= STAGE1_PROTOCOL.assembler_pickup_s:
                human_loaded = True
                cargo_at_drop = False
                pickup_timer = 0.0
        else:
            pickup_timer = 0.0

        if human_loaded and reserved is not None:
            goal = _install_pose(reserved, geom, walk_floor=reach)
            if _dist3(human, goal) < 0.40:
                erect_timer += oc.dt_s
                need = (
                    STAGE1_PROTOCOL.board_erect_s
                    if reserved.kind == "board"
                    else STAGE1_PROTOCOL.structural_erect_s
                )
                if erect_timer >= need:
                    if reserved.unlocks_deck:
                        unlocked_decks.add(reserved.floor)
                    if reserved.unlocks_stair_lift is not None:
                        completed_stairs.add(reserved.unlocks_stair_lift)
                    n_filled += 1
                    reserved = None
                    human_loaded = False
                    erect_timer = 0.0
                    keepout_waived = False
            else:
                erect_timer = 0.0

        t += oc.dt_s
        if oc.record_trace:
            realized = step / oc.dt_s if oc.dt_s > 0 else 0.0
            sp = protective_separation_m(max(realized, theta.vmax_mps * 0.01))
            si = safety_index(sep, sp) if keepout_enforced else 1.0
            trace.append(
                TraceSample(
                    t_s=t,
                    spot=spot,
                    human=human,
                    sep_m=sep,
                    spot_speed_mps=realized,
                    blocked=blocked,
                    in_corridor=not _on_scaffold(spot[0], geom),
                    n_filled=n_filled,
                    current_floor=work_floor,
                    violating=keepout_enforced and si < 1.0 and realized > SSM_STOPPED_MPS,
                    si=si,
                    sp_m=sp,
                    ssm_mode=ssm_mode,
                    unsafe=keepout_enforced and sep < protective_separation_m(realized),
                    retreating=evac_requested,
                )
            )

    completed = (
        n_filled >= n_work
        and reserved is None
        and not spot_loaded
        and not cargo_at_drop
        and not human_loaded
    )
    return OracleResult(
        completed=completed,
        makespan_s=t,
        path_length_m=path_m,
        corridor_time_s=corridor_time_s,
        min_separation_m=0.0 if min_sep is float("inf") else min_sep,
        wait_s=wait_s,
        violation_s=scaffold_unsafe_s,
        n_filled=n_filled,
        n_sockets=n_work,
        floors_completed=max(unlocked_decks) if unlocked_decks else 0,
        timeout_s=oc.timeout_s,
        trace=tuple(trace) if oc.record_trace else tuple(),
        ssm_s=scaffold_unsafe_s,
        si_min=1.0 if si_min is float("inf") else si_min,
        scaffold_safe_s=scaffold_safe_s,
        scaffold_unsafe_s=scaffold_unsafe_s,
        scaffold_time_s=scaffold_safe_s + wait_s,
    )
