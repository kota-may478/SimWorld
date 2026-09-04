"""Headless 3F erection oracle. Corridor unconstrained; scaffold uses θ.

Jsafe is keep-out violation time (sep < d_safe on the scaffold), not a
runtime hard filter and not body-collision SVR. The controller still
refuses to close further when already inside d_min.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from constraints.pareto import Theta
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


@dataclass(frozen=True)
class OracleConfig:
    dt_s: float = 0.1
    human_speed_mps: float = 1.2
    timeout_s: float = 3600.0
    handoff_spot_m: float = 0.50
    handoff_human_m: float = 0.50
    collision_m: float = 0.40
    d_safe_m: float = 1.00
    erect_s: float = 1.5
    sockets_per_floor: Optional[int] = None
    drop_x_m: float = 2.0
    standoff_x_m: float = 6.0
    record_trace: bool = True


@dataclass(frozen=True)
class OracleResult:
    completed: bool
    makespan_s: float
    path_length_m: float
    corridor_time_s: float
    min_separation_m: float
    wait_s: float
    violation_s: float
    n_filled: int
    n_sockets: int
    floors_completed: int
    timeout_s: float
    trace: Tuple[TraceSample, ...] = ()


def _horiz(a: Vec3, b: Vec3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _dist3(a: Vec3, b: Vec3) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _on_scaffold(x_m: float, geom: ScaffoldGeom) -> bool:
    return x_m >= -geom.stair_bay_m - 1e-6


def _same_level(a: Vec3, b: Vec3, *, tol_m: float = 0.40) -> bool:
    return abs(a[2] - b[2]) < tol_m


def _floor_of_z(geom: ScaffoldGeom, z_m: float) -> int:
    return max(1, min(geom.n_floors, 1 + int(math.floor((z_m + 1e-6) / geom.lift_m))))


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
    arrive = 0.45
    mid_y = geom.deck_width_m * 0.5
    stair_x = -geom.stair_bay_m * 0.5
    src_f = _floor_of_z(geom, src[2])
    dest_f = _floor_of_z(geom, dest[2])
    if src_f != dest_f:
        at_stair_column = abs(src[0] - stair_x) <= arrive and abs(src[1] - mid_y) <= arrive
        if not at_stair_column:
            return (stair_x, mid_y, geom.floor_z_m(src_f))
        step = 1 if dest_f > src_f else -1
        return (stair_x, mid_y, geom.floor_z_m(src_f + step))
    if _on_scaffold(src[0], geom) and not _on_scaffold(dest[0], geom):
        gate: Vec3 = (-geom.stair_bay_m - 0.5, mid_y, geom.floor_z_m(src_f))
        if src[0] > gate[0] + 0.05:
            return gate
        return dest
    if (not _on_scaffold(src[0], geom)) and _on_scaffold(dest[0], geom):
        gate = (stair_x, mid_y, geom.floor_z_m(src_f))
        if src[0] < -geom.stair_bay_m - 0.05:
            return gate
        return dest
    return dest


def _count_filled(spec: ScaffoldSpec) -> int:
    return sum(1 for s in spec.sockets if s.filled)


def _floor_done(spec: ScaffoldSpec, floor: int) -> bool:
    sockets = spec.sockets_on_floor(floor)
    return bool(sockets) and all(s.filled for s in sockets)


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
    t = 0.0
    wait_s = 0.0
    path_m = 0.0
    corridor_time_s = 0.0
    min_sep = 1e9
    violation_s = 0.0
    floors_completed = 0
    yield_drop = False
    trace: List[TraceSample] = []

    while t < config.timeout_s:
        drop = _pose(config.drop_x_m, geom, current_floor)
        standoff = _pose(config.standoff_x_m, geom, current_floor)
        unfilled = sum(1 for s in spec.sockets_on_floor(current_floor) if not s.filled)
        pipeline = int(spot_loaded) + int(cargo_at_drop) + int(human_loaded)
        need_fetch = unfilled > pipeline
        floor_clear = unfilled == 0 and not human_loaded and not cargo_at_drop
        next_sock = spec.next_empty_socket(current_floor)

        if human_loaded:
            if reserved is None:
                reserved = next_sock
            human_dest = (
                (reserved.x_m, reserved.y_m, reserved.z_m) if reserved is not None else standoff
            )
        elif cargo_at_drop:
            human_dest = drop
        elif yield_drop:
            human_dest = standoff
        elif floor_clear and current_floor < geom.n_floors:
            human_dest = _pose(config.standoff_x_m, geom, current_floor + 1)
        elif next_sock is not None:
            human_dest = (next_sock.x_m, next_sock.y_m, next_sock.z_m)
        else:
            human_dest = standoff

        if spot_loaded and current_floor <= spot_max_floor and not cargo_at_drop:
            spot_dest = drop
        elif need_fetch and not spot_loaded:
            spot_dest = store
        else:
            spot_dest = store

        in_corridor = not _on_scaffold(spot[0], geom)
        speed = 1.0 if in_corridor or not constraint_active else theta.vmax_mps
        hop = _next_hop(geom, spot, spot_dest)
        trial, _ = _move_toward(spot, hop, speed, config.dt_s)
        sep = _horiz(spot, human)
        on_site = (
            _on_scaffold(spot[0], geom)
            and _on_scaffold(human[0], geom)
            and _same_level(spot, human)
        )
        opens = _horiz(trial, human) > sep + 1e-9
        blocked = constraint_active and on_site and sep < theta.dmin_m and not opens
        if blocked:
            yield_drop = True
            wait_s += config.dt_s
            nxt, moved = spot, 0.0
        else:
            nxt, moved = trial, _dist3(spot, trial)
        if cargo_at_drop or not spot_loaded:
            yield_drop = False

        human_hop = _next_hop(geom, human, human_dest)
        human, _ = _move_toward(human, human_hop, config.human_speed_mps, config.dt_s)

        if (not spot_loaded) and need_fetch and _dist3(nxt, store) < 0.40:
            spot_loaded = True
        at_drop = _horiz(nxt, drop) < config.handoff_spot_m and _same_level(nxt, drop)
        human_at_drop = _horiz(human, drop) < theta.dmin_m and _same_level(human, drop)
        if (
            spot_loaded
            and at_drop
            and not cargo_at_drop
            and not (constraint_active and human_at_drop)
        ):
            spot_loaded = False
            cargo_at_drop = True

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
            human_loaded = True
            cargo_at_drop = False
            reserved = spec.next_empty_socket(current_floor)
            erect_timer = 0.0

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
        if in_corridor:
            corridor_time_s += config.dt_s
        sep = _horiz(spot, human)
        on_site = (
            _on_scaffold(spot[0], geom)
            and _on_scaffold(human[0], geom)
            and _same_level(spot, human)
        )
        violating = on_site and sep < config.d_safe_m
        if on_site:
            min_sep = min(min_sep, sep)
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
                    spot_speed_mps=0.0 if blocked else speed,
                    blocked=blocked,
                    in_corridor=in_corridor,
                    n_filled=max(0, filled_now),
                    current_floor=current_floor,
                    violating=violating,
                )
            )
        if floors_completed >= geom.n_floors and not human_loaded:
            break

    if min_sep > 1e8:
        min_sep = _horiz(spot, human)
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
    )
