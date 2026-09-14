#!/usr/bin/env python3
"""UE bare-ground progressive erect under a frozen theta.

Prereq: `ue/spawn_scaffold_pie.py --bare` (truck + yard human only).
Places posts/braces/ledgers/boards/stairs one-by-one from scene.erect_plan.
Keep-out: Assembler on scaffold from the first member. Yard / corridor unconstrained.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
ROOT = PKG.parent.parent
NAV = ROOT / "dev" / "grid_env_level_nav"
GEH = ROOT / "dev" / "grid_env_hri"
DEPTH = ROOT / "dev" / "grid_env_depth_perception"
for path in (str(ROOT), str(NAV), str(GEH), str(DEPTH), str(PKG)):
    if path not in sys.path:
        sys.path.insert(0, path)

import ue_client_guard  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402
from pie_spawn_safety import spawn_bp_resilient  # noqa: E402
from pie_safety import PieSessionLost, cooldown_before_spawn_batch  # noqa: E402
from level_nav_robot import ensure_level_spotdog  # noqa: E402
from oracle.simulate import _floor_of_z  # noqa: E402
from oracle.ssm import apply_ssm, protective_separation_m  # noqa: E402
from scene.erect_plan import (  # noqa: E402
    build_erect_sequence,
    limit_erect_sequence,
    select_erect_sequence,
    ue_protocol_limits,
)
from scene.field import ramp_yard_x_m  # noqa: E402
from scene.geometry import STAGE1_GEOM  # noqa: E402
from scene.mission_protocol import STAGE1_PROTOCOL  # noqa: E402
from ue.layout import ENTRANCE_LOCAL_XY_CM, STAGING_LOCAL_XY_CM, footprint_local_cm, scaffold_xyz_to_local_cm  # noqa: E402
from ue.hrc_pipeline import (  # noqa: E402
    assembler_holds_refuge,
    assembler_is_lost,
    assembler_should_yield_to_locked_spot,
    drive_duration_s,
    evac_cargo_action,
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
    spot_is_delivering,
    spot_on_scaffold_local_x,
    wall_from_sim,
    wall_tick_remainder_s,
    yaw_error_deg,
)
from ue.mission_poses import (  # noqa: E402
    ASSEMBLER_FOOT_Z_CM,
    ASSEMBLER_SCALE,
    SPOT_STAND_Z_CM,
    assembler_stand_local_cm,
    assembler_install_local_cm,
    extra_humanoid_actor_names,
    first_free_drop_slot,
    floor_drop_slot_local_cm,
    floor_refuge_local_cm,
    next_hop_standing_local_cm,
    reachable_floor,
)
from ue.placement import (  # noqa: E402
    ACTOR_PREFIX,
    ASSEMBLER_ACTOR,
    DROP_SLOT_COUNT,
    PIPE_KINDS,
    SPOT_YARD_LOCAL_XY_CM,
    TRUCK_ACTOR,
    YARD_HUMAN_ACTOR,
    YARD_HUMAN_ALONG_CM,
    YARD_HUMAN_SIDE_CM,
    YARD_HUMAN_YAW_DEG,
    YARD_HUMAN_Z_CM,
    module_to_spawn_box,
    spot_ramp_cheeks,
    stair_landings,
)

YARD_HUMAN = YARD_HUMAN_ACTOR
ASSEMBLER = ASSEMBLER_ACTOR
CARRY = f"{ACTOR_PREFIX}_carry"
CARRY_SCALE = [2.0, 0.4, 0.2]
SPOT_FOOT_Z_CM = SPOT_STAND_Z_CM
HUMAN_SPEED_CM_S = STAGE1_PROTOCOL.human_speed_mps * 100.0
HUMAN_RETREAT_CM_S = STAGE1_PROTOCOL.human_retreat_mps * 100.0
PARALLEL_DT_S = 0.16
CORRIDOR_VMAX_MPS = STAGE1_PROTOCOL.corridor_vmax_mps
HUMAN_BP = (
    geh.HUMAN_BP,
    "/Game/TrafficSystem/Pedestrian/Base_Pedestrian.Base_Pedestrian_C",
)
PIPE_COLOR = (168, 172, 176)
# Last slomo set by the mission loop; spawn_bp drops to 1x then restores this.
_PLAYBACK_RATE = 1.0


def _yaw_toward(src, dst) -> float:
    return math.degrees(math.atan2(dst[1] - src[1], dst[0] - src[0]))


def _horiz_m(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1]) / 100.0


def _ground_human(ucv, name: str) -> None:
    """Keep the skeletal mesh with the root: physics off, collision on, movable."""
    for fn in (
        lambda: ucv.set_physics(name, False),
        lambda: ucv.set_movable(name, True),
        lambda: ucv.set_collision(name, True),
        lambda: ucv.enable_controller(name, False),
        lambda: ucv.humanoid_stop(name),
        lambda: ucv.humanoid_stop_current_action(name),
    ):
        try:
            fn()
        except Exception:
            pass


def _park_human_offmap(ucv, name: str) -> None:
    loc = geh.try_get_location_cm(ucv, name) if hasattr(geh, "try_get_location_cm") else None
    if loc is None:
        try:
            loc = ucv.get_location(name)
        except Exception:
            return
    stash_z = lc.FLOOR_REF_Z_CM - 50_000.0
    _ground_human(ucv, name)
    try:
        ucv.set_collision(name, False)
    except Exception:
        pass
    wx, wy, _wz = lc.local_xyz_to_world(-50_000.0, -50_000.0, -50_000.0)
    ucv.set_location([wx, wy, stash_z], name)


def _park_extra_humanoids(ucv, keep: tuple[str, ...]) -> None:
    extras = extra_humanoid_actor_names(geh.actor_names(ucv), keep)
    for name in extras:
        _park_human_offmap(ucv, name)
        print("park leftover humanoid", name)


def _ensure_human(ucv, name: str, local_cm, yaw: float) -> bool:
    wx, wy, wz = lc.local_xyz_to_world(*local_cm)
    scale = [ASSEMBLER_SCALE, ASSEMBLER_SCALE, ASSEMBLER_SCALE]
    if geh.actor_exists(ucv, name):
        _ground_human(ucv, name)
        try:
            ucv.set_scale(scale, name)
        except Exception:
            pass
        ucv.set_location([wx, wy, wz], name)
        ucv.set_orientation([0.0, yaw, 0.0], name)
        _ground_human(ucv, name)
        print(
            f"[Human] reuse {name} world=({wx:.1f},{wy:.1f},{wz:.1f}) "
            f"local=({local_cm[0]:.1f},{local_cm[1]:.1f},{local_cm[2]:.1f}) "
            f"scale={ASSEMBLER_SCALE:.3f}"
        )
        return True
    for bp in HUMAN_BP:
        ok, _ = spawn_bp_resilient(
            ucv,
            bp,
            name,
            timeout_s=120.0,
            playback_rate=_PLAYBACK_RATE,
            pause_playback=False,
        )
        if not ok:
            continue
        _ground_human(ucv, name)
        try:
            ucv.set_scale(scale, name)
        except Exception:
            pass
        ucv.set_location([wx, wy, wz], name)
        ucv.set_orientation([0.0, yaw, 0.0], name)
        _ground_human(ucv, name)
        print(
            f"[Human] spawn {name} via {bp} "
            f"world=({wx:.1f},{wy:.1f},{wz:.1f}) "
            f"local=({local_cm[0]:.1f},{local_cm[1]:.1f},{local_cm[2]:.1f}) "
            f"scale={ASSEMBLER_SCALE:.3f}"
        )
        return True
    return False


def _spawn_box(ucv, box) -> bool:
    wx, wy, wz = lc.local_xyz_to_world(*box.local_cm)
    if not geh.actor_exists(ucv, box.actor):
        ok, _ = spawn_bp_resilient(
            ucv,
            geh.CUBE_BP,
            box.actor,
            timeout_s=60.0,
            playback_rate=_PLAYBACK_RATE,
            pause_playback=False,
        )
        if not ok:
            return False
    ucv.set_physics(box.actor, False)
    blocking = True
    try:
        geh.set_cube_blocking_mode(ucv, box.actor, blocking=blocking, apply_tint=True)
    except Exception:
        pass
    if box.kind in PIPE_KINDS:
        try:
            ucv.set_color(box.actor, list(PIPE_COLOR))
        except Exception:
            pass
    ucv.set_location([wx, wy, wz], box.actor)
    ucv.set_scale(list(box.scale), box.actor)
    ucv.set_orientation([box.pitch_deg, box.yaw_deg, box.roll_deg], box.actor)
    return True


def _spawn_member(ucv, item) -> bool:
    return _spawn_box(ucv, module_to_spawn_box(item.module, STAGE1_GEOM))


def _spawn_climb_aids(ucv, *, floor: int | None = None, stair_lift: int | None = None) -> None:
    """Landings and cheeks so Spot/assembler can leave the AABB stair onto the deck."""
    tags: list[str] = []
    if floor == 1:
        tags.append("landing_f1")
    elif floor == 2:
        tags.append("landing_f2")
    elif floor == 3:
        tags.append("landing_f3")
    if stair_lift is not None:
        tags.append(f"cheek_L{stair_lift}")
        tags.append("landing_f2" if stair_lift == 0 else "landing_f3")
    if not tags:
        return
    for box in (*stair_landings(), *spot_ramp_cheeks()):
        if any(tag in box.actor for tag in tags):
            _spawn_box(ucv, box)


def _world(local_cm) -> list[float]:
    wx, wy, wz = lc.local_xyz_to_world(*local_cm)
    return [wx, wy, wz]


def _local_of(world_xyz) -> tuple[float, float, float]:
    return lc.world_xyz_to_local(world_xyz[0], world_xyz[1], world_xyz[2])


def _hop_world(loc, goal_w, clearance_cm: float) -> list[float]:
    hop_l = next_hop_standing_local_cm(_local_of(loc), _local_of(goal_w), clearance_cm)
    return _world(hop_l)


def _nav_surface_z_cm(ucv, nav_actor, wx: float, wy: float, z_hint_cm: float):
    if not nav_actor:
        return None
    for probe_z in (
        z_hint_cm,
        z_hint_cm + 20.0,
        z_hint_cm - 15.0,
        z_hint_cm + 70.0,
    ):
        try:
            raw = nq.nav_project_point(ucv, nav_actor, wx, wy, probe_z)
        except Exception:
            continue
        if not raw.get("ok"):
            continue
        px, py, pz = float(raw["x"]), float(raw["y"]), float(raw["z"])
        if not nav_projection_is_usable(query_xy=(wx, wy), projected_xy=(px, py)):
            continue
        return pz
    return None


def _ground_root(
    ucv,
    nav_actor,
    wx: float,
    wy: float,
    *,
    clearance_cm: float,
    z_hint_cm: float,
    prev_root_z_cm=None,
) -> list[float]:
    """XY stays planned; Z comes from the live NavMesh / ground."""
    feet_hint = float(z_hint_cm) - float(clearance_cm)
    surface = _nav_surface_z_cm(ucv, nav_actor, wx, wy, feet_hint + 12.0)
    root_z = grounded_root_z_cm(
        surface_z_cm=surface,
        clearance_cm=clearance_cm,
        prev_root_z_cm=prev_root_z_cm if prev_root_z_cm is not None else z_hint_cm,
    )
    return [float(wx), float(wy), float(root_z)]


def _same_floor_w(a, b) -> bool:
    za = _local_of(a)[2] / 100.0
    zb = _local_of(b)[2] / 100.0
    return _floor_of_z(STAGE1_GEOM, za) == _floor_of_z(STAGE1_GEOM, zb)


def _arrived_w(a, b, *, horiz_m: float = 0.55, z_cm: float = 80.0) -> bool:
    return _horiz_m(a, b) < horiz_m and abs(a[2] - b[2]) < z_cm


def _cargo_no_hit(ucv, actor: str) -> None:
    """Visual cargo only. Collision on a hovering cube traps the Assembler."""
    try:
        ucv.set_physics(actor, False)
        ucv.set_collision(actor, False)
    except Exception:
        pass


def _host_yaw_deg(ucv, host: str) -> float:
    try:
        rot = ucv.get_orientation(host)
        return float(rot[1])
    except Exception:
        return 0.0


def _attach_actor(ucv, host: str, actor: str, *, dz_cm: float = 40.0) -> None:
    if not geh.actor_exists(ucv, actor):
        return
    loc = ucv.get_location(host)
    ucv.set_location([loc[0], loc[1], loc[2] + dz_cm], actor)
    try:
        ucv.set_orientation([0.0, _host_yaw_deg(ucv, host), 0.0], actor)
    except Exception:
        pass
    _cargo_no_hit(ucv, actor)


def _attach_carry(ucv, host: str, *, dz_cm: float = 40.0) -> None:
    _attach_actor(ucv, host, CARRY, dz_cm=dz_cm)


def _style_pipe_cube(ucv, actor: str) -> None:
    try:
        geh.set_cube_blocking_mode(ucv, actor, blocking=True, apply_tint=False)
    except Exception:
        pass
    try:
        ucv.set_color(actor, list(PIPE_COLOR))
    except Exception:
        pass
    ucv.set_scale(list(CARRY_SCALE), actor)
    _cargo_no_hit(ucv, actor)


def _make_carry_solid(ucv) -> None:
    """BP_TransparentCube defaults to glass; force opaque + collision off while carried."""
    if geh.actor_exists(ucv, CARRY):
        _style_pipe_cube(ucv, CARRY)


def _stage_actor(item_id: str) -> str:
    return f"{ACTOR_PREFIX}_stage_{item_id}"


def _place_stage(ucv, actor: str, world) -> None:
    if not geh.actor_exists(ucv, actor):
        spawn_bp_resilient(
            ucv,
            geh.CUBE_BP,
            actor,
            timeout_s=30.0,
            playback_rate=_PLAYBACK_RATE,
            pause_playback=False,
        )
    if not geh.actor_exists(ucv, actor):
        return
    _style_pipe_cube(ucv, actor)
    ucv.set_location([world[0], world[1], world[2] + 20.0], actor)
    ucv.set_orientation([0.0, 90.0, 0.0], actor)
    _cargo_no_hit(ucv, actor)


def _hide_actor(ucv, actor: str) -> None:
    if not geh.actor_exists(ucv, actor):
        return
    _cargo_no_hit(ucv, actor)
    loc = ucv.get_location(actor)
    ucv.set_location([loc[0], loc[1], loc[2] - 50000.0], actor)


def _set_slomo(ucv, rate: float) -> bool:
    """UE console slomo: N means N seconds of sim per wall second."""
    cmd = f"vrun slomo {float(rate):g}"
    try:
        with ucv.lock:
            ucv.client.request(cmd, -1)
        print("sim_rate", float(rate), "slomo")
        return True
    except Exception as exc:
        print("WARN slomo failed", exc)
        return False


def _kinematic_pawn(ucv, name: str) -> None:
    """Teleport-driven pawns: no gravity, no capsule shove, no AI walk."""
    for fn in (
        lambda: ucv.set_physics(name, False),
        lambda: ucv.set_collision(name, False),
        lambda: ucv.enable_controller(name, False),
    ):
        try:
            fn()
        except Exception:
            pass


def _teleport_body(ucv, name: str) -> None:
    """No gravity; leave collision as-is unless the caller sets it."""
    try:
        ucv.set_physics(name, False)
    except Exception:
        pass


def _spot_walk_body(ucv, name: str) -> None:
    """Anim via controller; collision off so the yard human does not pin Spot."""
    for fn in (
        lambda: ucv.set_physics(name, False),
        lambda: ucv.set_collision(name, False),
        lambda: ucv.enable_controller(name, True),
    ):
        try:
            fn()
        except Exception:
            pass


def _spot_stop_gait(ucv, robot: str) -> None:
    try:
        ucv.enable_controller(robot, False)
    except Exception:
        pass


def _spot_play_gait(
    ucv,
    robot: str,
    drive: dict,
    *,
    speed_cm_s: float,
    remain_cm: float,
    yaw_deg: float,
    start_xy,
    sim_rate: float,
) -> bool:
    """Issue one long Move_Speed. Returns True if issued."""
    now = time.time()
    if not should_reissue_drive(now=now, cmd_until=float(drive.get("until") or 0.0)):
        return False
    dur = drive_duration_s(
        remain_cm / 100.0,
        max(speed_cm_s / 100.0, 0.20),
        cap_s=2.4,
        min_s=0.40,
    )
    if dur <= 0.0:
        return False
    prev_yaw = drive.get("yaw")
    if prev_yaw is None or abs(yaw_error_deg(float(prev_yaw), yaw_deg)) > 25.0:
        try:
            ucv.set_orientation([0.0, yaw_deg, 0.0], robot)
        except Exception:
            pass
        drive["yaw"] = yaw_deg
    try:
        ucv.dog_move(robot, [max(float(speed_cm_s), 50.0), dur, 0], wait=False)
    except Exception:
        pass
    drive["until"] = now + wall_from_sim(dur, sim_rate)
    drive["expect_cm"] = max(float(speed_cm_s), 50.0) * dur
    drive["start_xy"] = (float(start_xy[0]), float(start_xy[1]))
    return True


def _spot_halt(ucv, robot: str, drive: dict, *, face_yaw: float | None = None) -> None:
    drive["moving"] = False
    drive["until"] = 0.0
    drive.pop("last_xy", None)
    drive.pop("start_xy", None)
    drive.pop("expect_cm", None)
    drive.pop("walk_body", None)
    pose = drive.get("pose")
    _teleport_body(ucv, robot)
    try:
        ucv.set_collision(robot, False)
    except Exception:
        pass
    _spot_stop_gait(ucv, robot)
    if pose is not None:
        ucv.set_location(list(pose), robot)
    if face_yaw is not None:
        try:
            ucv.set_orientation([0.0, float(face_yaw), 0.0], robot)
        except Exception:
            pass
        drive["yaw"] = float(face_yaw)


def _spot_drive(
    ucv,
    robot: str,
    assembler: str,
    goal_w,
    *,
    vmax: float,
    dmin: float,
    state: dict,
    keepout: bool,
    drive: dict,
    nav_actor: str | None = None,
    carry_host: str | None = None,
    sim_rate: float = 1.0,
    wall_dt_s: float = PARALLEL_DT_S,
    look_at=None,
) -> bool:
    """XY from hops (this is what actually translates). Move_Speed is the walk cycle."""
    if not drive.get("walk_body"):
        _spot_walk_body(ucv, robot)
        drive["walk_body"] = True
    try:
        eng = ucv.get_location(robot)
    except Exception:
        eng = drive.get("pose") or [0.0, 0.0, 0.0]
    pose = drive.get("pose")
    if pose is not None:
        loc = [float(pose[0]), float(pose[1]), float(eng[2])]
    else:
        loc = [float(eng[0]), float(eng[1]), float(eng[2])]
    loc = _ground_root(
        ucv,
        nav_actor,
        loc[0],
        loc[1],
        clearance_cm=SPOT_FOOT_Z_CM,
        z_hint_cm=loc[2],
        prev_root_z_cm=loc[2],
    )
    drive["pose"] = loc
    goal_w = _ground_root(
        ucv,
        nav_actor,
        goal_w[0],
        goal_w[1],
        clearance_cm=SPOT_FOOT_Z_CM,
        z_hint_cm=loc[2],
        prev_root_z_cm=loc[2],
    )
    if _arrived_w(loc, goal_w) and _same_floor_w(loc, goal_w):
        face = _yaw_toward(goal_w, look_at) if look_at is not None else None
        _spot_halt(ucv, robot, drive, face_yaw=face)
        drive["pose"] = list(goal_w)
        ucv.set_location(list(goal_w), robot)
        return True
    hop_w = _hop_world(loc, goal_w, SPOT_FOOT_Z_CM)
    on_sc = _spot_on_scaffold_w(loc)
    cmd = float(vmax)
    speed, mode = cmd, "free"
    sep = float("nan")
    if state.get("refuge_waived") or state.get("keepout_waived"):
        state.pop("spot_locked", None)
        state.pop("asm_lock_xy", None)
    enforce = bool(
        keepout
        and on_sc
        and not state.get("keepout_waived")
        and not state.get("refuge_waived")
    )
    if enforce:
        hum = ucv.get_location(assembler)
        sep = _horiz_m(loc, hum)
        if abs(loc[2] - hum[2]) <= 120.0:
            speed, mode = apply_ssm(sep, cmd, on_site=True)
            if sep < dmin:
                speed, mode = 0.0, "dmin_stop"
    sim_dt = playback_sim_dt_s(wall_dt_s, sim_rate)

    def _request_assembler_retreat(reason: str) -> None:
        # Oracle parity: only erect/pickup dwells are sticky; holding walks to refuge.
        if state.get("evac_requested") or state.get("asm_working"):
            return
        print("spot keepout", reason, "sep_m", round(sep, 3) if sep == sep else sep)
        print("assembler retreat")
        state["evac_requested"] = True

    if state.get("spot_locked"):
        hum = ucv.get_location(assembler)
        anchor = state.get("asm_lock_xy")
        if (
            anchor is not None
            and not state.get("asm_working")
            and math.hypot(hum[0] - anchor[0], hum[1] - anchor[1]) > 35.0
        ):
            state.pop("spot_locked", None)
            state.pop("asm_lock_xy", None)
            # Keep evac_requested so Assembler finishes the trip to refuge.
        else:
            _spot_halt(ucv, robot, drive)
            drive["pose"] = loc
            state["wait_s"] += sim_dt
            if enforce:
                _request_assembler_retreat(mode if mode in ("stop", "dmin_stop") else "locked")
            return False
    if mode in ("stop", "dmin_stop") or speed <= 1e-6:
        _spot_halt(ucv, robot, drive)
        drive["pose"] = loc
        state["wait_s"] += sim_dt
        if enforce:
            if state.get("asm_lock_xy") is None:
                hum = ucv.get_location(assembler)
                state["asm_lock_xy"] = (float(hum[0]), float(hum[1]))
            state["spot_locked"] = True
            _request_assembler_retreat(mode)
        return False
    hop_remain = math.hypot(hop_w[0] - loc[0], hop_w[1] - loc[1])
    desired = _yaw_toward(loc, hop_w)
    step = playback_step_cm(
        speed_cm_s=speed * 100.0,
        wall_dt_s=wall_dt_s,
        sim_rate=sim_rate,
        remain_cm=hop_remain,
    )
    if step > 0.5:
        t = step / max(hop_remain, 1e-6)
        loc = _ground_root(
            ucv,
            nav_actor,
            loc[0] + t * (hop_w[0] - loc[0]),
            loc[1] + t * (hop_w[1] - loc[1]),
            clearance_cm=SPOT_FOOT_Z_CM,
            z_hint_cm=loc[2],
            prev_root_z_cm=loc[2],
        )
        ucv.set_location(loc, robot)
    drive["pose"] = loc
    live = not should_reissue_drive(
        now=time.time(), cmd_until=float(drive.get("until") or 0.0)
    )
    if not live:
        goal_remain = math.hypot(goal_w[0] - loc[0], goal_w[1] - loc[1])
        remain = gait_command_remain_cm(
            hop_remain_cm=hop_remain,
            goal_remain_cm=goal_remain,
            hop_yaw_err_deg=yaw_error_deg(desired, _yaw_toward(loc, goal_w)),
        )
        _spot_play_gait(
            ucv,
            robot,
            drive,
            speed_cm_s=speed * 100.0,
            remain_cm=remain,
            yaw_deg=float(desired),
            start_xy=loc,
            sim_rate=sim_rate,
        )
    drive["moving"] = True
    if carry_host:
        _attach_carry(ucv, carry_host)
    if on_sc:
        state["scaffold_tt_s"] += sim_dt
    if keepout and on_sc and speed > 0.05 and sep == sep:
        state["min_sep"] = min(state["min_sep"], sep)
        state["si_min"] = min(
            state["si_min"], sep / max(protective_separation_m(speed), 1e-6)
        )
        state["n_samples"] += 1
    return False


def _human_stop(ucv, name: str) -> None:
    for fn in (
        lambda: ucv.humanoid_stop(name),
        lambda: ucv.humanoid_stop_current_action(name),
    ):
        try:
            fn()
        except Exception:
            pass


def _scaffold_depth_m_from_world(loc) -> float:
    """Oracle scaffold-x from UE local Y (entrance sill → deck), same as Spot keep-out."""
    ly = lc.world_xyz_to_local(float(loc[0]), float(loc[1]), 0.0)[1]
    return (ly - ENTRANCE_LOCAL_XY_CM[1]) / 100.0


def _spot_on_scaffold_w(loc) -> bool:
    return spot_on_scaffold_local_x(
        local_x_m=_scaffold_depth_m_from_world(loc),
        ramp_yard_x_m=ramp_yard_x_m(),
    )


def _clamp_work_world(wx: float, wy: float, wz: float) -> list[float]:
    lx, ly, lz = lc.world_xyz_to_local(wx, wy, wz)
    x0, y0, x1, y1 = footprint_local_cm()
    pad = 250.0
    lx = min(max(lx, x0 - pad), x1 + pad)
    ly = min(max(ly, y0 - pad), y1 + pad)
    nx, ny, nz = lc.local_xyz_to_world(lx, ly, lz)
    return [nx, ny, nz]


def _hold_assembler(ucv, name: str, pose) -> None:
    """Keep CharacterMovement from launching the pawn during a dwell."""
    _ground_human(ucv, name)
    if pose is None:
        return
    ucv.set_location([pose[0], pose[1], pose[2]], name)


def _human_tick(
    ucv,
    name: str,
    goal_w,
    state: dict,
    gait: dict,
    *,
    nav_actor: str | None = None,
    carry: bool = False,
    carry_actor: str | None = None,
    speed_cm_s: float | None = None,
    sim_rate: float = 1.0,
    wall_dt_s: float = PARALLEL_DT_S,
) -> bool:
    """Steer XY by set_location; Z follows hop on stairs, else NavProject."""
    loc = ucv.get_location(name)
    loc = _ground_root(
        ucv,
        nav_actor,
        loc[0],
        loc[1],
        clearance_cm=ASSEMBLER_FOOT_Z_CM,
        z_hint_cm=loc[2],
        prev_root_z_cm=loc[2],
    )
    ucv.set_location(loc, name)
    _ground_human(ucv, name)
    held = carry_actor or CARRY
    speed = HUMAN_SPEED_CM_S if speed_cm_s is None else float(speed_cm_s)
    remain_goal = math.hypot(goal_w[0] - loc[0], goal_w[1] - loc[1])
    if assembler_is_lost(remain_cm=remain_goal):
        print("assembler recover remain_cm", round(remain_goal, 1))
        clamped = _clamp_work_world(goal_w[0], goal_w[1], loc[2])
        loc = _ground_root(
            ucv,
            nav_actor,
            clamped[0],
            clamped[1],
            clearance_cm=ASSEMBLER_FOOT_Z_CM,
            z_hint_cm=clamped[2],
            prev_root_z_cm=None,
        )
        ucv.set_location(loc, name)
        gait.clear()
        remain_goal = math.hypot(goal_w[0] - loc[0], goal_w[1] - loc[1])
    try:
        ucv.set_physics(name, False)
    except Exception:
        pass
    if pawn_arrived(
        horiz_m=_horiz_m(loc, goal_w),
        same_floor=_same_floor_w(loc, goal_w),
        horiz_tol_m=0.50,
    ):
        _human_stop(ucv, name)
        gait.clear()
        if carry:
            _attach_actor(ucv, name, held, dz_cm=30.0)
        return True
    hop_w = _hop_world(loc, goal_w, ASSEMBLER_FOOT_Z_CM)
    dz = float(hop_w[2]) - float(loc[2])
    remain_xy = math.hypot(hop_w[0] - loc[0], hop_w[1] - loc[1])
    # Include vertical so stair hops are not treated as "already there".
    remain = math.hypot(remain_xy, dz)
    climbing = abs(dz) > 25.0
    key = (round(goal_w[0], 0), round(goal_w[1], 0), round(goal_w[2], 0))
    if gait.get("key") != key:
        _human_stop(ucv, name)
        gait.clear()
        gait["key"] = key
        gait["last_remain"] = remain
        ucv.set_orientation([0.0, _yaw_toward(loc, hop_w), 0.0], name)
    step = playback_step_cm(
        speed_cm_s=speed,
        wall_dt_s=wall_dt_s,
        sim_rate=sim_rate,
        remain_cm=remain,
    )
    if remain > 1e-6:
        t = step / remain
        nx = loc[0] + t * (hop_w[0] - loc[0])
        ny = loc[1] + t * (hop_w[1] - loc[1])
        if climbing:
            # NavMesh often lacks AABB tread polys — trust kinematic hop Z.
            posed = [nx, ny, loc[2] + t * dz]
        else:
            posed = _ground_root(
                ucv,
                nav_actor,
                nx,
                ny,
                clearance_cm=ASSEMBLER_FOOT_Z_CM,
                z_hint_cm=loc[2],
                prev_root_z_cm=loc[2],
            )
            posed = _clamp_work_world(posed[0], posed[1], posed[2])
            posed = _ground_root(
                ucv,
                nav_actor,
                posed[0],
                posed[1],
                clearance_cm=ASSEMBLER_FOOT_Z_CM,
                z_hint_cm=posed[2],
                prev_root_z_cm=loc[2],
            )
    else:
        posed = list(hop_w) if climbing else _ground_root(
            ucv,
            nav_actor,
            hop_w[0],
            hop_w[1],
            clearance_cm=ASSEMBLER_FOOT_Z_CM,
            z_hint_cm=loc[2],
            prev_root_z_cm=loc[2],
        )
    ucv.set_orientation([0.0, _yaw_toward(loc, hop_w), 0.0], name)
    ucv.set_location(posed, name)
    stall = int(gait.get("stall", 0))
    if remain > 80.0 and gait.get("last_remain") is not None and remain > float(gait["last_remain"]) - 1.0:
        stall += 1
        if stall % 20 == 1:
            loc_l = _local_of(posed)
            print(
                "assembler walk remain_cm",
                round(remain, 1),
                "local",
                tuple(round(v, 1) for v in loc_l),
            )
        if stall >= 25 and remain > 400.0:
            snap = min(remain, 400.0)
            t = snap / max(remain, 1e-6)
            if climbing:
                posed = [
                    loc[0] + t * (hop_w[0] - loc[0]),
                    loc[1] + t * (hop_w[1] - loc[1]),
                    loc[2] + t * dz,
                ]
            else:
                posed = _ground_root(
                    ucv,
                    nav_actor,
                    loc[0] + t * (hop_w[0] - loc[0]),
                    loc[1] + t * (hop_w[1] - loc[1]),
                    clearance_cm=ASSEMBLER_FOOT_Z_CM,
                    z_hint_cm=loc[2],
                    prev_root_z_cm=loc[2],
                )
                posed = _clamp_work_world(posed[0], posed[1], posed[2])
            ucv.set_location(posed, name)
            print("assembler unstuck snap_cm", round(snap, 1))
            gait["stall"] = 0
            gait["last_remain"] = None
            return False
    else:
        stall = 0
    gait["stall"] = stall
    gait["last_remain"] = remain
    if carry:
        _attach_actor(ucv, name, held, dz_cm=30.0)
    return False


def _poses_for(item, stairs: set[int], slot: int = 0):
    work_floor = min(item.floor, reachable_floor(stairs))
    drop_floor = floor_drop_slot_local_cm(work_floor, slot)
    install = assembler_install_local_cm(item, walk_floor=work_floor)
    return (
        _world(drop_floor),
        _world(assembler_stand_local_cm(drop_floor)),
        _world(assembler_stand_local_cm(install)),
    )


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--vmax", type=float, required=True)
    p.add_argument("--dmin", type=float, required=True)
    p.add_argument(
        "--protocol",
        choices=("smoke", "smin", "full"),
        default=None,
        help="smoke=3 posts; smin=first keep-out leg; full=all members",
    )
    p.add_argument("--max-items", type=int, default=8)
    p.add_argument("--max-floors", type=int, default=1)
    p.add_argument("--boards-per-floor", type=int, default=2)
    p.add_argument(
        "--require-smin",
        action="store_true",
        help="fail unless keep-out S_min was sampled (implied by protocol smin/full)",
    )
    p.add_argument("--out", type=Path, default=None)
    p.add_argument(
        "--sim-rate",
        type=float,
        default=4.0,
        help="UE playback rate (slomo). 4 is the UnrealCV sync ceiling.",
    )
    args = p.parse_args()

    ucv, _ = ue_client_guard.prepare_ue_connection()
    if not geh._ping_ucv(ucv):  # noqa: SLF001
        print("FAIL: unrealcv")
        return 1
    if TRUCK_ACTOR not in set(geh.actor_names(ucv)):
        print("FAIL: run spawn_scaffold_pie.py --bare first")
        return 1

    sx, sy = STAGING_LOCAL_XY_CM
    yard_local = (
        sx + YARD_HUMAN_SIDE_CM,
        sy + YARD_HUMAN_ALONG_CM,
        ASSEMBLER_FOOT_Z_CM,
    )
    asm_local = assembler_stand_local_cm(floor_drop_slot_local_cm(1, 0))
    _park_extra_humanoids(ucv, (YARD_HUMAN, ASSEMBLER))
    if not _ensure_human(ucv, YARD_HUMAN, yard_local, YARD_HUMAN_YAW_DEG):
        return 1
    if not _ensure_human(ucv, ASSEMBLER, asm_local, 180.0):
        return 1
    _park_extra_humanoids(ucv, (YARD_HUMAN, ASSEMBLER))

    ok, nav_actor = nq.ensure_nav_query_service(ucv)
    if not ok:
        return 1
    nq.nav_rebuild(ucv, nav_actor)
    robot_ok, robot = ensure_level_spotdog(ucv, SPOT_YARD_LOCAL_XY_CM)
    if not robot_ok:
        return 1
    _teleport_body(ucv, robot)
    wx, wy, wz = lc.local_xyz_to_world(
        SPOT_YARD_LOCAL_XY_CM[0], SPOT_YARD_LOCAL_XY_CM[1], 0.0
    )
    ucv.set_location(
        _ground_root(
            ucv,
            nav_actor,
            wx,
            wy,
            clearance_cm=SPOT_FOOT_Z_CM,
            z_hint_cm=wz + SPOT_FOOT_Z_CM,
            prev_root_z_cm=None,
        ),
        robot,
    )

    try:
        cooldown_before_spawn_batch(ucv, reason="hrc_mission start after bare")
    except PieSessionLost as exc:
        print("FAIL:", exc)
        return 1

    global _PLAYBACK_RATE
    sim_rate = max(0.25, float(args.sim_rate))
    if not _set_slomo(ucv, sim_rate):
        sim_rate = 1.0
        print("sim_rate fallback 1.0")
    _PLAYBACK_RATE = sim_rate

    if args.protocol:
        seq = select_erect_sequence(args.protocol)
        require_smin = (
            ue_protocol_limits(args.protocol).require_smin or args.require_smin
        )
        protocol = args.protocol
    else:
        seq = limit_erect_sequence(
            build_erect_sequence(STAGE1_GEOM),
            max_items=None if args.max_items <= 0 else args.max_items,
            max_floors=None if args.max_floors <= 0 else args.max_floors,
            boards_per_floor=(
                None if args.boards_per_floor <= 0 else args.boards_per_floor
            ),
        )
        require_smin = args.require_smin
        protocol = "custom"
    print("erect_items", len(seq), [i.item_id for i in seq[:5]], "...", "protocol", protocol)

    # Load/stop beside the kei bed, not on the truck centerline.
    yard_w = _world(
        (SPOT_YARD_LOCAL_XY_CM[0], SPOT_YARD_LOCAL_XY_CM[1], 0.0)
    )
    truck_w = _world(
        (STAGING_LOCAL_XY_CM[0], STAGING_LOCAL_XY_CM[1], 0.0)
    )

    state = {
        "tt_s": 0.0,
        "scaffold_tt_s": 0.0,
        "wait_s": 0.0,
        "min_sep": float("inf"),
        "si_min": float("inf"),
        "n_samples": 0,
        "n_filled": 0,
        "evac_requested": False,
        "keepout_waived": False,
        "spot_locked": False,
        "asm_lock_xy": None,
        "refuge_waived": False,
    }
    wall0 = time.time()

    unlocked_decks: set[int] = set()
    completed_stairs: set[int] = set()
    pending = deque(seq)
    spot_item = None
    staged: deque = deque()
    occupied: set[int] = set()
    asm_item = None
    spot_slot: int | None = None
    asm_slot: int | None = None
    asm_carry: str | None = None
    spot_loaded = False
    holding = False
    erecting = False
    picking = False
    drain_hold = False
    load_until: float | None = None
    place_until: float | None = None
    erect_until: float | None = None
    pickup_until: float | None = None
    sim_t = 0.0
    gait: dict = {}
    spot_drive: dict = {}
    asm_pose = None
    n_seq = len(seq)

    def _ensure_carry() -> None:
        if geh.actor_exists(ucv, CARRY):
            _make_carry_solid(ucv)
            _attach_carry(ucv, robot)
            return
        ok, _ = spawn_bp_resilient(
            ucv,
            geh.CUBE_BP,
            CARRY,
            timeout_s=12.0,
            playback_rate=_PLAYBACK_RATE,
            pause_playback=False,
        )
        if ok and geh.actor_exists(ucv, CARRY):
            _make_carry_solid(ucv)
            _attach_carry(ucv, robot)
        else:
            print("WARN carry missing; driving without cargo mesh")

    _ensure_carry()
    _hide_actor(ucv, CARRY)

    prev_tick = time.time()
    while pending or spot_item or staged or asm_item:
        tick0 = time.time()
        wall_dt = tick0 - prev_tick
        prev_tick = tick0
        sim_dt = playback_sim_dt_s(wall_dt, sim_rate)
        # Sticky only for dwell tasks (oracle erect_bare). Holding may retreat with cargo.
        state["asm_working"] = bool(erecting or picking)
        if spot_loaded and spot_slot is None:
            spot_slot = first_free_drop_slot(occupied)
        has_free_slot = (not spot_loaded) or spot_slot is not None
        slot_for_spot = spot_slot if spot_slot is not None else 0
        if spot_item is not None:
            drop_w, _, _ = _poses_for(spot_item, completed_stairs, slot_for_spot)
        else:
            drop_w = _world(floor_drop_slot_local_cm(1, slot_for_spot))
        loc_s = list(spot_drive["pose"]) if spot_drive.get("pose") else ucv.get_location(robot)
        at_yard = _horiz_m(loc_s, yard_w) < 0.70
        at_drop = _horiz_m(loc_s, drop_w) < 0.70 and _same_floor_w(loc_s, drop_w)
        if len(occupied) >= DROP_SLOT_COUNT:
            drain_hold = True
        elif not occupied:
            drain_hold = False
        if drain_hold and at_yard and not spot_loaded:
            # Spot is clear of the deck: Assembler must drain staging, not sit in refuge.
            state["keepout_waived"] = True
            state["evac_requested"] = False
            state["refuge_waived"] = False
            state.pop("spot_locked", None)
            state.pop("asm_lock_xy", None)
        intent = next_spot_mode(
            pending=bool(pending),
            loaded=spot_loaded,
            has_free_slot=bool(has_free_slot) if spot_loaded else True,
            at_yard=at_yard,
            at_drop=at_drop,
            drain_hold=drain_hold,
            assembler_busy=asm_item is not None,
        )
        if intent == "loading" and load_until is not None and sim_t < load_until:
            intent = "loading"
        elif intent == "loading" and load_until is None:
            load_until = sim_t + STAGE1_PROTOCOL.truck_load_s
            print(
                "spot load",
                "dwell_s",
                STAGE1_PROTOCOL.truck_load_s,
                "wall_s",
                round(wall_from_sim(STAGE1_PROTOCOL.truck_load_s, sim_rate), 2),
            )
        if (
            intent == "loading"
            and load_until is not None
            and sim_t >= load_until
        ):
            if pending and not spot_loaded:
                spot_item = pending.popleft()
                spot_loaded = True
                print("spot carry", spot_item.item_id, spot_item.kind)
                _ensure_carry()
            load_until = None
            if spot_slot is None:
                spot_slot = first_free_drop_slot(occupied)
            intent = "to_drop" if spot_slot is not None else "wait_drop"

        if intent == "placing" and place_until is None:
            place_until = sim_t + STAGE1_PROTOCOL.drop_place_s
            print(
                "spot place",
                "dwell_s",
                STAGE1_PROTOCOL.drop_place_s,
                "wall_s",
                round(wall_from_sim(STAGE1_PROTOCOL.drop_place_s, sim_rate), 2),
            )
        if intent == "placing" and place_until is not None and sim_t >= place_until:
            if spot_slot is None:
                spot_slot = first_free_drop_slot(occupied)
            if spot_item is not None and spot_slot is not None:
                drop_w, _, _ = _poses_for(spot_item, completed_stairs, spot_slot)
                actor = _stage_actor(spot_item.item_id)
                _place_stage(ucv, actor, drop_w)
                occupied.add(spot_slot)
                staged.append((spot_slot, spot_item, actor))
                print("spot drop", spot_item.item_id, "slot", spot_slot)
            _hide_actor(ucv, CARRY)
            spot_item = None
            spot_loaded = False
            spot_slot = None
            place_until = None
        elif intent == "placing":
            _spot_halt(ucv, robot, spot_drive)
            _attach_carry(ucv, robot)

        if intent == "to_yard":
            _spot_drive(
                ucv,
                robot,
                ASSEMBLER,
                yard_w,
                vmax=args.vmax,
                dmin=args.dmin,
                state=state,
                keepout=False,
                drive=spot_drive,
                sim_rate=sim_rate,
                wall_dt_s=wall_dt,
                nav_actor=nav_actor,
                look_at=truck_w,
            )
        elif intent == "to_drop" and spot_item is not None:
            if spot_slot is None:
                spot_slot = first_free_drop_slot(occupied)
            if spot_slot is None:
                intent = "wait_drop"
            else:
                drop_w, _, _ = _poses_for(spot_item, completed_stairs, spot_slot)
                _spot_drive(
                    ucv,
                    robot,
                    ASSEMBLER,
                    drop_w,
                    vmax=args.vmax,
                    dmin=args.dmin,
                    state=state,
                    keepout=STAGE1_PROTOCOL.assembler_keepout_on_scaffold,
                    drive=spot_drive,
                    carry_host=robot,
                    sim_rate=sim_rate,
                    wall_dt_s=wall_dt,
                nav_actor=nav_actor,
                )
        if intent == "wait_drop" or (intent == "idle" and drain_hold):
            if not state.get("drain_logged"):
                print(
                    "spot staging full waiting at yard",
                    len(occupied),
                    "/",
                    DROP_SLOT_COUNT,
                    "staged",
                    len(staged),
                )
                state["drain_logged"] = True
            _spot_drive(
                ucv,
                robot,
                ASSEMBLER,
                yard_w,
                vmax=args.vmax,
                dmin=args.dmin,
                state=state,
                keepout=False,
                drive=spot_drive,
                carry_host=robot if spot_loaded else None,
                sim_rate=sim_rate,
                wall_dt_s=wall_dt,
                nav_actor=nav_actor,
                look_at=truck_w,
            )
        elif intent in ("loading", "idle"):
            _spot_halt(ucv, robot, spot_drive)
            if spot_loaded:
                _attach_carry(ucv, robot)
        if not drain_hold:
            state["drain_logged"] = False
        if not spot_loaded:
            _hide_actor(ucv, CARRY)

        if erecting and erect_until is not None and sim_t >= erect_until:
            if asm_item is not None:
                if asm_carry:
                    _hide_actor(ucv, asm_carry)
                _hold_assembler(ucv, ASSEMBLER, asm_pose)
                if not _spawn_member(ucv, asm_item):
                    print("WARN spawn failed", asm_item.item_id)
                else:
                    state["n_filled"] += 1
                    print("assembler placed", asm_item.item_id, state["n_filled"], "/", n_seq)
                    if asm_item.unlocks_deck:
                        unlocked_decks.add(asm_item.floor)
                        _spawn_climb_aids(ucv, floor=asm_item.floor)
                    if asm_item.unlocks_stair_lift is not None:
                        completed_stairs.add(asm_item.unlocks_stair_lift)
                        _spawn_climb_aids(ucv, stair_lift=asm_item.unlocks_stair_lift)
                    nq.nav_rebuild(ucv, nav_actor)
                _hold_assembler(ucv, ASSEMBLER, asm_pose)
            asm_item = None
            asm_slot = None
            asm_carry = None
            holding = False
            erecting = False
            erect_until = None
            gait.clear()
            _human_stop(ucv, ASSEMBLER)
            _hold_assembler(ucv, ASSEMBLER, asm_pose)

        # Use Spot's pose AFTER this tick's drive/halt (not the stale start-of-tick copy).
        loc_s_now = (
            list(spot_drive["pose"])
            if spot_drive.get("pose")
            else ucv.get_location(robot)
        )
        spot_on_sc = _spot_on_scaffold_w(loc_s_now)
        delivering = spot_is_delivering(intent)
        if not delivering:
            # Returning / waiting at yard: release Assembler so staging can drain.
            state["evac_requested"] = False
            state["refuge_waived"] = False
            state.pop("spot_locked", None)
            state.pop("asm_lock_xy", None)
            if not drain_hold and not spot_on_sc:
                state["keepout_waived"] = False
        elif not spot_on_sc:
            state["evac_requested"] = False
            if state.get("refuge_waived"):
                state["refuge_waived"] = False
                if not drain_hold:
                    state["keepout_waived"] = False
        handoff_blocked = spot_blocks_staging_handoff(
            spot_at_drop=at_drop and spot_on_sc,
            spot_placing=intent == "placing",
            spot_loaded_en_route=spot_loaded and delivering,
        )
        if assembler_should_yield_to_locked_spot(
            spot_locked=bool(state.get("spot_locked")),
            erecting=erecting,
            picking=picking,
        ):
            state["evac_requested"] = True
        evac = bool(state.get("evac_requested"))
        refuge_hold = assembler_holds_refuge(
            refuge_waived=bool(state.get("refuge_waived")),
            spot_on_scaffold=spot_on_sc,
            spot_delivering=delivering,
        )
        assigned = asm_item is not None and not holding and not erecting and not picking
        asm_intent = next_asm_mode(
            cargo_at_drop=bool(staged) and not handoff_blocked,
            holding=holding,
            erecting=erecting,
            assigned=assigned,
            evac=evac,
            picking=picking,
            refuge_hold=refuge_hold,
        )
        if asm_intent == "to_refuge":
            refuge_w = _world(
                assembler_stand_local_cm(
                    floor_refuge_local_cm(reachable_floor(completed_stairs))
                )
            )
            if _human_tick(
                ucv,
                ASSEMBLER,
                refuge_w,
                state,
                gait,
                carry=holding,
                carry_actor=asm_carry,
                speed_cm_s=HUMAN_RETREAT_CM_S,
                sim_rate=sim_rate,
                wall_dt_s=wall_dt,
                nav_actor=nav_actor,
            ):
                if not state.get("refuge_logged"):
                    print("assembler refuge")
                    state["refuge_logged"] = True
                state["evac_requested"] = False
                state["refuge_waived"] = True
                state["keepout_waived"] = True
                state.pop("spot_locked", None)
                state.pop("asm_lock_xy", None)
            else:
                state["refuge_logged"] = False
        else:
            state["refuge_logged"] = False
            if asm_intent == "to_drop" and staged and asm_item is None:
                asm_slot, asm_item, asm_carry = staged.popleft()
                print("assembler pickup", asm_item.item_id, "slot", asm_slot)
        if asm_intent == "to_drop" and asm_item is not None:
            _, drop_hip, _ = _poses_for(asm_item, completed_stairs, asm_slot or 0)
            if _human_tick(
                ucv,
                ASSEMBLER,
                drop_hip,
                state,
                gait,
                sim_rate=sim_rate,
                wall_dt_s=wall_dt,
                nav_actor=nav_actor,
            ):
                picking = True
                pickup_until = sim_t + STAGE1_PROTOCOL.assembler_pickup_s
                asm_pose = ucv.get_location(ASSEMBLER)
                print(
                    "assembler lift",
                    asm_item.item_id,
                    "dwell_s",
                    STAGE1_PROTOCOL.assembler_pickup_s,
                    "wall_s",
                    round(
                        wall_from_sim(STAGE1_PROTOCOL.assembler_pickup_s, sim_rate),
                        2,
                    ),
                )
                gait.clear()
                _human_stop(ucv, ASSEMBLER)
                _hold_assembler(ucv, ASSEMBLER, asm_pose)
        elif asm_intent == "pickup":
            _hold_assembler(ucv, ASSEMBLER, asm_pose)
            if asm_carry:
                _attach_actor(ucv, ASSEMBLER, asm_carry, dz_cm=30.0)
            if pickup_until is not None and sim_t >= pickup_until:
                occupied.discard(asm_slot if asm_slot is not None else 0)
                picking = False
                holding = True
                pickup_until = None
        elif asm_intent == "to_socket" and asm_item is not None:
            _, _, socket_hip = _poses_for(asm_item, completed_stairs, asm_slot or 0)
            if _human_tick(
                ucv,
                ASSEMBLER,
                socket_hip,
                state,
                gait,
                carry=True,
                carry_actor=asm_carry,
                sim_rate=sim_rate,
                wall_dt_s=wall_dt,
                nav_actor=nav_actor,
            ):
                holding = False
                erecting = True
                dwell = (
                    STAGE1_PROTOCOL.board_erect_s
                    if asm_item.kind == "board"
                    else STAGE1_PROTOCOL.structural_erect_s
                )
                erect_until = sim_t + dwell
                print(
                    "assembler erect",
                    asm_item.item_id,
                    "dwell_s",
                    dwell,
                    "wall_s",
                    round(wall_from_sim(dwell, sim_rate), 2),
                )
                gait.clear()
                asm_pose = ucv.get_location(ASSEMBLER)
                _human_stop(ucv, ASSEMBLER)
                _hold_assembler(ucv, ASSEMBLER, asm_pose)
        elif asm_intent == "erect":
            _hold_assembler(ucv, ASSEMBLER, asm_pose)
            if asm_carry:
                _attach_actor(ucv, ASSEMBLER, asm_carry, dz_cm=30.0)
        elif asm_intent == "idle":
            _human_stop(ucv, ASSEMBLER)
            _hold_assembler(ucv, ASSEMBLER, asm_pose)

        sim_t += sim_dt
        state["tt_s"] = sim_t
        remain = wall_tick_remainder_s(
            spent_s=time.time() - tick0,
            sim_dt=sim_dt,
            sim_rate=sim_rate,
        )
        if remain > 0.0:
            time.sleep(remain)

        if time.time() - wall0 > 43200:
            print("FAIL: pipeline timeout")
            break
    wall = time.time() - wall0
    _set_slomo(ucv, 1.0)
    result = {
        "ok": state["n_filled"] == len(seq),
        "protocol": protocol,
        "vmax_mps": args.vmax,
        "dmin_m": args.dmin,
        "sim_rate": sim_rate,
        "n_items": len(seq),
        "n_filled": state["n_filled"],
        "n_unlocked_decks": len(unlocked_decks),
        "n_keepout_samples": state["n_samples"],
        "tt_s": round(state["tt_s"], 2),
        "scaffold_tt_s": round(state["scaffold_tt_s"], 2),
        "wait_s": round(state["wait_s"], 2),
        "smin_m": None
        if state["min_sep"] == float("inf")
        else round(state["min_sep"], 4),
        "si_min": None if state["si_min"] == float("inf") else round(state["si_min"], 4),
        "wall_s": round(wall, 2),
        "erect_from_bare": True,
        "keepout_scope": "assembler_on_scaffold_only",
    }
    if require_smin and result["smin_m"] is None:
        result["ok"] = False
        result["error"] = "smin_missing"
    print(json.dumps(result, indent=2))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2) + "\n")
        print(args.out)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
