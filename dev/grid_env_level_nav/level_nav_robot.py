#!/usr/bin/env python3
"""Spawn / place SpotDog on /Game/Maps/Level for layered nav smoke."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

_THIS_DIR = Path(__file__).resolve().parent
_ROOT = _THIS_DIR.parent.parent
_GEH_DIR = _ROOT / "dev" / "grid_env_hri"
for _p in (_GEH_DIR, _THIS_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import grid_env_hri_simulation as geh  # noqa: E402
import nav_query as nq  # noqa: E402
from level_coords import (  # noqa: E402
    FLOOR_REF_Z_CM,
    NAV_PROJECT_PROBE_Z_CM,
    foot_world_xyz_from_local_xy,
    world_xy_to_local,
)
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

LocalXY = Tuple[float, float]
LEVEL_ROBOT_NAME = geh.ROBOT_ACTOR_NAME
SPOTDOG_AI_CONTROLLER_BP = (
    "/Game/Robot_Dog/Blueprint/BP_SpotDogAIController.BP_SpotDogAIController_C"
)
SIGHT_AI_CONTROLLER_NAME = "SpotDogSightAI"
ROBOT_SETTLE_S = 0.5
ROBOT_NAV_XY_TOLERANCE_CM = 120.0
ROBOT_FOOT_Z_OFFSET_CM = 50.0
# Pawn destroy + UE GC on Level needs longer idle (no clean_garbage).
ROBOT_DESTROY_SETTLE_S = 3.0
# Stash far below floor before destroy (keeps camera/controller away from NavMesh).
ROBOT_STASH_WORLD_Z_CM = FLOOR_REF_Z_CM - 50_000.0


def _configure_robot_at(
    ucv: UnrealCV,
    loc: Tuple[float, float, float],
    *,
    actor_name: str = LEVEL_ROBOT_NAME,
    yaw_deg: float = 0.0,
) -> None:
    """Match grid_env_10k_pie_patrol SpotDog settings (controller required for dog_move)."""
    ucv.set_physics(actor_name, False)
    ucv.set_movable(actor_name, True)
    ucv.set_location(list(loc), actor_name)
    ucv.set_orientation((0.0, yaw_deg, 0.0), actor_name)
    ucv.set_collision(actor_name, True)
    ucv.enable_controller(actor_name, True)
    time.sleep(geh.PHYSICS_ENABLE_DELAY_S)


def get_robot_orientation_deg(ucv: UnrealCV, actor_name: str) -> Tuple[float, float, float]:
    ori = ucv.get_orientation(actor_name)
    return float(ori[0]), float(ori[1]), float(ori[2])


def is_robot_tipped(
    ucv: UnrealCV,
    actor_name: str,
    *,
    pitch_roll_thr_deg: float = 18.0,
) -> bool:
    pitch, _yaw, roll = get_robot_orientation_deg(ucv, actor_name)
    return abs(pitch) > pitch_roll_thr_deg or abs(roll) > pitch_roll_thr_deg


def recover_robot_upright(
    ucv: UnrealCV,
    actor_name: str,
    target_world_xy: Tuple[float, float],
    *,
    nav_actor: Optional[str] = None,
    yaw_deg: Optional[float] = None,
) -> Tuple[bool, Tuple[float, float]]:
    """Teleport robot upright onto NavMesh without destroy (fallen / wedged recovery)."""
    wx, wy = float(target_world_xy[0]), float(target_world_xy[1])
    wz = FLOOR_REF_Z_CM + ROBOT_FOOT_Z_OFFSET_CM
    if nav_actor:
        try:
            raw = nq.nav_project_point(ucv, nav_actor, wx, wy, NAV_PROJECT_PROBE_Z_CM)
        except Exception:
            raw = {"ok": False}
        if raw.get("ok"):
            px, py, pz = float(raw["x"]), float(raw["y"]), float(raw["z"])
            wx, wy, wz = px, py, pz + ROBOT_FOOT_Z_OFFSET_CM
    if yaw_deg is None:
        try:
            _pitch, yaw_deg_val, _roll = get_robot_orientation_deg(ucv, actor_name)
            yaw_deg = yaw_deg_val
        except Exception:
            yaw_deg = 0.0
    try:
        _configure_robot_at(ucv, (wx, wy, wz), actor_name=actor_name, yaw_deg=yaw_deg)
        lx, ly = world_xy_to_local(wx, wy)
        print(
            f"[LevelRobot] upright-recover {actor_name!r} world=({wx:.1f}, {wy:.1f}, {wz:.1f}) "
            f"local=({lx:.1f}, {ly:.1f}) yaw={yaw_deg:.1f}"
        )
        return True, (wx, wy)
    except Exception as exc:
        print(f"[LevelRobot] upright-recover failed for {actor_name!r}: {exc}")
        return False, (wx, wy)


def _spotdog_ai_controller_names(ucv: UnrealCV) -> list[str]:
    return sorted(
        n for n in geh.actor_names(ucv) if "SpotDogAIController" in n
    )


def _vbp_ok(raw: Optional[str]) -> bool:
    if raw is None:
        return False
    text = str(raw).strip().lower()
    return bool(text) and not text.startswith("error")


def _sight_vbp_returns_json(ucv: UnrealCV, robot_name: str) -> bool:
    """True when pawn sight vbp returns parseable JSON (controller cast OK)."""
    for cmd in ("GetVisibleSightTargetsJson", "GetSightPerceptionJson"):
        try:
            raw = geh._ue_request(ucv, f"vbp {robot_name} {cmd}", timeout_s=15.0)  # noqa: SLF001
        except (ConnectionError, OSError, RuntimeError, ValueError):
            continue
        if raw is None:
            continue
        text = str(raw).strip()
        if not text or text.lower().startswith("error"):
            continue
        if "targets" in text or "actors" in text or text.startswith("{"):
            return True
    return False


def _try_possess_pawn(ucv: UnrealCV, controller_name: str, pawn_name: str) -> bool:
    for cmd in (
        f"vbp {controller_name} Possess {pawn_name}",
        f"vbp {controller_name} K2_Possess {pawn_name}",
    ):
        try:
            raw = geh._ue_request(ucv, cmd, timeout_s=30.0)  # noqa: SLF001
        except (ConnectionError, OSError, RuntimeError, ValueError):
            continue
        print(f"[LevelRobot] possess probe {cmd!r} -> {str(raw).strip()[:120]}")
        if _vbp_ok(raw):
            return True
    return False


def ensure_spotdog_sight_controller(
    ucv: UnrealCV,
    robot_name: str = LEVEL_ROBOT_NAME,
    *,
    perception_warmup_s: float = 2.0,
    allow_spawn_controller: bool = False,
) -> Optional[str]:
    """Attach ``BP_SpotDogAIController`` to the robot (required for AI Sight vbp).

    Level-placed ``GridEnv_SpotRobot`` often keeps a generic ``AIController_0`` from an
    earlier session. ``GetVisibleSightTargetsJson`` on the Pawn casts to
    ``BP_SpotDogAIController``; when that fails, UE returns ``{"targets":[]}`` even
    though geom FOV would see props.
    """
    if _sight_vbp_returns_json(ucv, robot_name):
        controllers = _spotdog_ai_controller_names(ucv)
        ctrl = controllers[0] if controllers else None
        print(
            f"[LevelRobot] sight vbp already OK on {robot_name!r}"
            + (f" via {ctrl!r}" if ctrl else "")
        )
        return ctrl

    controllers = _spotdog_ai_controller_names(ucv)
    for ctrl in controllers:
        if _try_possess_pawn(ucv, ctrl, robot_name):
            print(f"[LevelRobot] sight AI possess OK via {ctrl!r}")
            if perception_warmup_s > 0:
                time.sleep(perception_warmup_s)
            for _ in range(5):
                try:
                    ucv.tick()
                except Exception:
                    break
                time.sleep(0.15)
            return ctrl

    if not allow_spawn_controller:
        print(
            "[LevelRobot] WARN: existing BP_SpotDogAIController could not Possess "
            f"{robot_name!r}; skipping controller spawn for PIE stability"
        )
        return None

    spawn_name = SIGHT_AI_CONTROLLER_NAME
    existing = set(geh.actor_names(ucv))
    if spawn_name in existing:
        spawn_name = f"{SIGHT_AI_CONTROLLER_NAME}_{len(controllers)}"
    if not geh.spawn_bp(ucv, SPOTDOG_AI_CONTROLLER_BP, spawn_name):
        print("[LevelRobot] WARN: spawn BP_SpotDogAIController failed")
        return None
    if _try_possess_pawn(ucv, spawn_name, robot_name):
        print(f"[LevelRobot] spawned+possessed sight AI {spawn_name!r}")
        if perception_warmup_s > 0:
            time.sleep(perception_warmup_s)
        for _ in range(8):
            try:
                ucv.tick()
            except Exception:
                break
            time.sleep(0.15)
        return spawn_name

    print(
        "[LevelRobot] WARN: BP_SpotDogAIController could not Possess "
        f"{robot_name!r} — check Auto Possess AI / vbp Possess support"
    )
    return None


def _spotdog_actor_names(ucv: UnrealCV) -> list[str]:
    """Managed label first, then level-placed BP_SpotRobot* actors."""
    names = sorted(geh.actor_names(ucv))
    ordered: list[str] = []
    if LEVEL_ROBOT_NAME in names:
        ordered.append(LEVEL_ROBOT_NAME)
    for name in names:
        if name == LEVEL_ROBOT_NAME:
            continue
        if "SpotRobot" in name:
            ordered.append(name)
    return ordered


def find_spotdog_actor(ucv: UnrealCV) -> Optional[str]:
    actors = _spotdog_actor_names(ucv)
    return actors[0] if actors else None


def _park_spotdog_offmap(ucv: UnrealCV, actor_name: str) -> None:
    """Hide duplicate pawns without destroy (destroy often crashes Level PIE)."""
    loc = geh.try_get_location_cm(ucv, actor_name)
    if loc is None:
        return
    stash = (float(loc[0]), float(loc[1]), ROBOT_STASH_WORLD_Z_CM)
    try:
        ucv.enable_controller(actor_name, False)
    except Exception:
        pass
    ucv.set_physics(actor_name, False)
    ucv.set_location(list(stash), actor_name)
    ucv.set_collision(actor_name, False)
    print(f"[LevelRobot] parked duplicate {actor_name!r} off-map")


def _resolve_robot_world_xyz(
    ucv: UnrealCV,
    start_local_xy: LocalXY,
    *,
    nav_actor: Optional[str] = None,
) -> Tuple[float, float, float]:
    wx, wy, wz = foot_world_xyz_from_local_xy(*start_local_xy)
    if not nav_actor:
        return wx, wy, wz
    try:
        raw = nq.nav_project_point(ucv, nav_actor, wx, wy, NAV_PROJECT_PROBE_Z_CM)
    except Exception:
        return wx, wy, wz
    if not raw.get("ok"):
        return wx, wy, wz
    px, py, pz = float(raw["x"]), float(raw["y"]), float(raw["z"])
    if abs(px - wx) > ROBOT_NAV_XY_TOLERANCE_CM or abs(py - wy) > ROBOT_NAV_XY_TOLERANCE_CM:
        return wx, wy, wz
    return px, py, pz + ROBOT_FOOT_Z_OFFSET_CM


def soft_reset_level_spotdog(
    ucv: UnrealCV,
    start_local_xy: LocalXY,
    *,
    nav_actor: Optional[str] = None,
) -> Tuple[bool, str]:
    """Teleport + reconfigure at start — no destroy (safe on Level PIE)."""
    wx, wy, wz = _resolve_robot_world_xyz(ucv, start_local_xy, nav_actor=nav_actor)
    actors = _spotdog_actor_names(ucv)
    if actors:
        primary = actors[0]
        for duplicate in actors[1:]:
            _park_spotdog_offmap(ucv, duplicate)
        try:
            _configure_robot_at(ucv, (wx, wy, wz), actor_name=primary)
            lx, ly = world_xy_to_local(wx, wy)
            print(
                f"[LevelRobot] soft-reset {primary!r} world=({wx:.1f}, {wy:.1f}, {wz:.1f}) "
                f"local=({lx:.1f}, {ly:.1f})"
            )
            return True, primary
        except Exception as exc:
            print(f"[LevelRobot] soft-reset failed for {primary!r}: {exc}")
            return False, primary
    ok, name, _ = _spawn_spotdog_at(ucv, start_local_xy, nav_actor=nav_actor)
    return ok, name


def destroy_level_spotdog(ucv: UnrealCV) -> bool:
    """Destroy only ``GridEnv_SpotRobot`` with pawn-safe teardown (may still stress UE)."""
    name = find_spotdog_actor(ucv)
    if not name:
        return True
    loc = geh.try_get_location_cm(ucv, name)
    stash = None
    if loc is not None:
        stash = (loc[0], loc[1], ROBOT_STASH_WORLD_Z_CM)
    print(f"[LevelRobot] hard-destroy {name!r} (pawn-safe) ...")
    geh.prepare_pawn_for_destroy(ucv, name, stash_xyz=stash)
    return geh.destroy_pawn_safely(ucv, name)


def _spawn_spotdog_at(
    ucv: UnrealCV,
    start_local_xy: LocalXY,
    *,
    actor_name: str = LEVEL_ROBOT_NAME,
    nav_actor: Optional[str] = None,
) -> Tuple[bool, str, UnrealCV]:
    wx, wy, wz = _resolve_robot_world_xyz(ucv, start_local_xy, nav_actor=nav_actor)
    geh._prepare_ue_spawn(ucv)
    spawned = geh.spawn_bp(ucv, geh.ROBOT_BP, actor_name)
    if not spawned:
        ucv, _ = geh.reconnect_if_needed(ucv=ucv, force_new=True)
        geh._prepare_ue_spawn(ucv)
        spawned = geh.spawn_bp(ucv, geh.ROBOT_BP, actor_name)
    if not spawned:
        fallback = find_spotdog_actor(ucv)
        if not fallback:
            return False, actor_name, ucv
        actor_name = fallback
    else:
        time.sleep(ROBOT_SETTLE_S)

    _configure_robot_at(ucv, (wx, wy, wz), actor_name=actor_name)
    if actor_name in geh.actor_names(ucv):
        print(f"[LevelRobot] spawned {actor_name!r} @ ({wx:.1f}, {wy:.1f}, {wz:.1f})")
        return True, actor_name, ucv
    fallback = find_spotdog_actor(ucv)
    ok = fallback is not None
    return ok, fallback or actor_name, ucv


def hard_respawn_level_spotdog(
    ucv: UnrealCV,
    start_local_xy: LocalXY,
    *,
    actor_name: str = LEVEL_ROBOT_NAME,
) -> Tuple[bool, str, UnrealCV]:
    """Destroy + spawn. Use only when soft-reset is insufficient (can crash fragile PIE)."""
    destroy_level_spotdog(ucv)
    geh.settle_after_actor_destroy(ucv, settle_s=ROBOT_DESTROY_SETTLE_S, run_clean_garbage=False)
    ucv, _ = geh.reconnect_if_needed(ucv=ucv)
    ok, name, ucv = _spawn_spotdog_at(ucv, start_local_xy, actor_name=actor_name)
    if ok:
        print(f"[LevelRobot] hard-respawned {name!r}")
    return ok, name, ucv


def respawn_level_spotdog(
    ucv: UnrealCV,
    start_local_xy: LocalXY,
    *,
    hard: bool = False,
    actor_name: str = LEVEL_ROBOT_NAME,
) -> Tuple[bool, str, UnrealCV]:
    """Default: soft-reset (teleport). ``hard=True``: destroy + spawn."""
    if not hard:
        ok, name = soft_reset_level_spotdog(ucv, start_local_xy)
        return ok, name, ucv
    return hard_respawn_level_spotdog(ucv, start_local_xy, actor_name=actor_name)


def ensure_level_spotdog(
    ucv: UnrealCV,
    start_local_xy: LocalXY,
    *,
    actor_name: str = LEVEL_ROBOT_NAME,
) -> Tuple[bool, str]:
    """Spawn or soft-reset SpotDog at start_local_xy."""
    ok, name = soft_reset_level_spotdog(ucv, start_local_xy)
    if ok:
        return ok, name
    ok, name, _ = _spawn_spotdog_at(ucv, start_local_xy, actor_name=actor_name)
    return ok, name


def ensure_robot_upright_at_start(
    ucv: UnrealCV,
    robot_name: str,
    start_local_xy: LocalXY,
    *,
    nav_actor: Optional[str] = None,
) -> bool:
    """Recover fallen SpotDog onto NavMesh at mission start."""
    if not is_robot_tipped(ucv, robot_name):
        return True
    wx, wy, _wz = _resolve_robot_world_xyz(ucv, start_local_xy, nav_actor=nav_actor)
    print(f"[LevelRobot] robot tipped — upright recover @ local={start_local_xy}")
    ok, _landed = recover_robot_upright(
        ucv,
        robot_name,
        (wx, wy),
        nav_actor=nav_actor,
        yaw_deg=0.0,
    )
    return ok


def verify_spotdog_at_start(
    ucv: UnrealCV,
    robot_name: str,
    start_local_xy: LocalXY,
    *,
    tolerance_cm: float = 180.0,
) -> bool:
    loc = geh.try_get_location_cm(ucv, robot_name)
    if loc is None:
        return False
    lx, ly = world_xy_to_local(float(loc[0]), float(loc[1]))
    dist = math.hypot(lx - start_local_xy[0], ly - start_local_xy[1])
    if dist > tolerance_cm:
        print(
            f"[LevelRobot] WARN: robot @ local=({lx:.1f}, {ly:.1f}) "
            f"far from start {start_local_xy} ({dist:.0f}cm)"
        )
        return False
    return True


def ensure_robot_in_work_region(
    ucv: UnrealCV,
    robot_name: str,
    reset_local_xy: LocalXY,
    *,
    nav_actor: Optional[str] = None,
    region_size_cm: float = 2000.0,
    margin_cm: float = 80.0,
) -> Tuple[bool, str]:
    """Soft-reset SpotDog when it has drifted outside the declared work bounds."""
    loc = geh.try_get_location_cm(ucv, robot_name)
    if loc is None:
        ok, name = soft_reset_level_spotdog(
            ucv, reset_local_xy, nav_actor=nav_actor
        )
        return ok, name
    lx, ly = world_xy_to_local(float(loc[0]), float(loc[1]))
    lo = margin_cm
    hi = region_size_cm - margin_cm
    if lo <= lx <= hi and lo <= ly <= hi:
        return True, robot_name
    print(
        f"[LevelRobot] robot outside work region @ local=({lx:.1f}, {ly:.1f}) "
        f"— soft-reset to {reset_local_xy}"
    )
    ok, name = soft_reset_level_spotdog(ucv, reset_local_xy, nav_actor=nav_actor)
    return ok, name


def prepare_spotdog_mission_start(
    ucv: UnrealCV,
    start_local_xy: LocalXY,
    *,
    nav_actor: Optional[str] = None,
    start_tolerance_cm: float = 180.0,
) -> Tuple[bool, str]:
    """Teleport SpotDog to mission start, upright yaw=0, controller on."""
    ok, name = soft_reset_level_spotdog(ucv, start_local_xy, nav_actor=nav_actor)
    if not ok or not geh.actor_exists(ucv, name):
        return False, name
    if not ensure_robot_upright_at_start(ucv, name, start_local_xy, nav_actor=nav_actor):
        return False, name
    try:
        ucv.enable_controller(name, True)
    except Exception:
        pass
    time.sleep(ROBOT_SETTLE_S)
    if not verify_spotdog_at_start(ucv, name, start_local_xy, tolerance_cm=start_tolerance_cm):
        print("[LevelRobot] retry soft-reset after start position mismatch")
        ok, name = soft_reset_level_spotdog(ucv, start_local_xy, nav_actor=nav_actor)
        if not ok:
            return False, name
        ensure_robot_upright_at_start(ucv, name, start_local_xy, nav_actor=nav_actor)
        time.sleep(ROBOT_SETTLE_S)
    return True, name
