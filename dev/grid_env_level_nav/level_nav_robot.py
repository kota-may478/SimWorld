#!/usr/bin/env python3
"""Spawn / place SpotDog on /Game/Maps/Level for layered nav smoke."""

from __future__ import annotations

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
from level_coords import FLOOR_REF_Z_CM, foot_world_xyz_from_local_xy  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

LocalXY = Tuple[float, float]
LEVEL_ROBOT_NAME = geh.ROBOT_ACTOR_NAME
ROBOT_SETTLE_S = 0.5
# Pawn destroy + UE GC on Level needs longer idle (no clean_garbage).
ROBOT_DESTROY_SETTLE_S = 3.0
# Stash far below floor before destroy (keeps camera/controller away from NavMesh).
ROBOT_STASH_WORLD_Z_CM = FLOOR_REF_Z_CM - 50_000.0


def _configure_robot_at(
    ucv: UnrealCV,
    loc: Tuple[float, float, float],
    *,
    actor_name: str = LEVEL_ROBOT_NAME,
) -> None:
    """Match grid_env_10k_pie_patrol SpotDog settings (controller required for dog_move)."""
    ucv.set_physics(actor_name, False)
    ucv.set_movable(actor_name, True)
    ucv.set_location(list(loc), actor_name)
    ucv.set_orientation((0.0, 0.0, 0.0), actor_name)
    ucv.set_collision(actor_name, True)
    ucv.enable_controller(actor_name, True)
    time.sleep(geh.PHYSICS_ENABLE_DELAY_S)


def find_spotdog_actor(ucv: UnrealCV) -> Optional[str]:
    """Return only the Python-managed SpotDog label (not level-placed BP_SpotRobot_C_*)."""
    if LEVEL_ROBOT_NAME in geh.actor_names(ucv):
        return LEVEL_ROBOT_NAME
    return None


def soft_reset_level_spotdog(
    ucv: UnrealCV,
    start_local_xy: LocalXY,
) -> Tuple[bool, str]:
    """Teleport + reconfigure at start — no destroy (safe on Level PIE)."""
    wx, wy, wz = foot_world_xyz_from_local_xy(*start_local_xy)
    existing = find_spotdog_actor(ucv)
    if existing:
        try:
            _configure_robot_at(ucv, (wx, wy, wz), actor_name=existing)
            print(f"[LevelRobot] soft-reset {existing!r} @ ({wx:.1f}, {wy:.1f}, {wz:.1f})")
            return True, existing
        except Exception as exc:
            print(f"[LevelRobot] soft-reset failed for {existing!r}: {exc}")
            return False, existing
    ok, name, _ = _spawn_spotdog_at(ucv, start_local_xy)
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
) -> Tuple[bool, str, UnrealCV]:
    wx, wy, wz = foot_world_xyz_from_local_xy(*start_local_xy)
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
