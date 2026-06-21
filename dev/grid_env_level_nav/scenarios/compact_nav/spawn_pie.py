#!/usr/bin/env python3
"""Spawn 3 compact-nav props + SpotDog at (1m, 1m) local (PIE required)."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Optional, Tuple

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="compact_nav")

import ue_client_guard  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import level_nav_robot as lnr  # noqa: E402
import nav_query as nq  # noqa: E402
from pie_spawn_safety import (  # noqa: E402
    destroy_actor_level,
    ensure_live_or_reconnect,
    spawn_bp_resilient,
)
from placement import (  # noqa: E402
    CompactNavRegistry,
    CompactPropSlot,
    ensure_registry,
    save_registry,
    to_placement_registry,
    update_slot_pose,
)
from object_mask_color import sync_registry_mask_colors  # noqa: E402
from pie_safety import PieSessionLost, batch_pause, tick_settle  # noqa: E402
from spawn_construction_vol1_props_pie import SPAWN_SETTLE_S  # noqa: E402

DEFAULT_FOOT_Z_OFFSET_CM = 5.0
NAV_XY_TOLERANCE_CM = 120.0
POST_DESTROY_BEFORE_SPAWN_S = 4.0


def _nav_spawn_xyz(ucv, nav_actor: str, lx: float, ly: float) -> Tuple[Optional[Tuple[float, float, float]], object]:
    ucv = ensure_live_or_reconnect(ucv, reason="nav_project_point")
    wx, wy = lc.local_xy_to_world(lx, ly)
    try:
        raw = nq.nav_project_point(ucv, nav_actor, wx, wy, lc.NAV_PROJECT_PROBE_Z_CM)
    except Exception as exc:
        print(f"[CompactSpawn] nav_project_point error: {exc}")
        ucv = ensure_live_or_reconnect(ucv, reason="nav_project_point exception")
        return None, ucv
    if not raw.get("ok"):
        if not geh._ping_ucv(ucv):  # noqa: SLF001
            ucv = ensure_live_or_reconnect(ucv, reason="nav_project_point connection lost")
        return None, ucv
    px, py, pz = float(raw["x"]), float(raw["y"]), float(raw["z"])
    if math.hypot(px - wx, py - wy) > NAV_XY_TOLERANCE_CM:
        return None, ucv
    return (px, py, pz + DEFAULT_FOOT_Z_OFFSET_CM), ucv


def _configure_prop_at(ucv, prop: CompactPropSlot, xyz: Tuple[float, float, float]) -> None:
    ucv.set_location(list(xyz), prop.slot_id)
    ucv.set_orientation((0.0, prop.yaw_deg, 0.0), prop.slot_id)
    geh._ue_request(ucv, f"vset /object/{prop.slot_id}/physics 0", timeout_s=15.0)  # noqa: SLF001
    ucv.set_color(prop.slot_id, list(prop.mask_color_rgb))


def _place_prop(
    ucv,
    prop: CompactPropSlot,
    xyz: Tuple[float, float, float],
    *,
    force_respawn: bool,
) -> Tuple[object, str]:
    """Place prop by teleport (safe) or destroy+spawn when forced."""
    if geh.actor_exists(ucv, prop.slot_id) and not force_respawn:
        _configure_prop_at(ucv, prop, xyz)
        tick_settle(ucv, settle_s=SPAWN_SETTLE_S, ticks=2)
        return ucv, "reused"

    if geh.actor_exists(ucv, prop.slot_id):
        _, ucv = destroy_actor_level(ucv, prop.slot_id)
        tick_settle(ucv, settle_s=POST_DESTROY_BEFORE_SPAWN_S, ticks=3)
        ucv = ensure_live_or_reconnect(ucv, reason=f"after destroy {prop.slot_id}")

    ok, ucv = spawn_bp_resilient(ucv, prop.bp_path, prop.slot_id, timeout_s=120.0)
    if not ok:
        raise PieSessionLost(f"spawn_bp failed {prop.slot_id} after retries")
    _configure_prop_at(ucv, prop, xyz)
    tick_settle(ucv, settle_s=SPAWN_SETTLE_S, ticks=2)
    return ucv, "spawned"


def _spawn_props(
    ucv,
    registry: CompactNavRegistry,
    nav_actor: str,
    *,
    force_respawn: bool,
) -> Tuple[CompactNavRegistry, object]:
    updated = registry
    placed = 0
    for prop in registry.props:
        if placed > 0:
            batch_pause(ucv, reason=f"before {prop.slot_id}")
        ucv = ensure_live_or_reconnect(ucv, reason=f"before {prop.slot_id}")
        tick_settle(ucv, settle_s=0.9, ticks=2)
        xyz, ucv = _nav_spawn_xyz(ucv, nav_actor, prop.local_xy_cm[0], prop.local_xy_cm[1])
        if xyz is None:
            raise PieSessionLost(f"no NavMesh for {prop.slot_id}")
        ucv, mode = _place_prop(ucv, prop, xyz, force_respawn=force_respawn)
        print(f"[CompactSpawn] {prop.slot_id} {prop.bp_name} @ {xyz} ({mode})")
        updated = update_slot_pose(updated, prop.slot_id, xyz, local_xy_cm=prop.local_xy_cm)
        placed += 1
    return updated, ucv


def spawn_compact_scene(
    *,
    force_rebuild: bool = False,
    force_respawn: bool = False,
    skip_cleanup: bool = False,
    ucv=None,
    manage_connection: bool = True,
) -> Tuple[int, Optional[object]]:
    """Spawn props + SpotDog. When manage_connection=False, caller keeps the UCV session."""
    registry = ensure_registry(force_rebuild=force_rebuild)
    respawn = force_respawn and not skip_cleanup
    own_session = manage_connection and ucv is None

    def _run(active_ucv) -> Tuple[int, object]:
        probe = lc.foot_world_xyz_from_local_xy(*registry.robot_start_local_cm)
        active_ucv = ensure_live_or_reconnect(active_ucv, reason="before NavQueryService")
        ok_nav, nav_actor = nq.ensure_nav_query_service(active_ucv, probe_xyz=probe)
        if not ok_nav:
            active_ucv = ensure_live_or_reconnect(active_ucv, reason="NavQueryService unavailable")
            ok_nav, nav_actor = nq.ensure_nav_query_service(active_ucv, probe_xyz=probe)
        if not ok_nav:
            print("[CompactSpawn] NavQueryService unavailable")
            return 1, active_ucv
        updated, active_ucv = _spawn_props(
            active_ucv,
            registry,
            nav_actor,
            force_respawn=respawn,
        )
        sync_registry_mask_colors(active_ucv, to_placement_registry(updated))
        save_registry(updated)
        ok, name = lnr.soft_reset_level_spotdog(active_ucv, updated.robot_start_local_cm)
        print(f"[CompactSpawn] SpotDog ok={ok} name={name!r} props={len(updated.props)}")
        return (0 if ok else 2), active_ucv

    if own_session:
        with ue_client_guard.exclusive_ue_client_lock():
            active_ucv, _ = ue_client_guard.prepare_ue_connection(force_new=True)
            rc, active_ucv = _run(active_ucv)
            geh.release_connection(active_ucv)
            return rc, None

    if ucv is None:
        raise ValueError("ucv required when manage_connection=False")
    return _run(ucv)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--force-rebuild", action="store_true", help="Rebuild registry JSON")
    parser.add_argument("--force-respawn", action="store_true", help="Destroy+spawn each prop")
    parser.add_argument("--skip-cleanup", action="store_true", help="Alias: reuse actors (no destroy)")
    args = parser.parse_args()
    try:
        rc, _ = spawn_compact_scene(
            force_rebuild=args.force_rebuild,
            force_respawn=args.force_respawn,
            skip_cleanup=args.skip_cleanup,
        )
        return rc
    except PieSessionLost as exc:
        print(f"[CompactSpawn] ABORT: {exc}")
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
