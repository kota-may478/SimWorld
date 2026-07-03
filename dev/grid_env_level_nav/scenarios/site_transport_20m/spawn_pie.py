#!/usr/bin/env python3
"""Spawn 20 m site props, material crate, humanoid, and SpotDog (PIE required)."""

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

setup_paths(scenario="site_transport_20m")

import ue_client_guard  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import level_nav_robot as lnr  # noqa: E402
import nav_query as nq  # noqa: E402
from pie_spawn_safety import (  # noqa: E402
    ensure_live_or_reconnect,
    spawn_bp_resilient,
)
from paths import site_transport_registry_path  # noqa: E402
from placement import (  # noqa: E402
    SitePropSlot,
    SiteTransportRegistry,
    apply_mask_colors_from_placement,
    ensure_registry,
    save_registry,
    to_placement_registry,
    update_slot_pose,
)
from object_mask_color import sync_registry_mask_colors  # noqa: E402
from pie_safety import PieSessionLost, require_live_ucv, tick_settle  # noqa: E402
from runtime_sight_sources import ensure_runtime_site20_sight_sources  # noqa: E402
from spawn_construction_vol1_props_pie import SPAWN_SETTLE_S  # noqa: E402
from simworld.communicator.communicator import Communicator  # noqa: E402

DEFAULT_FOOT_Z_OFFSET_CM = 5.0
NAV_XY_TOLERANCE_CM = 120.0
POST_DESTROY_BEFORE_SPAWN_S = 1.0
# Face +local Y (east / UE world +X).
HUMANOID_YAW_LOCAL_Y_POS_DEG = 0.0
PROP_BATCH_PAUSE_EVERY = 6
PROP_BATCH_PAUSE_S = 0.55
PROP_PRE_SPAWN_SETTLE_S = 0.2


def _actor_name_for_slot(registry: SiteTransportRegistry, slot: SitePropSlot) -> str:
    if slot.is_transport_target:
        return registry.material_actor_name
    return slot.slot_id


def _nav_spawn_xyz(ucv, nav_actor: str, lx: float, ly: float) -> Tuple[Optional[Tuple[float, float, float]], object]:
    ucv = ensure_live_or_reconnect(ucv, reason="nav_project_point")
    wx, wy = lc.local_xy_to_world(lx, ly)
    try:
        raw = nq.nav_project_point(ucv, nav_actor, wx, wy, lc.NAV_PROJECT_PROBE_Z_CM)
    except Exception as exc:
        print(f"[Site20Spawn] nav_project_point error: {exc}")
        ucv = ensure_live_or_reconnect(ucv, reason="nav_project_point exception")
        return None, ucv
    if not raw.get("ok"):
        if not geh._ping_ucv(ucv):  # noqa: SLF001
            ucv = ensure_live_or_reconnect(ucv, reason="nav_project_point connection lost")
        fx, fy, fz = lc.foot_world_xyz_from_local_xy(lx, ly)
        print(
            f"[Site20Spawn] nav snap miss local=({lx:.0f},{ly:.0f}) "
            f"-> layout foot ({fx:.0f},{fy:.0f},{fz:.0f})"
        )
        return (fx, fy, fz), ucv
    px, py, pz = float(raw["x"]), float(raw["y"]), float(raw["z"])
    if math.hypot(px - wx, py - wy) > NAV_XY_TOLERANCE_CM:
        fx, fy, fz = lc.foot_world_xyz_from_local_xy(lx, ly)
        print(
            f"[Site20Spawn] nav snap drift {math.hypot(px - wx, py - wy):.0f}cm "
            f"local=({lx:.0f},{ly:.0f}) -> layout foot ({fx:.0f},{fy:.0f},{fz:.0f})"
        )
        return (fx, fy, fz), ucv
    return (px, py, pz + DEFAULT_FOOT_Z_OFFSET_CM), ucv


def _ensure_navmesh_ready(
    ucv,
    nav_actor: str,
    probe_local: Tuple[float, float],
) -> object:
    """Build/warm up NavMesh before prop spawn (required when RuntimeGeneration=Dynamic)."""
    wx, wy = lc.local_xy_to_world(*probe_local)
    raw = nq.nav_project_point(ucv, nav_actor, wx, wy, lc.NAV_PROJECT_PROBE_Z_CM)
    if raw.get("ok"):
        return ucv
    print("[Site20Spawn] NavMesh empty — NavRebuild warmup...")
    rebuild = nq.nav_rebuild(ucv, nav_actor)
    if not rebuild.get("ok"):
        raise PieSessionLost(
            f"NavRebuild failed before spawn: {rebuild.get('error', rebuild)}"
        )
    tick_settle(ucv, settle_s=3.0, ticks=4)
    raw = nq.nav_project_point(ucv, nav_actor, wx, wy, lc.NAV_PROJECT_PROBE_Z_CM)
    if not raw.get("ok"):
        raise PieSessionLost(
            "NavMesh unavailable after NavRebuild — stop PIE, restart UE Editor "
            "(RuntimeGeneration=Dynamic), then Build → Build Paths before Play"
        )
    print(
        f"[Site20Spawn] NavMesh ready @ ({raw['x']:.0f},{raw['y']:.0f},{raw['z']:.0f})"
    )
    return ucv


def _is_barrier_prop(prop: SitePropSlot) -> bool:
    return prop.cluster_id in {"no_entry_fence", "no_entry_roadblock"}


def _enable_barrier_collisions(ucv, registry: SiteTransportRegistry) -> None:
    """Enable fence collision after spawn settle (avoids UE physics spike on spawn)."""
    for prop in registry.props:
        if not _is_barrier_prop(prop):
            continue
        actor_name = _actor_name_for_slot(registry, prop)
        if geh.actor_exists(ucv, actor_name):
            ucv.set_collision(actor_name, True)


def _configure_prop_at(
    ucv,
    prop: SitePropSlot,
    actor_name: str,
    xyz: Tuple[float, float, float],
    *,
    skip_color: bool = False,
) -> None:
    ucv.set_location(list(xyz), actor_name)
    ucv.set_orientation((0.0, prop.yaw_deg, 0.0), actor_name)
    geh._ue_request(ucv, f"vset /object/{actor_name}/physics 0", timeout_s=15.0)  # noqa: SLF001
    ucv.set_collision(actor_name, False)
    if not skip_color:
        ucv.set_color(actor_name, list(prop.mask_color_rgb))


def _light_prop_pause(ucv, placed: int) -> None:
    if placed <= 0 or placed % PROP_BATCH_PAUSE_EVERY != 0:
        return
    require_live_ucv(ucv, context=f"prop batch pause ({placed})")
    tick_settle(ucv, settle_s=PROP_BATCH_PAUSE_S, ticks=1)


def _stash_actor_offmap(ucv, actor_name: str) -> object:
    """Park actor below floor instead of destroy (Level PIE crashes on destroy)."""
    if not geh.actor_exists(ucv, actor_name):
        return ucv
    loc = ucv.get_location(actor_name)
    stash = (float(loc[0]), float(loc[1]), lc.FLOOR_REF_Z_CM - 50_000.0)
    ucv.set_physics(actor_name, False)
    ucv.set_collision(actor_name, False)
    ucv.set_location(list(stash), actor_name)
    tick_settle(ucv, settle_s=0.15, ticks=1)
    return ucv


def _place_prop(
    ucv,
    registry: SiteTransportRegistry,
    prop: SitePropSlot,
    xyz: Tuple[float, float, float],
    *,
    force_respawn: bool,
) -> Tuple[object, str]:
    actor_name = _actor_name_for_slot(registry, prop)
    if force_respawn and geh.actor_exists(ucv, actor_name):
        ucv = _stash_actor_offmap(ucv, actor_name)
        _configure_prop_at(ucv, prop, actor_name, xyz)
        tick_settle(ucv, settle_s=SPAWN_SETTLE_S, ticks=2)
        return ucv, "reused"
    if geh.actor_exists(ucv, actor_name) and not force_respawn:
        _configure_prop_at(ucv, prop, actor_name, xyz, skip_color=True)
        tick_settle(ucv, settle_s=SPAWN_SETTLE_S, ticks=2)
        return ucv, "reused"

    ok, ucv = spawn_bp_resilient(ucv, prop.bp_path, actor_name, timeout_s=120.0)
    if not ok:
        raise PieSessionLost(f"spawn_bp failed {actor_name} after retries")
    _configure_prop_at(ucv, prop, actor_name, xyz)
    tick_settle(ucv, settle_s=SPAWN_SETTLE_S, ticks=2)
    return ucv, "spawned"


def _spawn_props(
    ucv,
    registry: SiteTransportRegistry,
    nav_actor: str,
    *,
    force_respawn: bool,
) -> Tuple[SiteTransportRegistry, object, int]:
    updated = registry
    placed = 0
    spawned_count = 0
    for prop in registry.props:
        _light_prop_pause(ucv, placed)
        ucv = ensure_live_or_reconnect(ucv, reason=f"before {prop.slot_id}")
        tick_settle(ucv, settle_s=PROP_PRE_SPAWN_SETTLE_S, ticks=1)
        xyz, ucv = _nav_spawn_xyz(ucv, nav_actor, prop.local_xy_cm[0], prop.local_xy_cm[1])
        if xyz is None:
            raise PieSessionLost(f"no NavMesh for {prop.slot_id}")
        ucv, mode = _place_prop(ucv, registry, prop, xyz, force_respawn=force_respawn)
        actor = _actor_name_for_slot(registry, prop)
        print(f"[Site20Spawn] {actor} {prop.bp_name} @ {xyz} ({mode})")
        updated = update_slot_pose(updated, prop.slot_id, xyz, local_xy_cm=prop.local_xy_cm)
        placed += 1
        if mode == "spawned":
            spawned_count += 1
    return updated, ucv, spawned_count


def _yaw_toward_local(registry: SiteTransportRegistry, from_local: Tuple[float, float], to_local: Tuple[float, float]) -> float:
    fx, fy = lc.local_xy_to_world(*from_local)
    tx, ty = lc.local_xy_to_world(*to_local)
    return math.degrees(math.atan2(ty - fy, tx - fx))


def _find_existing_humanoid(ucv, preferred_name: str) -> Optional[str]:
    for name in sorted(geh.actor_names(ucv)):
        if name == preferred_name:
            return name
        if "Humanoid" in name or name.startswith("GEN_BP_Humanoid"):
            return name
    return None


def _place_humanoid(
    ucv,
    registry: SiteTransportRegistry,
    *,
    force_respawn: bool,
) -> Tuple[object, bool]:
    human_name = registry.humanoid_actor_name
    floor_z = geh.resolve_floor_top_z_cm(ucv)
    loc = lc.foot_world_xyz_from_local_xy(*registry.humanoid_local_cm)
    communicator = Communicator(ucv)

    existing = _find_existing_humanoid(ucv, human_name)
    if existing is not None:
        human_name = existing
        ucv.set_location(list(loc), human_name)
        ucv.set_orientation((0.0, HUMANOID_YAW_LOCAL_Y_POS_DEG, 0.0), human_name)
        ucv.set_physics(human_name, False)
        if geh.place_humanoid_on_floor(ucv, communicator, human_name, loc, floor_top_z_cm=floor_z):
            geh.configure_humanoid_kinematic(ucv, communicator, human_name, loc)
            print(
                f"[Site20Spawn] humanoid reused {human_name!r} @ {loc} "
                f"yaw={HUMANOID_YAW_LOCAL_Y_POS_DEG} (+local Y)"
            )
            return ucv, True

    if not geh.actor_exists(ucv, human_name):
        ok, ucv = spawn_bp_resilient(ucv, geh.HUMAN_BP, human_name, timeout_s=120.0)
        if not ok:
            print("[Site20Spawn] humanoid spawn failed")
            return ucv, False
    if not geh.place_humanoid_on_floor(ucv, communicator, human_name, loc, floor_top_z_cm=floor_z):
        print("[Site20Spawn] humanoid placement failed")
        return ucv, False
    ucv.set_orientation((0.0, HUMANOID_YAW_LOCAL_Y_POS_DEG, 0.0), human_name)
    geh.configure_humanoid_kinematic(ucv, communicator, human_name, loc)
    print(
        f"[Site20Spawn] humanoid {human_name} @ {loc} "
        f"yaw={HUMANOID_YAW_LOCAL_Y_POS_DEG} (+local Y)"
    )
    return ucv, True


def _orient_robot_toward_yard(ucv, registry: SiteTransportRegistry, robot_name: str) -> None:
    yaw = _yaw_toward_local(
        registry,
        registry.robot_start_local_cm,
        registry.material_pickup_local_cm,
    )
    ucv.set_orientation((0.0, yaw, 0.0), robot_name)
    print(f"[Site20Spawn] SpotDog yaw={yaw:.1f}° toward material yard")


def spawn_site_transport_scene(
    *,
    layout_id: str = "layout_01",
    force_rebuild: bool = False,
    force_respawn: bool = False,
    skip_cleanup: bool = False,
    ucv=None,
    manage_connection: bool = True,
) -> Tuple[int, Optional[object]]:
    registry = ensure_registry(layout_id=layout_id, force_rebuild=force_rebuild)
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
            print("[Site20Spawn] NavQueryService unavailable")
            return 1, active_ucv

        active_ucv = _ensure_navmesh_ready(
            active_ucv,
            nav_actor,
            registry.robot_start_local_cm,
        )
        updated, active_ucv, spawned_count = _spawn_props(
            active_ucv,
            registry,
            nav_actor,
            force_respawn=respawn,
        )
        placement = to_placement_registry(updated)
        placement = sync_registry_mask_colors(
            active_ucv, placement, reapply_colors=False
        )
        updated = apply_mask_colors_from_placement(updated, placement)
        print(
            f"[Site20Spawn] mask colors synced ({spawned_count} newly spawned, "
            f"{len(updated.props)} props)"
        )
        tick_settle(active_ucv, settle_s=2.0, ticks=2)
        _enable_barrier_collisions(active_ucv, updated)
        tick_settle(active_ucv, settle_s=1.5, ticks=2)
        print("[Site20Spawn] barrier collisions enabled")
        active_ucv, human_ok = _place_humanoid(active_ucv, updated, force_respawn=respawn)
        save_registry(updated, site_transport_registry_path(updated.layout_id))
        ok, name = lnr.soft_reset_level_spotdog(
            active_ucv,
            updated.robot_start_local_cm,
            nav_actor=nav_actor,
        )
        if ok:
            lnr.ensure_spotdog_sight_controller(active_ucv, name)
            ensure_runtime_site20_sight_sources(active_ucv)
        # Keep yaw=0 from soft-reset; pre-turning toward yard caused early dog_move crashes.
        print(
            f"[Site20Spawn] SpotDog ok={ok} name={name!r} props={len(updated.props)} "
            f"humanoid_ok={human_ok}"
        )
        return (0 if ok and human_ok else 2), active_ucv

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
    parser.add_argument("--force-rebuild", action="store_true")
    parser.add_argument("--force-respawn", action="store_true")
    parser.add_argument("--skip-cleanup", action="store_true")
    parser.add_argument("--layout-id", default="layout_01", help="layout_01 .. layout_10")
    args = parser.parse_args()
    try:
        rc, _ = spawn_site_transport_scene(
            layout_id=args.layout_id,
            force_rebuild=args.force_rebuild,
            force_respawn=args.force_respawn,
            skip_cleanup=args.skip_cleanup,
        )
        return rc
    except PieSessionLost as exc:
        print(f"[Site20Spawn] ABORT: {exc}")
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
