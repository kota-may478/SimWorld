#!/usr/bin/env python3
"""Spawn curated construction-site props on Level NavMesh (PIE required).

Usage (PIE on /Game/Maps/Level):
  conda run -n simworld python dev/grid_env_level_nav/spawn_construction_site_pie.py
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time
from pathlib import Path
from typing import Iterator, Optional, Tuple

THIS_DIR = Path(__file__).resolve().parent
ROOT = THIS_DIR.parent.parent
GEH_DIR = ROOT / "dev" / "grid_env_hri"
G10K_DIR = ROOT / "dev" / "grid_env_10k"
DEPTH_DIR = ROOT / "dev" / "grid_env_depth_perception"
for p in (str(ROOT), str(THIS_DIR), str(GEH_DIR), str(G10K_DIR), str(DEPTH_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

import ue_client_guard  # noqa: E402

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import level_nav_robot as lnr  # noqa: E402
import nav_query as nq  # noqa: E402
from construction_site_placement import (  # noqa: E402
    PROP_ACTOR_PREFIX,
    ConstructionSiteRegistry,
    SitePropSlot,
    ensure_registry,
    save_registry,
    update_slot_pose,
)
from pie_safety import (  # noqa: E402
    PieSessionLost,
    batch_pause,
    pause_between_spawns,
    require_live_ucv,
    settle_after_destroy_batch,
    tick_settle,
)
from spawn_construction_vol1_props_pie import SPAWN_SETTLE_S  # noqa: E402

DEFAULT_FOOT_Z_OFFSET_CM = 5.0
NAV_XY_TOLERANCE_CM = 120.0
CANDIDATE_JITTER_CM = 220.0
REUSE_POSITION_TOLERANCE_CM = 180.0
DESTROY_BETWEEN_S = 0.55


def _destroy_actor(ucv, name: str) -> bool:
    require_live_ucv(ucv, context=f"destroy {name}")
    if not geh.actor_exists(ucv, name):
        return True
    geh._ue_request(ucv, f"vset /object/{name}/destroy", timeout_s=30.0)  # noqa: SLF001
    gone = geh.wait_until_actor_gone(ucv, name, timeout_s=5.0)
    time.sleep(DESTROY_BETWEEN_S)
    tick_settle(ucv, settle_s=0.0, ticks=1)
    return gone


def _destroy_existing(ucv, registry: ConstructionSiteRegistry) -> int:
    prefixes = (f"{PROP_ACTOR_PREFIX}_", registry.material_actor_name, registry.carry_actor_name)
    names: list[str] = []
    for actor in geh.actor_names(ucv):
        if any(actor.startswith(p) if p.endswith("_") else actor == p for p in prefixes):
            names.append(actor)
    removed = 0
    for name in sorted(set(names)):
        if _destroy_actor(ucv, name):
            removed += 1
    if removed:
        settle_after_destroy_batch(ucv)
    return removed


def _candidate_positions(
    base_lx: float,
    base_ly: float,
    *,
    seed: int,
    slot_index: int,
) -> Iterator[Tuple[float, float]]:
    yield base_lx, base_ly
    rng = random.Random(seed + slot_index * 991)
    for _ in range(36):
        lx = base_lx + rng.uniform(-CANDIDATE_JITTER_CM, CANDIDATE_JITTER_CM)
        ly = base_ly + rng.uniform(-CANDIDATE_JITTER_CM, CANDIDATE_JITTER_CM)
        lx = min(max(lx, 400.0), 6600.0)
        ly = min(max(ly, 400.0), 7400.0)
        yield lx, ly


def _nav_spawn_xyz(
    ucv,
    nav_actor: str,
    lx: float,
    ly: float,
) -> Optional[Tuple[float, float, float]]:
    wx, wy = lc.local_xy_to_world(lx, ly)
    raw = nq.nav_project_point(ucv, nav_actor, wx, wy, lc.NAV_PROJECT_PROBE_Z_CM)
    if not raw.get("ok"):
        return None
    px = float(raw["x"])
    py = float(raw["y"])
    pz = float(raw["z"])
    if math.hypot(px - wx, py - wy) > NAV_XY_TOLERANCE_CM:
        return None
    return px, py, pz + DEFAULT_FOOT_Z_OFFSET_CM


def _actor_name_for_slot(registry: ConstructionSiteRegistry, slot: SitePropSlot) -> str:
    if slot.is_transport_target:
        return registry.material_actor_name
    return slot.slot_id


def _spawn_one(
    ucv,
    registry: ConstructionSiteRegistry,
    slot: SitePropSlot,
    nav_actor: str,
    *,
    slot_index: int,
) -> Tuple[Optional[Tuple[float, float, float]], Optional[Tuple[float, float]], object]:
    actor_name = _actor_name_for_slot(registry, slot)
    require_live_ucv(ucv, context=f"spawn {actor_name}")

    placed_xyz: Optional[Tuple[float, float, float]] = None
    placed_local: Optional[Tuple[float, float]] = None
    for lx, ly in _candidate_positions(
        slot.local_xy_cm[0],
        slot.local_xy_cm[1],
        seed=registry.seed,
        slot_index=slot_index,
    ):
        xyz = _nav_spawn_xyz(ucv, nav_actor, lx, ly)
        if xyz is not None:
            placed_xyz = xyz
            placed_local = (lx, ly)
            break
    if placed_xyz is None:
        return None, None, ucv

    if geh.actor_exists(ucv, actor_name):
        _destroy_actor(ucv, actor_name)

    ok = geh.spawn_bp(ucv, slot.bp_path, actor_name, timeout_s=120.0)
    if not ok:
        raise PieSessionLost(f"spawn_bp failed for {actor_name}")
    ucv.set_location(list(placed_xyz), actor_name)
    ucv.set_orientation((0.0, slot.yaw_deg, 0.0), actor_name)
    geh._ue_request(ucv, f"vset /object/{actor_name}/physics 0", timeout_s=15.0)  # noqa: SLF001
    tick_settle(ucv, settle_s=SPAWN_SETTLE_S, ticks=2)
    return placed_xyz, placed_local, ucv


def spawn_construction_site(
    *,
    dry_run: bool = False,
    force_rebuild: bool = False,
    skip_cleanup: bool = False,
    skip_spotdog: bool = False,
) -> int:
    registry = ensure_registry(force_rebuild=force_rebuild)
    print(
        f"[SiteSpawn] seed={registry.seed} types={len({p.bp_name for p in registry.props})} "
        f"instances={len(registry.props)}"
    )

    if dry_run:
        for slot in registry.props:
            actor = _actor_name_for_slot(registry, slot)
            print(
                f"  {actor} {slot.bp_name} cluster={slot.cluster_id} role={slot.role} "
                f"local={slot.local_xy_cm} target={slot.is_transport_target}"
            )
        return 0

    ucv, _ = ue_client_guard.prepare_ue_connection(force_new=True)
    require_live_ucv(ucv, context="initial connect")

    if not skip_cleanup:
        removed = _destroy_existing(ucv, registry)
        if removed:
            print(f"[SiteSpawn] removed {removed} existing construction-site actors")

    probe = lc.foot_world_xyz_from_local_xy(*registry.robot_start_local_cm)
    ok_nav, nav_actor = nq.ensure_nav_query_service(ucv, probe_xyz=probe)
    if not ok_nav:
        print("[SiteSpawn] NavQueryService unavailable — is PIE running on Level?")
        return 1

    updated = registry
    spawned = 0
    failed = 0
    for slot_index, slot in enumerate(registry.props, start=1):
        if pause_between_spawns(spawned):
            batch_pause(ucv, reason=f"after {spawned} spawns")
        placed_xyz, placed_local, ucv = _spawn_one(
            ucv,
            registry,
            slot,
            nav_actor,
            slot_index=slot_index,
        )
        if placed_xyz is None:
            print(f"[SiteSpawn] FAIL: no NavMesh for {slot.bp_name} ({slot.slot_id})")
            failed += 1
            continue
        actor = _actor_name_for_slot(registry, slot)
        print(
            f"[SiteSpawn] OK {actor} {slot.bp_name} cluster={slot.cluster_id} "
            f"local={placed_local} world={placed_xyz}"
        )
        updated = update_slot_pose(updated, slot.slot_id, placed_xyz, local_xy_cm=placed_local)
        spawned += 1

    save_registry(updated)
    print(f"[SiteSpawn] saved registry ({spawned} spawned, {failed} failed)")

    if not skip_spotdog:
        ok, name = lnr.soft_reset_level_spotdog(ucv, updated.robot_start_local_cm)
        print(f"[SiteSpawn] SpotDog soft-reset ok={ok} name={name}")

    return 0 if failed == 0 else 2


def main() -> int:
    parser = argparse.ArgumentParser(description="Spawn construction site props (PIE)")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--force-rebuild", action="store_true")
    parser.add_argument("--skip-cleanup", action="store_true")
    parser.add_argument("--skip-spotdog", action="store_true")
    args = parser.parse_args()
    try:
        return spawn_construction_site(
            dry_run=args.dry_run,
            force_rebuild=args.force_rebuild,
            skip_cleanup=args.skip_cleanup,
            skip_spotdog=args.skip_spotdog,
        )
    except PieSessionLost as exc:
        print(f"[SiteSpawn] ABORT: {exc}")
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
