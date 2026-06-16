#!/usr/bin/env python3
"""Spawn depth-perception test scene: 5 construction props + SpotDog (PIE required).

Usage (PIE on /Game/Maps/Level):
  conda run -n simworld python dev/grid_env_depth_perception/spawn_test_scene_pie.py

Options:
  --dry-run           print placements only
  --force-rebuild     rebuild placement registry JSON
  --force-respawn     destroy and re-spawn props (risky on Level PIE)
  --skip-cleanup      keep existing depth_test_prop_* actors
  --skip-spotdog      spawn props only
  --reapply-colors    vset mask colors before vget (use only if IDs missing)
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Iterator, List, Optional, Tuple

THIS_DIR = Path(__file__).resolve().parent
ROOT = THIS_DIR.parent.parent
GEH_DIR = ROOT / "dev" / "grid_env_hri"
G10K_DIR = ROOT / "dev" / "grid_env_10k"
NAV_DIR = ROOT / "dev" / "grid_env_level_nav"
for p in (str(ROOT), str(THIS_DIR), str(GEH_DIR), str(G10K_DIR), str(NAV_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import level_nav_robot as lnr  # noqa: E402
import nav_query as nq  # noqa: E402
from object_mask_color import sync_registry_mask_colors  # noqa: E402
from prop_signature import sync_registry_detection_signatures  # noqa: E402
from pie_safety import (  # noqa: E402
    BATCH_PAUSE_S,
    DESTROY_BETWEEN_S,
    PieSessionLost,
    SPAWN_SETTLE_S,
    batch_pause,
    pause_between_spawns,
    ping_ok,
    require_live_ucv,
    settle_after_destroy_batch,
    tick_settle,
)
from prop_catalog import CONTENT_ROOT  # noqa: E402
from prop_placement import (  # noqa: E402
    PROP_ACTOR_PREFIX,
    PlacementRegistry,
    ensure_registry,
    finalize_registry_after_spawn,
    save_registry,
    update_prop_world_pose,
)
from robot_sensor import (  # noqa: E402
    configure_sensor_camera,
    resolve_sensor_camera_id,
)
from simworld.communicator.communicator import Communicator  # noqa: E402


def _bp_path_exists(bp_path: str) -> bool:
    rel = bp_path.split("/Game/", 1)[-1].split(".", 1)[0]
    return (CONTENT_ROOT / f"{rel}.uasset").is_file()


DEFAULT_FOOT_Z_OFFSET_CM = 5.0
NAV_XY_TOLERANCE_CM = 120.0
CANDIDATE_JITTER_CM = 280.0
REUSE_POSITION_TOLERANCE_CM = 180.0


def _destroy_actor(ucv, name: str) -> bool:
    require_live_ucv(ucv, context=f"destroy {name}")
    if not geh.actor_exists(ucv, name):
        return True
    geh._ue_request(ucv, f"vset /object/{name}/destroy", timeout_s=30.0)  # noqa: SLF001
    gone = geh.wait_until_actor_gone(ucv, name, timeout_s=5.0)
    time.sleep(DESTROY_BETWEEN_S)
    tick_settle(ucv, settle_s=0.0, ticks=1)
    return gone


def _destroy_existing(ucv, prefix: str) -> int:
    names = sorted(n for n in geh.actor_names(ucv) if n.startswith(prefix))
    removed = 0
    for name in names:
        if _destroy_actor(ucv, name):
            removed += 1
        else:
            print(f"[DepthSpawn] WARN: {name} still present after destroy")
    if removed:
        settle_after_destroy_batch(ucv)
    return removed


def _all_props_present(ucv, registry: PlacementRegistry) -> bool:
    return all(geh.actor_exists(ucv, p.slot_id) for p in registry.props)


def _prop_at_registry_pose(ucv, prop) -> bool:
    if prop.world_xyz_cm is None:
        return False
    loc = geh.try_get_location_cm(ucv, prop.slot_id)
    if loc is None:
        return False
    dx = loc[0] - prop.world_xyz_cm[0]
    dy = loc[1] - prop.world_xyz_cm[1]
    return math.hypot(dx, dy) <= REUSE_POSITION_TOLERANCE_CM


def _candidate_positions(
    base_lx: float,
    base_ly: float,
    *,
    seed: int,
    slot_index: int,
) -> Iterator[Tuple[float, float]]:
    yield base_lx, base_ly
    import random

    rng = random.Random(seed + slot_index * 997)
    exclusion = 500.0
    for _ in range(40):
        lx = base_lx + rng.uniform(-CANDIDATE_JITTER_CM, CANDIDATE_JITTER_CM)
        ly = base_ly + rng.uniform(-CANDIDATE_JITTER_CM, CANDIDATE_JITTER_CM)
        if lx < exclusion and ly < exclusion:
            continue
        lx = min(max(lx, 200.0), 2800.0)
        ly = min(max(ly, 200.0), 2800.0)
        yield lx, ly


def _nav_spawn_xyz(
    ucv,
    nav_actor: str,
    lx: float,
    ly: float,
) -> Optional[Tuple[float, float, float]]:
    wx, wy = lc.local_xy_to_world(lx, ly)
    probe_z = lc.NAV_PROJECT_PROBE_Z_CM
    raw = nq.nav_project_point(ucv, nav_actor, wx, wy, probe_z)
    if not raw.get("ok"):
        return None
    px = float(raw["x"])
    py = float(raw["y"])
    pz = float(raw["z"])
    if math.hypot(px - wx, py - wy) > NAV_XY_TOLERANCE_CM:
        return None
    return px, py, pz + DEFAULT_FOOT_Z_OFFSET_CM


def _spawn_one_prop(
    ucv,
    prop,
    nav_actor: str,
    *,
    seed: int,
    slot_index: int,
) -> Tuple[Optional[Tuple[float, float, float]], Optional[Tuple[float, float]], object]:
    require_live_ucv(ucv, context=f"spawn {prop.slot_id}")
    if geh.actor_exists(ucv, prop.slot_id) and _prop_at_registry_pose(ucv, prop):
        print(f"[DepthSpawn] reuse existing {prop.slot_id}")
        if prop.world_xyz_cm is not None:
            return prop.world_xyz_cm, prop.local_xy_cm, ucv

    placed_xyz: Optional[Tuple[float, float, float]] = None
    placed_local: Optional[Tuple[float, float]] = None
    for lx, ly in _candidate_positions(
        prop.local_xy_cm[0],
        prop.local_xy_cm[1],
        seed=seed,
        slot_index=slot_index,
    ):
        xyz = _nav_spawn_xyz(ucv, nav_actor, lx, ly)
        if xyz is None:
            continue
        placed_xyz = xyz
        placed_local = (lx, ly)
        break
    if placed_xyz is None:
        return None, None, ucv

    set_rgb = prop.mask_color_set_rgb or prop.mask_color_rgb
    ok = geh.spawn_bp(ucv, prop.bp_path, prop.slot_id, timeout_s=120.0)
    if not ok:
        raise PieSessionLost(f"spawn_bp failed for {prop.slot_id} — UE may have crashed")
    ucv.set_location(list(placed_xyz), prop.slot_id)
    ucv.set_orientation((0.0, 0.0, 0.0), prop.slot_id)
    geh._ue_request(ucv, f"vset /object/{prop.slot_id}/physics 0", timeout_s=15.0)  # noqa: SLF001
    ucv.set_color(prop.slot_id, list(set_rgb))
    tick_settle(ucv, settle_s=SPAWN_SETTLE_S, ticks=2)
    return placed_xyz, placed_local, ucv


def _sync_colors(ucv, registry: PlacementRegistry, *, reapply: bool) -> PlacementRegistry:
    return sync_registry_mask_colors(ucv, registry, reapply_colors=reapply)


def _spawn_props(
    ucv,
    registry: PlacementRegistry,
    nav_actor: str,
) -> Tuple[PlacementRegistry, object]:
    updated = registry
    spawned_count = 0
    for slot_index, prop in enumerate(registry.props, start=1):
        if not _bp_path_exists(prop.bp_path):
            print(f"[DepthSpawn] WARN: missing BP asset {prop.bp_path}")
            continue
        if pause_between_spawns(spawned_count):
            batch_pause(ucv, reason=f"after {spawned_count} spawns")
        placed_xyz, placed_local, ucv = _spawn_one_prop(
            ucv,
            prop,
            nav_actor,
            seed=registry.seed,
            slot_index=slot_index,
        )
        if placed_xyz is None:
            print(f"[DepthSpawn] FAIL: no NavMesh placement for {prop.slot_id}")
            continue
        print(
            f"[DepthSpawn] {prop.slot_id} {prop.prop_type_id} "
            f"@ local={placed_local} world={placed_xyz}"
        )
        updated = update_prop_world_pose(updated, prop.slot_id, placed_xyz, local_xy_cm=placed_local)
        spawned_count += 1
    return updated, ucv


def main() -> int:
    parser = argparse.ArgumentParser(description="Spawn depth perception test scene (PIE)")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--force-rebuild", action="store_true")
    parser.add_argument(
        "--force-respawn",
        action="store_true",
        help="destroy existing props before spawn (can crash fragile Level PIE)",
    )
    parser.add_argument("--skip-cleanup", action="store_true")
    parser.add_argument("--skip-spotdog", action="store_true")
    parser.add_argument(
        "--reapply-colors",
        action="store_true",
        help="vset mask colors before vget (avoid unless colors are wrong)",
    )
    parser.add_argument(
        "--skip-signatures",
        action="store_true",
        help="skip per-prop standoff lit signature capture",
    )
    args = parser.parse_args()

    registry = ensure_registry(force_rebuild=args.force_rebuild)
    print(f"[DepthSpawn] registry seed={registry.seed} props={len(registry.props)}")

    if args.dry_run:
        for prop in registry.props:
            print(
                f"[DepthSpawn] dry-run {prop.slot_id} {prop.prop_type_id} "
                f"local={prop.local_xy_cm} set_rgb={prop.mask_color_set_rgb or prop.mask_color_rgb}"
            )
        registry = finalize_registry_after_spawn(registry)
        save_registry(registry)
        print("[DepthSpawn] dry-run complete")
        return 0

    try:
        ucv, _ = g10k.ensure_connection()
        reuse = _all_props_present(ucv, registry) and not args.force_respawn
        if reuse:
            print("[DepthSpawn] all props present — reuse mode (no destroy/spawn)")
        elif args.skip_cleanup:
            print("[DepthSpawn] --skip-cleanup: leaving existing actors in place")
        else:
            n_props = _destroy_existing(ucv, f"{PROP_ACTOR_PREFIX}_")
            print(f"[DepthSpawn] removed {n_props} old props")

        if not reuse:
            probe = lc.foot_world_xyz_from_local_xy(1500.0, 1500.0)
            ok_nav, nav_actor = nq.ensure_nav_query_service(ucv, probe_xyz=probe)
            if not ok_nav:
                print("[DepthSpawn] NavQueryService unavailable — is PIE running on Level?")
                return 1
            registry, ucv = _spawn_props(ucv, registry, nav_actor)

        registry = finalize_registry_after_spawn(registry)
        registry = _sync_colors(ucv, registry, reapply=args.reapply_colors)
        save_registry(registry)
        print("[DepthSpawn] saved placement registry with canonical mask colors")

        if not args.skip_spotdog:
            ok, name = lnr.soft_reset_level_spotdog(ucv, registry.spotdog_spawn_local_cm)
            print(f"[DepthSpawn] SpotDog soft-reset ok={ok} name={name}")
            if ok and not args.skip_signatures:
                camera_id = resolve_sensor_camera_id(ucv)
                configure_sensor_camera(ucv, camera_id)
                communicator = Communicator(ucv)
                registry = sync_registry_detection_signatures(
                    ucv, communicator, camera_id, name, registry
                )
                lnr.soft_reset_level_spotdog(ucv, registry.spotdog_spawn_local_cm)
                print("[DepthSpawn] lit/depth signatures captured at standoff")

        print("[DepthSpawn] done")
        return 0
    except PieSessionLost as exc:
        print(f"[DepthSpawn] ABORT: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
