#!/usr/bin/env python3
"""Spawn Construction VOL.1 LevelProps inside Level NavMesh (PIE required).

Usage (PIE running on /Game/Maps/Level):
  conda run -n simworld python dev/grid_env_level_nav/spawn_construction_vol1_props_pie.py

Optional:
  --dry-run          print placements only
  --max-props N      spawn first N props (smoke)
  --spacing-cm 520   grid spacing between props (73 props need ~520cm)
  --skip-cleanup     do not destroy existing prop_vol1_* actors first
  --allow-missing-bp skip entries whose generated BP .uasset is absent
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Iterator, List, Tuple

THIS_DIR = Path(__file__).resolve().parent
ROOT = THIS_DIR.parent.parent
GEH_DIR = ROOT / "dev" / "grid_env_hri"
G10K_DIR = ROOT / "dev" / "grid_env_10k"
for p in (str(ROOT), str(THIS_DIR), str(GEH_DIR), str(G10K_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402
from prop_catalog import (  # noqa: E402
    EXPECTED_MESH_COUNT,
    PropCatalogEntry,
    bp_asset_exists,
    ensure_catalog,
    missing_bp_entries,
)
from work_region import REGION_SIZE_X_CM, REGION_SIZE_Y_CM  # noqa: E402

PROP_ACTOR_PREFIX = "prop_vol1"
DEFAULT_MARGIN_LOCAL_CM = 900.0
DEFAULT_SPACING_CM = 520.0
DEFAULT_FOOT_Z_OFFSET_CM = 5.0
NAV_XY_TOLERANCE_CM = 120.0
SPAWN_SETTLE_S = 0.55
POST_DESTROY_SETTLE_S = 4.0
DESTROY_BETWEEN_S = 0.35
RECONNECT_BACKOFF_S = 5.0
BATCH_PAUSE_EVERY = 8
BATCH_PAUSE_S = 2.0
MAX_SPAWN_ATTEMPTS = 4


def _grid_local_centers(
    count: int,
    *,
    margin_cm: float,
    spacing_cm: float,
) -> Iterator[Tuple[float, float]]:
    usable_x = REGION_SIZE_X_CM - 2.0 * margin_cm
    usable_y = REGION_SIZE_Y_CM - 2.0 * margin_cm
    cols = max(1, int(math.floor(usable_x / spacing_cm)))
    rows = max(1, int(math.ceil(count / cols)))
    produced = 0
    for row in range(rows):
        for col in range(cols):
            if produced >= count:
                return
            lx = margin_cm + (col + 0.5) * spacing_cm
            ly = margin_cm + (row + 0.5) * spacing_cm
            if lx > REGION_SIZE_X_CM - margin_cm or ly > REGION_SIZE_Y_CM - margin_cm:
                continue
            yield lx, ly
            produced += 1


def _ping_ok(ucv) -> bool:
    try:
        return geh._ping_ucv(ucv)  # noqa: SLF001 — shared session health
    except Exception:
        return False


def _recover_ucv(ucv, *, reason: str):
    print(f"[PropSpawn] reconnect ({reason}) ...")
    ucv, _ = g10k.reconnect_if_needed(ucv, force_new=True)
    time.sleep(RECONNECT_BACKOFF_S)
    return ucv


def _ensure_live_ucv(ucv, *, reason: str):
    if _ping_ok(ucv):
        return ucv
    return _recover_ucv(ucv, reason=reason)


def _destroy_prop_light(ucv, name: str) -> bool:
    if not geh.actor_exists(ucv, name):
        return True
    geh._ue_request(ucv, f"vset /object/{name}/destroy", timeout_s=30.0)  # noqa: SLF001
    gone = geh.wait_until_actor_gone(ucv, name, timeout_s=4.0)
    time.sleep(DESTROY_BETWEEN_S)
    return gone


def _destroy_existing_props(ucv, *, skip_cleanup: bool) -> int:
    if skip_cleanup:
        return 0
    names = sorted(
        n for n in geh.actor_names(ucv) if n.startswith(f"{PROP_ACTOR_PREFIX}_")
    )
    if not names:
        return 0
    print(f"[PropSpawn] removing {len(names)} existing {PROP_ACTOR_PREFIX}_* (light destroy) ...")
    removed = 0
    for name in names:
        ucv = _ensure_live_ucv(ucv, reason=f"before destroy {name}")
        if _destroy_prop_light(ucv, name):
            removed += 1
        else:
            print(f"[PropSpawn] WARN: {name} still present after destroy")
    geh.settle_after_actor_destroy(ucv, settle_s=POST_DESTROY_SETTLE_S, run_clean_garbage=False)
    return removed


def _ensure_nav(ucv, probe_xyz: Tuple[float, float, float]) -> Tuple[bool, object, str]:
    ucv = _ensure_live_ucv(ucv, reason="before NavQueryService")
    ok_nav, nav_actor = nq.ensure_nav_query_service(ucv, probe_xyz=probe_xyz)
    if ok_nav:
        return True, ucv, nav_actor
    ucv = _recover_ucv(ucv, reason="NavQueryService unavailable")
    ok_nav, nav_actor = nq.ensure_nav_query_service(ucv, probe_xyz=probe_xyz)
    return ok_nav, ucv, nav_actor


def _spawn_one(
    ucv,
    entry: PropCatalogEntry,
    actor_name: str,
    location: Tuple[float, float, float],
) -> Tuple[bool, object]:
    for attempt in range(1, MAX_SPAWN_ATTEMPTS + 1):
        ucv = _ensure_live_ucv(ucv, reason=f"spawn {entry.bp_name} attempt {attempt}")
        if not geh.spawn_bp(ucv, entry.bp_path, actor_name, timeout_s=120.0):
            print(f"[PropSpawn] spawn_bp retry {attempt}/{MAX_SPAWN_ATTEMPTS} for {entry.bp_name}")
            ucv = _recover_ucv(ucv, reason=f"spawn_bp failed {entry.bp_name}")
            continue
        try:
            ucv.set_location(list(location), actor_name)
            ucv.set_orientation((0.0, 0.0, 0.0), actor_name)
            geh._ue_request(ucv, f"vset /object/{actor_name}/physics 0", timeout_s=15.0)  # noqa: SLF001
        except Exception as exc:
            print(f"[PropSpawn] WARN: post-spawn setup {actor_name}: {exc}")
            if not _ping_ok(ucv):
                ucv = _recover_ucv(ucv, reason=f"post-spawn lost connection {actor_name}")
                continue
        time.sleep(SPAWN_SETTLE_S)
        try:
            ucv.tick()
        except Exception:
            pass
        return True, ucv
    return False, ucv


def _nav_spawn_xyz(
    ucv,
    nav_actor: str,
    lx: float,
    ly: float,
) -> Tuple[Tuple[float, float, float] | None, object]:
    ucv = _ensure_live_ucv(ucv, reason="nav_project_point")
    wx, wy = lc.local_xy_to_world(lx, ly)
    probe_z = lc.NAV_PROJECT_PROBE_Z_CM
    try:
        raw = nq.nav_project_point(ucv, nav_actor, wx, wy, probe_z)
    except Exception as exc:
        print(f"[PropSpawn] nav_project_point error: {exc}")
        ucv = _recover_ucv(ucv, reason="nav_project_point exception")
        return None, ucv
    if not raw.get("ok"):
        if not _ping_ok(ucv):
            ucv = _recover_ucv(ucv, reason="nav_project_point connection lost")
        return None, ucv
    try:
        px = float(raw["x"])
        py = float(raw["y"])
        pz = float(raw["z"])
    except (KeyError, TypeError, ValueError):
        return None, ucv
    if math.hypot(px - wx, py - wy) > NAV_XY_TOLERANCE_CM:
        return None, ucv
    return (px, py, pz + DEFAULT_FOOT_Z_OFFSET_CM), ucv


def _candidate_local_points(count: int, spacing_cm: float) -> List[Tuple[float, float]]:
    """Return at least ``count`` grid centers; extra slots for NavMesh rejections."""
    target = max(count, int(math.ceil(count * 1.35)))
    primary = list(
        _grid_local_centers(target, margin_cm=DEFAULT_MARGIN_LOCAL_CM, spacing_cm=spacing_cm)
    )
    if len(primary) >= target:
        return primary
    fallback_spacing = spacing_cm * 0.85
    fallback = list(
        _grid_local_centers(
            target,
            margin_cm=DEFAULT_MARGIN_LOCAL_CM * 0.7,
            spacing_cm=fallback_spacing,
        )
    )
    if len(fallback) > len(primary):
        return fallback
    dense = list(
        _grid_local_centers(
            target,
            margin_cm=DEFAULT_MARGIN_LOCAL_CM * 0.55,
            spacing_cm=spacing_cm * 0.75,
        )
    )
    return dense if len(dense) > len(primary) else primary


def _filter_spawnable_entries(
    entries: list[PropCatalogEntry],
    *,
    allow_missing_bp: bool,
) -> tuple[list[PropCatalogEntry], list[PropCatalogEntry]]:
    missing = missing_bp_entries(entries)
    if not missing:
        return entries, []
    if allow_missing_bp:
        missing_names = {e.bp_name for e in missing}
        spawnable = [e for e in entries if e.bp_name not in missing_names]
        return spawnable, missing
    return [], missing


def spawn_props(
    *,
    dry_run: bool = False,
    max_props: int | None = None,
    spacing_cm: float = DEFAULT_SPACING_CM,
    skip_cleanup: bool = False,
    allow_missing_bp: bool = False,
    refresh_catalog: bool = True,
) -> int:
    entries = ensure_catalog(refresh=refresh_catalog, from_meshes=True)
    if not entries:
        print("FAIL: prop catalog empty — run rebuild_generated_level_props_editor.py first")
        return 1

    print(f"[PropSpawn] catalog meshes={len(entries)} (expected {EXPECTED_MESH_COUNT})")
    missing_on_disk = missing_bp_entries(entries)
    if missing_on_disk:
        print(
            f"[PropSpawn] WARN: {len(missing_on_disk)} BP(s) not found on Content disk "
            "(will still attempt spawn_bp — UE may have them loaded):"
        )
        for entry in missing_on_disk:
            print(f"  - {entry.bp_name} ({entry.mesh_name})")
        if allow_missing_bp:
            missing_names = {e.bp_name for e in missing_on_disk}
            entries = [e for e in entries if e.bp_name not in missing_names]
            print(f"[PropSpawn] --allow-missing-bp: skipping {len(missing_on_disk)} entries")
    missing_bps = missing_on_disk if allow_missing_bp else []

    if max_props is not None:
        entries = entries[: max(0, max_props)]

    print(f"[PropSpawn] spawning {len(entries)} props, spacing={spacing_cm}cm")
    candidates = _candidate_local_points(len(entries), spacing_cm)
    if len(candidates) < len(entries):
        print(
            f"WARN: only {len(candidates)} grid slots for {len(entries)} props "
            "(increase region or reduce spacing)"
        )

    if dry_run:
        for idx, entry in enumerate(entries):
            lx, ly = candidates[min(idx, len(candidates) - 1)]
            wx, wy = lc.local_xy_to_world(lx, ly)
            bp_ok = "ok" if bp_asset_exists(entry) else "MISSING_BP"
            print(
                f"  [{idx:02d}] {entry.bp_name} ({bp_ok}) actor={PROP_ACTOR_PREFIX}_{idx:03d} "
                f"local=({lx:.0f},{ly:.0f}) world=({wx:.0f},{wy:.0f})"
            )
        return 0

    ucv, _ = g10k.ensure_connection()
    ucv = _ensure_live_ucv(ucv, reason="initial connect")

    removed = _destroy_existing_props(ucv, skip_cleanup=skip_cleanup)
    if removed:
        print(f"[PropSpawn] removed {removed} existing {PROP_ACTOR_PREFIX}_* actors")

    probe = lc.local_xy_to_world(1500.0, 1500.0) + (lc.NAV_PROJECT_PROBE_Z_CM,)
    ok_nav, ucv, nav_actor = _ensure_nav(ucv, probe)
    if not ok_nav:
        print("FAIL: NavQueryService unavailable (PIE + BP_NavQueryService required)")
        return 2

    spawned = 0
    failed = 0
    skipped_bp = len(missing_bps) if allow_missing_bp else 0
    cand_idx = 0
    for idx, entry in enumerate(entries):
        if spawned > 0 and spawned % BATCH_PAUSE_EVERY == 0:
            print(f"[PropSpawn] batch pause after {spawned} spawns ({BATCH_PAUSE_S}s) ...")
            ucv = _ensure_live_ucv(ucv, reason="batch pause")
            time.sleep(BATCH_PAUSE_S)

        actor_name = f"{PROP_ACTOR_PREFIX}_{idx:03d}"
        if geh.actor_exists(ucv, actor_name):
            print(f"[PropSpawn] skip exists {actor_name}")
            spawned += 1
            continue
        location = None
        while cand_idx < len(candidates):
            lx, ly = candidates[cand_idx]
            cand_idx += 1
            location, ucv = _nav_spawn_xyz(ucv, nav_actor, lx, ly)
            if location is not None:
                break
        if location is None:
            print(f"FAIL: no NavMesh point for {entry.bp_name}")
            failed += 1
            if not _ping_ok(ucv):
                ucv = _recover_ucv(ucv, reason="after nav failure")
                ok_nav, ucv, nav_actor = _ensure_nav(ucv, probe)
                if not ok_nav:
                    print("FAIL: lost NavQueryService after connection recovery — aborting")
                    return 5
            continue

        ok_spawn, ucv = _spawn_one(ucv, entry, actor_name, location)
        if not ok_spawn:
            print(f"FAIL: spawn_bp {entry.bp_name}")
            failed += 1
            if not _ping_ok(ucv):
                ucv = _recover_ucv(ucv, reason="after spawn failure")
                ok_nav, ucv, nav_actor = _ensure_nav(ucv, probe)
                if not ok_nav:
                    print("FAIL: lost NavQueryService after spawn recovery — aborting")
                    return 5
            continue

        spawned += 1
        print(
            f"[PropSpawn] OK {actor_name} <- {entry.bp_name} "
            f"@ ({location[0]:.0f}, {location[1]:.0f}, {location[2]:.0f})"
        )

    print(
        f"[PropSpawn] done spawned={spawned} failed={failed} "
        f"skipped_missing_bp={skipped_bp} missing_on_disk={len(missing_on_disk)}"
    )
    disk_ok = len(missing_on_disk) == 0
    return 0 if failed == 0 and disk_ok else 3


def main() -> int:
    parser = argparse.ArgumentParser(description="Spawn Construction VOL.1 props in Level NavMesh")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--max-props", type=int, default=None)
    parser.add_argument("--spacing-cm", type=float, default=DEFAULT_SPACING_CM)
    parser.add_argument("--skip-cleanup", action="store_true")
    parser.add_argument(
        "--allow-missing-bp",
        action="store_true",
        help="spawn only entries whose generated BP .uasset exists",
    )
    parser.add_argument("--no-refresh-catalog", action="store_true")
    args = parser.parse_args()
    return spawn_props(
        dry_run=args.dry_run,
        max_props=args.max_props,
        spacing_cm=args.spacing_cm,
        skip_cleanup=args.skip_cleanup,
        allow_missing_bp=args.allow_missing_bp,
        refresh_catalog=not args.no_refresh_catalog,
    )


if __name__ == "__main__":
    raise SystemExit(main())
