#!/usr/bin/env python3
"""E2E smoke: L0 (+ optional RoomD closure) → A* → SpotDog follow."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
_ROOT = _THIS_DIR.parent.parent
for _p in (
    _ROOT,
    _ROOT / "dev" / "grid_env_hri",
    _ROOT / "dev" / "grid_env_10k",
    _THIS_DIR,
):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import nav_query as nq  # noqa: E402
from ue_client_guard import ensure_exclusive_ue_session  # noqa: E402
from costmap_layers import LayeredCostmap  # noqa: E402
from level_coords import local_xy_to_world  # noqa: E402
from l0_nav_mask import is_l0_cache_complete  # noqa: E402
from level_nav_robot import (  # noqa: E402
    ensure_level_spotdog,
    hard_respawn_level_spotdog,
    respawn_level_spotdog,
)
from spotdog_nav_follower import navigate_local_xy  # noqa: E402
from zone_catalog import ZoneCatalog, catalog_to_zone_registry  # noqa: E402
from zone_registry import ZoneRegistry  # noqa: E402


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--l0", type=Path, default=_THIS_DIR / "cache" / "l0_mask_10cm.npz")
    p.add_argument("--catalog", type=Path, default=None, help="zone_catalog.json (preferred)")
    p.add_argument("--zones", type=Path, default=None, help="legacy zone_registry.json")
    p.add_argument("--start-local", type=float, nargs=2, default=(500.0, 500.0))
    p.add_argument("--goal-local", type=float, nargs=2, default=(5000.0, 6000.0))
    p.add_argument("--close-zone", action="append", default=[])
    p.add_argument("--plan-only", action="store_true", help="A* only, no robot motion")
    p.add_argument(
        "--respawn",
        action="store_true",
        help="Soft-reset SpotDog at start (teleport + controller; no destroy).",
    )
    p.add_argument(
        "--hard-respawn",
        action="store_true",
        help="Destroy GridEnv_SpotRobot then spawn (can crash fragile PIE — use sparingly).",
    )
    p.add_argument("--delay-close-s", type=float, default=0.0, help="Close zones after N seconds")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    if not args.l0.is_file():
        print(f"[smoke] ERROR: L0 cache missing: {args.l0}")
        print("  Run: python dev/grid_env_level_nav/build_l0_nav_mask.py")
        return 1
    if not args.plan_only and not is_l0_cache_complete(args.l0):
        print(f"[smoke] ERROR: L0 cache incomplete (partial build): {args.l0}")
        print("  Wait for build_l0_nav_mask.py to finish, or use --plan-only")
        return 1

    layers = LayeredCostmap.from_l0_cache(args.l0)
    registry: ZoneRegistry | None = None
    if args.catalog and args.catalog.is_file():
        catalog = ZoneCatalog.load(args.catalog)
        registry = catalog_to_zone_registry(catalog, layers.resolution_cm)
    elif args.zones and args.zones.is_file():
        registry = ZoneRegistry.load(args.zones)

    for zid in args.close_zone:
        if registry is None:
            print(f"[smoke] WARN: --close-zone {zid} but no --zones file")
            continue
        n = layers.close_zone(zid, registry)
        print(f"[smoke] closed zone {zid!r} → {n} cells")

    start_xy = local_xy_to_world(*args.start_local)
    goal_xy = local_xy_to_world(*args.goal_local)
    plan = layers.plan_astar(start_xy, goal_xy)
    print(
        f"[smoke] plan start={args.start_local} goal={args.goal_local} "
        f"waypoints={len(plan.waypoints_xy)} cost={plan.total_cost:.1f}"
    )
    for i, wp in enumerate(plan.waypoints_xy[:6]):
        print(f"  WP{i+1}: ({wp[0]:.1f}, {wp[1]:.1f})")

    if args.plan_only:
        return 0 if plan.waypoints_xy else 1

    ucv, _ = ensure_exclusive_ue_session(force_new=True)
    ok, _ = nq.ensure_nav_query_service(ucv)
    if not ok:
        print("[smoke] WARN: NavQueryService missing (plan OK; motion needs PIE)")

    if args.delay_close_s > 0 and registry and args.close_zone:
        print(f"[smoke] will close zones after {args.delay_close_s}s motion")

    robot_ok: bool
    robot_name: str
    if args.hard_respawn:
        robot_ok, robot_name, ucv = hard_respawn_level_spotdog(ucv, tuple(args.start_local))
    elif args.respawn:
        robot_ok, robot_name, ucv = respawn_level_spotdog(ucv, tuple(args.start_local), hard=False)
    else:
        robot_ok, robot_name = ensure_level_spotdog(ucv, tuple(args.start_local))
    if not robot_ok:
        print("[smoke] ERROR: could not spawn/place SpotDog")
        return 1
    print(f"[smoke] robot ready: {robot_name!r}")

    if args.delay_close_s > 0 and registry:

        def _delayed_close() -> None:
            time.sleep(args.delay_close_s)
            for zid in args.close_zone:
                layers.close_zone(zid, registry)
                print(f"[smoke] delayed close {zid!r}")

        import threading

        threading.Thread(target=_delayed_close, daemon=True).start()

    arrived = navigate_local_xy(
        ucv,
        layers,
        tuple(args.goal_local),
        label="layered-smoke",
    )
    print(f"[smoke] arrived={arrived}")
    return 0 if arrived else 1


if __name__ == "__main__":
    raise SystemExit(main())
