#!/usr/bin/env python3
"""E2E: construction site spawn → L0+L2 nav → pickup carry → return home.

Prerequisites:
  - UE Editor: /Game/Maps/Level in PIE
  - L0 cache built: dev/grid_env_level_nav/cache/l0_mask_30cm_strict.npz

Usage:
  conda run -n simworld python dev/grid_env_level_nav/run_construction_site_transport_test.py
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
_ROOT = _THIS_DIR.parent.parent
for _p in (
    _ROOT,
    _ROOT / "dev" / "grid_env_hri",
    _ROOT / "dev" / "grid_env_10k",
    _ROOT / "dev" / "grid_env_depth_perception",
    _THIS_DIR,
):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import ue_client_guard  # noqa: E402

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import level_nav_robot as lnr  # noqa: E402
import nav_query as nq  # noqa: E402
from construction_site_carry import (  # noqa: E402
    begin_carry_from_material,
    deliver_carry_at_home,
    pickup_standoff_xy,
)
from construction_site_placement import ensure_registry  # noqa: E402
from costmap_layers import LayeredCostmap  # noqa: E402
from grid_env_10k_pie_patrol import dist2d, get_pos2d  # noqa: E402
from l0_nav_mask import is_l0_cache_complete  # noqa: E402
from layered_nav_perception import navigate_layered_local  # noqa: E402
from pie_safety import PieSessionLost, require_live_ucv, tick_settle  # noqa: E402
from spawn_construction_site_pie import spawn_construction_site  # noqa: E402

DEFAULT_L0 = _THIS_DIR / "cache" / "l0_mask_30cm_strict.npz"
ROBOT_ACTOR = geh.ROBOT_ACTOR_NAME
ARRIVE_TOLERANCE_CM = 130.0


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Construction site material transport E2E")
    p.add_argument("--l0", type=Path, default=DEFAULT_L0)
    p.add_argument("--spawn-only", action="store_true")
    p.add_argument("--skip-spawn", action="store_true", help="reuse existing scene actors")
    p.add_argument("--plan-only", action="store_true", help="validate L0 path only")
    p.add_argument("--force-rebuild-registry", action="store_true")
    p.add_argument("--max-nav-steps", type=int, default=450, help="cap per navigation leg")
    return p.parse_args()


def _material_goal_xy(registry) -> tuple[float, float]:
    transport = registry.transport_slot()
    if transport is not None and transport.world_xyz_cm is not None:
        return transport.world_xyz_cm[0], transport.world_xyz_cm[1]
    return lc.local_xy_to_world(*registry.material_pickup_local_cm)


def _home_goal_xy(registry) -> tuple[float, float]:
    return lc.local_xy_to_world(*registry.home_local_cm)


def main() -> int:
    args = _parse_args()
    if not args.l0.is_file():
        print(f"[CSTest] ERROR: missing L0 cache: {args.l0}")
        return 1

    registry = ensure_registry(force_rebuild=args.force_rebuild_registry)
    layers = LayeredCostmap.from_l0_cache(args.l0)

    start_local = registry.robot_start_local_cm
    material_local = registry.material_pickup_local_cm
    plan_out = layers.plan_astar_local(start_local, material_local)
    plan_home = layers.plan_astar_local(material_local, start_local)
    print(
        f"[CSTest] L0 plan to material: {len(plan_out.waypoints_xy)} WP cost={plan_out.total_cost:.1f}"
    )
    print(
        f"[CSTest] L0 plan return home: {len(plan_home.waypoints_xy)} WP cost={plan_home.total_cost:.1f}"
    )
    if not plan_out.waypoints_xy or not plan_home.waypoints_xy:
        print("[CSTest] ERROR: no L0 path — adjust placement or rebuild L0 mask")
        return 1
    if args.plan_only:
        return 0

    if not is_l0_cache_complete(args.l0):
        print(f"[CSTest] WARN: L0 cache may be partial: {args.l0}")

    if not args.skip_spawn:
        spawn_rc = spawn_construction_site(force_rebuild=args.force_rebuild_registry)
        if spawn_rc != 0:
            print(f"[CSTest] spawn failed rc={spawn_rc}")
            return spawn_rc
        registry = ensure_registry()
    if args.spawn_only:
        return 0

    try:
        ucv, _ = ue_client_guard.prepare_ue_connection(force_new=True)
        require_live_ucv(ucv, context="transport test start")
        ok_nav, _ = nq.ensure_nav_query_service(
            ucv,
            probe_xyz=lc.foot_world_xyz_from_local_xy(*start_local),
        )
        if not ok_nav:
            print("[CSTest] NavQueryService unavailable")
            return 2

        ok_robot, robot_name = lnr.soft_reset_level_spotdog(ucv, start_local)
        if not ok_robot:
            print("[CSTest] SpotDog soft-reset failed")
            return 2
        print(f"[CSTest] robot={robot_name!r} at start {start_local}")
        tick_settle(ucv, settle_s=0.8, ticks=2)

        material_xy = _material_goal_xy(registry)
        robot_xy = get_pos2d(ucv, robot_name)
        approach_xy = pickup_standoff_xy(material_xy, robot_xy)
        approach_local = lc.world_xy_to_local(*approach_xy)
        print(f"[CSTest] leg1 goal material @ {material_xy}, approach @ {approach_xy}")

        layers.reset_l2()
        leg1_ok = navigate_layered_local(
            ucv,
            layers,
            approach_local,
            robot_name=robot_name,
            tolerance_cm=ARRIVE_TOLERANCE_CM,
            label="to-material",
            max_total_steps=args.max_nav_steps,
        )
        if not leg1_ok:
            print("[CSTest] FAIL: could not reach material approach")
            return 3

        carry_name = begin_carry_from_material(ucv, registry, robot_name=robot_name)
        if not carry_name:
            print("[CSTest] FAIL: carry visual could not start")
            return 4
        print(f"[CSTest] carry started: {carry_name}")

        home_local = registry.home_local_cm
        layers.reset_l2()
        leg2_ok = navigate_layered_local(
            ucv,
            layers,
            home_local,
            robot_name=robot_name,
            tolerance_cm=ARRIVE_TOLERANCE_CM,
            label="return-home",
            carry_sync_name=carry_name,
            max_total_steps=args.max_nav_steps,
        )
        if not leg2_ok:
            print("[CSTest] FAIL: could not return home")
            return 5

        delivered = deliver_carry_at_home(ucv, registry, robot_name=robot_name)
        pos_xy = get_pos2d(ucv, robot_name)
        home_xy = _home_goal_xy(registry)
        home_dist = dist2d(pos_xy, home_xy)
        print(f"[CSTest] delivered={delivered} home_dist={home_dist:.1f}cm")
        if not delivered or home_dist > ARRIVE_TOLERANCE_CM * 1.5:
            return 6

        print("[CSTest] PASS")
        return 0
    except PieSessionLost as exc:
        print(f"[CSTest] ABORT: {exc}")
        return 10


if __name__ == "__main__":
    raise SystemExit(main())
