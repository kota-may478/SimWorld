#!/usr/bin/env python3
"""Quick PIE smoke test: sync mask colors, sample perception, rotate, re-sample.

Usage (PIE Play on /Game/Maps/Level):
  conda run -n simworld python dev/grid_env_depth_perception/run_perception_smoke_test.py

Exits 0 when at least one prop is detected after aiming toward nearest target.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

THIS_DIR = Path(__file__).resolve().parent
ROOT = THIS_DIR.parent.parent
GEH_DIR = ROOT / "dev" / "grid_env_hri"
G10K_DIR = ROOT / "dev" / "grid_env_10k"
NAV_DIR = ROOT / "dev" / "grid_env_level_nav"
for p in (str(ROOT), str(THIS_DIR), str(GEH_DIR), str(G10K_DIR), str(NAV_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

import ue_client_guard  # noqa: E402

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_nav_robot as lnr  # noqa: E402
from depth_object_perception import (  # noqa: E402
    PerceptionConfig,
    depth_npy_to_meters,
    detect_objects,
    estimates_by_prop_type,
    fetch_depth_npy,
)
from object_mask_color import sync_registry_mask_colors  # noqa: E402
from pie_safety import PieSessionLost, require_live_ucv, tick_settle  # noqa: E402
from prop_placement import load_registry, save_registry  # noqa: E402
from robot_sensor import (  # noqa: E402
    SENSOR_FOV_DEG,
    configure_sensor_camera,
    fetch_lit_bgr,
    fetch_mask_rgb,
    get_pos2d,
    get_yaw_deg,
    resolve_sensor_camera_id,
    update_sensor_camera_pose,
)
from simworld.communicator.communicator import Communicator  # noqa: E402


def _bearing_to_target_deg(
    robot_xy: Tuple[float, float],
    robot_yaw_deg: float,
    target_xy: Tuple[float, float],
) -> float:
    """Signed bearing from robot forward to target (degrees)."""
    dx = target_xy[0] - robot_xy[0]
    dy = target_xy[1] - robot_xy[1]
    target_yaw = math.degrees(math.atan2(dy, dx))
    delta = (target_yaw - robot_yaw_deg + 180.0) % 360.0 - 180.0
    return delta


def _perceive(
    ucv,
    communicator: Communicator,
    camera_id: int,
    registry,
    robot_name: str,
    cfg: PerceptionConfig,
) -> Dict[str, Dict[str, float]]:
    update_sensor_camera_pose(ucv, robot_name, camera_id)
    tick_settle(ucv, settle_s=0.35, ticks=1)
    mask = fetch_mask_rgb(communicator, camera_id)
    lit = fetch_lit_bgr(communicator, camera_id)
    depth_raw = fetch_depth_npy(ucv, camera_id)
    if depth_raw is None or mask is None:
        return {}
    depth_m = depth_npy_to_meters(depth_raw)
    estimates = detect_objects(mask, depth_m, registry, lit_bgr=lit, config=cfg)
    by_type = estimates_by_prop_type(estimates)
    return {
        pid: {
            "distance_m": est.distance_m,
            "bearing_deg": est.bearing_deg,
            "confidence": est.confidence,
            "mask_pixels": float(est.mask_pixels),
        }
        for pid, est in by_type.items()
    }


def _rotate_toward(
    ucv,
    robot_name: str,
    delta_yaw_deg: float,
    *,
    step_deg: float = 12.0,
) -> None:
    remaining = delta_yaw_deg
    while abs(remaining) > step_deg * 0.5:
        step = max(-step_deg, min(step_deg, remaining))
        yaw = get_yaw_deg(ucv, robot_name)
        ucv.set_orientation((0.0, yaw + step, 0.0), robot_name)
        tick_settle(ucv, settle_s=0.25, ticks=1)
        remaining -= step


def main() -> int:
    parser = argparse.ArgumentParser(description="Perception smoke test (PIE)")
    parser.add_argument(
        "--sync-colors",
        action="store_true",
        help="vget canonical mask colors from UE before sampling",
    )
    parser.add_argument(
        "--reapply-colors",
        action="store_true",
        help="vset intended colors before vget (only if IDs are wrong)",
    )
    args = parser.parse_args()

    registry = load_registry()
    missing = [p.slot_id for p in registry.props if p.world_xyz_cm is None]
    if missing:
        print(f"[Smoke] registry missing world poses: {missing}")
        print("[Smoke] run spawn_test_scene_pie.py first")
        return 1

    try:
        with ue_client_guard.exclusive_ue_client_lock():
            ucv, _ = g10k.ensure_connection()
            require_live_ucv(ucv, context="smoke start")

            ok, robot_name = lnr.soft_reset_level_spotdog(ucv, registry.spotdog_spawn_local_cm)
            if not ok:
                print("[Smoke] SpotDog not available")
                return 1
            tick_settle(ucv, settle_s=1.0, ticks=2)

            if args.sync_colors or args.reapply_colors:
                registry = sync_registry_mask_colors(
                    ucv, registry, reapply_colors=args.reapply_colors
                )
                save_registry(registry)

            camera_id = resolve_sensor_camera_id(ucv)
            configure_sensor_camera(ucv, camera_id)
            communicator = Communicator(ucv)
            cfg = PerceptionConfig(fov_deg=SENSOR_FOV_DEG)

            est0 = _perceive(ucv, communicator, camera_id, registry, robot_name, cfg)
            print(f"[Smoke] initial detections ({len(est0)}): {list(est0.keys())}")

            nearest = registry.visit_order_props()[0]
            goal_xy = (nearest.world_xyz_cm[0], nearest.world_xyz_cm[1])  # type: ignore[index]
            robot_xy = get_pos2d(ucv, robot_name)
            robot_yaw = get_yaw_deg(ucv, robot_name)
            delta = _bearing_to_target_deg(robot_xy, robot_yaw, goal_xy)
            print(
                f"[Smoke] aiming at {nearest.prop_type_id} "
                f"delta_yaw={delta:.1f}° robot={robot_xy} goal={goal_xy}"
            )
            _rotate_toward(ucv, robot_name, delta)

            est1 = _perceive(ucv, communicator, camera_id, registry, robot_name, cfg)
            print(f"[Smoke] after aim detections ({len(est1)}): {est1}")

            if not est1:
                print("[Smoke] FAIL: no detections after aiming")
                return 1

            if nearest.prop_type_id not in est1:
                print(
                    f"[Smoke] WARN: nearest target {nearest.prop_type_id} not in detections; "
                    f"got {list(est1.keys())}"
                )

            print("[Smoke] PASS")
            return 0
    except PieSessionLost as exc:
        print(f"[Smoke] ABORT: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
