#!/usr/bin/env python3
"""Run depth + object_mask recognition test with time-series logging and plots.

Usage (after spawn_test_scene_pie.py):
  conda run -n simworld python dev/grid_env_depth_perception/run_depth_recognition_test.py

Options:
  --spawn-first       run spawn_test_scene_pie before navigation
  --output-dir PATH   directory for JSON + PNG outputs
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Dict, List

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
from pie_safety import PieSessionLost, ping_ok, require_live_ucv, tick_settle  # noqa: E402
from depth_object_perception import (  # noqa: E402
    PerceptionConfig,
    depth_npy_to_meters,
    detect_objects,
    estimates_by_prop_type,
    fetch_depth_npy,
)
from plot_results import plot_distance_and_bearing, summarize_rmse  # noqa: E402
from prop_placement import load_registry  # noqa: E402
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
from simple_nav import NavigationRunResult, navigate_to_target  # noqa: E402
from simworld.communicator.communicator import Communicator  # noqa: E402

CACHE_DIR = THIS_DIR / "cache"
DEFAULT_OUTPUT_DIR = CACHE_DIR / "runs"


def _spawn_scene() -> None:
    script = THIS_DIR / "spawn_test_scene_pie.py"
    subprocess.check_call([sys.executable, str(script)])


def _perceive(
    ucv,
    communicator: Communicator,
    camera_id: int,
    registry,
    robot_name: str,
    cfg: PerceptionConfig,
    *,
    only_prop_type_id: Optional[str] = None,
) -> Dict[str, Dict[str, float]]:
    update_sensor_camera_pose(ucv, robot_name, camera_id)
    tick_settle(ucv, settle_s=0.2, ticks=1)
    mask = fetch_mask_rgb(communicator, camera_id)
    lit = fetch_lit_bgr(communicator, camera_id)
    depth_raw = fetch_depth_npy(ucv, camera_id)
    if depth_raw is None or mask is None:
        return {}
    depth_m = depth_npy_to_meters(depth_raw)
    estimates = detect_objects(
        mask, depth_m, registry, lit_bgr=lit, only_prop_type_id=only_prop_type_id, config=cfg
    )
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


def _serialize_run(run: NavigationRunResult) -> Dict:
    return {
        "target_prop_type_id": run.target_prop_type_id,
        "reached": run.reached,
        "aborted": run.aborted,
        "abort_reason": run.abort_reason,
        "samples": [
            {
                "t_s": s.t_s,
                "robot_xy": list(s.robot_xy),
                "robot_yaw_deg": s.robot_yaw_deg,
                "estimates": s.estimates,
                "ground_truth": s.ground_truth,
            }
            for s in run.samples
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Depth recognition PIE test")
    parser.add_argument("--spawn-first", action="store_true")
    parser.add_argument("--max-targets", type=int, default=0, help="limit navigation legs (0=all)")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()

    if args.spawn_first:
        _spawn_scene()

    registry = load_registry()
    missing_world = [p.slot_id for p in registry.props if p.world_xyz_cm is None]
    if missing_world:
        print(f"[DepthTest] registry missing world poses: {missing_world}")
        print("[DepthTest] run spawn_test_scene_pie.py first")
        return 1

    try:
        with ue_client_guard.exclusive_ue_client_lock():
            ucv, _ = g10k.ensure_connection()
            robot_name = geh.ROBOT_ACTOR_NAME
            ok, _ = lnr.soft_reset_level_spotdog(ucv, registry.spotdog_spawn_local_cm)
            if not ok:
                print("[DepthTest] SpotDog not available")
                return 1

            camera_id = resolve_sensor_camera_id(ucv)
            configure_sensor_camera(ucv, camera_id)
            communicator = Communicator(ucv)
            cfg = PerceptionConfig(fov_deg=SENSOR_FOV_DEG)

            t0 = time.time()
            all_runs: List[NavigationRunResult] = []
            targets = list(registry.visit_order_props())
            if args.max_targets > 0:
                targets = targets[: args.max_targets]
            for prop in targets:
                require_live_ucv(ucv, context=f"nav leg to {prop.prop_type_id}")
                goal_xy = (prop.world_xyz_cm[0], prop.world_xyz_cm[1])  # type: ignore[index]
                print(f"[DepthTest] navigating to {prop.prop_type_id} (order {prop.visit_order}) ...")
                run = navigate_to_target(
                    ucv,
                    robot_name,
                    goal_xy,
                    registry=registry,
                    fov_deg=SENSOR_FOV_DEG,
                    perceive_fn=lambda ptype=prop.prop_type_id: _perceive(
                        ucv, communicator, camera_id, registry, robot_name, cfg,
                        only_prop_type_id=ptype,
                    ),
                    get_pose_fn=lambda: (get_pos2d(ucv, robot_name), get_yaw_deg(ucv, robot_name)),
                    target_prop_type_id=prop.prop_type_id,
                    t0=t0,
                    connection_check=lambda: ping_ok(ucv),
                )
                if run.aborted:
                    print(f"[DepthTest] ABORT leg {prop.prop_type_id}: {run.abort_reason}")
                    all_runs.append(run)
                    break
                print(f"[DepthTest] reached={run.reached} samples={len(run.samples)}")
                all_runs.append(run)
                tick_settle(ucv, settle_s=1.0, ticks=2)

            out_dir = args.output_dir
            out_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            json_path = out_dir / f"depth_recognition_{stamp}.json"
            payload = {
                "registry_seed": registry.seed,
                "props": [p.to_dict() for p in registry.props],
                "runs": [_serialize_run(r) for r in all_runs],
            }
            json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            print(f"[DepthTest] wrote {json_path}")

            rmse_summary = summarize_rmse(all_runs, registry)
            rmse_path = out_dir / f"depth_recognition_{stamp}_rmse.json"
            rmse_path.write_text(json.dumps(rmse_summary, indent=2), encoding="utf-8")
            print(f"[DepthTest] RMSE summary: {rmse_summary}")

            dist_png = out_dir / f"depth_recognition_{stamp}_distance.png"
            bear_png = out_dir / f"depth_recognition_{stamp}_bearing.png"
            plot_distance_and_bearing(all_runs, registry, dist_png, bear_png, rmse_summary)
            print(f"[DepthTest] plots: {dist_png} , {bear_png}")
            return 0
    except PieSessionLost as exc:
        print(f"[DepthTest] ABORT: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
