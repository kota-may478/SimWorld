#!/usr/bin/env python3
"""30 m × 30 m compact nav test: L0 NavMesh + FusionCam L2 + SpotDog to (25m, 25m)."""

from __future__ import annotations

import os

os.environ["MPLBACKEND"] = "Agg"

import argparse
import sys
import time
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="compact_nav")

import grid_env_hri_simulation as geh  # noqa: E402
import ue_client_guard  # noqa: E402
import level_coords as lc  # noqa: E402
import level_nav_robot as lnr  # noqa: E402
import nav_query as nq  # noqa: E402
from depth_object_perception import (  # noqa: E402
    PerceptionConfig,
    depth_npy_to_meters,
    detect_objects,
    estimates_by_prop_type,
)
from ground_truth import ground_truth_all_props  # noqa: E402
from grid_env_10k_pie_patrol import get_pos2d, get_yaw  # noqa: E402
from l0_crop import crop_l0_to_local_region  # noqa: E402
from l2_fusion import estimate_world_xy_from_detection  # noqa: E402
from layered_nav import navigate_layered_with_fusion  # noqa: E402
from paths import COMPACT_NAV_RUN_DIR, L0_MASK_STRICT  # noqa: E402
from pie_safety import PieSessionLost, require_live_ucv, tick_settle  # noqa: E402
from placement import ensure_registry, to_placement_registry  # noqa: E402
from region import REGION_SIZE_CM  # noqa: E402
from robot_sensor import (  # noqa: E402
    SENSOR_FOV_DEG,
    configure_sensor_camera,
    fetch_lit_bgr,
    fetch_mask_rgb,
    resolve_mask_camera_id,
    resolve_sensor_camera_id,
    restore_editor_viewmode_lit,
    update_sensor_camera_pose,
)
from simworld.communicator.communicator import Communicator  # noqa: E402
from simple_nav import TimeSeriesSample  # noqa: E402
from spawn_pie import spawn_compact_scene  # noqa: E402
from viz import DEFAULT_ARTIFACT_DIR, NavTrace, save_compact_nav_artifacts  # noqa: E402

DEFAULT_L0 = L0_MASK_STRICT
ARRIVE_TOLERANCE_CM = 130.0


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Compact 30m nav test with FusionCam L2")
    p.add_argument("--l0", type=Path, default=DEFAULT_L0)
    p.add_argument("--skip-spawn", action="store_true")
    p.add_argument("--spawn-only", action="store_true")
    p.add_argument("--plan-only", action="store_true")
    p.add_argument("--max-nav-steps", type=int, default=500)
    p.add_argument("--force-rebuild-registry", action="store_true")
    p.add_argument("--artifact-dir", type=Path, default=DEFAULT_ARTIFACT_DIR)
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    if not args.l0.is_file():
        print(f"[CompactNav] missing L0: {args.l0}")
        return 1

    registry = ensure_registry(force_rebuild=args.force_rebuild_registry)
    layers = crop_l0_to_local_region(args.l0, size_x_cm=REGION_SIZE_CM, size_y_cm=REGION_SIZE_CM)
    start = registry.robot_start_local_cm
    goal = registry.goal_local_cm
    plan = layers.plan_astar_local(start, goal)
    print(f"[CompactNav] L0 plan {start} → {goal}: {len(plan.waypoints_xy)} WP cost={plan.total_cost:.1f}")
    if not plan.waypoints_xy:
        return 1
    if args.plan_only:
        return 0

    if args.spawn_only:
        rc, _ = spawn_compact_scene(force_rebuild=args.force_rebuild_registry)
        return rc

    try:
        ucv, _ = ue_client_guard.prepare_ue_connection(force_new=True)
        if not args.skip_spawn:
            spawn_rc, ucv = spawn_compact_scene(
                force_rebuild=args.force_rebuild_registry,
                ucv=ucv,
                manage_connection=False,
            )
            if spawn_rc != 0:
                return spawn_rc
            registry = ensure_registry()

        require_live_ucv(ucv, context="compact nav start")
        ok_nav, _ = nq.ensure_nav_query_service(
            ucv, probe_xyz=lc.foot_world_xyz_from_local_xy(*start)
        )
        if not ok_nav:
            print("[CompactNav] NavQueryService unavailable")
            return 2

        ok_robot, robot_name = lnr.soft_reset_level_spotdog(ucv, start)
        if not ok_robot:
            print("[CompactNav] SpotDog unavailable")
            return 2
        tick_settle(ucv, settle_s=0.8, ticks=2)

        fusion_id = resolve_sensor_camera_id(ucv)
        configure_sensor_camera(ucv, fusion_id)
        communicator = Communicator(ucv)
        mask_id = resolve_mask_camera_id(communicator, ucv, fusion_id)
        configure_sensor_camera(ucv, mask_id)
        placement_reg = to_placement_registry(registry)
        perceive_cfg = PerceptionConfig(
            fov_deg=SENSOR_FOV_DEG,
            camera_offset_forward_cm=22.0,
            camera_pitch_deg=-5.0,
        )
        t0 = time.time()
        trace = NavTrace()

        def _perceive():
            update_sensor_camera_pose(ucv, robot_name, fusion_id)
            tick_settle(ucv, settle_s=0.2, ticks=1)
            mask = fetch_mask_rgb(communicator, mask_id)
            lit = fetch_lit_bgr(communicator, fusion_id)
            from depth_object_perception import fetch_depth_npy  # noqa: WPS433

            depth_raw = fetch_depth_npy(ucv, fusion_id)
            if mask is None or depth_raw is None:
                return []
            depth_m = depth_npy_to_meters(depth_raw)
            detections = detect_objects(
                mask, depth_m, placement_reg, lit_bgr=lit, config=perceive_cfg
            )
            robot_xy = get_pos2d(ucv, robot_name)
            robot_yaw = get_yaw(ucv, robot_name)
            for det in detections:
                wx, wy = estimate_world_xy_from_detection(
                    robot_xy,
                    robot_yaw,
                    distance_m=float(det.distance_m),
                    bearing_deg=float(det.bearing_deg),
                    camera_offset_forward_cm=perceive_cfg.camera_offset_forward_cm,
                )
                trace.record_l2_estimate(lc.world_xy_to_local(wx, wy))
            gt_all = ground_truth_all_props(
                robot_xy,
                robot_yaw,
                placement_reg,
                fov_deg=SENSOR_FOV_DEG,
            )
            by_type = estimates_by_prop_type(detections)
            trace.perception_samples.append(
                TimeSeriesSample(
                    t_s=time.time() - t0,
                    robot_xy=robot_xy,
                    robot_yaw_deg=robot_yaw,
                    estimates={
                        pid: {
                            "distance_m": est.distance_m,
                            "bearing_deg": est.bearing_deg,
                            "confidence": est.confidence,
                        }
                        for pid, est in by_type.items()
                    },
                    ground_truth={
                        pid: {
                            "distance_m": gt.distance_m,
                            "bearing_deg": gt.bearing_deg,
                            "in_fov": float(gt.in_fov),
                        }
                        for pid, gt in gt_all.items()
                    },
                )
            )
            return detections

        layers.reset_l2()
        arrived = navigate_layered_with_fusion(
            ucv,
            layers,
            goal,
            perceive_fn=_perceive,
            robot_name=robot_name,
            tolerance_cm=ARRIVE_TOLERANCE_CM,
            label="to-goal",
            max_total_steps=args.max_nav_steps,
            trace=trace,
        )
        restore_editor_viewmode_lit(ucv)
        artifact_paths = save_compact_nav_artifacts(
            layers,
            registry,
            trace,
            output_dir=args.artifact_dir,
            placement_registry=placement_reg,
        )
        for key, path in sorted(artifact_paths.items()):
            print(f"[CompactNav] artifact {key}: {path}")
        if not arrived:
            print("[CompactNav] FAIL: did not reach goal")
            return 3
        print("[CompactNav] PASS")
        return 0
    except PieSessionLost as exc:
        print(f"[CompactNav] ABORT: {exc}")
        return 10
    except (ValueError, RuntimeError) as exc:
        print(f"[CompactNav] ABORT: navigation planning failed: {exc}")
        return 11
    finally:
        try:
            geh.release_connection()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
