#!/usr/bin/env python3
"""E2E: 20 m site transport — spawn → L0+L1+L2 nav → carry → deliver to humanoid."""

from __future__ import annotations

import os

os.environ["MPLBACKEND"] = "Agg"

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import ue_client_guard  # noqa: E402
import level_coords as lc  # noqa: E402
import level_nav_robot as lnr  # noqa: E402
import nav_query as nq  # noqa: E402
from carry import (  # noqa: E402
    begin_carry_from_material,
    deliver_carry_at_humanoid,
    pickup_standoff_xy,
    reset_carry_attach_state,
    reset_carry_from_previous_mission,
)
from grid_env_10k_pie_patrol import dist2d, get_pos2d, get_yaw  # noqa: E402
from l0_crop import crop_l0_to_local_region  # noqa: E402
from depth_object_perception import depth_npy_to_meters, fetch_depth_npy  # noqa: E402
from l2_fusion import estimate_world_xy_from_detection  # noqa: E402
from l2_geom import GeomPerceptionConfig, geom_detections  # noqa: E402
from layered_nav import (  # noqa: E402
    PerceiveOutcome,
    SITE_DEFAULT_PERCEPTION_INTERVAL_S,
    navigate_layered_with_fusion,
)
from l2_sight import (  # noqa: E402
    L2SlotCellTracker,
    SightConfig,
    SightMemory,
    estimate_local_xy_from_detection,
    soft_l2_reset,
    update_l2_from_sight,
)
from metrics import MissionRecorder, save_metrics_json  # noqa: E402
from paths import L0_MASK_STRICT  # noqa: E402
from pie_safety import PieSessionLost, require_live_ucv, tick_settle  # noqa: E402
from perception_layer import (  # noqa: E402
    EgocentricPerceptionConfig,
    apply_l2_obstacle_cells,
    close_range_keepout_cells_from_depth,
    obstacle_cells_from_depth_gated_by_detections,
    update_l2_from_depth_image,
)
from placement import ensure_registry, to_placement_registry  # noqa: E402
from region import REGION_SIZE_CM  # noqa: E402
from robot_sensor import (  # noqa: E402
    SENSOR_CAM_FORWARD_OFFSET_CM,
    SENSOR_CAM_HEIGHT_OFFSET_CM,
    SENSOR_CAM_PITCH_DEG,
    SENSOR_FOV_DEG,
    configure_sensor_camera,
    fetch_mask_rgb,
    resolve_sensor_camera_id,
    restore_editor_viewmode_lit,
    update_sensor_camera_pose,
)
from pie_spawn_safety import ensure_live_or_reconnect  # noqa: E402
from runtime_sight_sources import ensure_runtime_site20_sight_sources  # noqa: E402
from spawn_pie import spawn_site_transport_scene  # noqa: E402
from viz import DEFAULT_ARTIFACT_DIR, NavTrace, save_site_transport_artifacts  # noqa: E402
from zones import apply_forbidden_zones_l1  # noqa: E402

DEFAULT_L0 = L0_MASK_STRICT
ARRIVE_TOLERANCE_CM = 130.0
DEPTH_CLEARANCE_TRIGGER_CM = 125.0
DEPTH_KEEP_OUT_RADIUS_CM = 100.0
_RELEASE_UE = Path(__file__).resolve().parents[2] / "release_ue_connection.py"


def _connect_ue_with_retry(*, attempts: int = 4, pause_s: float = 15.0):
    """Connect to UnrealCV; retry after release when PIE has stale CloseWait."""
    import subprocess

    last_exc: Optional[Exception] = None
    for attempt in range(1, attempts + 1):
        try:
            if attempt == 1:
                ue_client_guard.wait_for_unrealcv_banner_ready(timeout_s=30.0)
            return ue_client_guard.prepare_ue_connection(force_new=True)
        except ConnectionError as exc:
            last_exc = exc
            print(f"[Site20] UE connect attempt {attempt}/{attempts} failed")
            if attempt >= attempts:
                break
            print(
                "[Site20] release_ue_connection + wait "
                "(if this repeats: PIE Stop → Play on Level)"
            )
            if _RELEASE_UE.is_file():
                subprocess.run(
                    [sys.executable, str(_RELEASE_UE)],
                    check=False,
                    timeout=120,
                )
            time.sleep(pause_s)
    if last_exc is not None:
        raise last_exc
    raise ConnectionError("UnrealCV connect failed")


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Site transport 20m E2E with metrics")
    p.add_argument("--l0", type=Path, default=DEFAULT_L0)
    p.add_argument("--skip-spawn", action="store_true")
    p.add_argument("--spawn-only", action="store_true")
    p.add_argument("--plan-only", action="store_true")
    p.add_argument("--max-nav-steps", type=int, default=600)
    p.add_argument("--force-rebuild-registry", action="store_true")
    p.add_argument(
        "--l2-mode",
        choices=("sight", "geom", "camera", "off"),
        default="sight",
        help="L2 perception: sight (AI Perception, default), geom, camera, or off",
    )
    p.add_argument(
        "--no-l2",
        action="store_true",
        help="Alias for --l2-mode off (L0+L1 navigation only)",
    )
    p.add_argument(
        "--no-l1",
        action="store_true",
        help="Skip L1 forbidden-zone rasterization (no forbidden rects on L1)",
    )
    p.add_argument("--artifact-dir", type=Path, default=DEFAULT_ARTIFACT_DIR)
    return p.parse_args()


def _material_goal_xy(registry) -> tuple[float, float]:
    transport = registry.transport_slot()
    if transport is not None and transport.world_xyz_cm is not None:
        return transport.world_xyz_cm[0], transport.world_xyz_cm[1]
    return lc.local_xy_to_world(*registry.material_pickup_local_cm)


def _humanoid_goal_local(registry) -> tuple[float, float]:
    return registry.humanoid_local_cm


def main() -> int:
    args = _parse_args()
    l2_mode = "off" if args.no_l2 else args.l2_mode
    if not args.l0.is_file():
        print(f"[Site20] missing L0: {args.l0}")
        return 1

    registry = ensure_registry(force_rebuild=args.force_rebuild_registry)
    layers = crop_l0_to_local_region(args.l0, size_x_cm=REGION_SIZE_CM, size_y_cm=REGION_SIZE_CM)
    if args.no_l1:
        n_l1 = 0
        print(f"[Site20] L1 forbidden zones DISABLED (--no-l1); props → L2 via {l2_mode}")
    else:
        n_l1 = apply_forbidden_zones_l1(layers, registry.forbidden_zones)
        print(f"[Site20] L1 forbidden cells: {n_l1} (props → L2 via {l2_mode})")

    start = registry.robot_start_local_cm
    material_local = registry.material_pickup_local_cm
    human_local = registry.humanoid_local_cm
    plan_out = layers.plan_astar_local(start, material_local)
    plan_back = layers.plan_astar_local(material_local, human_local)
    print(
        f"[Site20] L0+L1 plan to material: {len(plan_out.waypoints_xy)} WP cost={plan_out.total_cost:.1f}"
    )
    print(
        f"[Site20] L0+L1 plan to humanoid: {len(plan_back.waypoints_xy)} WP cost={plan_back.total_cost:.1f}"
    )
    if not plan_out.waypoints_xy or not plan_back.waypoints_xy:
        return 1
    if args.plan_only:
        return 0

    if args.spawn_only:
        rc, _ = spawn_site_transport_scene(force_rebuild=args.force_rebuild_registry)
        return rc

    mission_t0 = time.time()
    reset_carry_attach_state()
    success = False
    metrics: dict = {}
    trace = NavTrace()
    ucv = None

    try:
        ucv, _ = _connect_ue_with_retry()
        fresh_spawn = False
        if not args.skip_spawn:
            spawn_rc, ucv = spawn_site_transport_scene(
                force_rebuild=args.force_rebuild_registry,
                force_respawn=args.force_rebuild_registry,
                ucv=ucv,
                manage_connection=False,
            )
            if spawn_rc != 0:
                return spawn_rc
            fresh_spawn = True
            registry = ensure_registry()
            tick_settle(ucv, settle_s=5.0, ticks=4)
            ucv = ensure_live_or_reconnect(ucv, reason="post spawn settle")

        placement_reg_holder: dict = {}

        require_live_ucv(ucv, context="site transport start")
        ok_nav, nav_actor = nq.ensure_nav_query_service(
            ucv, probe_xyz=lc.foot_world_xyz_from_local_xy(*start)
        )
        if not ok_nav:
            print("[Site20] NavQueryService unavailable")
            return 2

        robot_name = lnr.find_spotdog_actor(ucv) or geh.ROBOT_ACTOR_NAME
        reset_carry_from_previous_mission(ucv, registry.carry_actor_name, robot_name=robot_name)
        if fresh_spawn:
            if not lnr.ensure_robot_upright_at_start(
                ucv, robot_name, start, nav_actor=nav_actor
            ):
                print("[Site20] SpotDog upright recovery failed")
                return 2
            if not lnr.verify_spotdog_at_start(ucv, robot_name, start):
                ok_robot, robot_name = lnr.prepare_spotdog_mission_start(
                    ucv, start, nav_actor=nav_actor
                )
                if not ok_robot:
                    print("[Site20] SpotDog unavailable after spawn verify")
                    return 2
        else:
            ok_robot, robot_name = lnr.prepare_spotdog_mission_start(
                ucv, start, nav_actor=nav_actor
            )
            if not ok_robot:
                print("[Site20] SpotDog unavailable")
                return 2
        try:
            ucv.enable_controller(robot_name, True)
        except Exception:
            pass
        if l2_mode == "sight":
            lnr.ensure_spotdog_sight_controller(ucv, robot_name)
            if not fresh_spawn:
                ensure_runtime_site20_sight_sources(ucv)
        robot_xy = get_pos2d(ucv, robot_name)
        robot_local = lc.world_xy_to_local(*robot_xy)
        print(
            f"[Site20] SpotDog start local=({robot_local[0]:.1f}, {robot_local[1]:.1f}) "
            f"target=({start[0]:.1f}, {start[1]:.1f})"
        )
        tick_settle(ucv, settle_s=0.8, ticks=2)
        ucv = ensure_live_or_reconnect(ucv, reason="before nav")

        if l2_mode == "off":
            print("[Site20] L2 perception disabled (--l2-mode off)")

        def _perceive_disabled():
            return []

        _perceive = _perceive_disabled
        _reset_l2_perceive_counter = None
        sight_memory = SightMemory()
        sight_tracker = L2SlotCellTracker()
        sight_cfg = SightConfig(
            fov_deg=SENSOR_FOV_DEG,
            max_range_cm=650.0,
            sensor_forward_cm=SENSOR_CAM_FORWARD_OFFSET_CM,
            prop_radius_cm=80.0,
        )
        sight_depth_cfg = EgocentricPerceptionConfig(
            fov_deg=SENSOR_FOV_DEG,
            max_range_cm=650.0,
            min_obstacle_height_cm=35.0,
            stride_px=6,
            use_lethal=True,
            camera_offset_forward_cm=SENSOR_CAM_FORWARD_OFFSET_CM,
            camera_height_cm=SENSOR_CAM_HEIGHT_OFFSET_CM,
        )
        sight_depth_cam: dict = {"ready": False, "fusion_id": None}

        def _reset_sight_memory(label: str, *, keep_static: bool = False) -> None:
            sight_memory.dynamic_last_seen_xy.clear()
            if keep_static:
                # Between legs: evict only dynamic slots, preserve static map.
                for slot_id in list(sight_tracker.slot_to_cells.keys()):
                    if slot_id in (registry.humanoid_actor_name,) or slot_id.startswith("__"):
                        for gx, gy in sight_tracker.slot_to_cells.pop(slot_id, set()):
                            layers.l2[gy, gx] = 0
                print(f"[Site20] L2 sight memory soft-reset ({label}, static map preserved)")
            else:
                sight_memory.static_last_seen_xy.clear()
                for cells in sight_tracker.slot_to_cells.values():
                    for gx, gy in cells:
                        layers.l2[gy, gx] = 0
                sight_tracker.slot_to_cells.clear()
                print(f"[Site20] L2 sight memory reset ({label})")

        def _soft_reset_l2(l2_seen_cells: set, stuck_world_xy=None) -> None:
            soft_l2_reset(
                sight_memory, sight_tracker, layers, l2_seen_cells,
                stuck_world_xy=stuck_world_xy,
            )

        def _ensure_sight_depth_camera() -> int:
            if not sight_depth_cam["ready"]:
                fusion_id = resolve_sensor_camera_id(ucv)
                configure_sensor_camera(ucv, fusion_id)
                sight_depth_cam["fusion_id"] = fusion_id
                sight_depth_cam["ready"] = True
                print(f"[Site20] L2 sight depth camera ready fusion={fusion_id}")
            return int(sight_depth_cam["fusion_id"])

        def _apply_depth_for_ai_sight(detections) -> int:
            nonlocal ucv
            fusion_id = _ensure_sight_depth_camera()
            update_sensor_camera_pose(ucv, robot_name, fusion_id)
            tick_settle(ucv, settle_s=0.25, ticks=1)
            depth_raw = fetch_depth_npy(ucv, fusion_id)
            if depth_raw is None:
                print("[Site20] L2 sight depth fetch skipped: no depth")
                return 0
            depth_m = depth_npy_to_meters(depth_raw)
            finite_depth = depth_m[np.isfinite(depth_m) & (depth_m > 0.05) & (depth_m < 80.0)]
            if finite_depth.size:
                print(
                    f"[Site20] depth sample: min={float(np.min(finite_depth)):.2f}m "
                    f"p10={float(np.percentile(finite_depth, 10)):.2f}m "
                    f"near1m={int(np.sum(finite_depth <= 1.0))}px "
                    f"near{DEPTH_CLEARANCE_TRIGGER_CM / 100.0:.2f}m="
                    f"{int(np.sum(finite_depth <= DEPTH_CLEARANCE_TRIGGER_CM / 100.0))}px "
                    f"visible={len(detections)}"
                )
            robot_xy = get_pos2d(ucv, robot_name)
            robot_yaw = get_yaw(ucv, robot_name)
            cells = []
            if detections:
                cells.extend(
                    obstacle_cells_from_depth_gated_by_detections(
                        depth_m,
                        layers,
                        robot_xy=robot_xy,
                        robot_yaw_deg=robot_yaw,
                        detections=detections,
                        config=sight_depth_cfg,
                    )
                )
            cells.extend(
                close_range_keepout_cells_from_depth(
                    depth_m,
                    layers,
                    robot_xy=robot_xy,
                    robot_yaw_deg=robot_yaw,
                    config=sight_depth_cfg,
                    min_clearance_cm=DEPTH_CLEARANCE_TRIGGER_CM,
                    keepout_radius_cm=DEPTH_KEEP_OUT_RADIUS_CM,
                    camera_pitch_deg=SENSOR_CAM_PITCH_DEG,
                )
            )
            prev_depth = sight_tracker.pop_cells("__depth__")
            if prev_depth:
                other_owned: set = set()
                for other_id, other_cells in sight_tracker.slot_to_cells.items():
                    other_owned.update(other_cells)
                for gx, gy in prev_depth:
                    if (gx, gy) in other_owned:
                        continue
                    layers.l2[gy, gx] = 0
                    l2_seen_cells.discard((gx, gy))
            if not cells:
                return 0
            if finite_depth.size:
                print(f"[Site20] depth keepout cells: frame={len(cells)}")
            n = apply_l2_obstacle_cells(layers, cells, config=sight_depth_cfg)
            sight_tracker.set_cells("__depth__", set(cells))
            for cell in cells:
                l2_seen_cells.add(cell)
            return n

        if l2_mode == "sight":
            placement_reg = to_placement_registry(registry)
            placement_reg_holder["reg"] = placement_reg
            print(
                f"[Site20] L2 sight enabled: FOV={sight_cfg.fov_deg}° "
                f"range={sight_cfg.max_range_cm}cm interval={SITE_DEFAULT_PERCEPTION_INTERVAL_S}s "
                f"static=persist dynamic=sticky-until-rescan"
            )

            def _perceive_sight(*, layers, l2_seen_cells):
                result = update_l2_from_sight(
                    ucv,
                    layers,
                    robot_name=robot_name,
                    placement_reg=placement_reg_holder["reg"],
                    humanoid_actor_name=registry.humanoid_actor_name,
                    material_actor_name=registry.material_actor_name,
                    memory=sight_memory,
                    tracker=sight_tracker,
                    l2_seen_cells=l2_seen_cells,
                    config=sight_cfg,
                    apply_cells=True,
                )
                visible_slots = set(result.visible_slot_ids)
                visible_detections = [
                    det for det in result.detections if det.slot_id in visible_slots
                ]
                n_depth_cells = _apply_depth_for_ai_sight(visible_detections)
                n_cells_added = result.cells_added + n_depth_cells
                for cell in sight_tracker.cells_for("__depth__"):
                    l2_seen_cells.add(cell)
                for det in result.detections:
                    local_xy = estimate_local_xy_from_detection(
                        get_pos2d(ucv, robot_name),
                        get_yaw(ucv, robot_name),
                        det,
                        sensor_forward_cm=sight_cfg.sensor_forward_cm,
                    )
                    trace.record_l2_estimate(local_xy)
                if result.visible_actor_names or result.l2_changed:
                    print(
                        f"[Site20] L2 sight ({result.backend}): "
                        f"visible={len(result.visible_actor_names)} "
                        f"+{result.cells_added} ai +{n_depth_cells} depth "
                        f"/-{result.cells_removed} cells"
                    )
                return PerceiveOutcome(
                    detections=result.detections,
                    cells_added=n_cells_added,
                    cells_removed=result.cells_removed,
                    l2_applied=True,
                )

            _perceive = _perceive_sight

        elif l2_mode == "geom":
            placement_reg = to_placement_registry(registry)
            placement_reg_holder["reg"] = placement_reg
            geom_cfg = GeomPerceptionConfig(
                fov_deg=SENSOR_FOV_DEG,
                max_range_cm=650.0,
                sensor_forward_cm=SENSOR_CAM_FORWARD_OFFSET_CM,
            )
            print(
                f"[Site20] L2 geom enabled: FOV={geom_cfg.fov_deg}° "
                f"range={geom_cfg.max_range_cm}cm interval={SITE_DEFAULT_PERCEPTION_INTERVAL_S}s "
                f"({len(placement_reg.props)} props in registry)"
            )

            def _perceive_geom():
                robot_xy = get_pos2d(ucv, robot_name)
                robot_yaw = get_yaw(ucv, robot_name)
                detections = geom_detections(
                    robot_xy,
                    robot_yaw,
                    placement_reg_holder["reg"],
                    config=geom_cfg,
                )
                for det in detections:
                    wx, wy = estimate_world_xy_from_detection(
                        robot_xy,
                        robot_yaw,
                        distance_m=float(det.distance_m),
                        bearing_deg=float(det.bearing_deg),
                        camera_offset_forward_cm=geom_cfg.sensor_forward_cm,
                    )
                    trace.record_l2_estimate(lc.world_xy_to_local(wx, wy))
                if detections:
                    summary = ", ".join(
                        f"{d.prop_type_id}@{d.distance_m:.1f}m" for d in detections
                    )
                    print(f"[Site20] L2 geom: {len(detections)} props [{summary}]")
                return detections

            _perceive = _perceive_geom

        elif l2_mode == "camera":
            from depth_object_perception import (  # noqa: WPS433
                PerceptionConfig,
                detect_objects,
            )
            from object_mask_color import sync_registry_mask_colors  # noqa: WPS433
            from placement import apply_mask_colors_from_placement  # noqa: WPS433
            from simworld.communicator.communicator import Communicator  # noqa: WPS433

            placement_reg = to_placement_registry(registry)
            if args.skip_spawn:
                placement_reg = sync_registry_mask_colors(
                    ucv, placement_reg, reapply_colors=True
                )
                registry = apply_mask_colors_from_placement(registry, placement_reg)
                print(f"[Site20] L2 mask colors synced for {len(placement_reg.props)} props")
            placement_reg_holder["reg"] = placement_reg
            perceive_cfg = PerceptionConfig(
                fov_deg=SENSOR_FOV_DEG,
                camera_offset_forward_cm=SENSOR_CAM_FORWARD_OFFSET_CM,
                camera_pitch_deg=-5.0,
            )
            l2_cam: dict = {
                "ready": False,
                "fusion_id": None,
                "mask_id": None,
                "communicator": None,
            }
            l2_perceive_count = {"n": 0}
            l2_depth_scan_count = {"n": 0}
            L2_MAX_DEPTH_SCANS_PER_LEG = 1
            depth_l2_cfg = EgocentricPerceptionConfig(
                fov_deg=SENSOR_FOV_DEG,
                max_range_cm=650.0,
                min_obstacle_height_cm=35.0,
                stride_px=8,
                use_lethal=True,
                camera_offset_forward_cm=perceive_cfg.camera_offset_forward_cm,
                camera_height_cm=perceive_cfg.camera_height_cm,
            )

            def _reset_l2_perceive_counter(label: str) -> None:
                l2_perceive_count["n"] = 0
                l2_depth_scan_count["n"] = 0
                print(f"[Site20] L2 perceive counter reset ({label})")

            def _ensure_l2_cameras() -> None:
                nonlocal ucv
                if l2_cam["ready"]:
                    return
                fusion_id = resolve_sensor_camera_id(ucv)
                configure_sensor_camera(ucv, fusion_id)
                communicator = Communicator(ucv)
                mask_id = fusion_id
                restore_editor_viewmode_lit(ucv)
                tick_settle(ucv, settle_s=1.0, ticks=2)
                l2_cam["fusion_id"] = fusion_id
                l2_cam["mask_id"] = mask_id
                l2_cam["communicator"] = communicator
                l2_cam["ready"] = True
                print(f"[Site20] L2 cameras ready fusion={fusion_id} mask={mask_id}")

            def _perceive_camera():
                nonlocal ucv
                try:
                    require_live_ucv(ucv, context="perceive")
                    _ensure_l2_cameras()
                    l2_perceive_count["n"] += 1
                    n = l2_perceive_count["n"]
                    fusion_id = l2_cam["fusion_id"]
                    mask_id = l2_cam["mask_id"]
                    communicator = l2_cam["communicator"]
                    if l2_depth_scan_count["n"] >= L2_MAX_DEPTH_SCANS_PER_LEG:
                        print(
                            f"[Site20] L2 depth fetch skipped n={n} "
                            f"(scan limit {L2_MAX_DEPTH_SCANS_PER_LEG}/leg)"
                        )
                        return []
                    update_sensor_camera_pose(ucv, robot_name, fusion_id)
                    tick_settle(ucv, settle_s=0.8, ticks=2)
                    from depth_object_perception import fetch_depth_npy  # noqa: WPS433

                    print(f"[Site20] L2 depth fetch start n={n} cam={fusion_id}")
                    depth_raw = fetch_depth_npy(ucv, fusion_id)
                    print(f"[Site20] L2 depth fetch ok n={n}")
                    if depth_raw is None:
                        return []
                    depth_m = depth_npy_to_meters(depth_raw)
                    robot_xy = get_pos2d(ucv, robot_name)
                    robot_yaw = get_yaw(ucv, robot_name)
                    n_depth_cells = update_l2_from_depth_image(
                        depth_m,
                        layers,
                        robot_xy=robot_xy,
                        robot_yaw_deg=robot_yaw,
                        config=depth_l2_cfg,
                    )
                    l2_depth_scan_count["n"] += 1
                    print(f"[Site20] L2 depth cells written={n_depth_cells}")
                    mask = fetch_mask_rgb(communicator, mask_id, mode="fast")
                    if mask is None:
                        return []
                    detections = detect_objects(
                        mask,
                        depth_m,
                        placement_reg_holder["reg"],
                        config=perceive_cfg,
                    )
                    for det in detections:
                        wx, wy = estimate_world_xy_from_detection(
                            robot_xy,
                            robot_yaw,
                            distance_m=float(det.distance_m),
                            bearing_deg=float(det.bearing_deg),
                            camera_offset_forward_cm=perceive_cfg.camera_offset_forward_cm,
                        )
                        trace.record_l2_estimate(lc.world_xy_to_local(wx, wy))
                    return detections
                except (ConnectionError, OSError, ValueError, RuntimeError) as exc:
                    print(f"[Site20] perceive skipped: {exc}")
                    ucv = ensure_live_or_reconnect(ucv, reason="perceive failure")
                    return []

            _perceive = _perceive_camera
            print("[Site20] L2 perception enabled (camera mode; depth+mask)")

        recorder = MissionRecorder(mission_t0, registry.forbidden_zones)

        def _on_pose(pos_xy, now_t: float) -> None:
            recorder.record_pose(pos_xy, now=now_t)

        material_xy = _material_goal_xy(registry)
        robot_xy = get_pos2d(ucv, robot_name)
        approach_xy = pickup_standoff_xy(material_xy, robot_xy)
        approach_local = lc.world_xy_to_local(*approach_xy)
        print(f"[Site20] leg1 material @ {material_xy}, approach @ {approach_xy}")
        tick_settle(ucv, settle_s=6.0, ticks=4)
        ucv = ensure_live_or_reconnect(ucv, reason="pre leg1 settle")

        layers.reset_l2()
        if l2_mode == "sight":
            _reset_sight_memory("leg1")
        leg1_ok = navigate_layered_with_fusion(
            ucv,
            layers,
            approach_local,
            perceive_fn=_perceive,
            soft_reset_fn=_soft_reset_l2 if l2_mode == "sight" else None,
            robot_name=robot_name,
            nav_actor=nav_actor,
            tolerance_cm=ARRIVE_TOLERANCE_CM,
            label="to-material",
            perception_interval_s=SITE_DEFAULT_PERCEPTION_INTERVAL_S,
            max_total_steps=args.max_nav_steps,
            trace=trace,
            on_pose_sample=_on_pose,
        )
        if not leg1_ok:
            print("[Site20] FAIL: leg1 material approach")
            mission_end = time.time()
            metrics = recorder.finalize(success=False, mission_end_t=mission_end, layout_id=registry.layout_id)
            save_metrics_json(metrics, args.artifact_dir)
            save_site_transport_artifacts(layers, registry, trace, metrics, output_dir=args.artifact_dir)
            return 3

        carry_name = begin_carry_from_material(ucv, registry, robot_name=robot_name)
        if not carry_name:
            print("[Site20] FAIL: carry start")
            mission_end = time.time()
            metrics = recorder.finalize(success=False, mission_end_t=mission_end, layout_id=registry.layout_id)
            save_metrics_json(metrics, args.artifact_dir)
            save_site_transport_artifacts(layers, registry, trace, metrics, output_dir=args.artifact_dir)
            return 4
        print(f"[Site20] carry: {carry_name}")
        tick_settle(ucv, settle_s=2.0, ticks=3)
        robot_xy = get_pos2d(ucv, robot_name)
        print(
            f"[Site20] leg2 start local={lc.world_xy_to_local(*robot_xy)} "
            f"goal={human_local}"
        )
        if l2_mode == "camera" and _reset_l2_perceive_counter is not None:
            _reset_l2_perceive_counter("leg2")

        if l2_mode == "sight":
            # Preserve static obstacle map from leg1; only reset dynamic tracking.
            _reset_sight_memory("leg2", keep_static=True)
        else:
            layers.reset_l2()
        leg2_ok = navigate_layered_with_fusion(
            ucv,
            layers,
            human_local,
            perceive_fn=_perceive,
            soft_reset_fn=_soft_reset_l2 if l2_mode == "sight" else None,
            robot_name=robot_name,
            nav_actor=nav_actor,
            tolerance_cm=ARRIVE_TOLERANCE_CM,
            label="to-humanoid",
            perception_interval_s=SITE_DEFAULT_PERCEPTION_INTERVAL_S,
            max_total_steps=args.max_nav_steps,
            trace=trace,
            carry_sync_name=carry_name,
            on_pose_sample=_on_pose,
        )
        if not leg2_ok:
            print("[Site20] FAIL: leg2 humanoid approach")
            mission_end = time.time()
            metrics = recorder.finalize(success=False, mission_end_t=mission_end, layout_id=registry.layout_id)
            save_metrics_json(metrics, args.artifact_dir)
            save_site_transport_artifacts(layers, registry, trace, metrics, output_dir=args.artifact_dir)
            return 5

        delivered = deliver_carry_at_humanoid(ucv, registry, robot_name=robot_name)
        pos_xy = get_pos2d(ucv, robot_name)
        human_xy = lc.local_xy_to_world(*human_local)
        human_dist = dist2d(pos_xy, human_xy)
        print(f"[Site20] delivered={delivered} human_dist={human_dist:.1f}cm")
        success = delivered and human_dist <= ARRIVE_TOLERANCE_CM * 2.0

        if l2_mode == "camera":
            try:
                restore_editor_viewmode_lit(ucv)
            except (ConnectionError, OSError, ValueError, RuntimeError):
                pass
        mission_end = time.time()
        metrics = recorder.finalize(
            success=success,
            mission_end_t=mission_end,
            layout_id=registry.layout_id,
        )
        metrics_path = save_metrics_json(metrics, args.artifact_dir)
        artifact_paths = save_site_transport_artifacts(
            layers, registry, trace, metrics, output_dir=args.artifact_dir
        )
        print(f"[Site20] metrics: {metrics_path}")
        for key, path in sorted(artifact_paths.items()):
            print(f"[Site20] artifact {key}: {path}")
        if not success:
            print("[Site20] FAIL")
            return 6
        print("[Site20] PASS")
        return 0
    except PieSessionLost as exc:
        print(f"[Site20] ABORT: {exc}")
        return 10
    except (ValueError, RuntimeError) as exc:
        print(f"[Site20] ABORT: {exc}")
        return 11
    finally:
        try:
            geh.release_connection(ucv)
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
