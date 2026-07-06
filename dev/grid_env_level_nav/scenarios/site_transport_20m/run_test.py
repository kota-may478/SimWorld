#!/usr/bin/env python3
"""E2E: 20 m site transport — spawn → L0+L1+L2 nav → carry → deliver to humanoid."""

from __future__ import annotations

import os

os.environ["MPLBACKEND"] = "Agg"

import argparse
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

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
import nav_move as nm  # noqa: E402
from carry import (  # noqa: E402
    begin_carry_from_material,
    deliver_carry_at_humanoid,
    reset_carry_attach_state,
    reset_carry_from_previous_mission,
)
from grid_env_10k_pie_patrol import dist2d, get_pos2d, get_yaw  # noqa: E402
from l0_crop import crop_l0_to_local_region  # noqa: E402
from depth_object_perception import depth_npy_to_meters, depth_npy_unit_hint, fetch_depth_npy  # noqa: E402
from l2_fusion import estimate_world_xy_from_detection  # noqa: E402
from l2_geom import GeomPerceptionConfig, geom_detections  # noqa: E402
from layered_nav import (  # noqa: E402
    PerceiveOutcome,
    deliver_to,
    navigate_to_slot,
)
from nav_stack.nav_context import build_nav_context  # noqa: E402
from nav_stack.perception_server import PerceptionServer, SightPerceptionDeps  # noqa: E402
from object_registry import (  # noqa: E402
    ObjectRegistry,
    RegistryUpdateResult,
    SightConfig,
    estimate_local_xy_from_detection,
    update_object_registry_from_sight,
)
from l2_depth import DepthCellTracker, DepthUpdateResult, soft_l2_depth_reset, update_l2_depth  # noqa: E402
from depth_frame_cache import DepthFrameCache  # noqa: E402
from depth_nav_iter import DepthNavIterBudget  # noqa: E402
from metrics import (  # noqa: E402
    MissionRecorder,
    NavTimingAccumulator,
    build_timing_summary,
    save_metrics_json,
    save_timing_json,
)
from site_transport_config import apply_profile_to_layered_nav, resolve_profile  # noqa: E402
from nav_stack.mission_bt import MissionRunner  # noqa: E402
from paths import L0_MASK_STRICT  # noqa: E402
from pie_safety import PieSessionLost, require_live_ucv, tick_settle  # noqa: E402
from perception_layer import (  # noqa: E402
    EgocentricPerceptionConfig,
    min_forward_depth_m,
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
    uses_engine_follow_camera,
)
from pie_spawn_safety import ensure_live_or_reconnect  # noqa: E402
from runtime_sight_sources import ensure_runtime_site20_sight_sources  # noqa: E402
from spawn_pie import spawn_site_transport_scene  # noqa: E402
from navmesh_mission_nav import deliver_to_navmesh, navigate_to_slot_navmesh
from navmesh_config import SPOTDOG_BODY_RADIUS_CM
from navmesh_obstacles import fetch_actor_bounds, setup_static_navmesh_obstacles
from surface_distance import build_surface_obstacles_from_bounds, nearest_surface_distance_cm
from viz import DEFAULT_ARTIFACT_DIR, NavTrace, save_site_transport_artifacts  # noqa: E402
from zones import apply_forbidden_zones_l1  # noqa: E402

DEFAULT_L0 = L0_MASK_STRICT
ARRIVE_TOLERANCE_CM = 130.0
DEPTH_CLEARANCE_TRIGGER_CM = 100.0
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
        "--force-respawn",
        action="store_true",
        help="Destroy and re-spawn scene props (without rebuilding registry JSON)",
    )
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
    p.add_argument(
        "--profile",
        choices=("default", "careful", "fast"),
        default="default",
        help="Navigation profile: default/careful (conservative) or fast (quick-wins)",
    )
    p.add_argument(
        "--layout-id",
        default="layout_01",
        help="Site layout variant (layout_01 .. layout_10)",
    )
    p.add_argument("--artifact-dir", type=Path, default=DEFAULT_ARTIFACT_DIR)
    p.add_argument(
        "--run-label",
        default=None,
        help="Stable artifact label (e.g. L0andL2withSLAM); use with --trial-index",
    )
    p.add_argument(
        "--trial-index",
        type=int,
        default=None,
        help="Trial number for labeled artifacts (e.g. 1..5); requires --run-label",
    )
    p.add_argument(
        "--nav-mode",
        choices=("costmap", "navmesh"),
        default="costmap",
        help="costmap: L0+L1+L2 A* (default); navmesh: Dynamic NavMesh NavFindPath (no L2)",
    )
    p.add_argument(
        "--nav-exec",
        choices=("vbp", "moveto"),
        default="vbp",
        help="navmesh only: vbp=open-loop Move_Speed (default); moveto=UE SpotDogNavController",
    )
    p.add_argument(
        "--artifact-suffix",
        default=None,
        help="Fixed artifact suffix (e.g. layout_01_test) for costMap/timing/metrics files",
    )
    return p.parse_args()


def _material_goal_xy(registry) -> tuple[float, float]:
    transport = registry.transport_slot()
    if transport is not None and transport.world_xyz_cm is not None:
        return transport.world_xyz_cm[0], transport.world_xyz_cm[1]
    return lc.local_xy_to_world(*registry.material_pickup_local_cm)


def _humanoid_goal_local(registry) -> tuple[float, float]:
    return registry.humanoid_local_cm


def _artifact_kwargs(args: argparse.Namespace) -> dict[str, Any]:
    if args.artifact_suffix:
        return {"artifact_suffix": args.artifact_suffix}
    if (args.run_label is None) != (args.trial_index is None):
        raise ValueError("--run-label and --trial-index must be used together")
    if args.run_label is None:
        return {}
    return {"run_label": args.run_label, "trial_index": args.trial_index}


def main() -> int:
    args = _parse_args()
    try:
        artifact_kw = _artifact_kwargs(args)
    except ValueError as exc:
        print(f"[Site20] {exc}")
        return 1
    nav_mode = args.nav_mode
    nav_exec = args.nav_exec
    if nav_mode != "navmesh" and nav_exec != "vbp":
        print("[Site20] --nav-exec moveto requires --nav-mode navmesh")
        return 1
    if nav_mode == "navmesh":
        nav_profile = resolve_profile("navmesh")
        apply_profile_to_layered_nav(nav_profile)
        l2_mode = "off"
        print(
            f"[Site20] nav-mode=navmesh nav-exec={nav_exec} profile={nav_profile.name} "
            f"(NavFindPath, L2 off, surface-distance metrics)"
        )
    else:
        try:
            nav_profile = resolve_profile(args.profile)
        except ValueError as exc:
            print(f"[Site20] {exc}")
            return 1
        apply_profile_to_layered_nav(nav_profile)
        l2_mode = "off" if args.no_l2 else args.l2_mode
    print(
        f"[Site20] profile={nav_profile.name} perception_interval={nav_profile.perception_interval_s}s "
        f"standoff={nav_profile.perception_standoff_cm:.0f}cm"
    )
    if not args.l0.is_file():
        print(f"[Site20] missing L0: {args.l0}")
        return 1

    registry = ensure_registry(
        layout_id=args.layout_id,
        force_rebuild=args.force_rebuild_registry,
    )
    print(f"[Site20] layout={registry.layout_id} transport={registry.transport_slot().bp_name if registry.transport_slot() else '?'}")
    layers = crop_l0_to_local_region(args.l0, size_x_cm=REGION_SIZE_CM, size_y_cm=REGION_SIZE_CM)
    if args.no_l1:
        n_l1 = 0
        print(f"[Site20] L1 forbidden zones DISABLED (--no-l1); props → L2 via {l2_mode}")
    elif nav_profile.enable_l1_by_default:
        n_l1 = apply_forbidden_zones_l1(layers, registry.forbidden_zones)
        print(f"[Site20] L1 forbidden cells: {n_l1} (props → L2 via {l2_mode})")
    else:
        n_l1 = 0
        print(f"[Site20] L1 skipped by profile={nav_profile.name}")

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
        if nav_mode != "navmesh":
            return 1
        print("[Site20] navmesh mode: skipping offline L0+L1 plan sanity check")
    if args.plan_only:
        return 0

    if args.spawn_only:
        rc, _ = spawn_site_transport_scene(
            layout_id=args.layout_id,
            force_rebuild=args.force_rebuild_registry,
        )
        return rc

    mission_t0 = time.time()
    reset_carry_attach_state()
    success = False
    metrics: dict = {}
    trace = NavTrace()
    ucv = None
    leg1_time_s: Optional[float] = None
    leg2_time_s: Optional[float] = None
    leg1_timing = NavTimingAccumulator(label="leg1")
    leg2_timing = NavTimingAccumulator(label="leg2")
    navmesh_setup_timing = NavTimingAccumulator(label="navmesh_setup")
    timing_summary: Optional[dict] = None

    try:
        ucv, _ = _connect_ue_with_retry()
        fresh_spawn = False
        if not args.skip_spawn:
            spawn_rc, ucv = spawn_site_transport_scene(
                layout_id=args.layout_id,
                force_rebuild=args.force_rebuild_registry,
                force_respawn=args.force_respawn or args.force_rebuild_registry,
                ucv=ucv,
                manage_connection=False,
            )
            if spawn_rc != 0:
                return spawn_rc
            fresh_spawn = True
            registry = ensure_registry(layout_id=args.layout_id)
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

        bounds_cache: Dict[str, Any] = {}
        surface_obstacles = ()
        if nav_mode == "navmesh":
            bounds_cache, navmesh_ready = setup_static_navmesh_obstacles(
                ucv, nav_actor, registry, nav_timing=navmesh_setup_timing
            )
            if not navmesh_ready:
                print(
                    "[Site20] FAIL: NavMesh runtime API unavailable — "
                    "rebuild UE with ue_native/NavQueryService (see NAVMESH_UE_SETUP.md)"
                )
                return 2
            if nav_exec == "moveto" and not nm.nav_move_api_available(ucv, robot_name):
                print(
                    "[Site20] FAIL: NavMove API unavailable — "
                    "complete NAVMESH_PHASE5_UE_SETUP.md Steps 1–3 and PIE Play"
                )
                return 2
            human_bounds = fetch_actor_bounds(
                ucv,
                nav_actor,
                registry.humanoid_actor_name,
                nav_timing=navmesh_setup_timing,
            )
            if human_bounds is not None:
                bounds_cache[registry.humanoid_actor_name] = human_bounds
            surface_obstacles = build_surface_obstacles_from_bounds(bounds_cache)

        if l2_mode == "off":
            print("[Site20] L2 perception disabled (--l2-mode off)")

        def _perceive_disabled():
            return []

        _perceive = _perceive_disabled
        _reset_l2_perceive_counter = None
        object_registry = ObjectRegistry()
        nav_ctx = build_nav_context(
            ucv=ucv,
            layers=layers,
            profile=nav_profile,
            trace=trace,
            object_registry=object_registry,
            robot_name=robot_name,
        )
        depth_tracker = DepthCellTracker()

        def _registry_obstacle_positions(*, exclude_slot: Optional[str] = None):
            return [
                entry.last_world_xy
                for slot_id, entry in object_registry.entries.items()
                if slot_id != exclude_slot and not entry.is_dynamic
            ]
        sight_cfg = SightConfig(
            fov_deg=SENSOR_FOV_DEG,
            max_range_cm=650.0,
            sensor_forward_cm=SENSOR_CAM_FORWARD_OFFSET_CM,
        )
        depth_cfg = EgocentricPerceptionConfig(
            fov_deg=SENSOR_FOV_DEG,
            max_range_cm=650.0,
            min_obstacle_height_cm=55.0 if nav_profile.name == "fast" else 45.0,
            stride_px=nav_profile.depth_stride_px,
            use_lethal=True,
            camera_offset_forward_cm=SENSOR_CAM_FORWARD_OFFSET_CM,
            camera_height_cm=SENSOR_CAM_HEIGHT_OFFSET_CM,
            camera_pitch_deg=SENSOR_CAM_PITCH_DEG,
            self_exclude_radius_cm=70.0,
            use_log_odds=True,
            latch_static=True,
        )
        sight_depth_cam: dict = {"ready": False, "fusion_id": None}
        depth_frame = DepthFrameCache(
            ttl_s=nav_profile.depth_cache_ttl_s,
            pose_delta_max_cm=nav_profile.depth_pose_delta_max_cm,
            move_invalidate_cm=nav_profile.depth_move_invalidate_cm,
        )
        SIGHT_REGISTRY_EVERY_N = nav_profile.sight_registry_every_n
        depth_camera_settle_s = nav_profile.depth_camera_settle_s
        perceive_cycle = {"n": 0}
        depth_perceive_ctx = {"gate_fetched": False, "last_l2_changed": True}
        depth_iter_budget = DepthNavIterBudget()
        depth_log_next_fetch = {"enabled": False}
        active_nav_timing: dict = {"acc": None}
        nav_pose_cache: Dict[str, Any] = {"xy": None, "yaw": None}
        from nav_pose_query import init_pose_cache, invalidate_robot_pose  # noqa: WPS433

        init_pose_cache(nav_pose_cache)
        depth_meta: dict = {"unit": "unknown", "min_scene_m": None}

        def _cached_nav_pose() -> tuple[float, float]:
            cached = nav_pose_cache.get("xy")
            if cached is not None:
                return cached
            return get_pos2d(ucv, robot_name)

        def _record_depth_sample(depth_raw: np.ndarray, depth_m: np.ndarray) -> Optional[float]:
            unit = depth_npy_unit_hint(depth_raw)
            depth_meta["unit"] = unit
            finite_depth = depth_m[np.isfinite(depth_m) & (depth_m > 0.05) & (depth_m < 80.0)]
            min_fwd_m = min_forward_depth_m(
                depth_m,
                fov_deg=SENSOR_FOV_DEG,
            )
            min_fwd_cm = min_fwd_m * 100.0 if min_fwd_m is not None else None
            if depth_log_next_fetch["enabled"] and finite_depth.size:
                depth_meta["min_scene_m"] = float(np.min(finite_depth))
                fwd_txt = f"forward={min_fwd_cm:.0f}cm " if min_fwd_cm is not None else ""
                print(
                    f"[Site20] depth sample unit={unit} min={float(np.min(finite_depth)):.2f}m "
                    f"{fwd_txt}"
                    f"p10={float(np.percentile(finite_depth, 10)):.2f}m "
                    f"near{DEPTH_CLEARANCE_TRIGGER_CM / 100.0:.2f}m="
                    f"{int(np.sum(finite_depth <= DEPTH_CLEARANCE_TRIGGER_CM / 100.0))}px"
                )
            return min_fwd_cm

        def _fetch_depth_raw_for_cache() -> Optional[np.ndarray]:
            nonlocal ucv
            fusion_id = _ensure_sight_depth_camera()
            acc = active_nav_timing["acc"]
            if not uses_engine_follow_camera(ucv, fusion_id):
                cam_t0 = time.perf_counter()
                update_sensor_camera_pose(ucv, robot_name, fusion_id)
                tick_settle(ucv, settle_s=depth_camera_settle_s, ticks=1)
                if acc is not None:
                    acc.camera_settle_ms += (time.perf_counter() - cam_t0) * 1000.0
            return fetch_depth_npy(ucv, fusion_id)

        def _sync_depth_timing_stats() -> None:
            acc = active_nav_timing["acc"]
            if acc is None:
                return
            acc.sync_cache_stats(
                depth_frame.hits,
                depth_frame.misses,
                async_wait_ms=depth_frame.async_wait_ms,
                prefetch_hits=depth_frame.prefetch_hits,
            )

        def _begin_depth_nav_iter() -> None:
            depth_iter_budget.begin_iter()

        def _refresh_forward_depth_cm(
            *, in_perceive: bool = False, mark_gate: bool = False, force: bool = False
        ) -> Optional[float]:
            nonlocal ucv
            pose = _cached_nav_pose()
            ttl = depth_frame.ttl_s
            fresh = depth_frame.try_get_fresh_forward_cm(pose, max_age_s=ttl)
            if fresh is not None:
                if mark_gate:
                    depth_perceive_ctx["gate_fetched"] = True
                return fresh
            if depth_iter_budget.can_reuse_in_iter(depth_frame) and not force:
                reused = depth_frame.reuse_cached_forward_cm()
                if mark_gate and reused is not None:
                    depth_perceive_ctx["gate_fetched"] = True
                return reused
            if not depth_iter_budget.should_fetch_ue(
                depth_frame, pose, max_age_s=ttl, force=force
            ):
                reused = depth_frame.reuse_cached_forward_cm()
                if mark_gate and reused is not None:
                    depth_perceive_ctx["gate_fetched"] = True
                return reused

            t0 = time.perf_counter()
            prev_misses = depth_frame.misses
            prev_async_wait = depth_frame.async_wait_ms
            depth_log_next_fetch["enabled"] = True
            try:
                result = depth_frame.get_or_wait(
                    pose,
                    _fetch_depth_raw_for_cache,
                    _record_depth_sample,
                    max_wait_s=0.15,
                    force=True,
                    max_age_s=ttl,
                )
            finally:
                depth_log_next_fetch["enabled"] = False
            depth_iter_budget.note_ue_fetch()
            acc = active_nav_timing["acc"]
            if acc is not None:
                elapsed = (time.perf_counter() - t0) * 1000.0
                if depth_frame.misses > prev_misses:
                    async_delta = depth_frame.async_wait_ms - prev_async_wait
                    sync_elapsed = max(0.0, elapsed - async_delta)
                    if in_perceive:
                        acc.depth_fetch_ms += sync_elapsed
                    else:
                        acc.depth_refresh_ms += sync_elapsed
            if mark_gate:
                depth_perceive_ctx["gate_fetched"] = True
            _sync_depth_timing_stats()
            return result

        def _perceive_gate_depth_refresh() -> Optional[float]:
            return _refresh_forward_depth_cm(in_perceive=True, mark_gate=True)

        def _get_robot_pose_cached() -> tuple[tuple[float, float], float]:
            xy = nav_pose_cache.get("xy")
            yaw = nav_pose_cache.get("yaw")
            if xy is not None and yaw is not None:
                return (xy, float(yaw))
            pose_t0 = time.perf_counter()
            xy = get_pos2d(ucv, robot_name)
            yaw = get_yaw(ucv, robot_name)
            nav_pose_cache["xy"] = xy
            nav_pose_cache["yaw"] = yaw
            acc = active_nav_timing["acc"]
            if acc is not None:
                acc.perceive_pose_ms += (time.perf_counter() - pose_t0) * 1000.0
            return (xy, yaw)

        def _consume_gate_depth_fetch() -> bool:
            fetched = bool(depth_perceive_ctx["gate_fetched"])
            depth_perceive_ctx["gate_fetched"] = False
            return fetched

        def _should_skip_depth(_cycle: int, reg_result) -> bool:
            if reg_result.entries_added or reg_result.entries_evicted:
                return False
            if not depth_frame.is_fresh(_cached_nav_pose(), max_age_s=depth_frame.ttl_s):
                return False
            return not depth_perceive_ctx["last_l2_changed"]

        def _cached_forward_depth_cm() -> Optional[float]:
            return depth_frame.min_fwd_cm

        def _depth_invalidate_fn(reason: str) -> None:
            depth_frame.invalidate(reason)
            depth_iter_budget.on_invalidate()

        def _on_move_cm_fn(move_cm: float) -> None:
            depth_frame.note_move_cm(move_cm)

        def _depth_prefetch_fn(*, perceive_due: bool = False) -> None:
            nonlocal ucv
            if l2_mode != "sight" or not perceive_due:
                return
            pose = _cached_nav_pose()
            if depth_frame.try_get_fresh_forward_cm(pose, max_age_s=depth_frame.ttl_s) is not None:
                return
            if depth_iter_budget.can_reuse_in_iter(depth_frame):
                return
            t0 = time.perf_counter()
            prev_misses = depth_frame.misses
            depth_log_next_fetch["enabled"] = True
            try:
                depth_frame.prefetch_async(
                    pose, _fetch_depth_raw_for_cache, _record_depth_sample
                )
            finally:
                depth_log_next_fetch["enabled"] = False
            if depth_frame.misses > prev_misses:
                depth_iter_budget.note_ue_fetch()
            acc = active_nav_timing["acc"]
            if acc is not None:
                acc.prefetch_hit_ms += (time.perf_counter() - t0) * 1000.0
            _sync_depth_timing_stats()

        def _reset_depth_state(label: str, *, carry_forward: bool = False) -> None:
            from nav_pose_query import init_pose_cache, invalidate_robot_pose  # noqa: WPS433

            depth_frame.invalidate(label)
            perceive_cycle["n"] = 0
            depth_perceive_ctx["gate_fetched"] = False
            depth_perceive_ctx["last_l2_changed"] = True
            depth_iter_budget.begin_iter()
            init_pose_cache(nav_pose_cache)
            if carry_forward:
                depth_tracker.snapshot_occupied(layers)
                object_registry.clear_dynamic()
                print(
                    f"[Site20] L2 depth carry-forward mask={len(depth_tracker.carry_forward_mask)} "
                    f"({label})"
                )
            else:
                depth_tracker.active_cells.clear()
                depth_tracker.clear_carry_forward()
                object_registry.entries.clear()
                print(f"[Site20] L2 depth + registry reset ({label})")

        def _soft_reset_l2(l2_seen_cells: set, stuck_world_xy=None, *, aggressive: bool = False) -> None:
            removed = soft_l2_depth_reset(
                layers,
                depth_tracker,
                l2_seen_cells,
                stuck_world_xy=stuck_world_xy,
                aggressive=aggressive,
            )
            if removed:
                print(
                    f"[Site20] L2 soft reset evicted {removed} cells"
                    f"{' (aggressive)' if aggressive else ''}"
                )

        nav_ctx.soft_reset_fn = _soft_reset_l2 if l2_mode == "sight" else None

        def _ensure_sight_depth_camera() -> int:
            if not sight_depth_cam["ready"]:
                fusion_id = resolve_sensor_camera_id(ucv)
                configure_sensor_camera(ucv, fusion_id)
                sight_depth_cam["fusion_id"] = fusion_id
                sight_depth_cam["ready"] = True
                print(f"[Site20] L2_depth camera ready fusion={fusion_id}")
            return int(sight_depth_cam["fusion_id"])

        def _apply_l2_depth(
            l2_seen_cells: set,
            *,
            robot_xy: Optional[tuple[float, float]] = None,
            robot_yaw: Optional[float] = None,
            skip_depth_fetch: bool = False,
        ) -> DepthUpdateResult:
            nonlocal ucv
            if robot_xy is None:
                robot_xy = get_pos2d(ucv, robot_name)
            standoff_cm = nav_profile.perception_standoff_cm
            forward_depth_cm = depth_frame.min_fwd_cm
            if standoff_cm > 0.0:
                from perception_standoff import (  # noqa: WPS433
                    check_perception_standoff,
                    depth_confirms_clearance,
                    depth_shows_obstacle,
                    evict_stale_l2_in_forward_cone,
                )

                if depth_confirms_clearance(forward_depth_cm, standoff_cm):
                    if robot_yaw is None:
                        robot_yaw = get_yaw(ucv, robot_name)
                    removed = evict_stale_l2_in_forward_cone(
                        robot_xy,
                        robot_yaw,
                        layers,
                        forward_depth_cm=float(forward_depth_cm),  # type: ignore[arg-type]
                        standoff_cm=standoff_cm,
                        l2_seen_cells=l2_seen_cells,
                        registry_positions=_registry_obstacle_positions(),
                        cone_half_deg=nav_profile.standoff_evict_cone_half_deg,
                        depth_margin_cm=nav_profile.standoff_evict_depth_margin_cm,
                    )
                standoff = check_perception_standoff(
                    robot_xy,
                    layers,
                    registry_positions=_registry_obstacle_positions(),
                    standoff_cm=standoff_cm,
                    forward_depth_cm=forward_depth_cm,
                )
                depth_obstacle = depth_shows_obstacle(forward_depth_cm, standoff_cm)
                if standoff.needs_backoff(standoff_cm) and not depth_obstacle:
                    print(
                        f"[Site20] L2_depth gated: {standoff.nearest_dist_cm:.0f}cm "
                        f"< {standoff_cm:.0f}cm ({standoff.source})"
                    )
                    return DepthUpdateResult(0, 0, 0, 0)
            if depth_frame.get_depth_m() is None:
                if not skip_depth_fetch:
                    _refresh_forward_depth_cm(in_perceive=True)
            elif not skip_depth_fetch:
                pose = _cached_nav_pose()
                if not depth_frame.is_fresh(pose, max_age_s=depth_frame.ttl_s):
                    _refresh_forward_depth_cm(in_perceive=True)
            depth_m = depth_frame.get_depth_m()
            if depth_m is None:
                print("[Site20] L2_depth fetch skipped: no depth")
                return DepthUpdateResult(0, 0, 0, 0)
            l2_t0 = time.perf_counter()
            if robot_yaw is None:
                robot_yaw = get_yaw(ucv, robot_name)
            result = update_l2_depth(
                depth_m,
                layers,
                robot_xy=robot_xy,
                robot_yaw_deg=robot_yaw,
                config=depth_cfg,
                tracker=depth_tracker,
                camera_pitch_deg=SENSOR_CAM_PITCH_DEG,
                close_range_clearance_cm=DEPTH_CLEARANCE_TRIGGER_CM,
                close_range_keepout_cm=DEPTH_KEEP_OUT_RADIUS_CM,
            )
            acc = active_nav_timing["acc"]
            if acc is not None:
                acc.l2_update_ms += (time.perf_counter() - l2_t0) * 1000.0
            for cell in depth_tracker.active_cells:
                l2_seen_cells.add(cell)
            if result.l2_changed:
                print(
                    f"[Site20] L2_depth: +{result.hit_cells} hits "
                    f"-{result.cleared_cells} cleared +{result.keepout_cells} keepout"
                )
            depth_perceive_ctx["last_l2_changed"] = bool(result.l2_changed)
            return result

        if l2_mode == "sight":
            placement_reg = to_placement_registry(registry)
            placement_reg_holder["reg"] = placement_reg
            for prop in placement_reg.props:
                from l2_geom import _prop_world_xy  # noqa: WPS433

                object_registry.upsert(
                    slot_id=prop.slot_id,
                    prop_type_id=prop.prop_type_id,
                    world_xy=_prop_world_xy(prop),
                    is_dynamic=False,
                )
            object_registry.upsert(
                slot_id=registry.material_actor_name,
                prop_type_id="shipping_crate",
                world_xy=_material_goal_xy(registry),
                is_dynamic=False,
            )
            object_registry.upsert(
                slot_id=registry.humanoid_actor_name,
                prop_type_id="human_worker",
                world_xy=lc.local_xy_to_world(*human_local),
                is_dynamic=True,
            )
            print(
                f"[Site20] L2_depth + ObjectRegistry: FOV={sight_cfg.fov_deg}° "
                f"range={sight_cfg.max_range_cm}cm interval={nav_profile.perception_interval_s}s "
                f"log-odds=on static-latch=2-hit"
            )

            def _update_sight_registry():
                return update_object_registry_from_sight(
                    ucv,
                    object_registry,
                    robot_name=robot_name,
                    placement_reg=placement_reg_holder["reg"],
                    humanoid_actor_name=registry.humanoid_actor_name,
                    material_actor_name=registry.material_actor_name,
                    config=sight_cfg,
                )

            def _record_sight_detection(det) -> None:
                robot_xy = nav_pose_cache.get("xy")
                robot_yaw = nav_pose_cache.get("yaw")
                if robot_xy is None or robot_yaw is None:
                    robot_xy = get_pos2d(ucv, robot_name)
                    robot_yaw = get_yaw(ucv, robot_name)
                local_xy = estimate_local_xy_from_detection(
                    robot_xy,
                    robot_yaw,
                    det,
                    sensor_forward_cm=sight_cfg.sensor_forward_cm,
                )
                trace.record_l2_estimate(local_xy)

            def _on_perceive_pose_timing(ms: float) -> None:
                acc = active_nav_timing["acc"]
                if acc is not None:
                    acc.perceive_pose_ms += ms

            def _on_sight_registry_timing(ms: float) -> None:
                acc = active_nav_timing["acc"]
                if acc is not None:
                    acc.sight_registry_ms += ms

            perception_server = PerceptionServer(
                SightPerceptionDeps(
                    get_robot_pose=_get_robot_pose_cached,
                    apply_l2_depth=_apply_l2_depth,
                    update_registry=_update_sight_registry,
                    should_run_registry=lambda n: (
                        SIGHT_REGISTRY_EVERY_N <= 1 or n % SIGHT_REGISTRY_EVERY_N == 1
                    ),
                    record_detection_local=_record_sight_detection,
                    should_skip_depth=_should_skip_depth,
                    consume_gate_depth_fetch=_consume_gate_depth_fetch,
                    on_timing_pose_ms=_on_perceive_pose_timing,
                    on_timing_registry_ms=_on_sight_registry_timing,
                )
            )

            def _perceive_sight(*, layers, l2_seen_cells):
                outcome = perception_server.perceive(layers=layers, l2_seen_cells=l2_seen_cells)
                if outcome.detections or outcome.l2_changed:
                    print(
                        f"[Site20] perceive (sight): detections={len(outcome.detections)} "
                        f"depth+{outcome.cells_added}/-{outcome.cells_removed} cells"
                    )
                return PerceiveOutcome(
                    detections=outcome.detections,
                    cells_added=outcome.cells_added,
                    cells_removed=outcome.cells_removed,
                    l2_applied=outcome.l2_applied,
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
                f"range={geom_cfg.max_range_cm}cm interval={nav_profile.perception_interval_s}s "
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
                stride_px=nav_profile.depth_stride_px,
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
            if nav_mode == "navmesh":
                surface_dist_cm, _ = nearest_surface_distance_cm(
                    pos_xy, surface_obstacles
                )
                body_edge_dist_cm = (
                    surface_dist_cm - SPOTDOG_BODY_RADIUS_CM
                    if surface_dist_cm is not None
                    else None
                )
                recorder.record_pose(
                    pos_xy,
                    now=now_t,
                    surface_dist_cm=surface_dist_cm,
                    proximity_dist_cm=surface_dist_cm,
                    body_edge_dist_cm=body_edge_dist_cm,
                )
                return
            from perception_standoff import nearest_environment_distance_cm  # noqa: WPS433

            forward_depth_cm = depth_frame.min_fwd_cm
            proximity_dist_cm, _ = nearest_environment_distance_cm(
                pos_xy,
                layers,
                registry_positions=_registry_obstacle_positions(),
                forward_depth_cm=forward_depth_cm,
            )
            recorder.record_pose(
                pos_xy,
                now=now_t,
                proximity_dist_cm=proximity_dist_cm,
            )

        def _persist_mission_metrics(*, success: bool, mission_end: float) -> dict:
            nonlocal timing_summary, metrics
            timing_legs = [leg1_timing, leg2_timing]
            if nav_mode == "navmesh":
                timing_legs = [navmesh_setup_timing, leg1_timing, leg2_timing]
            timing_summary = build_timing_summary(
                legs=timing_legs,
                leg1_time_s=leg1_time_s,
                leg2_time_s=leg2_time_s,
                profile=nav_profile.name,
            )
            metrics = recorder.finalize(
                success=success,
                mission_end_t=mission_end,
                layout_id=registry.layout_id,
                leg1_time_s=leg1_time_s,
                leg2_time_s=leg2_time_s,
                timing_summary=timing_summary,
                profile=nav_profile.name,
                nav_kpi=nav_ctx.kpi.to_dict() if nav_ctx.kpi is not None else None,
            )
            metrics_path = save_metrics_json(metrics, args.artifact_dir, **artifact_kw)
            timing_path = save_timing_json(timing_summary, args.artifact_dir, **artifact_kw)
            print(f"[Site20] metrics: {metrics_path}")
            print(f"[Site20] timing: {timing_path}")
            print(
                f"[Site20] timing_summary total_ms={timing_summary['totals']['total_ms']:.0f} "
                f"leg1_s={timing_summary.get('leg1_time_s')} leg2_s={timing_summary.get('leg2_time_s')}"
            )
            if nav_mode == "navmesh":
                totals = timing_summary["totals"]
                print(
                    "[Site20] navmesh_timing_ms "
                    f"rebuild={totals.get('nav_rebuild_ms', 0):.0f} "
                    f"find_path={totals.get('nav_find_path_ms', 0):.0f} "
                    f"project={totals.get('nav_project_ms', 0):.0f} "
                    f"bounds={totals.get('nav_bounds_ms', 0):.0f} "
                    f"register={totals.get('nav_register_ms', 0):.0f} "
                    f"move={totals.get('move_ms', 0):.0f} "
                    f"pose={totals.get('pose_query_ms', 0):.0f} "
                    f"settle={totals.get('settle_ms', 0):.0f} "
                    f"loop_oh={totals.get('loop_overhead_ms', 0):.0f} "
                    f"accounted={totals.get('accounted_ms', 0):.0f} "
                    f"residual={totals.get('residual_ms', 0):.0f}"
                )
                print(
                    "[Site20] navmesh_timing_counts "
                    f"rebuild={totals.get('nav_rebuild_count', 0)} "
                    f"find_path={totals.get('nav_find_path_count', 0)} "
                    f"project={totals.get('nav_project_count', 0)} "
                    f"stuck_replan={totals.get('stuck_replan_count', 0)} "
                    f"humanoid_replan={totals.get('humanoid_replan_count', 0)} "
                    f"loop_iter={totals.get('nav_loop_iterations', 0)} "
                    f"pose_cache_hits={totals.get('pose_cache_hits', 0)}"
                )
            return metrics

        material_xy = _material_goal_xy(registry)
        robot_xy = get_pos2d(ucv, robot_name)
        print(f"[Site20] leg1 material @ {material_xy}")
        tick_settle(ucv, settle_s=nav_profile.pre_leg1_settle_s, ticks=4)
        ucv = ensure_live_or_reconnect(ucv, reason="pre leg1 settle")

        mission_state: Dict[str, Any] = {
            "carry_name": None,
            "leg1_time_s": 0.0,
            "leg2_time_s": 0.0,
            "fail_exit": 0,
        }

        def _run_leg1() -> bool:
            nonlocal ucv, leg1_time_s
            layers.reset_l2()
            if l2_mode == "sight":
                _reset_depth_state("leg1")
            invalidate_robot_pose(nav_pose_cache, reason="leg1_start")
            active_nav_timing["acc"] = leg1_timing
            leg1_t0 = time.time()
            if nav_mode == "navmesh":
                leg1_ok = navigate_to_slot_navmesh(
                    ucv,
                    registry.material_actor_name,
                    object_registry=object_registry,
                    nav_actor=nav_actor,
                    robot_name=robot_name,
                    profile=nav_profile,
                    fallback_goal_local=material_local,
                    tolerance_cm=ARRIVE_TOLERANCE_CM,
                    label="to-material",
                    perception_interval_s=nav_profile.perception_interval_s,
                    max_total_steps=args.max_nav_steps,
                    trace=trace,
                    on_pose_sample=_on_pose,
                    nav_timing=leg1_timing,
                    pose_cache=nav_pose_cache,
                    nav_exec=nav_exec,
                    path_obstacles=surface_obstacles,
                )
            else:
                leg1_ok = navigate_to_slot(
                    ucv,
                    layers,
                    registry.material_actor_name,
                    object_registry=object_registry,
                    perceive_fn=_perceive,
                    soft_reset_fn=_soft_reset_l2 if l2_mode == "sight" else None,
                    fallback_goal_local=material_local,
                    robot_name=robot_name,
                    nav_actor=nav_actor,
                    tolerance_cm=ARRIVE_TOLERANCE_CM,
                    label="to-material",
                    perception_interval_s=nav_profile.perception_interval_s,
                    max_total_steps=args.max_nav_steps,
                    trace=trace,
                    on_pose_sample=_on_pose,
                    nav_timing=leg1_timing,
                    extra_obstacle_positions_fn=lambda: _registry_obstacle_positions(
                        exclude_slot=registry.material_actor_name
                    ),
                    forward_depth_cm_fn=_cached_forward_depth_cm,
                    depth_refresh_fn=(
                        (lambda: _refresh_forward_depth_cm(in_perceive=False))
                        if l2_mode == "sight"
                        else None
                    ),
                    depth_invalidate_fn=_depth_invalidate_fn if l2_mode == "sight" else None,
                    on_move_cm_fn=_on_move_cm_fn if l2_mode == "sight" else None,
                    depth_prefetch_fn=_depth_prefetch_fn if l2_mode == "sight" else None,
                    on_nav_iter_start_fn=_begin_depth_nav_iter if l2_mode == "sight" else None,
                    pose_cache=nav_pose_cache,
                    nav_ctx=nav_ctx,
                    perceive_depth_refresh_fn=(
                        _perceive_gate_depth_refresh if l2_mode == "sight" else None
                    ),
                )
            mission_state["leg1_time_s"] = time.time() - leg1_t0
            leg1_time_s = mission_state["leg1_time_s"]
            _sync_depth_timing_stats()
            print(f"[Site20] leg1_time_s={mission_state['leg1_time_s']:.1f}")
            if not leg1_ok:
                print("[Site20] FAIL: leg1 material approach")
                mission_state["fail_exit"] = 3
            return leg1_ok

        def _run_carry() -> bool:
            nonlocal ucv
            carry_name = begin_carry_from_material(ucv, registry, robot_name=robot_name)
            mission_state["carry_name"] = carry_name
            if not carry_name:
                print("[Site20] FAIL: carry start")
                mission_state["fail_exit"] = 4
                return False
            print(f"[Site20] carry: {carry_name}")
            tick_settle(ucv, settle_s=2.0, ticks=3)
            robot_xy_local = get_pos2d(ucv, robot_name)
            print(
                f"[Site20] leg2 start local={lc.world_xy_to_local(*robot_xy_local)} "
                f"goal={human_local}"
            )
            return True

        def _run_leg2() -> bool:
            nonlocal ucv, leg2_time_s
            if l2_mode == "camera" and _reset_l2_perceive_counter is not None:
                _reset_l2_perceive_counter("leg2")
            if l2_mode == "sight":
                _reset_depth_state("leg2", carry_forward=True)
            else:
                layers.reset_l2()
            invalidate_robot_pose(nav_pose_cache, reason="leg2_start")
            active_nav_timing["acc"] = leg2_timing
            leg2_t0 = time.time()
            if nav_mode == "navmesh":
                leg2_ok = deliver_to_navmesh(
                    ucv,
                    registry.humanoid_actor_name,
                    object_registry=object_registry,
                    nav_actor=nav_actor,
                    robot_name=robot_name,
                    profile=nav_profile,
                    fallback_goal_local=human_local,
                    humanoid_actor_name=registry.humanoid_actor_name,
                    tolerance_cm=ARRIVE_TOLERANCE_CM,
                    label="to-humanoid",
                    perception_interval_s=nav_profile.perception_interval_s,
                    max_total_steps=args.max_nav_steps,
                    trace=trace,
                    carry_sync_name=mission_state["carry_name"],
                    on_pose_sample=_on_pose,
                    nav_timing=leg2_timing,
                    pose_cache=nav_pose_cache,
                    nav_exec=nav_exec,
                    path_obstacles=surface_obstacles,
                )
            else:
                leg2_ok = deliver_to(
                    ucv,
                    layers,
                    registry.humanoid_actor_name,
                    object_registry=object_registry,
                    perceive_fn=_perceive,
                    soft_reset_fn=_soft_reset_l2 if l2_mode == "sight" else None,
                    fallback_goal_local=human_local,
                    robot_name=robot_name,
                    nav_actor=nav_actor,
                    tolerance_cm=ARRIVE_TOLERANCE_CM,
                    label="to-humanoid",
                    perception_interval_s=nav_profile.perception_interval_s,
                    max_total_steps=args.max_nav_steps,
                    trace=trace,
                    carry_sync_name=mission_state["carry_name"],
                    on_pose_sample=_on_pose,
                    nav_timing=leg2_timing,
                    extra_obstacle_positions_fn=lambda: _registry_obstacle_positions(
                        exclude_slot=registry.humanoid_actor_name
                    ),
                    forward_depth_cm_fn=_cached_forward_depth_cm,
                    depth_refresh_fn=(
                        (lambda: _refresh_forward_depth_cm(in_perceive=False))
                        if l2_mode == "sight"
                        else None
                    ),
                    depth_invalidate_fn=_depth_invalidate_fn if l2_mode == "sight" else None,
                    on_move_cm_fn=_on_move_cm_fn if l2_mode == "sight" else None,
                    depth_prefetch_fn=_depth_prefetch_fn if l2_mode == "sight" else None,
                    on_nav_iter_start_fn=_begin_depth_nav_iter if l2_mode == "sight" else None,
                    pose_cache=nav_pose_cache,
                    nav_ctx=nav_ctx,
                    perceive_depth_refresh_fn=(
                        _perceive_gate_depth_refresh if l2_mode == "sight" else None
                    ),
                )
            mission_state["leg2_time_s"] = time.time() - leg2_t0
            leg2_time_s = mission_state["leg2_time_s"]
            _sync_depth_timing_stats()
            print(f"[Site20] leg2_time_s={mission_state['leg2_time_s']:.1f}")
            if not leg2_ok:
                print("[Site20] FAIL: leg2 humanoid approach")
                mission_state["fail_exit"] = 5
            return leg2_ok

        mission_runner = MissionRunner(
            leg1_fn=_run_leg1,
            carry_fn=_run_carry,
            leg2_fn=_run_leg2,
        )
        print("[Site20] mission_bt: Leg1 → carry → Leg2")
        if not mission_runner.run_to_completion():
            mission_end = time.time()
            _persist_mission_metrics(success=False, mission_end=mission_end)
            save_site_transport_artifacts(
                layers, registry, trace, metrics, output_dir=args.artifact_dir, **artifact_kw
            )
            return int(mission_state["fail_exit"] or 6)

        carry_name = mission_state["carry_name"]
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
        _persist_mission_metrics(success=success, mission_end=mission_end)
        artifact_paths = save_site_transport_artifacts(
            layers, registry, trace, metrics, output_dir=args.artifact_dir, **artifact_kw
        )
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
