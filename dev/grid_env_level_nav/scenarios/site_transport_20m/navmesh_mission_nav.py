#!/usr/bin/env python3
"""NavFindPath-based mission navigation for site_transport_20m (Pattern A)."""

from __future__ import annotations

import math
import time
from typing import Callable, List, Optional, Sequence, Tuple

import nav_query as nq
import nav_move as nm
from carry import is_carry_ue_attached, sync_carry_pose
from grid_env_10k_pie_patrol import dist2d
from level_coords import NAV_PROJECT_PROBE_Z_CM, local_xy_to_world, world_xy_to_local
from layered_nav import _fetch_nav_pose, _site_dog_move, _site_dog_rotate
from metrics import NavTimingAccumulator
from nav_pose_query import PoseCache, invalidate_robot_pose
from navmesh_config import (
    HUMANOID_REPLAN_DELTA_CM,
    NAVMESH_GOAL_TOLERANCE_CM,
    NAVMESH_MAX_OPEN_LOOP_MOVE_CM,
    NAVMESH_MAX_TURN_DEG_PER_STEP,
    NAVMESH_REPLAN_STUCK_STEPS,
    NAVMESH_ROTATE_THRESHOLD_DEG,
    NAVMESH_STUCK_MOVE_THRESHOLD_CM,
    NAVMESH_STUCK_UNCHANGED_STEPS,
    NAVMESH_WAYPOINT_SPACING_CM,
    NAVMESH_WP_REACH_TOLERANCE_CM,
    PROXIMITY_CENTER_FROM_SURFACE_CM,
)
from navmesh_obstacles import _timed_rebuild, update_humanoid_obstacle
from pie_safety import tick_settle
from site_transport_config import NavProfile
from viz import NavTrace

WorldXY = Tuple[float, float]
WorldXYZ = Tuple[float, float, float]
PoseSampleFn = Callable[[WorldXY, float], None]
PerceiveFn = Callable[[], object]


def _normalize_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _yaw_to_target(pos_xy: WorldXY, target_xy: WorldXY) -> float:
    return math.degrees(math.atan2(target_xy[1] - pos_xy[1], target_xy[0] - pos_xy[0]))


def _nav_plan_xyz(
    ucv,
    nav_actor: str,
    xy: WorldXY,
    *,
    nav_timing: Optional[NavTimingAccumulator] = None,
    z_probe_cm: Optional[float] = None,
) -> Optional[WorldXYZ]:
    """Project XY onto NavMesh for NavFindPath. Returns None when projection fails."""
    z_candidates: list[float] = []
    if z_probe_cm is not None:
        z_candidates.append(float(z_probe_cm))
    if NAV_PROJECT_PROBE_Z_CM not in z_candidates:
        z_candidates.append(NAV_PROJECT_PROBE_Z_CM)

    t0 = time.perf_counter()
    for z_cm in z_candidates:
        raw = nq.nav_project_point(ucv, nav_actor, xy[0], xy[1], z_cm)
        if raw.get("ok"):
            if nav_timing is not None:
                nav_timing.record_elapsed("nav_project_ms", t0)
                nav_timing.nav_project_count += 1
            return (
                float(raw["x"]),
                float(raw["y"]),
                float(raw.get("z", z_cm)),
            )
    if nav_timing is not None:
        nav_timing.record_elapsed("nav_project_ms", t0)
        nav_timing.nav_project_count += len(z_candidates)
    return None


def _start_xyz_for_robot(
    ucv,
    nav_actor: str,
    pos_xy: WorldXY,
    robot_name: str,
    *,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> Optional[WorldXYZ]:
    """Project robot XY using its current Z first, then the default nav probe."""
    z_probe: Optional[float] = None
    try:
        loc = ucv.get_location(robot_name)
        z_probe = float(loc[2])
    except (ConnectionError, OSError, ValueError, RuntimeError, TypeError, IndexError):
        z_probe = None
    return _nav_plan_xyz(
        ucv,
        nav_actor,
        pos_xy,
        nav_timing=nav_timing,
        z_probe_cm=z_probe,
    )


def _densify_waypoints(
    points: Sequence[WorldXY],
    *,
    spacing_cm: float = NAVMESH_WAYPOINT_SPACING_CM,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> List[WorldXY]:
    t0 = time.perf_counter()
    if len(points) < 2:
        dense = list(points)
    else:
        dense = [points[0]]
        for target in points[1:]:
            start = dense[-1]
            seg_len = dist2d(start, target)
            if seg_len <= spacing_cm:
                dense.append(target)
                continue
            steps = max(1, int(math.ceil(seg_len / spacing_cm)))
            for step in range(1, steps + 1):
                t = step / steps
                dense.append(
                    (
                        start[0] + (target[0] - start[0]) * t,
                        start[1] + (target[1] - start[1]) * t,
                    )
                )
    if nav_timing is not None:
        nav_timing.record_elapsed("nav_densify_ms", t0)
    return dense


def plan_navmesh_waypoints(
    ucv,
    nav_actor: str,
    start_xyz: WorldXYZ,
    goal_xyz: WorldXYZ,
    *,
    agent_radius_cm: float = PROXIMITY_CENTER_FROM_SURFACE_CM,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> List[WorldXY]:
    t_find = time.perf_counter()
    raw = nq.nav_find_path(
        ucv,
        nav_actor,
        start_xyz,
        goal_xyz,
        agent_radius_cm=agent_radius_cm,
    )
    if nav_timing is not None:
        nav_timing.record_elapsed("nav_find_path_ms", t_find)
        nav_timing.nav_find_path_count += 1
    points = nq.path_points_xy(raw)
    if not points:
        print(f"[NavMeshNav] plan failed: {raw.get('error', raw)}")
        return []
    dense = _densify_waypoints(points, nav_timing=nav_timing)
    if len(dense) != len(points):
        print(
            f"[NavMeshNav] densified path {len(points)} → {len(dense)} WP "
            f"(spacing={NAVMESH_WAYPOINT_SPACING_CM:.0f}cm)"
        )
    return dense


def _segment_command(
    pos_xy: WorldXY,
    yaw_deg: float,
    waypoint_xy: WorldXY,
    *,
    max_move_cm: float,
) -> Optional[Tuple[float, int, float]]:
    distance_cm = dist2d(pos_xy, waypoint_xy)
    if distance_cm < 1e-3:
        return None
    target_yaw = _yaw_to_target(pos_xy, waypoint_xy)
    angle_diff = _normalize_angle_deg(target_yaw - yaw_deg)
    if abs(angle_diff) > NAVMESH_ROTATE_THRESHOLD_DEG:
        clockwise = 1 if angle_diff < 0.0 else -1
        return (
            min(abs(angle_diff), NAVMESH_MAX_TURN_DEG_PER_STEP),
            clockwise,
            0.0,
        )
    return (0.0, 1, min(distance_cm, max_move_cm))


def _close_loop_overhead(
    nav_timing: Optional[NavTimingAccumulator],
    loop_t0: float,
    accounted_at_start: float,
) -> None:
    if nav_timing is None:
        return
    slice_ms = (time.perf_counter() - loop_t0) * 1000.0
    bucketed_ms = nav_timing.accounted_ms() - accounted_at_start
    gap = slice_ms - bucketed_ms
    if gap > 0.0:
        nav_timing.loop_overhead_ms += gap


def _waypoints_to_path_xyz(
    ucv,
    nav_actor: str,
    waypoints: Sequence[WorldXY],
    *,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> List[WorldXYZ]:
    out: List[WorldXYZ] = []
    fallback_z = NAV_PROJECT_PROBE_Z_CM
    for xy in waypoints:
        pt = _nav_plan_xyz(ucv, nav_actor, xy, nav_timing=nav_timing)
        if pt is None:
            out.append((float(xy[0]), float(xy[1]), fallback_z))
        else:
            fallback_z = pt[2]
            out.append(pt)
    return out


def _dispatch_ue_path_follow(
    ucv,
    robot_name: str,
    path_xyz: Sequence[WorldXYZ],
    *,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> bool:
    t0 = time.perf_counter()
    raw = nm.nav_follow_path_json(ucv, robot_name, path_xyz)
    if nav_timing is not None:
        nav_timing.record_elapsed("nav_moveto_wait_ms", t0)
        nav_timing.nav_moveto_request_count += 1
    if not raw.get("ok"):
        print(f"[NavMeshNav] NavFollowPathJson failed: {raw.get('error', raw)}")
        return False
    return True


def _navigate_navmesh_leg_moveto(
    ucv,
    goal_xy: WorldXY,
    *,
    nav_actor: str,
    robot_name: str,
    profile: NavProfile,
    perceive_fn: Optional[PerceiveFn] = None,
    perception_interval_s: float = 5.0,
    tolerance_cm: float = NAVMESH_GOAL_TOLERANCE_CM,
    max_total_steps: int = 600,
    label: str = "",
    trace: Optional[NavTrace] = None,
    on_pose_sample: Optional[PoseSampleFn] = None,
    nav_timing: Optional[NavTimingAccumulator] = None,
    pose_cache: Optional[PoseCache] = None,
    humanoid_actor_name: Optional[str] = None,
    dynamic_humanoid: bool = False,
    last_humanoid_xy: Optional[WorldXY] = None,
) -> Tuple[bool, Optional[WorldXY]]:
    """Follow NavFindPath polyline via UE SpotDogNavController (Phase 5)."""
    if not nm.moveto_use_ue_controller():
        print(
            "[NavMeshNav/moveto] NAV_MOVETO_UE unset — using vbp open-loop fallback "
            "(set NAV_MOVETO_UE=1 after Step 4 UE smoke passes)"
        )
        return _navigate_navmesh_leg_vbp(
            ucv,
            goal_xy,
            nav_actor=nav_actor,
            robot_name=robot_name,
            robot_speed=profile.site_robot_speed,
            profile=profile,
            perceive_fn=perceive_fn,
            perception_interval_s=perception_interval_s,
            tolerance_cm=tolerance_cm,
            max_total_steps=max_total_steps,
            label=label or "moveto-fallback",
            trace=trace,
            on_pose_sample=on_pose_sample,
            nav_timing=nav_timing,
            pose_cache=pose_cache,
            humanoid_actor_name=humanoid_actor_name,
            dynamic_humanoid=dynamic_humanoid,
            last_humanoid_xy=last_humanoid_xy,
        )
    poll_s = 0.25
    leg_timeout_s = max(120.0, max_total_steps * 2.0)
    goal_xyz = _nav_plan_xyz(ucv, nav_actor, goal_xy, nav_timing=nav_timing)
    if goal_xyz is None:
        print(
            f"[NavMeshNav/moveto] goal projection failed @ "
            f"({goal_xy[0]:.0f},{goal_xy[1]:.0f})"
        )
        return False, last_humanoid_xy
    pos_xy, _, ucv = _fetch_nav_pose(ucv, robot_name, nav_timing, pose_cache)
    start_xyz = _start_xyz_for_robot(
        ucv, nav_actor, pos_xy, robot_name, nav_timing=nav_timing
    )
    if start_xyz is None:
        print(
            f"[NavMeshNav/moveto] start projection failed @ "
            f"({pos_xy[0]:.0f},{pos_xy[1]:.0f})"
        )
        return False, last_humanoid_xy
    waypoints = plan_navmesh_waypoints(
        ucv, nav_actor, start_xyz, goal_xyz, nav_timing=nav_timing
    )
    if not waypoints:
        return False, last_humanoid_xy

    if trace is not None:
        trace.record_plan(waypoints, reason=label or "initial")

    human_xy = last_humanoid_xy
    last_perceive_t = -1e9
    path_xyz = _waypoints_to_path_xyz(
        ucv, nav_actor, waypoints, nav_timing=nav_timing
    )
    if not _dispatch_ue_path_follow(
        ucv, robot_name, path_xyz, nav_timing=nav_timing
    ):
        return False, human_xy

    print(
        f"[NavMeshNav/moveto] {label}: {len(waypoints)} WP "
        f"goal=({goal_xy[0]:.0f},{goal_xy[1]:.0f})"
    )

    deadline = time.perf_counter() + leg_timeout_s
    last_pose_t = -1e9
    while time.perf_counter() < deadline:
        loop_t0 = time.perf_counter()
        if nav_timing is not None:
            nav_timing.nav_loop_iterations += 1

        now = time.perf_counter()
        if perceive_fn is not None and now - last_perceive_t >= perception_interval_s:
            t_perceive = time.perf_counter()
            perceive_fn()
            if nav_timing is not None:
                nav_timing.record_elapsed("perceive_ms", t_perceive)
            last_perceive_t = now

        if dynamic_humanoid and humanoid_actor_name:
            try:
                t_loc = time.perf_counter()
                hloc = ucv.get_location(humanoid_actor_name)
                if nav_timing is not None:
                    nav_timing.record_elapsed("pose_query_ms", t_loc)
                new_hxy = (float(hloc[0]), float(hloc[1]))
                if human_xy is None or dist2d(new_hxy, human_xy) >= HUMANOID_REPLAN_DELTA_CM:
                    nm.nav_stop_move(ucv, robot_name)
                    update_humanoid_obstacle(
                        ucv,
                        nav_actor,
                        humanoid_actor_name,
                        nav_timing=nav_timing,
                    )
                    _timed_rebuild(ucv, nav_actor, nav_timing)
                    robot_xy, _, ucv = _fetch_nav_pose(
                        ucv, robot_name, nav_timing, pose_cache, force=True
                    )
                    cur_xyz = _start_xyz_for_robot(
                        ucv, nav_actor, robot_xy, robot_name, nav_timing=nav_timing
                    )
                    if cur_xyz is None:
                        return False, new_hxy
                    waypoints = plan_navmesh_waypoints(
                        ucv, nav_actor, cur_xyz, goal_xyz, nav_timing=nav_timing
                    )
                    path_xyz = _waypoints_to_path_xyz(
                        ucv, nav_actor, waypoints, nav_timing=nav_timing
                    )
                    if not _dispatch_ue_path_follow(
                        ucv, robot_name, path_xyz, nav_timing=nav_timing
                    ):
                        return False, new_hxy
                    human_xy = new_hxy
                    if nav_timing is not None:
                        nav_timing.humanoid_replan_count += 1
                    print(
                        f"[NavMeshNav/moveto] humanoid replan @ "
                        f"({new_hxy[0]:.0f},{new_hxy[1]:.0f}) → {len(waypoints)} WP"
                    )
            except Exception as exc:
                print(f"[NavMeshNav/moveto] humanoid update skipped: {exc}")

        if now - last_pose_t >= 1.0:
            pos_xy, _, ucv = _fetch_nav_pose(
                ucv, robot_name, nav_timing, pose_cache, force=True
            )
            last_pose_t = now
            if trace is not None:
                trace.record_position(world_xy_to_local(*pos_xy))
            if on_pose_sample is not None:
                on_pose_sample(pos_xy, time.time())
            if dist2d(pos_xy, goal_xy) <= tolerance_cm:
                nm.nav_stop_move(ucv, robot_name)
                return True, human_xy

        t_poll = time.perf_counter()
        status = nm.get_nav_move_status(ucv, robot_name)
        if nav_timing is not None:
            nav_timing.record_elapsed("nav_moveto_poll_ms", t_poll)
        move_status = str(status.get("status", "")).lower()
        if move_status == "success":
            pos_xy, _, ucv = _fetch_nav_pose(
                ucv, robot_name, nav_timing, pose_cache, force=True
            )
            if dist2d(pos_xy, goal_xy) <= tolerance_cm:
                return True, human_xy
        if move_status == "failed":
            if nav_timing is not None:
                nav_timing.stuck_replan_count += 1
                nav_timing.nav_moveto_replan_count += 1
            nm.nav_stop_move(ucv, robot_name)
            pos_xy, _, ucv = _fetch_nav_pose(
                ucv, robot_name, nav_timing, pose_cache, force=True
            )
            if dist2d(pos_xy, goal_xy) <= tolerance_cm:
                return True, human_xy
            cur_xyz = _start_xyz_for_robot(
                ucv, nav_actor, pos_xy, robot_name, nav_timing=nav_timing
            )
            if cur_xyz is None:
                return False, human_xy
            waypoints = plan_navmesh_waypoints(
                ucv, nav_actor, cur_xyz, goal_xyz, nav_timing=nav_timing
            )
            if not waypoints:
                return False, human_xy
            path_xyz = _waypoints_to_path_xyz(
                ucv, nav_actor, waypoints, nav_timing=nav_timing
            )
            if not _dispatch_ue_path_follow(
                ucv, robot_name, path_xyz, nav_timing=nav_timing
            ):
                return False, human_xy
            print(
                f"[NavMeshNav/moveto] ue-failed replan @ "
                f"({pos_xy[0]:.0f},{pos_xy[1]:.0f}) → {len(waypoints)} WP"
            )

        wait_t0 = time.perf_counter()
        time.sleep(poll_s)
        if nav_timing is not None:
            nav_timing.record_elapsed("nav_moveto_wait_ms", wait_t0)
        if nav_timing is not None:
            gap = (time.perf_counter() - loop_t0) * 1000.0
            nav_timing.loop_overhead_ms += max(0.0, gap)

    nm.nav_stop_move(ucv, robot_name)
    return False, human_xy


def _navigate_navmesh_leg_vbp(
    ucv,
    goal_xy: WorldXY,
    *,
    nav_actor: str,
    robot_name: str,
    robot_speed: float,
    profile: NavProfile,
    perceive_fn: Optional[PerceiveFn] = None,
    perception_interval_s: float = 5.0,
    tolerance_cm: float = NAVMESH_GOAL_TOLERANCE_CM,
    max_total_steps: int = 600,
    label: str = "",
    trace: Optional[NavTrace] = None,
    on_pose_sample: Optional[PoseSampleFn] = None,
    nav_timing: Optional[NavTimingAccumulator] = None,
    pose_cache: Optional[PoseCache] = None,
    carry_sync_name: Optional[str] = None,
    humanoid_actor_name: Optional[str] = None,
    dynamic_humanoid: bool = False,
    last_humanoid_xy: Optional[WorldXY] = None,
    post_motion_settle_s: Optional[float] = None,
) -> Tuple[bool, Optional[WorldXY]]:
    """Follow NavFindPath polyline with open-loop VBP. Returns (reached, last_humanoid_xy)."""
    settle_s = (
        profile.post_motion_settle_s
        if post_motion_settle_s is None
        else post_motion_settle_s
    )
    goal_xyz = _nav_plan_xyz(ucv, nav_actor, goal_xy, nav_timing=nav_timing)
    if goal_xyz is None:
        print(
            f"[NavMeshNav] goal projection failed @ "
            f"({goal_xy[0]:.0f},{goal_xy[1]:.0f})"
        )
        return False, last_humanoid_xy
    pos_xy, _, ucv = _fetch_nav_pose(
        ucv, robot_name, nav_timing, pose_cache
    )
    start_xyz = _start_xyz_for_robot(
        ucv, nav_actor, pos_xy, robot_name, nav_timing=nav_timing
    )
    if start_xyz is None:
        print(
            f"[NavMeshNav] start projection failed @ "
            f"({pos_xy[0]:.0f},{pos_xy[1]:.0f})"
        )
        return False, last_humanoid_xy
    waypoints = plan_navmesh_waypoints(
        ucv, nav_actor, start_xyz, goal_xyz, nav_timing=nav_timing
    )
    if not waypoints:
        return False, last_humanoid_xy

    if trace is not None:
        trace.record_plan(waypoints, reason=label or "initial")

    wp_index = 0
    steps_on_wp = 0
    unchanged_steps = 0
    last_move_xy: Optional[WorldXY] = None
    last_perceive_t = -1e9
    human_xy = last_humanoid_xy
    use_carry_sync = bool(carry_sync_name) and not is_carry_ue_attached()
    last_iter_had_motion = True

    print(
        f"[NavMeshNav] {label}: {len(waypoints)} WP "
        f"goal=({goal_xy[0]:.0f},{goal_xy[1]:.0f})"
    )

    for step_i in range(max_total_steps):
        loop_t0 = time.perf_counter()
        accounted_at_start = nav_timing.accounted_ms() if nav_timing is not None else 0.0
        if nav_timing is not None:
            nav_timing.nav_loop_iterations += 1

        if last_iter_had_motion:
            invalidate_robot_pose(pose_cache, reason="navmesh_iter_after_motion")
        pos_xy, yaw_deg, ucv = _fetch_nav_pose(
            ucv,
            robot_name,
            nav_timing,
            pose_cache,
            force=last_iter_had_motion,
        )
        last_iter_had_motion = False
        if dist2d(pos_xy, goal_xy) <= tolerance_cm:
            if trace is not None:
                trace.record_position(world_xy_to_local(*pos_xy))
            if on_pose_sample is not None:
                on_pose_sample(pos_xy, time.time())
            _close_loop_overhead(nav_timing, loop_t0, accounted_at_start)
            return True, human_xy

        now = time.perf_counter()
        if perceive_fn is not None and now - last_perceive_t >= perception_interval_s:
            t_perceive = time.perf_counter()
            perceive_fn()
            if nav_timing is not None:
                nav_timing.record_elapsed("perceive_ms", t_perceive)
            last_perceive_t = now

        if dynamic_humanoid and humanoid_actor_name:
            try:
                t_loc = time.perf_counter()
                hloc = ucv.get_location(humanoid_actor_name)
                if nav_timing is not None:
                    nav_timing.record_elapsed("pose_query_ms", t_loc)
                new_hxy = (float(hloc[0]), float(hloc[1]))
                if human_xy is None or dist2d(new_hxy, human_xy) >= HUMANOID_REPLAN_DELTA_CM:
                    update_humanoid_obstacle(
                        ucv,
                        nav_actor,
                        humanoid_actor_name,
                        nav_timing=nav_timing,
                    )
                    _timed_rebuild(ucv, nav_actor, nav_timing)
                    invalidate_robot_pose(pose_cache, reason="humanoid_replan")
                    robot_xy, _, ucv = _fetch_nav_pose(
                        ucv,
                        robot_name,
                        nav_timing,
                        pose_cache,
                        force=True,
                    )
                    cur_xyz = _start_xyz_for_robot(
                        ucv,
                        nav_actor,
                        robot_xy,
                        robot_name,
                        nav_timing=nav_timing,
                    )
                    if cur_xyz is None:
                        continue
                    waypoints = plan_navmesh_waypoints(
                        ucv, nav_actor, cur_xyz, goal_xyz, nav_timing=nav_timing
                    )
                    wp_index = 0
                    steps_on_wp = 0
                    human_xy = new_hxy
                    if nav_timing is not None:
                        nav_timing.humanoid_replan_count += 1
                    print(
                        f"[NavMeshNav] humanoid replan @ ({new_hxy[0]:.0f},{new_hxy[1]:.0f}) "
                        f"→ {len(waypoints)} WP"
                    )
            except Exception as exc:
                print(f"[NavMeshNav] humanoid update skipped: {exc}")

        if wp_index >= len(waypoints):
            waypoint_xy = goal_xy
        else:
            waypoint_xy = waypoints[wp_index]
            if dist2d(pos_xy, waypoint_xy) <= NAVMESH_WP_REACH_TOLERANCE_CM:
                wp_index += 1
                steps_on_wp = 0
                _close_loop_overhead(nav_timing, loop_t0, accounted_at_start)
                continue

        cmd = _segment_command(
            pos_xy,
            yaw_deg,
            waypoint_xy,
            max_move_cm=min(
                profile.site_max_open_loop_move_cm,
                NAVMESH_MAX_OPEN_LOOP_MOVE_CM,
            ),
        )
        if cmd is None:
            wp_index += 1
            steps_on_wp = 0
            _close_loop_overhead(nav_timing, loop_t0, accounted_at_start)
            continue

        turn_deg, clockwise, move_cm = cmd
        did_motion = False
        t_move = time.perf_counter()
        if turn_deg > NAVMESH_ROTATE_THRESHOLD_DEG:
            turn_duration_s = max(0.12, turn_deg / max(robot_speed, 1.0))
            _site_dog_rotate(ucv, robot_name, turn_duration_s, turn_deg, clockwise)
            if nav_timing is not None:
                nav_timing.rotate_ms += (time.perf_counter() - t_move) * 1000.0
            did_motion = True
        if move_cm > 1e-3:
            move_t0 = time.perf_counter()
            move_duration_s = max(0.12, move_cm / max(robot_speed, 1.0))
            _site_dog_move(ucv, robot_name, robot_speed, move_duration_s, 0)
            if nav_timing is not None:
                nav_timing.translate_ms += (time.perf_counter() - move_t0) * 1000.0
            if use_carry_sync and carry_sync_name:
                t_carry = time.perf_counter()
                sync_carry_pose(
                    ucv,
                    carry_sync_name,
                    robot_name,
                    refresh_collision=False,
                )
                if nav_timing is not None:
                    nav_timing.record_elapsed("carry_sync_ms", t_carry)
            did_motion = True
        if nav_timing is not None:
            nav_timing.move_ms += (time.perf_counter() - t_move) * 1000.0
        if did_motion:
            invalidate_robot_pose(pose_cache, reason="navmesh_segment_motion")
            last_iter_had_motion = True
        if settle_s > 0:
            t_settle = time.perf_counter()
            tick_settle(ucv, settle_s=settle_s, ticks=1)
            if nav_timing is not None:
                nav_timing.record_elapsed("settle_ms", t_settle)

        if did_motion:
            pos_xy, _, ucv = _fetch_nav_pose(
                ucv,
                robot_name,
                nav_timing,
                pose_cache,
                force=True,
            )
        if last_move_xy is not None and dist2d(pos_xy, last_move_xy) < NAVMESH_STUCK_MOVE_THRESHOLD_CM:
            unchanged_steps += 1
        else:
            unchanged_steps = 0
        last_move_xy = pos_xy

        if trace is not None:
            trace.record_position(world_xy_to_local(*pos_xy))
        if on_pose_sample is not None:
            on_pose_sample(pos_xy, time.time())

        steps_on_wp += 1
        should_replan = (
            steps_on_wp >= NAVMESH_REPLAN_STUCK_STEPS
            or unchanged_steps >= NAVMESH_STUCK_UNCHANGED_STEPS
        )
        if should_replan:
            if dist2d(pos_xy, goal_xy) <= tolerance_cm:
                _close_loop_overhead(nav_timing, loop_t0, accounted_at_start)
                return True, human_xy
            reason = (
                "stuck"
                if unchanged_steps >= NAVMESH_STUCK_UNCHANGED_STEPS
                else "wp-timeout"
            )
            if nav_timing is not None:
                if reason == "stuck":
                    nav_timing.stuck_replan_count += 1
                else:
                    nav_timing.wp_timeout_replan_count += 1
            if unchanged_steps >= NAVMESH_STUCK_UNCHANGED_STEPS:
                _timed_rebuild(ucv, nav_actor, nav_timing)
            invalidate_robot_pose(pose_cache, reason="stuck_replan")
            cur_xyz = _start_xyz_for_robot(
                ucv, nav_actor, pos_xy, robot_name, nav_timing=nav_timing
            )
            if cur_xyz is None:
                _close_loop_overhead(nav_timing, loop_t0, accounted_at_start)
                return False, human_xy
            waypoints = plan_navmesh_waypoints(
                ucv, nav_actor, cur_xyz, goal_xyz, nav_timing=nav_timing
            )
            wp_index = 0
            steps_on_wp = 0
            unchanged_steps = 0
            print(
                f"[NavMeshNav] {reason} replan @ ({pos_xy[0]:.0f},{pos_xy[1]:.0f}) "
                f"→ {len(waypoints)} WP (step {step_i})"
            )

        _close_loop_overhead(nav_timing, loop_t0, accounted_at_start)

    return False, human_xy


def navigate_navmesh_leg(
    ucv,
    goal_xy: WorldXY,
    *,
    nav_actor: str,
    robot_name: str,
    robot_speed: float,
    profile: NavProfile,
    perceive_fn: Optional[PerceiveFn] = None,
    perception_interval_s: float = 5.0,
    tolerance_cm: float = NAVMESH_GOAL_TOLERANCE_CM,
    max_total_steps: int = 600,
    label: str = "",
    trace: Optional[NavTrace] = None,
    on_pose_sample: Optional[PoseSampleFn] = None,
    nav_timing: Optional[NavTimingAccumulator] = None,
    pose_cache: Optional[PoseCache] = None,
    carry_sync_name: Optional[str] = None,
    humanoid_actor_name: Optional[str] = None,
    dynamic_humanoid: bool = False,
    last_humanoid_xy: Optional[WorldXY] = None,
    post_motion_settle_s: Optional[float] = None,
    nav_exec: str = "vbp",
) -> Tuple[bool, Optional[WorldXY]]:
    """Follow NavFindPath polyline (vbp open-loop or UE moveto)."""
    if nav_exec == "moveto":
        return _navigate_navmesh_leg_moveto(
            ucv,
            goal_xy,
            nav_actor=nav_actor,
            robot_name=robot_name,
            profile=profile,
            perceive_fn=perceive_fn,
            perception_interval_s=perception_interval_s,
            tolerance_cm=tolerance_cm,
            max_total_steps=max_total_steps,
            label=label,
            trace=trace,
            on_pose_sample=on_pose_sample,
            nav_timing=nav_timing,
            pose_cache=pose_cache,
            humanoid_actor_name=humanoid_actor_name,
            dynamic_humanoid=dynamic_humanoid,
            last_humanoid_xy=last_humanoid_xy,
        )
    return _navigate_navmesh_leg_vbp(
        ucv,
        goal_xy,
        nav_actor=nav_actor,
        robot_name=robot_name,
        robot_speed=robot_speed,
        profile=profile,
        perceive_fn=perceive_fn,
        perception_interval_s=perception_interval_s,
        tolerance_cm=tolerance_cm,
        max_total_steps=max_total_steps,
        label=label,
        trace=trace,
        on_pose_sample=on_pose_sample,
        nav_timing=nav_timing,
        pose_cache=pose_cache,
        carry_sync_name=carry_sync_name,
        humanoid_actor_name=humanoid_actor_name,
        dynamic_humanoid=dynamic_humanoid,
        last_humanoid_xy=last_humanoid_xy,
        post_motion_settle_s=post_motion_settle_s,
    )


def navigate_to_slot_navmesh(
    ucv,
    slot_id: str,
    *,
    object_registry,
    nav_actor: str,
    robot_name: str,
    profile: NavProfile,
    perceive_fn: Optional[PerceiveFn] = None,
    fallback_goal_local: Optional[Tuple[float, float]] = None,
    tolerance_cm: float = NAVMESH_GOAL_TOLERANCE_CM,
    label: str = "",
    perception_interval_s: float = 5.0,
    max_total_steps: int = 600,
    trace: Optional[NavTrace] = None,
    on_pose_sample: Optional[PoseSampleFn] = None,
    nav_timing: Optional[NavTimingAccumulator] = None,
    pose_cache: Optional[PoseCache] = None,
    nav_exec: str = "vbp",
) -> bool:
    from carry import pickup_standoff_xy

    goal_xy = (
        object_registry.goal_xy(slot_id)
        if hasattr(object_registry, "goal_xy")
        else None
    )
    if goal_xy is None and fallback_goal_local is not None:
        goal_xy = local_xy_to_world(*fallback_goal_local)
    if goal_xy is None:
        print(f"[NavMeshNav] unknown slot {slot_id}")
        return False

    robot_xy, _, ucv = _fetch_nav_pose(
        ucv, robot_name, nav_timing, pose_cache
    )
    approach_xy = pickup_standoff_xy(goal_xy, robot_xy, standoff_cm=160.0)
    nav_label = label or f"to-slot-{slot_id}"
    reached, _ = navigate_navmesh_leg(
        ucv,
        approach_xy,
        nav_actor=nav_actor,
        robot_name=robot_name,
        robot_speed=profile.site_robot_speed,
        profile=profile,
        perceive_fn=perceive_fn,
        perception_interval_s=perception_interval_s,
        tolerance_cm=tolerance_cm,
        max_total_steps=max_total_steps,
        label=nav_label,
        trace=trace,
        on_pose_sample=on_pose_sample,
        nav_timing=nav_timing,
        pose_cache=pose_cache,
        nav_exec=nav_exec,
    )
    return reached


def deliver_to_navmesh(
    ucv,
    slot_id: str,
    *,
    object_registry,
    nav_actor: str,
    robot_name: str,
    profile: NavProfile,
    perceive_fn: Optional[PerceiveFn] = None,
    fallback_goal_local: Optional[Tuple[float, float]] = None,
    humanoid_actor_name: Optional[str] = None,
    tolerance_cm: float = NAVMESH_GOAL_TOLERANCE_CM,
    label: str = "",
    perception_interval_s: float = 5.0,
    max_total_steps: int = 600,
    trace: Optional[NavTrace] = None,
    carry_sync_name: Optional[str] = None,
    on_pose_sample: Optional[PoseSampleFn] = None,
    nav_timing: Optional[NavTimingAccumulator] = None,
    pose_cache: Optional[PoseCache] = None,
    nav_exec: str = "vbp",
) -> bool:
    goal_local = (
        object_registry.goal_local(slot_id)
        if hasattr(object_registry, "goal_local")
        else None
    )
    if goal_local is None:
        goal_local = fallback_goal_local
    if goal_local is None:
        print(f"[NavMeshNav] deliver_to unknown slot {slot_id}")
        return False
    goal_xy = local_xy_to_world(*goal_local)
    nav_label = label or f"deliver-to-{slot_id}"
    reached, _ = navigate_navmesh_leg(
        ucv,
        goal_xy,
        nav_actor=nav_actor,
        robot_name=robot_name,
        robot_speed=profile.site_robot_speed,
        profile=profile,
        perceive_fn=perceive_fn,
        perception_interval_s=perception_interval_s,
        tolerance_cm=tolerance_cm,
        max_total_steps=max_total_steps,
        label=nav_label,
        trace=trace,
        on_pose_sample=on_pose_sample,
        nav_timing=nav_timing,
        pose_cache=pose_cache,
        carry_sync_name=carry_sync_name,
        humanoid_actor_name=humanoid_actor_name,
        dynamic_humanoid=True,
        nav_exec=nav_exec,
    )
    return reached
