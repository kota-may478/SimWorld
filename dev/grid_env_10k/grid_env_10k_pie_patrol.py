#!/usr/bin/env python3
"""grid_100x100 PIE patrol: perimeter solidification, A* navigation, SpotDog round trip.

Prerequisites:
  - UE Editor: open grid_100x100 and start PIE
  - Map already contains floor + block_* actors (initially translucent)
  - WSL: conda activate simworld

Grid cell indices (gx, gy) are 1-based (see grid_env_10k).
"""

from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

BlockIndex = Tuple[int, int]
WorldXY = Tuple[float, float]


def _find_project_root() -> Path:
    here = Path(__file__).resolve().parent
    for candidate in (here, *here.parents):
        if (candidate / "setup.py").exists() and (candidate / "simworld").is_dir():
            return candidate
    return here.parent.parent


ROOT = _find_project_root()
GEH_DIR = ROOT / "dev" / "grid_env_hri"
G10K_DIR = ROOT / "dev" / "grid_env_10k"
MT_DIR = ROOT / "dev" / "llm_material_transport"
for p in (ROOT, GEH_DIR, G10K_DIR, MT_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
from path_planning_costmap import (  # noqa: E402
    COSTMAP_LETHAL_COST,
    AStarPlanResult,
    Costmap2D,
    build_uniform_costmap,
    plan_waypoints_grid_astar,
)
from simworld.agent.humanoid import Humanoid  # noqa: E402
from simworld.communicator.communicator import Communicator  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402
from simworld.utils.vector import Vector  # noqa: E402

# ---- Scenario defaults ----
GRID_N = 100
# Costmap cell size matches block width (0.30 m) for 1:1 grid alignment.
COSTMAP_RESOLUTION_CM = geh.CUBE_SIZE_CM
HUMAN_CELL: BlockIndex = (5, 5)
ROBOT_START_CELL: BlockIndex = (10, 5)
ROBOT_GOAL_CELL: BlockIndex = (50, 50)
GOAL_DWELL_S = 5.0

# ---- Robot control (same family as llm_material_transport) ----
ROBOT_SPEED = 200.0
ROBOT_MOVE_SLICE_S = 0.2
ROBOT_TURN_DUR_S = 1.0
ROTATE_THR_DEG = 20.0
ARRIVE_TOLERANCE_CM = 100.0
PATH_WP_SPACING_CM = 300.0
PATH_WP_REACH_TOLERANCE_CM = 80.0
PATH_MAX_OPEN_LOOP_MOVE_CM = 350.0
PATH_MAX_STEPS_PER_WP = 50
PATH_REPLAN_STUCK_STEPS = 14
PATH_MAX_TOTAL_STEPS = 1200
RETURN_ARRIVE_TOLERANCE_CM = 120.0
ROBOT_BP = geh.ROBOT_BP
REGISTRY_CACHE_PATH = G10K_DIR / ".pie_block_registry.json"
ROBOT_PROBE_NAME = "__GridEnv_SpotRobot_probe__"


@dataclass(frozen=True)
class SegmentCommand:
    turn_deg: float
    turn_clockwise: int
    move_cm: float


@dataclass
class PatrolResult:
    perimeter_ok: bool
    human_name: Optional[str]
    robot_spawned: bool
    outbound_arrived: bool
    return_arrived: bool
    start_xy: WorldXY
    goal_xy: WorldXY
    final_xy: WorldXY
    return_dist_cm: float


def block_index_to_map_xy_m(gx: int, gy: int) -> Tuple[float, float]:
    """Return cell-center map coordinates [m] (lower-left origin) for (gx, gy)."""
    g10k.validate_block_index(gx, gy, grid_n=GRID_N)
    col = gx - 1
    row = gy - 1
    return (col + 0.5) * geh.CUBE_SIZE_M, (row + 0.5) * geh.CUBE_SIZE_M


def block_index_to_world_xy_cm(gx: int, gy: int) -> WorldXY:
    mx, my = block_index_to_map_xy_m(gx, gy)
    return geh.map_xy_m_to_world_cm((mx, my))


def world_cm_to_block_index(
    x_cm: float,
    y_cm: float,
    *,
    grid_n: int = GRID_N,
) -> Optional[BlockIndex]:
    """Map UE world XY [cm] to nearest cell (gx, gy); None if out of range."""
    ox, oy = geh.MAP_ORIGIN_XY_CM
    col = int(round((x_cm - ox - geh.CUBE_HALF_CM) / geh.CUBE_SIZE_CM))
    row = int(round((y_cm - oy - geh.CUBE_HALF_CM) / geh.CUBE_SIZE_CM))
    gx, gy = col + 1, row + 1
    if 1 <= gx <= grid_n and 1 <= gy <= grid_n:
        return gx, gy
    return None


def _map_cube_actor_to_cell(
    ucv: UnrealCV,
    name: str,
    *,
    grid_n: int,
) -> Optional[Tuple[BlockIndex, str]]:
    if name.startswith(g10k.BLOCK_ACTOR_PREFIX + "_"):
        parsed = g10k.parse_block_actor_name(name)
        if parsed is not None:
            return parsed, name
        return None
    if "TransparentCube" not in name:
        return None
    loc = geh.try_get_location_cm(ucv, name)
    if loc is None:
        return None
    cell = world_cm_to_block_index(loc[0], loc[1], grid_n=grid_n)
    if cell is None:
        return None
    return cell, name


def _load_registry_cache() -> Dict[str, str]:
    if not REGISTRY_CACHE_PATH.is_file():
        return {}
    try:
        raw = json.loads(REGISTRY_CACHE_PATH.read_text(encoding="utf-8"))
        return {str(k): str(v) for k, v in raw.items()}
    except (OSError, json.JSONDecodeError, TypeError, ValueError):
        return {}


def _save_registry_cache(registry: Dict[BlockIndex, str]) -> None:
    payload = {f"{gx:03d}_{gy:03d}": name for (gx, gy), name in registry.items()}
    try:
        REGISTRY_CACHE_PATH.write_text(
            json.dumps(payload, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        print(f"[Registry] cache saved: {REGISTRY_CACHE_PATH}")
    except OSError as exc:
        print(f"[Registry] warn: cache save failed: {exc}")


def build_pie_block_registry(
    ucv: UnrealCV,
    *,
    grid_n: int = GRID_N,
    cells: Optional[Set[BlockIndex]] = None,
    use_cache: bool = True,
) -> Dict[BlockIndex, str]:
    """Map PIE actor names to (gx, gy); vget /objects returns UAID names only."""
    needed = set(cells) if cells is not None else None
    registry: Dict[BlockIndex, str] = {}
    cache = _load_registry_cache() if use_cache else {}

    if needed is not None and cache:
        for cell in list(needed):
            key = f"{cell[0]:03d}_{cell[1]:03d}"
            name = cache.get(key)
            if name and geh.actor_exists(ucv, name):
                registry[cell] = name
                needed.discard(cell)
        if needed:
            print(f"[Registry] cache hit {len(registry)}, still need {len(needed)} cells")

    names = geh.actor_names(ucv)
    cube_names = [
        n
        for n in names
        if n.startswith(g10k.BLOCK_ACTOR_PREFIX + "_") or "TransparentCube" in n
    ]
    t0 = time.monotonic()
    print(
        f"[Registry] scanning {len(cube_names)} cube actors"
        f"{f' for {len(needed)} cells' if needed else ''} ..."
    )

    for i, name in enumerate(cube_names, start=1):
        if needed is not None and not needed:
            break
        mapped = _map_cube_actor_to_cell(ucv, name, grid_n=grid_n)
        if mapped is None:
            continue
        cell, actor_name = mapped
        if needed is not None and cell not in needed:
            continue
        registry.setdefault(cell, actor_name)
        if needed is not None and cell in needed:
            needed.discard(cell)
        if i % 500 == 0 or i == len(cube_names):
            print(
                f"[Registry] {i}/{len(cube_names)} mapped={len(registry)} "
                f"elapsed={time.monotonic() - t0:.0f}s",
                flush=True,
            )

    if cache:
        merged = dict(cache)
        merged.update(
            {f"{gx:03d}_{gy:03d}": name for (gx, gy), name in registry.items()}
        )
        registry_full: Dict[BlockIndex, str] = {}
        for key, name in merged.items():
            parts = key.split("_")
            if len(parts) == 2:
                registry_full[(int(parts[0]), int(parts[1]))] = name
        _save_registry_cache(registry_full)
    elif registry:
        _save_registry_cache(registry)

    expected = len(cells) if cells is not None else grid_n * grid_n
    print(
        f"[Registry] mapped {len(registry)}/{expected} blocks "
        f"in {time.monotonic() - t0:.1f}s"
    )
    if cells is None and len(registry) < grid_n * grid_n:
        print(
            "[Registry] warn: incomplete map — World Partition で未ロードのセルがあると "
            "apply_scenario が失敗します。Editor で grid 領域をロードしてください。"
        )
    return registry


def resolve_block_actor_name(
    registry: Dict[BlockIndex, str],
    gx: int,
    gy: int,
) -> str:
    return registry.get((gx, gy), g10k.block_actor_name(gx, gy))


def set_block_mode_pie(
    ucv: UnrealCV,
    registry: Dict[BlockIndex, str],
    gx: int,
    gy: int,
    mode: g10k.BlockMode,
) -> bool:
    name = resolve_block_actor_name(registry, gx, gy)
    if not geh.actor_exists(ucv, name):
        return False
    blocking = g10k.mode_to_set_blocking(mode)
    return geh.set_cube_blocking_mode(
        ucv, name, blocking=blocking, apply_tint=blocking
    )


def set_blocks_mode_pie(
    ucv: UnrealCV,
    registry: Dict[BlockIndex, str],
    cells: Iterable[BlockIndex],
    mode: g10k.BlockMode,
    *,
    progress_every: int = 100,
    label: str = "",
) -> Tuple[int, int]:
    ok_n = 0
    fail_n = 0
    cells_list = list(cells)
    total = len(cells_list)
    t0 = time.monotonic()
    prefix = f"[Layout]{f' {label}' if label else ''}"
    print(f"{prefix} apply {mode} to {total} cells (PIE registry) ...")
    for i, (gx, gy) in enumerate(cells_list, start=1):
        if set_block_mode_pie(ucv, registry, gx, gy, mode):
            ok_n += 1
        else:
            fail_n += 1
        if progress_every > 0 and (i % progress_every == 0 or i == total):
            print(
                f"{prefix} {i}/{total} ok={ok_n} fail={fail_n} "
                f"elapsed={time.monotonic() - t0:.0f}s"
            )
    print(f"{prefix} done ok={ok_n} fail={fail_n} elapsed={time.monotonic() - t0:.1f}s")
    return ok_n, fail_n


def verify_robot_bp_available(ucv: UnrealCV) -> bool:
    """BP_SpotRobot が PIE からスポーン可能か短いプローブ（同一セッションは1回）。"""
    from mount_simworld_runtime_paks_pie import probe_robot_spawn  # noqa: WPS433

    return probe_robot_spawn(ucv)


def robot_bp_setup_hint() -> str:
    return (
        "SpotDog BP が PIE からスポーンできません。\n"
        "  1. PIE 停止 → C:\\UEProjects\\SimWorld\\apply_unrealcv_dll.bat 実行\n"
        "     （spawn_bp_asset + /Game UFS 登録入り DLL を適用）\n"
        "  2. UE Editor 再起動 → grid_100x100 → PIE\n"
        "  3. WSL: python dev/grid_env_10k/grid_env_10k_pie_patrol.py\n"
        "  （パトロールは起動時に pak マウントも自動実行）\n"
        "  pak: C:\\SimWorldServer\\SimWorld\\Content\\Paks\\pakchunk1000/0"
    )


def cleanup_runtime_agents(ucv: UnrealCV) -> None:
    """Remove patrol Humanoid / SpotDog / probe actors from the PIE session."""
    geh.destroy_actor_safely(ucv, geh.ROBOT_ACTOR_NAME)
    geh.destroy_actor_safely(ucv, ROBOT_PROBE_NAME)
    raw = geh._ue_request(ucv, "vget /objects", timeout_s=90.0)
    if raw:
        for name in raw.split():
            if name.startswith("GEN_BP_Humanoid"):
                geh.destroy_actor_safely(ucv, name)
    geh._prepare_ue_spawn(ucv)


def prepare_pie_rerun(ucv: UnrealCV) -> None:
    """Early cleanup when re-running patrol in the same PIE session."""
    print("[Scenario] prepare PIE rerun (clear prior agents) ...")
    cleanup_runtime_agents(ucv)


def _configure_robot_at(ucv: UnrealCV, loc: Sequence[float]) -> None:
    """Apply standard SpotDog spawn settings at ``loc`` [cm]."""
    ucv.set_physics(geh.ROBOT_ACTOR_NAME, False)
    ucv.set_movable(geh.ROBOT_ACTOR_NAME, True)
    ucv.set_location(list(loc), geh.ROBOT_ACTOR_NAME)
    ucv.set_orientation((0.0, 0.0, 0.0), geh.ROBOT_ACTOR_NAME)
    ucv.set_collision(geh.ROBOT_ACTOR_NAME, True)
    ucv.enable_controller(geh.ROBOT_ACTOR_NAME, True)
    time.sleep(geh.PHYSICS_ENABLE_DELAY_S)


def _find_existing_humanoid_name(ucv: UnrealCV) -> Optional[str]:
    raw = geh._ue_request(ucv, "vget /objects", timeout_s=60.0)
    if not raw:
        return None
    names = [n for n in raw.split() if n.startswith("GEN_BP_Humanoid")]
    return names[0] if names else None


def spawn_humanoid_at(
    communicator: Communicator,
    ucv: UnrealCV,
    cell: BlockIndex,
) -> Optional[str]:
    gx, gy = cell
    map_xy = block_index_to_map_xy_m(gx, gy)
    loc = geh.agent_spawn_xyz_cm(map_xy, spawn_z_cm=geh.HUMAN_SPAWN_Z_CM)

    existing = _find_existing_humanoid_name(ucv)
    if existing and geh.actor_exists(ucv, existing):
        print(f"[Humanoid] reuse {existing!r} (teleport to cell)")
        ucv.set_physics(existing, False)
        ucv.set_movable(existing, True)
        ucv.set_location(list(loc), existing)
        ucv.set_collision(existing, True)
        return existing

    human = Humanoid(position=Vector(loc[0], loc[1]), direction=Vector(1, 0))
    communicator.spawn_agent(
        agent=human,
        name=None,
        position=loc,
        model_path=geh.HUMAN_BP,
        type="humanoid",
    )
    human_name = communicator.get_humanoid_name(human.id)
    ucv.set_physics(human_name, False)
    ucv.set_movable(human_name, True)
    ucv.set_collision(human_name, True)
    try:
        communicator.humanoid_set_speed(human.id, 0.0)
    except Exception:
        pass
    print(f"[Humanoid] {human_name} @ cell({gx},{gy}) map={map_xy} world={geh._fmt_xyz(loc)}")
    return human_name


def spawn_robot_at(ucv: UnrealCV, cell: BlockIndex) -> bool:
    gx, gy = cell
    map_xy = block_index_to_map_xy_m(gx, gy)
    loc = geh.agent_spawn_xyz_cm(map_xy, spawn_z_cm=geh.ROBOT_SPAWN_Z_CM)
    robot_name = geh.ROBOT_ACTOR_NAME

    gone = geh.destroy_actor_safely(ucv, robot_name)
    if not gone and geh.actor_exists(ucv, robot_name):
        print(
            f"[Robot] reuse existing {robot_name!r} at goal/start "
            "(teleport to start cell; skip spawn_bp rename)"
        )
        _configure_robot_at(ucv, loc)
        print(
            f"[Robot] {robot_name} @ cell({gx},{gy}) "
            f"map={map_xy} world={geh._fmt_xyz(loc)}"
        )
        return True

    if not geh.spawn_bp(ucv, ROBOT_BP, robot_name):
        if geh.actor_exists(ucv, robot_name):
            print(f"[Robot] spawn failed but {robot_name!r} exists — reusing via teleport")
            _configure_robot_at(ucv, loc)
            return True
        print("[Robot] spawn failed")
        return False

    _configure_robot_at(ucv, loc)
    print(
        f"[Robot] {robot_name} @ cell({gx},{gy}) "
        f"map={map_xy} world={geh._fmt_xyz(loc)}"
    )
    return True


def _mark_rect_lethal(
    costmap: Costmap2D,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
) -> None:
    res = costmap.resolution_cm
    gx0 = int(math.floor((x0 - costmap.origin_xy[0]) / res))
    gy0 = int(math.floor((y0 - costmap.origin_xy[1]) / res))
    gx1 = int(math.ceil((x1 - costmap.origin_xy[0]) / res))
    gy1 = int(math.ceil((y1 - costmap.origin_xy[1]) / res))
    gx0 = max(0, gx0)
    gy0 = max(0, gy0)
    gx1 = min(costmap.width_cells, gx1)
    gy1 = min(costmap.height_cells, gy1)
    for gy in range(gy0, gy1):
        for gx in range(gx0, gx1):
            costmap.costs[gy, gx] = COSTMAP_LETHAL_COST


def build_costmap_from_blocking_cells(
    blocking_cells: Set[BlockIndex],
    *,
    grid_n: int = GRID_N,
) -> Costmap2D:
    """Build a 30 m costmap; solid (T) cells are marked lethal at block resolution."""
    size_m = grid_n * geh.CUBE_SIZE_M
    costmap = build_uniform_costmap(
        origin_xy=geh.MAP_ORIGIN_XY_CM,
        size_m=size_m,
        resolution_cm=COSTMAP_RESOLUTION_CM,
    )
    ox, oy = geh.MAP_ORIGIN_XY_CM
    for gx, gy in blocking_cells:
        row, col = g10k.block_index_to_row_col(gx, gy)
        x0 = ox + col * geh.CUBE_SIZE_CM
        x1 = x0 + geh.CUBE_SIZE_CM
        y0 = oy + row * geh.CUBE_SIZE_CM
        y1 = y0 + geh.CUBE_SIZE_CM
        _mark_rect_lethal(costmap, x0, y0, x1, y1)
    print(
        f"[Costmap] {size_m:.0f} m, resolution={costmap.resolution_cm / 100:.2f} m, "
        f"lethal from {len(blocking_cells)} blocks, grid={costmap.costs.shape}"
    )
    return costmap


def perimeter_blocking_set(*, grid_n: int = GRID_N) -> Set[BlockIndex]:
    return set(g10k.iter_perimeter_indices(grid_n))


def get_pos2d(ucv: UnrealCV, actor_name: str) -> WorldXY:
    loc = ucv.get_location(actor_name)
    return float(loc[0]), float(loc[1])


def get_yaw(ucv: UnrealCV, actor_name: str) -> float:
    ori = ucv.get_orientation(actor_name)
    return float(ori[1])


def dist2d(a: WorldXY, b: WorldXY) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def normalize_angle(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def yaw_to_target(from_xy: WorldXY, to_xy: WorldXY) -> float:
    return math.degrees(math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0]))


def plan_astar_waypoints(
    costmap: Costmap2D,
    start_xy: WorldXY,
    goal_xy: WorldXY,
) -> AStarPlanResult:
    return plan_waypoints_grid_astar(
        costmap,
        start_xy,
        goal_xy,
        max_segment_cm=PATH_WP_SPACING_CM,
    )


def segment_command_toward_waypoint(
    pos_xy: WorldXY,
    yaw_deg: float,
    waypoint_xy: WorldXY,
) -> Optional[SegmentCommand]:
    distance_cm = dist2d(pos_xy, waypoint_xy)
    if distance_cm < 1e-3:
        return None
    target_yaw = yaw_to_target(pos_xy, waypoint_xy)
    angle_diff = normalize_angle(target_yaw - yaw_deg)
    if abs(angle_diff) > ROTATE_THR_DEG:
        clockwise = 1 if angle_diff < 0.0 else -1
        return SegmentCommand(
            turn_deg=abs(angle_diff),
            turn_clockwise=clockwise,
            move_cm=0.0,
        )
    move_cm = min(distance_cm, PATH_MAX_OPEN_LOOP_MOVE_CM)
    return SegmentCommand(turn_deg=0.0, turn_clockwise=1, move_cm=move_cm)


def execute_segment_command(ucv: UnrealCV, command: SegmentCommand) -> None:
    if command.turn_deg > ROTATE_THR_DEG:
        turn_duration_s = max(0.15, ROBOT_TURN_DUR_S * command.turn_deg / 90.0)
        ucv.dog_rotate(
            geh.ROBOT_ACTOR_NAME,
            [turn_duration_s, command.turn_deg, command.turn_clockwise],
        )
        time.sleep(turn_duration_s * 0.15)
    if command.move_cm > 1e-3:
        move_duration_s = max(ROBOT_MOVE_SLICE_S, command.move_cm / ROBOT_SPEED)
        ucv.dog_move(
            geh.ROBOT_ACTOR_NAME,
            [ROBOT_SPEED, move_duration_s, 0],
        )
        time.sleep(move_duration_s * 0.1)


def _nearest_waypoint_index_ahead(
    pos_xy: WorldXY,
    waypoints: Sequence[WorldXY],
    current_index: int,
) -> int:
    if not waypoints:
        return 0
    start = min(max(current_index, 0), len(waypoints) - 1)
    best_index = start
    best_dist = dist2d(pos_xy, waypoints[start])
    for index in range(start, len(waypoints)):
        dist = dist2d(pos_xy, waypoints[index])
        if dist < best_dist:
            best_dist = dist
            best_index = index
    return best_index


def robot_navigate_astar(
    ucv: UnrealCV,
    costmap: Costmap2D,
    goal_xy: WorldXY,
    *,
    tolerance_cm: float = ARRIVE_TOLERANCE_CM,
    label: str = "",
) -> bool:
    """Global A* waypoints + open-loop rotate-then-drive control."""
    start_xy = get_pos2d(ucv, geh.ROBOT_ACTOR_NAME)
    plan = plan_astar_waypoints(costmap, start_xy, goal_xy)
    waypoints = plan.waypoints_xy
    wp_index = 0
    steps_on_wp = 0
    total_steps = 0

    print(
        f"  [Nav]{f' {label}' if label else ''} A* cost={plan.total_cost:.1f}, "
        f"grid_cells={len(plan.grid_path)}, waypoints={len(waypoints)}"
    )
    for index, waypoint in enumerate(waypoints[:8]):
        print(f"    WP{index + 1}: ({waypoint[0]:.1f}, {waypoint[1]:.1f})")
    if len(waypoints) > 8:
        print(f"    ... ({len(waypoints) - 8} more)")

    while total_steps < PATH_MAX_TOTAL_STEPS:
        total_steps += 1
        pos_xy = get_pos2d(ucv, geh.ROBOT_ACTOR_NAME)
        if dist2d(pos_xy, goal_xy) <= tolerance_cm:
            print(f"  [Nav] Arrived (dist={dist2d(pos_xy, goal_xy):.1f} cm)")
            return True

        if wp_index >= len(waypoints):
            waypoint_xy = goal_xy
        else:
            waypoint_xy = waypoints[wp_index]
            if dist2d(pos_xy, waypoint_xy) <= PATH_WP_REACH_TOLERANCE_CM:
                wp_index += 1
                steps_on_wp = 0
                continue

        command = segment_command_toward_waypoint(
            pos_xy,
            get_yaw(ucv, geh.ROBOT_ACTOR_NAME),
            waypoint_xy,
        )
        if command is None:
            if wp_index >= len(waypoints):
                if dist2d(pos_xy, goal_xy) <= max(tolerance_cm, ARRIVE_TOLERANCE_CM):
                    return True
            else:
                wp_index += 1
            steps_on_wp = 0
            continue

        execute_segment_command(ucv, command)
        steps_on_wp += 1

        if steps_on_wp >= PATH_REPLAN_STUCK_STEPS:
            pos_xy = get_pos2d(ucv, geh.ROBOT_ACTOR_NAME)
            if dist2d(pos_xy, goal_xy) <= max(tolerance_cm, ARRIVE_TOLERANCE_CM):
                return True
            if wp_index < len(waypoints):
                replan = plan_astar_waypoints(costmap, pos_xy, goal_xy)
                if replan.waypoints_xy:
                    waypoints = replan.waypoints_xy
                    wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                    print(
                        f"  [Nav] Replan @ ({pos_xy[0]:.1f},{pos_xy[1]:.1f}) "
                        f"→ {len(waypoints)} WP, resume WP{wp_index + 1}"
                    )
            steps_on_wp = 0

        if steps_on_wp >= PATH_MAX_STEPS_PER_WP and wp_index < len(waypoints):
            print(f"  [Nav] WP{wp_index + 1} step limit; skip")
            wp_index += 1
            steps_on_wp = 0

    print(f"  [Nav] ERROR: exceeded PATH_MAX_TOTAL_STEPS={PATH_MAX_TOTAL_STEPS}")
    return False


def apply_perimeter_solid(
    ucv: UnrealCV,
    registry: Dict[BlockIndex, str],
    *,
    grid_n: int = GRID_N,
) -> Tuple[int, int]:
    return set_blocks_mode_pie(
        ucv,
        registry,
        g10k.iter_perimeter_indices(grid_n),
        "T",
        progress_every=50,
        label="perimeter T",
    )


def run_patrol_scenario(
    *,
    human_cell: BlockIndex = HUMAN_CELL,
    robot_start_cell: BlockIndex = ROBOT_START_CELL,
    robot_goal_cell: BlockIndex = ROBOT_GOAL_CELL,
    goal_dwell_s: float = GOAL_DWELL_S,
    grid_n: int = GRID_N,
    skip_perimeter_if_already_solid: bool = False,
    skip_robot_probe: bool = False,
) -> PatrolResult:
    """Perimeter T -> spawn agents -> outbound A* -> dwell -> return A*."""
    ucv, communicator = g10k.ensure_connection()
    if not ucv.client.isconnected():
        raise ConnectionError("UnrealCV not connected — start PIE in UE Editor first.")

    prepare_pie_rerun(ucv)

    scripts_dir = str(G10K_DIR / "scripts")
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    from mount_simworld_runtime_paks_pie import mount_paks, probe_robot_spawn  # noqa: WPS433

    if not mount_paks(ucv):
        raise RuntimeError(robot_bp_setup_hint())
    if skip_robot_probe:
        print("[Robot] skip BP probe (reuse session)")
    elif not probe_robot_spawn(ucv):
        raise RuntimeError(robot_bp_setup_hint())

    start_xy = block_index_to_world_xy_cm(*robot_start_cell)
    goal_xy = block_index_to_world_xy_cm(*robot_goal_cell)

    perimeter_cells = set(g10k.iter_perimeter_indices(grid_n))
    registry = build_pie_block_registry(
        ucv, grid_n=grid_n, cells=perimeter_cells
    )

    if skip_perimeter_if_already_solid:
        print("[Scenario] skip perimeter (assume already T)")
        ok_n, fail_n = len(perimeter_cells), 0
    else:
        ok_n, fail_n = apply_perimeter_solid(ucv, registry, grid_n=grid_n)
    perimeter_ok = fail_n == 0

    blocking = perimeter_blocking_set(grid_n=grid_n)
    costmap = build_costmap_from_blocking_cells(blocking, grid_n=grid_n)

    human_name = spawn_humanoid_at(communicator, ucv, human_cell)
    robot_spawned = spawn_robot_at(ucv, robot_start_cell)
    if not robot_spawned:
        return PatrolResult(
            perimeter_ok=perimeter_ok,
            human_name=human_name,
            robot_spawned=False,
            outbound_arrived=False,
            return_arrived=False,
            start_xy=start_xy,
            goal_xy=goal_xy,
            final_xy=start_xy,
            return_dist_cm=float("inf"),
        )

    time.sleep(0.5)
    actual_start = get_pos2d(ucv, geh.ROBOT_ACTOR_NAME)
    print(f"[Scenario] outbound {robot_start_cell} → {robot_goal_cell}")
    outbound_arrived = robot_navigate_astar(
        ucv,
        costmap,
        goal_xy,
        tolerance_cm=ARRIVE_TOLERANCE_CM,
        label="outbound",
    )

    if outbound_arrived and goal_dwell_s > 0:
        print(f"[Scenario] dwell {goal_dwell_s:.1f}s at goal ...")
        time.sleep(goal_dwell_s)

    print(f"[Scenario] return {robot_goal_cell} → {robot_start_cell}")
    return_arrived = robot_navigate_astar(
        ucv,
        costmap,
        start_xy,
        tolerance_cm=RETURN_ARRIVE_TOLERANCE_CM,
        label="return",
    )

    final_xy = get_pos2d(ucv, geh.ROBOT_ACTOR_NAME)
    return_dist = dist2d(final_xy, start_xy)
    print(
        f"[Scenario] done outbound={outbound_arrived} return={return_arrived} "
        f"final_dist_from_start={return_dist:.1f} cm"
    )
    return PatrolResult(
        perimeter_ok=perimeter_ok,
        human_name=human_name,
        robot_spawned=True,
        outbound_arrived=outbound_arrived,
        return_arrived=return_arrived,
        start_xy=start_xy,
        goal_xy=goal_xy,
        final_xy=final_xy,
        return_dist_cm=return_dist,
    )


def main() -> int:
    result = run_patrol_scenario()
    ok = (
        result.perimeter_ok
        and result.robot_spawned
        and result.outbound_arrived
        and result.return_arrived
        and result.return_dist_cm <= RETURN_ARRIVE_TOLERANCE_CM
    )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
