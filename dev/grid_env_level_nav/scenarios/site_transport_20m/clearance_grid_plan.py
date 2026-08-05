#!/usr/bin/env python3
"""Grid A* planner with center-to-AABB clearance and open-corridor preference."""

from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

from level_coords import REGION_ORIGIN_WORLD_XY, local_xy_to_world, world_xy_to_local
from region import REGION_SIZE_CM
from surface_distance import SurfaceObstacle, center_to_aabb_surface_distance_cm

WorldXY = Tuple[float, float]
GridCell = Tuple[int, int]


def min_center_clearance_at(
    wx: float,
    wy: float,
    obstacles: Sequence[SurfaceObstacle],
) -> float:
    if not obstacles:
        return float("inf")
    return min(
        center_to_aabb_surface_distance_cm((wx, wy), obstacle)
        for obstacle in obstacles
    )


def clearance_hug_penalty_cm(
    clearance_cm: float,
    *,
    min_clearance_cm: float,
    hug_penalty_weight: float,
    open_margin_cm: float = 250.0,
) -> float:
    """Penalty for steps that skim the minimum-clearance band (prop outer perimeter)."""
    if hug_penalty_weight <= 0.0:
        return 0.0
    margin = clearance_cm - min_clearance_cm
    if margin < 0.0:
        return float("inf")
    if margin >= open_margin_cm:
        return 0.0
    return hug_penalty_weight * hug_penalty_weight / max(margin, 8.0)


def _cell_center_world(col: int, row: int, *, resolution_cm: float) -> WorldXY:
    ox, oy = REGION_ORIGIN_WORLD_XY
    lx = (col + 0.5) * resolution_cm
    ly = (row + 0.5) * resolution_cm
    return local_xy_to_world(lx, ly)


def _world_to_cell(wx: float, wy: float, *, resolution_cm: float) -> GridCell:
    lx, ly = world_xy_to_local(wx, wy)
    col = int(math.floor(lx / resolution_cm))
    row = int(math.floor(ly / resolution_cm))
    return col, row


def _cell_blocked(
    col: int,
    row: int,
    obstacles: Sequence[SurfaceObstacle],
    *,
    center_clearance_cm: float,
    resolution_cm: float,
    block_margin_cm: float = 0.0,
) -> bool:
    wx, wy = _cell_center_world(col, row, resolution_cm=resolution_cm)
    return (
        min_center_clearance_at(wx, wy, obstacles)
        < center_clearance_cm + block_margin_cm
    )


def _neighbors(col: int, row: int, cols: int, rows: int) -> List[GridCell]:
    out: List[GridCell] = []
    for dc, dr in (
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
    ):
        nc, nr = col + dc, row + dr
        if 0 <= nc < cols and 0 <= nr < rows:
            out.append((nc, nr))
    return out


def _snap_to_free_cell(
    cell: GridCell,
    obstacles: Sequence[SurfaceObstacle],
    *,
    center_clearance_cm: float,
    resolution_cm: float,
    cols: int,
    rows: int,
    block_margin_cm: float = 0.0,
    max_radius: int = 50,
) -> Optional[GridCell]:
    if not _cell_blocked(
        cell[0],
        cell[1],
        obstacles,
        center_clearance_cm=center_clearance_cm,
        resolution_cm=resolution_cm,
        block_margin_cm=block_margin_cm,
    ):
        return cell
    for radius in range(1, max_radius + 1):
        for dc in range(-radius, radius + 1):
            for dr in range(-radius, radius + 1):
                if abs(dc) != radius and abs(dr) != radius:
                    continue
                nc, nr = cell[0] + dc, cell[1] + dr
                if not (0 <= nc < cols and 0 <= nr < rows):
                    continue
                if not _cell_blocked(
                    nc,
                    nr,
                    obstacles,
                    center_clearance_cm=center_clearance_cm,
                    resolution_cm=resolution_cm,
                    block_margin_cm=block_margin_cm,
                ):
                    return nc, nr
    return None


def plan_clearance_grid_waypoints(
    start_xy: WorldXY,
    goal_xy: WorldXY,
    obstacles: Sequence[SurfaceObstacle],
    *,
    center_clearance_cm: float,
    resolution_cm: float = 40.0,
    block_margin_cm: float = 0.0,
    hug_penalty_weight: float = 0.0,
    hug_open_margin_cm: float = 250.0,
) -> List[WorldXY]:
    """A* on work-region grid; prefers open corridors over prop perimeter skating."""
    if not obstacles:
        return [start_xy, goal_xy]

    cols = max(1, int(math.ceil(REGION_SIZE_CM / resolution_cm)))
    rows = cols
    start_cell = _world_to_cell(start_xy[0], start_xy[1], resolution_cm=resolution_cm)
    goal_cell = _world_to_cell(goal_xy[0], goal_xy[1], resolution_cm=resolution_cm)

    start_cell = _snap_to_free_cell(
        start_cell,
        obstacles,
        center_clearance_cm=center_clearance_cm,
        resolution_cm=resolution_cm,
        cols=cols,
        rows=rows,
        block_margin_cm=block_margin_cm,
    )
    goal_cell = _snap_to_free_cell(
        goal_cell,
        obstacles,
        center_clearance_cm=center_clearance_cm,
        resolution_cm=resolution_cm,
        cols=cols,
        rows=rows,
        block_margin_cm=block_margin_cm,
    )
    if start_cell is None or goal_cell is None:
        return []

    def in_bounds(cell: GridCell) -> bool:
        c, r = cell
        return 0 <= c < cols and 0 <= r < rows

    if not in_bounds(start_cell) or not in_bounds(goal_cell):
        return []

    def heuristic(cell: GridCell) -> float:
        wx, wy = _cell_center_world(cell[0], cell[1], resolution_cm=resolution_cm)
        return math.hypot(wx - goal_xy[0], wy - goal_xy[1])

    open_heap: List[Tuple[float, GridCell]] = [(heuristic(start_cell), start_cell)]
    came_from: Dict[GridCell, GridCell] = {}
    g_score: Dict[GridCell, float] = {start_cell: 0.0}
    closed: set[GridCell] = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal_cell:
            path_cells: List[GridCell] = [current]
            while current in came_from:
                current = came_from[current]
                path_cells.append(current)
            path_cells.reverse()
            return [
                _cell_center_world(c, r, resolution_cm=resolution_cm)
                for c, r in path_cells
            ]

        closed.add(current)
        cx, cy = _cell_center_world(current[0], current[1], resolution_cm=resolution_cm)
        for nxt in _neighbors(current[0], current[1], cols, rows):
            if nxt in closed:
                continue
            if _cell_blocked(
                nxt[0],
                nxt[1],
                obstacles,
                center_clearance_cm=center_clearance_cm,
                resolution_cm=resolution_cm,
                block_margin_cm=block_margin_cm,
            ):
                continue
            nx, ny = _cell_center_world(nxt[0], nxt[1], resolution_cm=resolution_cm)
            step = math.hypot(nx - cx, ny - cy)
            clearance = min_center_clearance_at(nx, ny, obstacles)
            hug = clearance_hug_penalty_cm(
                clearance,
                min_clearance_cm=center_clearance_cm,
                hug_penalty_weight=hug_penalty_weight,
                open_margin_cm=hug_open_margin_cm,
            )
            if math.isinf(hug):
                continue
            tentative = g_score[current] + step + hug
            if tentative >= g_score.get(nxt, float("inf")):
                continue
            came_from[nxt] = current
            g_score[nxt] = tentative
            heapq.heappush(open_heap, (tentative + heuristic(nxt), nxt))

    return []


def mean_path_clearance_cm(
    path: Sequence[WorldXY],
    obstacles: Sequence[SurfaceObstacle],
) -> float:
    if not path or not obstacles:
        return float("inf")
    values = [min_center_clearance_at(wx, wy, obstacles) for wx, wy in path]
    return sum(values) / len(values)
