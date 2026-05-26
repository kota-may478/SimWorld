"""
2D costmap + grid A* for material transport navigation.

Costmap frame:
  - origin_xy is the world (UE) position of the map corner (humanoid anchor).
  - The map extends +X and +Y for size_m meters.
  - Cell size resolution_cm (default 10 cm).
  - costs[gy, gx]: row gy along +Y, column gx along +X.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np

GridCell = Tuple[int, int]
WorldXY = Tuple[float, float]

COSTMAP_SIZE_M = 30.0
COSTMAP_RESOLUTION_CM = 10.0
COSTMAP_DEFAULT_CELL_COST = 1.0
COSTMAP_LETHAL_COST = 1.0e9

_NEIGHBOR_OFFSETS: Tuple[Tuple[int, int, float], ...] = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (-1, -1, math.sqrt(2.0)),
)


@dataclass(frozen=True)
class Costmap2D:
    """Fixed 2D cost grid in UE horizontal plane (cm)."""

    costs: np.ndarray
    origin_xy: WorldXY
    resolution_cm: float = COSTMAP_RESOLUTION_CM
    lethal_cost: float = COSTMAP_LETHAL_COST

    @property
    def height_cells(self) -> int:
        return int(self.costs.shape[0])

    @property
    def width_cells(self) -> int:
        return int(self.costs.shape[1])

    @property
    def size_x_cm(self) -> float:
        return self.width_cells * self.resolution_cm

    @property
    def size_y_cm(self) -> float:
        return self.height_cells * self.resolution_cm

    def contains_world_xy(self, world_xy: WorldXY) -> bool:
        grid = self.world_xy_to_grid(world_xy, clamp=False)
        return grid is not None

    def world_xy_to_grid(
        self,
        world_xy: WorldXY,
        clamp: bool = True,
    ) -> Optional[GridCell]:
        gx = int(math.floor((world_xy[0] - self.origin_xy[0]) / self.resolution_cm))
        gy = int(math.floor((world_xy[1] - self.origin_xy[1]) / self.resolution_cm))
        if clamp:
            gx = min(max(gx, 0), self.width_cells - 1)
            gy = min(max(gy, 0), self.height_cells - 1)
            return (gx, gy)
        if gx < 0 or gy < 0 or gx >= self.width_cells or gy >= self.height_cells:
            return None
        return (gx, gy)

    def grid_to_world_xy_center(self, grid_xy: GridCell) -> WorldXY:
        gx, gy = grid_xy
        return (
            self.origin_xy[0] + (gx + 0.5) * self.resolution_cm,
            self.origin_xy[1] + (gy + 0.5) * self.resolution_cm,
        )

    def cell_cost(self, grid_xy: GridCell) -> float:
        gx, gy = grid_xy
        return float(self.costs[gy, gx])

    def is_traversable(self, grid_xy: GridCell) -> bool:
        return self.cell_cost(grid_xy) < self.lethal_cost

    def matplotlib_extent(self) -> Tuple[float, float, float, float]:
        """extent for imshow(origin='lower'): [x0, x1, y0, y1] in world cm."""
        x0 = self.origin_xy[0]
        y0 = self.origin_xy[1]
        return (x0, x0 + self.size_x_cm, y0, y0 + self.size_y_cm)


def costmap_cell_count(size_m: float, resolution_cm: float) -> int:
    size_cm = size_m * 100.0
    return int(round(size_cm / resolution_cm))


def build_uniform_costmap(
    origin_xy: WorldXY,
    size_m: float = COSTMAP_SIZE_M,
    resolution_cm: float = COSTMAP_RESOLUTION_CM,
    default_cost: float = COSTMAP_DEFAULT_CELL_COST,
) -> Costmap2D:
    """Humanoid 付近を原点とした size_m 四方の均一コストマップ。"""
    cells = costmap_cell_count(size_m, resolution_cm)
    costs = np.full((cells, cells), default_cost, dtype=np.float32)
    return Costmap2D(costs=costs, origin_xy=origin_xy, resolution_cm=resolution_cm)


def costmap_from_array(
    costs: np.ndarray,
    origin_xy: WorldXY,
    resolution_cm: float = COSTMAP_RESOLUTION_CM,
    lethal_cost: float = COSTMAP_LETHAL_COST,
) -> Costmap2D:
    """
    外部 2D 配列から Costmap2D を構築する。

    costs.shape = (height_cells, width_cells), 行=+Y, 列=+X。
    """
    array = np.asarray(costs, dtype=np.float32)
    if array.ndim != 2:
        raise ValueError("costs must be a 2D array")
    return Costmap2D(
        costs=array,
        origin_xy=origin_xy,
        resolution_cm=resolution_cm,
        lethal_cost=lethal_cost,
    )


def add_circular_cost_region(
    costmap: Costmap2D,
    center_xy: WorldXY,
    radius_cm: float,
    cost_value: float,
    *,
    replace: bool = False,
) -> None:
    """円形領域に高コスト（または任意コスト）を書き込む。"""
    grid_center = costmap.world_xy_to_grid(center_xy, clamp=True)
    if grid_center is None:
        return
    radius_cells = int(math.ceil(radius_cm / costmap.resolution_cm))
    cx, cy = grid_center
    for gy in range(max(0, cy - radius_cells), min(costmap.height_cells, cy + radius_cells + 1)):
        for gx in range(max(0, cx - radius_cells), min(costmap.width_cells, cx + radius_cells + 1)):
            wx, wy = costmap.grid_to_world_xy_center((gx, gy))
            if math.hypot(wx - center_xy[0], wy - center_xy[1]) <= radius_cm:
                if replace:
                    costmap.costs[gy, gx] = cost_value
                else:
                    costmap.costs[gy, gx] = max(float(costmap.costs[gy, gx]), cost_value)


def _edge_traversal_cost(costmap: Costmap2D, from_cell: GridCell, to_cell: GridCell) -> float:
    if not costmap.is_traversable(from_cell) or not costmap.is_traversable(to_cell):
        return math.inf
    step_cells = math.hypot(to_cell[0] - from_cell[0], to_cell[1] - from_cell[1])
    step_cm = step_cells * costmap.resolution_cm
    cost_from = costmap.cell_cost(from_cell)
    cost_to = costmap.cell_cost(to_cell)
    return step_cm * 0.5 * (cost_from + cost_to)


def _heuristic_cost(costmap: Costmap2D, cell: GridCell, goal_cell: GridCell) -> float:
    wx, wy = costmap.grid_to_world_xy_center(cell)
    gx, gy = costmap.grid_to_world_xy_center(goal_cell)
    return math.hypot(gx - wx, gy - wy)


@dataclass(frozen=True)
class AStarPlanResult:
    waypoints_xy: List[WorldXY]
    grid_path: List[GridCell]
    total_cost: float
    start_xy: WorldXY
    goal_xy: WorldXY


def astar_grid_path(
    costmap: Costmap2D,
    start_xy: WorldXY,
    goal_xy: WorldXY,
) -> AStarPlanResult:
    """コスト付き格子 A*。経路コストは辺コスト（距離×平均セルコスト）の総和。"""
    start_cell = costmap.world_xy_to_grid(start_xy, clamp=True)
    goal_cell = costmap.world_xy_to_grid(goal_xy, clamp=True)
    assert start_cell is not None and goal_cell is not None

    if not costmap.is_traversable(start_cell) or not costmap.is_traversable(goal_cell):
        raise ValueError("Start or goal lies on a lethal cost cell.")

    if start_cell == goal_cell:
        waypoint = costmap.grid_to_world_xy_center(goal_cell)
        return AStarPlanResult(
            waypoints_xy=[waypoint],
            grid_path=[start_cell],
            total_cost=0.0,
            start_xy=start_xy,
            goal_xy=goal_xy,
        )

    open_heap: List[Tuple[float, GridCell]] = []
    heapq.heappush(open_heap, (0.0, start_cell))
    came_from: dict[GridCell, Optional[GridCell]] = {start_cell: None}
    g_score: dict[GridCell, float] = {start_cell: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal_cell:
            grid_path = _reconstruct_grid_path(came_from, goal_cell)
            total_cost = compute_grid_path_total_cost(costmap, grid_path)
            waypoints_xy = simplify_world_path(
                [costmap.grid_to_world_xy_center(cell) for cell in grid_path]
            )
            return AStarPlanResult(
                waypoints_xy=waypoints_xy,
                grid_path=grid_path,
                total_cost=total_cost,
                start_xy=start_xy,
                goal_xy=goal_xy,
            )

        current_g = g_score[current]
        for dx, dy, _step_scale in _NEIGHBOR_OFFSETS:
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor[0] < 0 or neighbor[1] < 0:
                continue
            if neighbor[0] >= costmap.width_cells or neighbor[1] >= costmap.height_cells:
                continue
            edge_cost = _edge_traversal_cost(costmap, current, neighbor)
            if not math.isfinite(edge_cost):
                continue
            tentative_g = current_g + edge_cost
            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + _heuristic_cost(costmap, neighbor, goal_cell)
                heapq.heappush(open_heap, (f_score, neighbor))

    raise RuntimeError(f"A* failed: no path from {start_xy} to {goal_xy}")


def _reconstruct_grid_path(
    came_from: dict[GridCell, Optional[GridCell]],
    goal_cell: GridCell,
) -> List[GridCell]:
    path: List[GridCell] = []
    cell: Optional[GridCell] = goal_cell
    while cell is not None:
        path.append(cell)
        cell = came_from.get(cell)
    path.reverse()
    return path


def compute_grid_path_total_cost(
    costmap: Costmap2D,
    grid_path: Sequence[GridCell],
) -> float:
    if len(grid_path) < 2:
        return 0.0
    total = 0.0
    for index in range(len(grid_path) - 1):
        total += _edge_traversal_cost(costmap, grid_path[index], grid_path[index + 1])
    return total


def simplify_world_path(path_xy: List[WorldXY]) -> List[WorldXY]:
    """共線な中間点を間引く。"""
    if len(path_xy) <= 2:
        return list(path_xy)

    simplified: List[WorldXY] = [path_xy[0]]
    for index in range(1, len(path_xy) - 1):
        prev_xy = simplified[-1]
        mid_xy = path_xy[index]
        next_xy = path_xy[index + 1]
        v1 = (mid_xy[0] - prev_xy[0], mid_xy[1] - prev_xy[1])
        v2 = (next_xy[0] - mid_xy[0], next_xy[1] - mid_xy[1])
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        if abs(cross) > 1e-6:
            simplified.append(mid_xy)
    simplified.append(path_xy[-1])
    return simplified


def subdivide_waypoints(
    waypoints_xy: List[WorldXY],
    max_segment_cm: float,
) -> List[WorldXY]:
    """長い辺を max_segment_cm 以下に分割（実行用 FB）。"""
    if not waypoints_xy:
        return []
    if len(waypoints_xy) == 1:
        return list(waypoints_xy)

    subdivided: List[WorldXY] = []
    for start, end in zip(waypoints_xy[:-1], waypoints_xy[1:]):
        segment_len = math.hypot(end[0] - start[0], end[1] - start[1])
        if segment_len <= max_segment_cm:
            if not subdivided:
                subdivided.append(start)
            subdivided.append(end)
            continue
        segment_count = max(1, int(math.ceil(segment_len / max_segment_cm)))
        if not subdivided:
            subdivided.append(start)
        for step in range(1, segment_count + 1):
            t = step / segment_count
            subdivided.append(
                (
                    start[0] + t * (end[0] - start[0]),
                    start[1] + t * (end[1] - start[1]),
                )
            )
    return subdivided


def plan_waypoints_grid_astar(
    costmap: Costmap2D,
    start_xy: WorldXY,
    goal_xy: WorldXY,
    max_segment_cm: float,
) -> AStarPlanResult:
    plan = astar_grid_path(costmap, start_xy, goal_xy)
    subdivided = subdivide_waypoints(plan.waypoints_xy, max_segment_cm)
    return AStarPlanResult(
        waypoints_xy=subdivided,
        grid_path=plan.grid_path,
        total_cost=plan.total_cost,
        start_xy=start_xy,
        goal_xy=goal_xy,
    )


@dataclass(frozen=True)
class PathLegVisualization:
    label: str
    plan: AStarPlanResult
    color: str


def plot_costmap_with_paths(
    costmap: Costmap2D,
    legs: Sequence[PathLegVisualization],
    *,
    title: str = "Costmap + Planned Paths",
    save_path: Optional[str] = None,
    show: bool = True,
) -> plt.Figure:
    """コストマップ上に経路と総コストを表示。"""
    fig, ax = plt.subplots(figsize=(9, 8))
    extent = costmap.matplotlib_extent()
    image = ax.imshow(
        costmap.costs,
        origin="lower",
        extent=extent,
        cmap="YlOrRd",
        aspect="equal",
        interpolation="nearest",
    )
    fig.colorbar(image, ax=ax, fraction=0.046, pad=0.04, label="Cell cost")

    for leg in legs:
        xs = [point[0] for point in leg.plan.waypoints_xy]
        ys = [point[1] for point in leg.plan.waypoints_xy]
        ax.plot(xs, ys, "-o", color=leg.color, lw=2, ms=4, label=leg.label)
        ax.scatter(
            leg.plan.start_xy[0],
            leg.plan.start_xy[1],
            color=leg.color,
            marker="s",
            s=60,
            zorder=5,
        )
        ax.scatter(
            leg.plan.goal_xy[0],
            leg.plan.goal_xy[1],
            color=leg.color,
            marker="*",
            s=140,
            zorder=5,
        )
        ax.annotate(
            f"{leg.label}\ncost={leg.plan.total_cost:.1f}",
            xy=leg.plan.goal_xy,
            xytext=(8, 8),
            textcoords="offset points",
            fontsize=8,
            color=leg.color,
        )

    ax.scatter(
        costmap.origin_xy[0],
        costmap.origin_xy[1],
        c="cyan",
        marker="X",
        s=120,
        zorder=6,
        label="Map origin (human corner)",
    )
    ax.set_xlabel("X [cm]")
    ax.set_ylabel("Y [cm]")
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150)
    if show:
        plt.show()
    return fig
