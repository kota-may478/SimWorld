"""
2D costmap + grid A* for material transport navigation.

Costmap frame:
  - origin_xy is the world (UE) position of the map minimum-X/minimum-Y corner.
  - With MAP_WORLD_ORIGIN_XY (lower-left), the map center is at origin + (size/2, size/2).
  - The map extends +X and +Y for size_m meters.
  - Cell size resolution_cm (default 10 cm).
  - costs[gy, gx]: row gy along +Y, column gx along +X.
"""
from __future__ import annotations

import heapq
import math
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap


def _matplotlib_show_enabled() -> bool:
    backend = matplotlib.get_backend().lower()
    return backend not in {"agg", "svg", "pdf", "ps", "cairo"}

GridCell = Tuple[int, int]
WorldXY = Tuple[float, float]

COSTMAP_SIZE_M = 30.0
COSTMAP_RESOLUTION_CM = 10.0
COSTMAP_DEFAULT_CELL_COST = 1.0
COSTMAP_LETHAL_COST = 1.0e9

COSTMAP_ROBOT_COLOR = "lime"
COSTMAP_HUMANOID_COLOR = "forestgreen"
COSTMAP_WAYPOINT_COLOR = "black"
COSTMAP_WAYPOINT_SIZE = 14
COSTMAP_ROBOT_LINE_WIDTH = 2.2
COSTMAP_ROBOT_MARKER_SIZE = 72
COSTMAP_HUMANOID_MARKER_SIZE = 88


def _costmap_display_cmap() -> LinearSegmentedColormap:
    """低コストを白、高コストを赤系にするカラーマップ。"""
    return LinearSegmentedColormap.from_list(
        "costmap_white_red",
        ["#ffffff", "#fff5f0", "#fcbba1", "#fb6a4a", "#cb181d"],
    )


def collect_planned_waypoints(legs: Sequence[PathLegVisualization]) -> List[WorldXY]:
    waypoints: List[WorldXY] = []
    for leg in legs:
        waypoints.extend(leg.plan.waypoints_xy)
    return waypoints


def draw_costmap_visualization(
    ax: plt.Axes,
    costmap: Costmap2D,
    *,
    planned_legs: Sequence[PathLegVisualization] = (),
    traveled_xy: Sequence[WorldXY] = (),
    human_xy: Optional[WorldXY] = None,
    title: str = "Costmap (Y horizontal, X vertical)",
    show_colorbar: bool = False,
) -> None:
    """白背景・ライム実線軌跡・黒WP・Humanoid 凡例付きでコストマップを描画。"""
    ax.clear()
    ax.set_facecolor("white")

    extent = costmap.plot_extent_y_horizontal()
    cost_display = costmap.costs_for_plot_y_horizontal()
    cost_min = float(np.min(cost_display)) if cost_display.size else 0.0
    cost_max = float(np.max(cost_display)) if cost_display.size else 1.0
    if cost_max <= cost_min:
        cost_max = cost_min + 1.0

    image = ax.imshow(
        cost_display,
        origin="lower",
        extent=extent,
        cmap=_costmap_display_cmap(),
        vmin=cost_min,
        vmax=cost_max,
        aspect="equal",
        interpolation="nearest",
    )

    for leg in planned_legs:
        if len(leg.plan.waypoints_xy) < 2:
            continue
        leg_plot = [world_xy_to_plot_xy(point) for point in leg.plan.waypoints_xy]
        ax.plot(
            [point[0] for point in leg_plot],
            [point[1] for point in leg_plot],
            "--",
            color=leg.color,
            lw=1.2,
            alpha=0.55,
            zorder=4,
        )

    planned_waypoints = collect_planned_waypoints(planned_legs)
    if planned_waypoints:
        waypoint_plot = [world_xy_to_plot_xy(point) for point in planned_waypoints]
        ax.scatter(
            [point[0] for point in waypoint_plot],
            [point[1] for point in waypoint_plot],
            c=COSTMAP_WAYPOINT_COLOR,
            s=COSTMAP_WAYPOINT_SIZE,
            zorder=5,
            label="Waypoint",
        )

    if traveled_xy:
        traveled_plot = [world_xy_to_plot_xy(point) for point in traveled_xy]
        ax.plot(
            [point[0] for point in traveled_plot],
            [point[1] for point in traveled_plot],
            "-",
            color=COSTMAP_ROBOT_COLOR,
            lw=COSTMAP_ROBOT_LINE_WIDTH,
            zorder=6,
            label="Traveled",
        )
        current_plot = traveled_plot[-1]
        ax.scatter(
            current_plot[0],
            current_plot[1],
            c=COSTMAP_ROBOT_COLOR,
            s=COSTMAP_ROBOT_MARKER_SIZE,
            edgecolors="black",
            linewidths=0.5,
            zorder=8,
            label="Robot",
        )

    if human_xy is not None:
        human_plot = world_xy_to_plot_xy(human_xy)
        ax.scatter(
            human_plot[0],
            human_plot[1],
            c=COSTMAP_HUMANOID_COLOR,
            marker="s",
            s=COSTMAP_HUMANOID_MARKER_SIZE,
            edgecolors="black",
            linewidths=0.5,
            zorder=7,
            label="Humanoid",
        )

    apply_fixed_costmap_axes(ax, costmap)

    ax.set_xlabel("Y [cm]")
    ax.set_ylabel("X [cm]")
    ax.set_title(title)
    if show_colorbar:
        ax.figure.colorbar(image, ax=ax, fraction=0.046, pad=0.04, label="Cell cost")

    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(loc="upper right", fontsize=8)

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

    def plot_extent_y_horizontal(self) -> Tuple[float, float, float, float]:
        """imshow extent when horizontal axis=Y and vertical axis=X: [y0, y1, x0, x1]."""
        x0, x1, y0, y1 = self.matplotlib_extent()
        return (y0, y1, x0, x1)

    def costs_for_plot_y_horizontal(self) -> np.ndarray:
        """Display array for Y-horizontal / X-vertical plot (transpose of costs)."""
        return self.costs.T


def world_xy_to_plot_xy(world_xy: WorldXY) -> Tuple[float, float]:
    """Matplotlib 座標: 横軸=Y [cm], 縦軸=X [cm]（UE 水平面）。"""
    return (world_xy[1], world_xy[0])


def apply_fixed_costmap_axes(ax: plt.Axes, costmap: Costmap2D) -> None:
    """全フレームで同一の表示範囲・アスペクト比を維持する。"""
    y_min, y_max, x_min, x_max = costmap.plot_extent_y_horizontal()
    ax.set_xlim(y_min, y_max)
    ax.set_ylim(x_min, x_max)
    ax.set_aspect("equal", adjustable="box")


def costmap_cell_count(size_m: float, resolution_cm: float) -> int:
    size_cm = size_m * 100.0
    return int(round(size_cm / resolution_cm))


def costmap_origin_for_centered_agent(
    agent_xy: WorldXY,
    size_m: float = COSTMAP_SIZE_M,
) -> WorldXY:
    """agent_xy がコストマップの幾何中心に来るよう origin（最小 XY 隅）を返す。"""
    half_cm = size_m * 50.0
    return (float(agent_xy[0]) - half_cm, float(agent_xy[1]) - half_cm)


def build_uniform_costmap(
    origin_xy: WorldXY,
    size_m: float = COSTMAP_SIZE_M,
    resolution_cm: float = COSTMAP_RESOLUTION_CM,
    default_cost: float = COSTMAP_DEFAULT_CELL_COST,
) -> Costmap2D:
    """origin を最小 XY 隅とした size_m 四方の均一コストマップ。"""
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
    traveled_xy: Optional[Sequence[WorldXY]] = None,
) -> plt.Figure:
    """コストマップ上に経路と総コストを表示（横軸=Y, 縦軸=X）。"""
    fig, ax = plt.subplots(figsize=(9, 8), facecolor="white")
    draw_costmap_visualization(
        ax,
        costmap,
        planned_legs=legs,
        traveled_xy=traveled_xy or [],
        human_xy=costmap.origin_xy,
        title=title,
        show_colorbar=True,
    )
    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150, facecolor="white")
    if show and _matplotlib_show_enabled():
        plt.show()
    else:
        plt.close(fig)
    return fig


LIVE_COSTMAP_FIGSIZE = (9.0, 8.0)
LIVE_COSTMAP_DPI = 120
LIVE_COSTMAP_SUBPLOTS = {
    "left": 0.10,
    "right": 0.98,
    "top": 0.94,
    "bottom": 0.08,
}


@dataclass
class LiveCostmapVisualizer:
    """
    コストマップ上にロボット軌跡をリアルタイム表示し、フレーム PNG → MP4/GIF を生成する。

    横軸=Y [cm], 縦軸=X [cm]。更新間隔は RECORD_INTERVAL 等と揃える想定。
    """

    costmap: Costmap2D
    output_dir: Path
    update_interval_s: float = 0.3
    live_window: bool = True
    delete_frames_after_video: bool = True
    planned_legs: List[PathLegVisualization] = field(default_factory=list)
    traveled_xy: List[WorldXY] = field(default_factory=list)
    human_xy: Optional[WorldXY] = None
    _frame_paths: List[Path] = field(default_factory=list, init=False, repr=False)
    _last_update_ts: float = field(default_factory=time.time, init=False, repr=False)
    _fig: Optional[plt.Figure] = field(default=None, init=False, repr=False)
    _ax: Optional[plt.Axes] = field(default=None, init=False, repr=False)
    _draw_lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)

    def __post_init__(self) -> None:
        self.output_dir = Path(self.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        clear_intermediate_frame_pngs(self.output_dir)
        self.live_window = self.live_window and _matplotlib_show_enabled()
        if not self.live_window:
            matplotlib.use("Agg")
        elif self.live_window:
            plt.ion()
        self._fig, self._ax = plt.subplots(figsize=LIVE_COSTMAP_FIGSIZE, facecolor="white")
        if self._fig is not None:
            self._fig.set_dpi(LIVE_COSTMAP_DPI)
        self._apply_figure_layout()
        with self._draw_lock:
            self._draw_frame(save=False)

    def set_planned_legs(self, legs: Sequence[PathLegVisualization]) -> None:
        self.planned_legs = list(legs)

    def set_human_xy(self, human_xy: Optional[WorldXY]) -> None:
        self.human_xy = human_xy

    def _apply_figure_layout(self) -> None:
        if self._fig is not None:
            self._fig.subplots_adjust(**LIVE_COSTMAP_SUBPLOTS)

    def redraw_current(self, *, save: bool = False) -> None:
        """軌跡を増やさず現在状態で再描画（計画 WP 追加直後など）。"""
        with self._draw_lock:
            self._draw_frame(save=save)

    def maybe_update(
        self,
        robot_xy: WorldXY,
        *,
        human_xy: Optional[WorldXY] = None,
        force: bool = False,
    ) -> None:
        """update_interval_s ごとに表示更新とフレーム保存。"""
        if human_xy is not None:
            self.human_xy = human_xy
        self.traveled_xy.append(robot_xy)
        now = time.time()
        if not force and (now - self._last_update_ts) < self.update_interval_s:
            return
        self._last_update_ts = now
        with self._draw_lock:
            self._draw_frame(save=True)

    def finalize(self) -> dict:
        """最終フレームを保存し、MP4/GIF を生成。必要なら PNG を削除。"""
        with self._draw_lock:
            if self.traveled_xy:
                self._draw_frame(save=True)
            elif self._ax is not None:
                self._draw_frame(save=True)

        mp4_path = self.output_dir / "costmap_live.mp4"
        gif_path = self.output_dir / "costmap_live.gif"
        video_paths = export_frames_to_videos(
            self._frame_paths,
            mp4_path=mp4_path,
            gif_path=gif_path,
            fps=max(1.0, 1.0 / self.update_interval_s),
            delete_frames_after=self.delete_frames_after_video,
        )
        self._frame_paths.clear()

        if self.live_window:
            plt.ioff()
        if self._fig is not None:
            plt.close(self._fig)
            self._fig = None
            self._ax = None

        return {
            "frame_count": len(video_paths.get("frames_used", [])),
            "mp4": str(video_paths.get("mp4", "")),
            "gif": str(video_paths.get("gif", "")),
            "frames_deleted": video_paths.get("frames_deleted", 0),
        }

    def _draw_frame(self, *, save: bool) -> None:
        if self._ax is None or self._fig is None:
            return
        try:
            if self._fig.canvas is None:
                return
        except (AttributeError, RuntimeError):
            return
        draw_costmap_visualization(
            self._ax,
            self.costmap,
            planned_legs=self.planned_legs,
            traveled_xy=self.traveled_xy,
            human_xy=self.human_xy,
            title="Live costmap (Y horizontal, X vertical)",
            show_colorbar=False,
        )

        self._apply_figure_layout()

        if save:
            frame_path = self.output_dir / f"frame_{len(self._frame_paths) + 1:05d}.png"
            backend = matplotlib.get_backend().lower()
            if backend != "agg":
                try:
                    self._fig.canvas.draw()
                except (AttributeError, RuntimeError) as exc:
                    print(f"[LiveCostmap] canvas draw skipped: {exc}")
                    return
            try:
                self._fig.savefig(
                    frame_path,
                    dpi=LIVE_COSTMAP_DPI,
                    facecolor="white",
                    bbox_inches=None,
                    pad_inches=0.05,
                )
            except (AttributeError, RuntimeError) as exc:
                print(f"[LiveCostmap] savefig skipped: {exc}")
                return
            self._frame_paths.append(frame_path)

        if self.live_window and self._fig.canvas is not None:
            try:
                self._fig.canvas.draw_idle()
                self._fig.canvas.flush_events()
            except (AttributeError, RuntimeError):
                pass
            plt.pause(0.001)


def clear_intermediate_frame_pngs(directory: Path) -> int:
    """live_costmap 内の frame_*.png を削除（再実行時の残骸対策）。"""
    deleted = 0
    for frame_path in sorted(Path(directory).glob("frame_*.png")):
        frame_path.unlink(missing_ok=True)
        deleted += 1
    return deleted


def delete_intermediate_frame_pngs(frame_paths: Sequence[Path]) -> int:
    """動画化に使った中間 PNG を削除する。"""
    deleted = 0
    for frame_path in frame_paths:
        path = Path(frame_path)
        if path.name.startswith("frame_") and path.suffix.lower() == ".png":
            if path.exists():
                path.unlink()
                deleted += 1
    return deleted


def _export_gif_with_pillow(frame_paths: Sequence[Path], gif_path: Path, fps: float) -> bool:
    from PIL import Image

    images = [Image.open(path) for path in frame_paths]
    duration_ms = max(1, int(1000.0 / fps))
    images[0].save(
        gif_path,
        save_all=True,
        append_images=images[1:],
        duration=duration_ms,
        loop=0,
    )
    return True


def _export_mp4_with_ffmpeg(frame_paths: Sequence[Path], mp4_path: Path, fps: float) -> bool:
    if not frame_paths:
        return False
    first = Path(frame_paths[0])
    pattern = str(first.parent / "frame_%05d.png")
    command = [
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-framerate",
        str(fps),
        "-i",
        pattern,
        "-pix_fmt",
        "yuv420p",
        str(mp4_path),
    ]
    try:
        completed = subprocess.run(command, capture_output=True, text=True, check=False)
    except FileNotFoundError:
        return False
    if completed.returncode != 0:
        print(f"[LiveCostmap] ffmpeg failed: {completed.stderr.strip()}")
        return False
    return True


def export_frames_to_videos(
    frame_paths: Sequence[Path],
    *,
    mp4_path: Path,
    gif_path: Path,
    fps: float,
    delete_frames_after: bool = True,
) -> dict:
    """PNG フレーム列から MP4 と GIF を生成し、成功後に中間 PNG を削除する。"""
    paths = [Path(path) for path in frame_paths]
    result: dict = {
        "frames_used": [str(path) for path in paths],
        "gif_ok": False,
        "mp4_ok": False,
        "frames_deleted": 0,
    }
    if not paths:
        print("[LiveCostmap] No frames to export.")
        return result

    mp4_path.parent.mkdir(parents=True, exist_ok=True)
    gif_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        if _export_gif_with_pillow(paths, gif_path, fps):
            result["gif"] = gif_path
            result["gif_ok"] = True
            print(f"[LiveCostmap] GIF saved: {gif_path}")
    except Exception as exc:
        print(f"[LiveCostmap] GIF export failed ({exc}).")

    if _export_mp4_with_ffmpeg(paths, mp4_path, fps):
        result["mp4"] = mp4_path
        result["mp4_ok"] = True
        print(f"[LiveCostmap] MP4 saved: {mp4_path}")
    else:
        try:
            import imageio

            frames = [imageio.imread(path) for path in paths]
            imageio.mimsave(str(mp4_path), frames, fps=fps)
            result["mp4"] = mp4_path
            result["mp4_ok"] = True
            print(f"[LiveCostmap] MP4 saved via imageio: {mp4_path}")
        except Exception as exc:
            print(
                f"[LiveCostmap] MP4 export failed ({exc}). "
                "Install ffmpeg or: pip install imageio[ffmpeg]"
            )

    if delete_frames_after and (result["gif_ok"] or result["mp4_ok"]):
        deleted_count = delete_intermediate_frame_pngs(paths)
        result["frames_deleted"] = deleted_count
        print(f"[LiveCostmap] Deleted {deleted_count} intermediate frame PNG(s).")
    elif delete_frames_after:
        print("[LiveCostmap] Keeping frame PNGs (GIF and MP4 export both failed).")

    return result
