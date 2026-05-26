#!/usr/bin/env python3
"""Costmap viz style smoke test (no UE)."""
from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ["MPLBACKEND"] = "Agg"

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import matplotlib.pyplot as plt
import numpy as np

from path_planning_costmap import (
    AStarPlanResult,
    Costmap2D,
    LiveCostmapVisualizer,
    PathLegVisualization,
    apply_fixed_costmap_axes,
    draw_costmap_visualization,
    plot_costmap_with_paths,
)

OUT = Path(__file__).resolve().parent / "live_costmap_test_run"
OUT.mkdir(parents=True, exist_ok=True)

origin = (100.0, 200.0)
costs = np.ones((300, 300), dtype=np.float64)
costmap = Costmap2D(costs=costs, origin_xy=origin)

plan = AStarPlanResult(
    waypoints_xy=[
        (origin[0] + 500, origin[1] + 300),
        (origin[0] + 1200, origin[1] + 800),
        (origin[0] + 2000, origin[1] + 1500),
    ],
    grid_path=[],
    total_cost=42.0,
    start_xy=(origin[0] + 100, origin[1] + 100),
    goal_xy=(origin[0] + 2000, origin[1] + 1500),
)
leg = PathLegVisualization(label="to material", plan=plan, color="blue")

traveled = [
    (origin[0] + 100, origin[1] + 100),
    (origin[0] + 400, origin[1] + 250),
    (origin[0] + 900, origin[1] + 600),
]

def test_fixed_axes_across_frames() -> None:
    origin = (100.0, 200.0)
    costs = np.ones((300, 300), dtype=np.float64)
    costmap = Costmap2D(costs=costs, origin_xy=origin)
    fig, ax = plt.subplots(figsize=(9, 8), facecolor="white")
    draw_costmap_visualization(ax, costmap, traveled_xy=[])
    lim_empty = (ax.get_xlim(), ax.get_ylim())
    ax.clear()
    traveled = [
        (origin[0] + 500, origin[1] + 300),
        (origin[0] + 2500, origin[1] + 2500),
    ]
    draw_costmap_visualization(ax, costmap, traveled_xy=traveled)
    lim_full = (ax.get_xlim(), ax.get_ylim())
    plt.close(fig)
    assert lim_empty == lim_full, f"axis limits changed: {lim_empty} vs {lim_full}"


test_fixed_axes_across_frames()

static_png = OUT / "costmap_static.png"
plot_costmap_with_paths(
    costmap,
    [leg],
    traveled_xy=traveled,
    save_path=str(static_png),
    show=False,
    title="Style test",
)

viz = LiveCostmapVisualizer(
    costmap=costmap,
    output_dir=OUT,
    update_interval_s=0.05,
    live_window=False,
    delete_frames_after_video=True,
)
viz.set_planned_legs([leg])
viz.redraw_current(save=True)
viz.set_human_xy(origin)
for pose in traveled:
    viz.maybe_update(pose, human_xy=origin, force=True)
result = viz.finalize()

fig_check, ax_check = plt.subplots()
drawn = plt.imread(static_png)
assert static_png.exists()
assert Path(result.get("mp4", "")).exists() or Path(result.get("gif", "")).exists()

# Re-open static with legend inspection via draw helper
from path_planning_costmap import draw_costmap_visualization

fig, ax = plt.subplots(facecolor="white")
draw_costmap_visualization(
    ax,
    costmap,
    planned_legs=[leg],
    traveled_xy=traveled,
    human_xy=origin,
)
labels = [text.get_text() for text in ax.get_legend().get_texts()]
required = {"Waypoint", "Traveled", "Robot", "Humanoid"}
missing = required - set(labels)
if missing:
    raise SystemExit(f"Missing legend entries: {missing}; got {labels}")

print("OK costmap viz style test")
print(f"  static: {static_png}")
print(f"  live:   {result}")
