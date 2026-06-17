#!/usr/bin/env python3
"""Save compact-nav costmap and trajectory artifacts for post-run review."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import os

os.environ["MPLBACKEND"] = "Agg"
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="compact_nav")

from placement import CompactNavRegistry  # noqa: E402
from paths import COMPACT_NAV_RUN_DIR  # noqa: E402
from region import REGION_SIZE_CM  # noqa: E402
from costmap_layers import LayeredCostmap  # noqa: E402

THIS_DIR = Path(__file__).resolve().parent
DEFAULT_ARTIFACT_DIR = COMPACT_NAV_RUN_DIR

LocalXY = Tuple[float, float]


@dataclass
class NavTrace:
    trajectory_local_cm: List[LocalXY] = field(default_factory=list)
    planned_paths_local_cm: List[List[LocalXY]] = field(default_factory=list)
    replan_events: List[Dict[str, Any]] = field(default_factory=list)
    l2_estimate_local_cm: List[LocalXY] = field(default_factory=list)
    perception_samples: List[Any] = field(default_factory=list)
    l2_cell_count: int = 0
    arrived: bool = False

    def record_position(self, local_xy: LocalXY) -> None:
        if self.trajectory_local_cm and self.trajectory_local_cm[-1] == local_xy:
            return
        self.trajectory_local_cm.append(local_xy)

    def record_l2_estimate(self, local_xy: LocalXY) -> None:
        if local_xy not in self.l2_estimate_local_cm:
            self.l2_estimate_local_cm.append(local_xy)

    def record_plan(self, waypoints_world: Sequence[Tuple[float, float]], *, reason: str) -> None:
        from level_coords import world_xy_to_local  # noqa: WPS433

        path = [world_xy_to_local(wx, wy) for wx, wy in waypoints_world]
        self.planned_paths_local_cm.append(path)
        self.replan_events.append({"reason": reason, "waypoint_count": len(path)})


def save_compact_nav_artifacts(
    layers: LayeredCostmap,
    registry: CompactNavRegistry,
    trace: NavTrace,
    *,
    output_dir: Path = DEFAULT_ARTIFACT_DIR,
    placement_registry: Optional[Any] = None,
) -> Dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    paths: Dict[str, Path] = {}

    snap = layers.snapshot_layers()
    npz_path = output_dir / f"compact_nav_costmap_{stamp}.npz"
    np.savez_compressed(
        npz_path,
        l0=snap["l0"],
        l1=snap["l1"],
        l2=snap["l2"],
        merged=snap["merged"],
        resolution_cm=layers.resolution_cm,
        origin_xy=np.array(layers.origin_xy),
        lethal_cost=layers.lethal_cost,
    )
    paths["costmap_npz"] = npz_path

    png_path = output_dir / f"compact_nav_costmap_{stamp}.png"
    _save_costmap_png(layers, registry, trace, png_path)
    paths["costmap_png"] = png_path

    traj_path = output_dir / f"compact_nav_trajectory_{stamp}.json"
    traj_path.write_text(
        json.dumps(_trace_to_dict(registry, trace, layers), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    paths["trajectory_json"] = traj_path

    if trace.perception_samples and placement_registry is not None:
        rmse_paths = _save_perception_rmse_artifacts(
            trace,
            placement_registry,
            output_dir,
            stamp,
        )
        paths.update(rmse_paths)

    latest = _write_latest_symlinks(output_dir, paths, stamp)
    paths.update(latest)
    return paths


def _trace_to_dict(
    registry: CompactNavRegistry,
    trace: NavTrace,
    layers: LayeredCostmap,
) -> Dict[str, Any]:
    return {
        "arrived": trace.arrived,
        "l2_cell_count": trace.l2_cell_count,
        "trajectory_local_cm": [list(p) for p in trace.trajectory_local_cm],
        "trajectory_local_m": [[p[0] / 100.0, p[1] / 100.0] for p in trace.trajectory_local_cm],
        "planned_paths_local_cm": [[list(p) for p in path] for path in trace.planned_paths_local_cm],
        "replan_events": trace.replan_events,
        "robot_start_local_cm": list(registry.robot_start_local_cm),
        "goal_local_cm": list(registry.goal_local_cm),
        "props": [
            {
                "slot_id": p.slot_id,
                "bp_name": p.bp_name,
                "local_xy_cm": list(p.local_xy_cm),
                "local_xy_m": [p.local_xy_cm[0] / 100.0, p.local_xy_cm[1] / 100.0],
            }
            for p in registry.props
        ],
        "region_size_cm": REGION_SIZE_CM,
        "l2_nonzero_cells": int(np.count_nonzero(layers.l2)),
        "prop_marker_source": "registry_gt_local_xy",
        "l2_estimate_positions_local_cm": [list(p) for p in trace.l2_estimate_local_cm],
        "perception_sample_count": len(trace.perception_samples),
    }


def _save_costmap_png(
    layers: LayeredCostmap,
    registry: CompactNavRegistry,
    trace: NavTrace,
    output_path: Path,
) -> None:
    lethal = float(layers.lethal_cost)
    extent = [0.0, REGION_SIZE_CM, 0.0, REGION_SIZE_CM]
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.5))
    titles = ("L0 (NavMesh)", "L2 (FusionCam)", "Merged + path")
    arrays = (layers.l0, layers.l2, layers.merged_costs())

    for ax, title, arr in zip(axes, titles, arrays):
        display = np.where(arr >= lethal * 0.5, lethal, arr)
        # costs[gy, gx]: gy ∥ local X (UE Y), gx ∥ local Y (UE X) → transpose for imshow
        im = ax.imshow(
            display.T,
            origin="lower",
            extent=extent,
            cmap="RdYlGn_r",
            vmin=0,
            vmax=max(10.0, lethal),
        )
        if title.startswith("L2"):
            _overlay_props(ax, registry, gt=True)
            _overlay_l2_estimates(ax, trace)
        ax.set_title(title)
        ax.set_xlabel("local X (cm)")
        ax.set_ylabel("local Y (cm)")
        ax.set_aspect("equal")
        fig.colorbar(im, ax=ax, fraction=0.046)

    merged_ax = axes[2]
    _overlay_scene(merged_ax, registry, trace)
    fig.suptitle("Compact nav 30m × 30m — final costmap and SpotDog path")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def _overlay_props(ax, registry: CompactNavRegistry, *, gt: bool = False) -> None:
    suffix = " (GT)" if gt else ""
    for prop in registry.props:
        px, py = prop.local_xy_cm
        r, g, b = prop.mask_color_rgb
        ax.plot(px, py, "s", color=(r / 255.0, g / 255.0, b / 255.0), markersize=9)
        ax.annotate(
            f"{prop.prop_type_id}{suffix}",
            (px, py),
            textcoords="offset points",
            xytext=(5, 5),
            fontsize=7,
            color="black",
            bbox={"facecolor": "white", "alpha": 0.7, "pad": 1.5, "edgecolor": "none"},
        )


def _overlay_l2_estimates(ax, trace: NavTrace) -> None:
    if not trace.l2_estimate_local_cm:
        return
    xs = [p[0] for p in trace.l2_estimate_local_cm]
    ys = [p[1] for p in trace.l2_estimate_local_cm]
    ax.plot(xs, ys, "+", color="blue", markersize=10, mew=1.5, label="L2 est. position")
    ax.legend(loc="lower right", fontsize=7)


def _save_perception_rmse_artifacts(
    trace: NavTrace,
    placement_registry: Any,
    output_dir: Path,
    stamp: str,
) -> Dict[str, Path]:
    import sys

    _dp = THIS_DIR.parent / "grid_env_depth_perception"
    if str(_dp) not in sys.path:
        sys.path.insert(0, str(_dp))
    from plot_results import plot_distance_and_bearing, summarize_rmse  # noqa: WPS433
    from simple_nav import NavigationRunResult  # noqa: WPS433

    run = NavigationRunResult(target_prop_type_id="compact_nav", samples=trace.perception_samples)
    rmse_summary = summarize_rmse([run], placement_registry)
    rmse_json = output_dir / f"compact_nav_rmse_{stamp}.json"
    rmse_json.write_text(json.dumps(rmse_summary, indent=2) + "\n", encoding="utf-8")
    dist_png = output_dir / f"compact_nav_distance_{stamp}.png"
    bear_png = output_dir / f"compact_nav_bearing_{stamp}.png"
    plot_distance_and_bearing([run], placement_registry, dist_png, bear_png, rmse_summary)
    return {
        "rmse_json": rmse_json,
        "distance_png": dist_png,
        "bearing_png": bear_png,
    }


def _overlay_scene(ax, registry: CompactNavRegistry, trace: NavTrace) -> None:
    sx, sy = registry.robot_start_local_cm
    gx, gy = registry.goal_local_cm
    ax.plot(sx, sy, "o", color="cyan", markersize=8, label="start")
    ax.plot(gx, gy, "*", color="gold", markersize=14, label="goal")

    for prop in registry.props:
        px, py = prop.local_xy_cm
        ax.plot(px, py, "s", color="orange", markersize=7)
        ax.annotate(
            f"{prop.prop_type_id} (GT)",
            (px, py),
            textcoords="offset points",
            xytext=(4, 4),
            fontsize=7,
            color="white",
        )

    if trace.trajectory_local_cm:
        xs = [p[0] for p in trace.trajectory_local_cm]
        ys = [p[1] for p in trace.trajectory_local_cm]
        ax.plot(xs, ys, "-", color="deepskyblue", linewidth=1.5, label="trajectory")

    for idx, path in enumerate(trace.planned_paths_local_cm):
        if not path:
            continue
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(
            px,
            py,
            "--",
            color="magenta",
            linewidth=0.9,
            alpha=0.55,
            label="planned path" if idx == 0 else None,
        )

    ax.legend(loc="upper left", fontsize=7)


def _write_latest_symlinks(
    output_dir: Path,
    paths: Dict[str, Path],
    stamp: str,
) -> Dict[str, Path]:
    latest: Dict[str, Path] = {}
    for key, src in paths.items():
        if key.startswith("latest_"):
            continue
        ext = src.suffix
        link = output_dir / f"latest_{key}{ext}"
        try:
            if link.is_symlink() or link.exists():
                link.unlink()
        except OSError:
            pass
        try:
            link.symlink_to(src.name)
            latest[f"latest_{key}"] = link
        except OSError:
            # Fallback: copy for filesystems without symlinks
            import shutil

            shutil.copy2(src, link)
            latest[f"latest_{key}"] = link
    marker = output_dir / "latest_run_stamp.txt"
    marker.write_text(stamp + "\n", encoding="utf-8")
    latest["latest_stamp"] = marker
    return latest
