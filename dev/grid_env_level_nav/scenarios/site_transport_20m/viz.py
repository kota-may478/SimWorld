#!/usr/bin/env python3
"""Costmap overlays and mission metrics summary PNG."""

from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

os.environ["MPLBACKEND"] = "Agg"

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from costmap_layers import LayeredCostmap  # noqa: E402
from paths import SITE_TRANSPORT_20M_OUT_DIR  # noqa: E402
from placement import SiteTransportRegistry  # noqa: E402
from region import REGION_SIZE_CM  # noqa: E402

DEFAULT_ARTIFACT_DIR = SITE_TRANSPORT_20M_OUT_DIR
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


def _artifact_suffix(
    *,
    run_label: Optional[str] = None,
    trial_index: Optional[int] = None,
    stamp: Optional[str] = None,
) -> str:
    if run_label is not None and trial_index is not None:
        return f"{run_label}_{trial_index}"
    if stamp is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return stamp


def save_site_transport_artifacts(
    layers: LayeredCostmap,
    registry: SiteTransportRegistry,
    trace: NavTrace,
    metrics: Dict[str, Any],
    *,
    output_dir: Path = DEFAULT_ARTIFACT_DIR,
    run_label: Optional[str] = None,
    trial_index: Optional[int] = None,
) -> Dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    suffix = _artifact_suffix(run_label=run_label, trial_index=trial_index, stamp=stamp)
    labeled = run_label is not None and trial_index is not None
    paths: Dict[str, Path] = {}

    snap = layers.snapshot_layers()
    npz_path = output_dir / f"site_transport_costmap_{suffix}.npz"
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

    costmap_png = output_dir / f"costMap_{suffix}.png"
    _save_costmap_png(layers, registry, trace, costmap_png)
    paths["costMap"] = costmap_png

    if labeled:
        summary_png = output_dir / f"metricsSummary_{suffix}.png"
    else:
        summary_png = output_dir / f"site_transport_metrics_summary_{suffix}.png"
    _save_metrics_summary_png(registry, trace, metrics, summary_png)
    paths["metrics_summary_png"] = summary_png

    traj_path = output_dir / f"site_transport_trajectory_{suffix}.json"
    traj_path.write_text(
        json.dumps(_trace_to_dict(registry, trace, layers, metrics), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    paths["trajectory_json"] = traj_path

    latest = _write_latest_symlinks(output_dir, paths, stamp)
    paths.update(latest)
    return paths


def _save_metrics_summary_png(
    registry: SiteTransportRegistry,
    trace: NavTrace,
    metrics: Dict[str, Any],
    output_path: Path,
) -> None:
    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.2, 1.0], width_ratios=[1.4, 1.0])

    ax_map = fig.add_subplot(gs[:, 0])
    _draw_mission_map(ax_map, registry, trace, metrics)

    ax_metrics = fig.add_subplot(gs[0, 1])
    _draw_metrics_bars(ax_metrics, metrics)

    ax_viol = fig.add_subplot(gs[1, 1])
    _draw_violation_pie(ax_viol, metrics)

    success = metrics.get("success", False)
    title_color = "#1a7f37" if success else "#b42318"
    fig.suptitle(
        f"Site transport 20m — {'SUCCESS' if success else 'FAIL'} "
        f"(layout {metrics.get('layout_id', '?')})",
        fontsize=14,
        fontweight="bold",
        color=title_color,
    )
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def _draw_mission_map(
    ax,
    registry: SiteTransportRegistry,
    trace: NavTrace,
    metrics: Dict[str, Any],
) -> None:
    extent = [0.0, REGION_SIZE_CM, 0.0, REGION_SIZE_CM]
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.set_xlabel("local X (cm)")
    ax.set_ylabel("local Y (cm)")
    ax.set_title("Mission map (trajectory + zones)")

    for zone in registry.forbidden_zones:
        x0, y0, x1, y1 = zone.rect_local_cm
        ax.add_patch(
            plt.Rectangle(
                (min(x0, x1), min(y0, y1)),
                abs(x1 - x0),
                abs(y1 - y0),
                facecolor="#ff6b6b",
                edgecolor="#c92a2a",
                alpha=0.35,
                linewidth=1.5,
                label="forbidden (L1)" if zone.zone_id == registry.forbidden_zones[0].zone_id else None,
            )
        )

    sx, sy = registry.robot_start_local_cm
    hx, hy = registry.humanoid_local_cm
    mx, my = registry.material_pickup_local_cm
    ax.plot(sx, sy, "o", color="cyan", markersize=9, label="SpotDog start")
    ax.plot(hx, hy, "^", color="magenta", markersize=10, label="Humanoid")
    ax.plot(mx, my, "s", color="gold", markersize=11, label="transport crate")

    for prop in registry.props:
        if prop.is_transport_target:
            continue
        px, py = prop.local_xy_cm
        ax.plot(px, py, ".", color="#888888", markersize=4)

    if trace.trajectory_local_cm:
        xs = [p[0] for p in trace.trajectory_local_cm]
        ys = [p[1] for p in trace.trajectory_local_cm]
        ax.plot(xs, ys, "-", color="deepskyblue", linewidth=1.8, label="trajectory")

    ax.legend(loc="upper left", fontsize=8)


def _draw_metrics_bars(ax, metrics: Dict[str, Any]) -> None:
    labels = ["Success\n(0/1)", "Total time\n(s)"]
    values = [
        float(metrics.get("success_rate", 0.0)),
        float(metrics.get("total_time_s", 0.0)),
    ]
    colors = ["#2f9e44" if metrics.get("success") else "#e03131", "#1971c2"]
    bars = ax.bar(labels, values, color=colors, edgecolor="#333333", linewidth=0.8)
    ax.set_title("Mission metrics")
    ax.set_ylabel("value")
    for bar, val in zip(bars, values):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + max(0.02 * max(values + [1.0]), 0.05),
            f"{val:.2f}",
            ha="center",
            va="bottom",
            fontsize=10,
        )
    ax.set_ylim(0, max(values + [1.0]) * 1.25)


def _draw_violation_pie(ax, metrics: Dict[str, Any]) -> None:
    viol = metrics.get("violations", {})
    forbidden = float(viol.get("forbidden_zone_rate", 0.0))
    overspeed = float(viol.get("overspeed_rate", 0.0))
    compliant = max(0.0, 1.0 - forbidden - overspeed)
    sizes = [compliant, forbidden, overspeed]
    labels = ["compliant", "forbidden zone", f"overspeed (>{metrics.get('rules', {}).get('speed_limit_kmh', 5)} km/h)"]
    colors = ["#51cf66", "#ff6b6b", "#ffa94d"]
    nonzero = [(s, l, c) for s, l, c in zip(sizes, labels, colors) if s > 1e-6]
    if not nonzero:
        ax.text(0.5, 0.5, "no motion samples", ha="center", va="center", transform=ax.transAxes)
        ax.set_title("Violation rates (time fraction)")
        ax.axis("off")
        return
    sizes_nz, labels_nz, colors_nz = zip(*nonzero)
    ax.pie(
        sizes_nz,
        labels=[f"{l}\n{v*100:.1f}%" for l, v in zip(labels_nz, sizes_nz)],
        colors=colors_nz,
        autopct="",
        startangle=90,
    )
    ax.set_title("Violation rates (time fraction)")


def _overlay_registry_props(ax, registry: SiteTransportRegistry, *, label_gt: bool = False) -> None:
    for prop in registry.props:
        px, py = prop.local_xy_cm
        r, g, b = prop.mask_color_rgb
        ax.plot(px, py, "s", color=(r / 255.0, g / 255.0, b / 255.0), markersize=6)
        if label_gt:
            ax.annotate(
                prop.prop_type_id,
                (px, py),
                textcoords="offset points",
                xytext=(4, 4),
                fontsize=6,
                color="black",
                bbox={"facecolor": "white", "alpha": 0.65, "pad": 1.0, "edgecolor": "none"},
            )


def _overlay_l2_estimates(ax, trace: NavTrace) -> None:
    if not trace.l2_estimate_local_cm:
        return
    xs = [p[0] for p in trace.l2_estimate_local_cm]
    ys = [p[1] for p in trace.l2_estimate_local_cm]
    ax.plot(xs, ys, "+", color="blue", markersize=9, mew=1.4, label="L2 est.")
    ax.legend(loc="lower right", fontsize=7)


def _overlay_forbidden_zones(ax, registry: SiteTransportRegistry) -> None:
    for zone in registry.forbidden_zones:
        x0, y0, x1, y1 = zone.rect_local_cm
        ax.add_patch(
            plt.Rectangle(
                (min(x0, x1), min(y0, y1)),
                abs(x1 - x0),
                abs(y1 - y0),
                facecolor="none",
                edgecolor="#c92a2a",
                linewidth=1.5,
                linestyle="--",
            )
        )


def _save_costmap_png(
    layers: LayeredCostmap,
    registry: SiteTransportRegistry,
    trace: NavTrace,
    output_path: Path,
) -> None:
    lethal = float(layers.lethal_cost)
    extent = [0.0, REGION_SIZE_CM, 0.0, REGION_SIZE_CM]
    fig, axes = plt.subplots(1, 4, figsize=(22, 5.5))
    titles = ("L0 (NavMesh)", "L1 (forbidden)", "L2 (perception)", "Merged + path")
    arrays = (layers.l0, layers.l1, layers.l2, layers.merged_costs())

    for ax, title, arr in zip(axes, titles, arrays):
        display = np.where(arr >= lethal * 0.5, lethal, arr)
        im = ax.imshow(
            display.T,
            origin="lower",
            extent=extent,
            cmap="RdYlGn_r",
            vmin=0,
            vmax=max(10.0, lethal),
        )
        ax.set_title(title)
        ax.set_xlabel("local X (cm)")
        ax.set_ylabel("local Y (cm)")
        ax.set_aspect("equal")
        fig.colorbar(im, ax=ax, fraction=0.046)
        if title.startswith("L1"):
            _overlay_forbidden_zones(ax, registry)
        if title.startswith("L2"):
            _overlay_registry_props(ax, registry, label_gt=True)
            _overlay_l2_estimates(ax, trace)

    merged_ax = axes[3]
    _overlay_registry_props(merged_ax, registry, label_gt=False)
    _overlay_forbidden_zones(merged_ax, registry)
    if trace.trajectory_local_cm:
        xs = [p[0] for p in trace.trajectory_local_cm]
        ys = [p[1] for p in trace.trajectory_local_cm]
        merged_ax.plot(xs, ys, "-", color="deepskyblue", linewidth=1.5)
    sx, sy = registry.robot_start_local_cm
    mx, my = registry.material_pickup_local_cm
    hx, hy = registry.humanoid_local_cm
    merged_ax.plot(sx, sy, "o", color="cyan", markersize=7)
    merged_ax.plot(mx, my, "s", color="gold", markersize=9)
    merged_ax.plot(hx, hy, "^", color="magenta", markersize=8)

    fig.suptitle(
        f"Site transport 20m × 20m — costmaps (layout {registry.layout_id})"
    )
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def _trace_to_dict(
    registry: SiteTransportRegistry,
    trace: NavTrace,
    layers: LayeredCostmap,
    metrics: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "metrics": metrics,
        "arrived": trace.arrived,
        "l2_cell_count": trace.l2_cell_count,
        "trajectory_local_cm": [list(p) for p in trace.trajectory_local_cm],
        "planned_paths_local_cm": [[list(p) for p in path] for path in trace.planned_paths_local_cm],
        "replan_events": trace.replan_events,
        "robot_start_local_cm": list(registry.robot_start_local_cm),
        "humanoid_local_cm": list(registry.humanoid_local_cm),
        "material_pickup_local_cm": list(registry.material_pickup_local_cm),
        "forbidden_zones": [
            {"zone_id": z.zone_id, "rect_local_cm": list(z.rect_local_cm)} for z in registry.forbidden_zones
        ],
        "region_size_cm": REGION_SIZE_CM,
        "l2_nonzero_cells": int(np.count_nonzero(layers.l2)),
        "l1_nonzero_cells": int(np.count_nonzero(layers.l1)),
    }


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
            import shutil

            shutil.copy2(src, link)
            latest[f"latest_{key}"] = link
    marker = output_dir / "latest_run_stamp.txt"
    marker.write_text(stamp + "\n", encoding="utf-8")
    latest["latest_stamp"] = marker
    return latest
