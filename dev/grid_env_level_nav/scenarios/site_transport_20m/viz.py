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
    _save_metrics_summary_png(metrics, summary_png)
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


def _save_metrics_summary_png(metrics: Dict[str, Any], output_path: Path) -> None:
    from metrics import timing_breakdown_rows  # noqa: WPS433

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(
        4,
        2,
        height_ratios=[0.11, 0.07, 1.55, 0.62],
        hspace=0.42,
        wspace=0.28,
    )

    ax_status = fig.add_subplot(gs[0, :])
    _draw_status_banner(ax_status, metrics)

    ax_stats = fig.add_subplot(gs[1, :])
    _draw_compact_mission_stats(ax_stats, metrics)

    ax_table = fig.add_subplot(gs[2, :])
    _draw_timing_breakdown_table(ax_table, timing_breakdown_rows(metrics))

    ax_viol_time = fig.add_subplot(gs[3, 0])
    _draw_violation_time_rates(ax_viol_time, metrics)

    ax_viol_vel = fig.add_subplot(gs[3, 1])
    _draw_violation_velocity_rates(ax_viol_vel, metrics)

    fig.subplots_adjust(top=0.97, bottom=0.05, left=0.07, right=0.97)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _draw_status_banner(ax, metrics: Dict[str, Any]) -> None:
    success = bool(metrics.get("success", False))
    status = "SUCCESS" if success else "FAILED"
    bg = "#d3f9d8" if success else "#ffe3e3"
    fg = "#1a7f37" if success else "#b42318"
    total_s = float(metrics.get("total_time_s", 0.0))
    layout = metrics.get("layout_id", "?")
    profile = metrics.get("profile") or (metrics.get("timing_summary") or {}).get("profile")
    profile_txt = f"  |  profile: {profile}" if profile else ""
    ax.set_facecolor(bg)
    ax.text(
        0.5,
        0.62,
        status,
        ha="center",
        va="center",
        fontsize=28,
        fontweight="bold",
        color=fg,
        transform=ax.transAxes,
    )
    ax.text(
        0.5,
        0.22,
        f"Site transport 20m  ·  layout {layout}  ·  total time {total_s:.1f} s{profile_txt}",
        ha="center",
        va="center",
        fontsize=11,
        color="#333333",
        transform=ax.transAxes,
    )
    ax.axis("off")


def _draw_merged_costmap_panel(
    ax,
    layers: LayeredCostmap,
    registry: SiteTransportRegistry,
    trace: NavTrace,
    metrics: Dict[str, Any],
) -> None:
    lethal = float(layers.lethal_cost)
    extent = [0.0, REGION_SIZE_CM, 0.0, REGION_SIZE_CM]
    merged = layers.merged_costs()
    display = np.where(merged >= lethal * 0.5, lethal, merged)
    im = ax.imshow(
        display.T,
        origin="lower",
        extent=extent,
        cmap="RdYlGn_r",
        vmin=0,
        vmax=max(10.0, lethal),
    )
    _overlay_registry_props(ax, registry, label_gt=False)
    _overlay_forbidden_zones(ax, registry)
    if trace.trajectory_local_cm:
        xs = [p[0] for p in trace.trajectory_local_cm]
        ys = [p[1] for p in trace.trajectory_local_cm]
        ax.plot(xs, ys, "-", color="deepskyblue", linewidth=1.8, label="trajectory")
    sx, sy = registry.robot_start_local_cm
    mx, my = registry.material_pickup_local_cm
    hx, hy = registry.humanoid_local_cm
    ax.plot(sx, sy, "o", color="cyan", markersize=8, label="start")
    ax.plot(mx, my, "s", color="gold", markersize=10, label="crate")
    ax.plot(hx, hy, "^", color="magenta", markersize=9, label="humanoid")
    l1_active = int(np.count_nonzero(layers.l1)) > 0
    layer_label = "L0+L1+L2" if l1_active else "L0+L2"
    ax.set_title(f"Merged costmap ({layer_label}) + trajectory")
    ax.set_xlabel("local X (cm)")
    ax.set_ylabel("local Y (cm)")
    ax.set_aspect("equal")
    fig = ax.get_figure()
    if fig is not None:
        fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02)
    ax.legend(loc="upper left", fontsize=7)


def _draw_violation_time_rates(ax, metrics: Dict[str, Any]) -> None:
    viol = metrics.get("violations", {})
    forbidden = float(viol.get("forbidden_zone_rate", 0.0))
    proximity = float(viol.get("proximity_violation_rate", 0.0))
    compliant = max(0.0, 1.0 - forbidden - proximity)
    labels = ["compliant", "forbidden zone", "object proximity (≤1m)"]
    values = [compliant, forbidden, proximity]
    colors = ["#51cf66", "#ff6b6b", "#fab005"]
    _draw_violation_rate_bars(
        ax,
        labels,
        values,
        colors,
        title="Violation rates (time)",
        xlabel="fraction of mission time",
    )


def _draw_violation_velocity_rates(ax, metrics: Dict[str, Any]) -> None:
    viol = metrics.get("violations", {})
    overspeed = float(viol.get("overspeed_rate", 0.0))
    compliant = max(0.0, 1.0 - overspeed)
    speed_limit = metrics.get("rules", {}).get("speed_limit_kmh", 5)
    labels = ["compliant", f"overspeed (>{speed_limit} km/h)"]
    values = [compliant, overspeed]
    colors = ["#51cf66", "#ffa94d"]
    _draw_violation_rate_bars(
        ax,
        labels,
        values,
        colors,
        title="Violation rates (velocity)",
    )


def _draw_violation_rate_bars(
    ax,
    labels: Sequence[str],
    values: Sequence[float],
    colors: Sequence[str],
    *,
    title: str,
    xlabel: str = "fraction of tracked motion time",
) -> None:
    nonzero = [(l, v, c) for l, v, c in zip(labels, values, colors) if v > 1e-6]
    if not nonzero:
        ax.text(0.5, 0.5, "no motion samples", ha="center", va="center", transform=ax.transAxes)
        ax.set_title(title)
        ax.axis("off")
        return
    labels_nz, values_nz, colors_nz = zip(*nonzero)
    y_pos = np.arange(len(labels_nz))
    bars = ax.barh(y_pos, values_nz, color=colors_nz, edgecolor="#333333", linewidth=0.6)
    ax.set_yticks(y_pos)
    ax.set_yticklabels(labels_nz, fontsize=9)
    ax.set_xlim(0.0, 1.0)
    ax.set_xlabel(xlabel)
    ax.set_title(title)
    ax.invert_yaxis()
    for bar, val in zip(bars, values_nz):
        ax.text(
            min(0.98, val + 0.02),
            bar.get_y() + bar.get_height() / 2,
            f"{val * 100:.1f}%",
            va="center",
            ha="left",
            fontsize=9,
        )


def _draw_compact_mission_stats(ax, metrics: Dict[str, Any]) -> None:
    timing = metrics.get("timing_summary") or {}
    totals = timing.get("totals") or {}
    viol = metrics.get("violations") or {}
    parts: List[str] = []
    nav_wall = timing.get("nav_wall_time_s")
    if nav_wall is not None:
        parts.append(f"nav wall {float(nav_wall):.1f} s")
    leg1 = timing.get("leg1_time_s", metrics.get("leg1_time_s"))
    leg2 = timing.get("leg2_time_s", metrics.get("leg2_time_s"))
    if leg1 is not None and leg2 is not None:
        parts.append(f"leg1 {float(leg1):.1f} s · leg2 {float(leg2):.1f} s")
    standoff_events = totals.get("standoff_events")
    if standoff_events is not None:
        parts.append(f"standoff events {int(standoff_events)}")
    hits = totals.get("depth_cache_hits")
    misses = totals.get("depth_cache_misses")
    if hits is not None and misses is not None:
        parts.append(f"depth cache {int(hits)}/{int(misses)} hits/misses")
    prefetch_hits = totals.get("prefetch_hits")
    if prefetch_hits is not None:
        parts.append(f"prefetch hits {int(prefetch_hits)}")
    tracked = viol.get("tracked_motion_time_s")
    if tracked is not None:
        parts.append(f"tracked motion {float(tracked):.1f} s")
    ax.axis("off")
    ax.text(
        0.5,
        0.5,
        "  ·  ".join(parts) if parts else "—",
        ha="center",
        va="center",
        fontsize=9,
        color="#444444",
        transform=ax.transAxes,
    )


def _draw_timing_breakdown_table(ax, rows: Sequence[Tuple[str, str]]) -> None:
    ax.axis("off")
    ax.set_title("Time breakdown", loc="left", fontsize=13, fontweight="bold", pad=8)
    if not rows:
        ax.text(0.02, 0.5, "No timing data", transform=ax.transAxes, fontsize=10)
        return
    table = ax.table(
        cellText=[[label, value] for label, value in rows],
        colLabels=["Metric", "Value"],
        loc="upper center",
        cellLoc="left",
        colWidths=[0.68, 0.22],
        bbox=[0.0, 0.0, 1.0, 0.92],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9.5)
    table.scale(1.0, 1.42)
    for (row, col), cell in table.get_celld().items():
        if row == 0:
            cell.set_facecolor("#e7f5ff")
            cell.set_text_props(fontweight="bold")
        elif col == 1:
            cell.set_text_props(ha="right")


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
