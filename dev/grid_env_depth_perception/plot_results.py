#!/usr/bin/env python3
"""Plot time-series distance/bearing with ground truth and RMSE."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from ground_truth import rmse
from prop_placement import PlacementRegistry
from simple_nav import NavigationRunResult, TimeSeriesSample

# Distinct colors for up to 5 props
PROP_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]


def _paired_values(
    samples: Sequence[TimeSeriesSample],
    prop_type_id: str,
) -> Tuple[List[float], List[float], List[float]]:
    """Return t, gt, est where estimate exists and GT in FOV."""
    times: List[float] = []
    gt_vals: List[float] = []
    est_vals: List[float] = []
    for sample in samples:
        gt = sample.ground_truth.get(prop_type_id)
        est = sample.estimates.get(prop_type_id)
        if gt is None or est is None:
            continue
        if not gt.get("in_fov", 0.0):
            continue
        times.append(sample.t_s)
        gt_vals.append(float(gt["distance_m"]))
        est_vals.append(float(est["distance_m"]))
    return times, gt_vals, est_vals


def _paired_bearing(
    samples: Sequence[TimeSeriesSample],
    prop_type_id: str,
) -> Tuple[List[float], List[float], List[float]]:
    times: List[float] = []
    gt_vals: List[float] = []
    est_vals: List[float] = []
    for sample in samples:
        gt = sample.ground_truth.get(prop_type_id)
        est = sample.estimates.get(prop_type_id)
        if gt is None or est is None:
            continue
        if not gt.get("in_fov", 0.0):
            continue
        times.append(sample.t_s)
        gt_vals.append(float(gt["bearing_deg"]))
        est_vals.append(float(est["bearing_deg"]))
    return times, gt_vals, est_vals


def summarize_rmse(
    runs: Sequence[NavigationRunResult],
    registry: PlacementRegistry,
) -> Dict[str, object]:
    all_samples: List[TimeSeriesSample] = []
    for run in runs:
        all_samples.extend(run.samples)

    per_prop: Dict[str, Dict[str, Optional[float]]] = {}
    dist_gt_all: List[float] = []
    dist_est_all: List[float] = []
    bear_gt_all: List[float] = []
    bear_est_all: List[float] = []

    for idx, prop in enumerate(registry.props):
        _, gt_d, est_d = _paired_values(all_samples, prop.prop_type_id)
        _, gt_b, est_b = _paired_bearing(all_samples, prop.prop_type_id)
        per_prop[prop.prop_type_id] = {
            "distance_rmse_m": rmse(gt_d, est_d),
            "bearing_rmse_deg": rmse(gt_b, est_b),
            "n_distance_pairs": len(gt_d),
            "n_bearing_pairs": len(gt_b),
        }
        dist_gt_all.extend(gt_d)
        dist_est_all.extend(est_d)
        bear_gt_all.extend(gt_b)
        bear_est_all.extend(est_b)

    return {
        "per_prop": per_prop,
        "overall": {
            "distance_rmse_m": rmse(dist_gt_all, dist_est_all),
            "bearing_rmse_deg": rmse(bear_gt_all, bear_est_all),
            "n_distance_pairs": len(dist_gt_all),
            "n_bearing_pairs": len(bear_gt_all),
        },
    }


def plot_distance_and_bearing(
    runs: Sequence[NavigationRunResult],
    registry: PlacementRegistry,
    distance_png: Path,
    bearing_png: Path,
    rmse_summary: Dict[str, object],
) -> None:
    all_samples: List[TimeSeriesSample] = []
    for run in runs:
        all_samples.extend(run.samples)

    _plot_metric(
        all_samples,
        registry,
        metric="distance",
        ylabel="Distance [m]",
        title="Distance vs time (GT dashed, estimate solid)",
        outfile=distance_png,
        rmse_key="distance_rmse_m",
        rmse_summary=rmse_summary,
    )
    _plot_metric(
        all_samples,
        registry,
        metric="bearing",
        ylabel="Bearing [deg] (rel. forward)",
        title="Bearing vs time (GT dashed, estimate solid)",
        outfile=bearing_png,
        rmse_key="bearing_rmse_deg",
        rmse_summary=rmse_summary,
    )


def _plot_metric(
    samples: Sequence[TimeSeriesSample],
    registry: PlacementRegistry,
    *,
    metric: str,
    ylabel: str,
    title: str,
    outfile: Path,
    rmse_key: str,
    rmse_summary: Dict[str, object],
) -> None:
    fig, ax = plt.subplots(figsize=(12, 6))
    per_prop = rmse_summary.get("per_prop", {})
    overall = rmse_summary.get("overall", {})

    for idx, prop in enumerate(registry.props):
        color = PROP_COLORS[idx % len(PROP_COLORS)]
        if metric == "distance":
            times, gt, est = _paired_values(samples, prop.prop_type_id)
        else:
            times, gt, est = _paired_bearing(samples, prop.prop_type_id)

        if not times:
            continue
        label_base = prop.prop_type_id
        prop_rmse = per_prop.get(prop.prop_type_id, {}).get(rmse_key)
        rmse_txt = f"{prop_rmse:.3f}" if prop_rmse is not None and math.isfinite(prop_rmse) else "n/a"
        ax.plot(times, gt, color=color, linestyle="--", linewidth=1.2, alpha=0.85, label=f"{label_base} GT")
        ax.plot(
            times,
            est,
            color=color,
            linestyle="-",
            linewidth=1.6,
            alpha=0.95,
            label=f"{label_base} est (RMSE={rmse_txt})",
        )

    overall_rmse = overall.get(rmse_key)
    if overall_rmse is not None and math.isfinite(overall_rmse):
        ax.set_title(f"{title}\nOverall RMSE={overall_rmse:.3f}")
    else:
        ax.set_title(title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8, ncol=2)
    fig.tight_layout()
    outfile.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(outfile, dpi=150)
    plt.close(fig)
