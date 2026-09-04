"""Plots for discovered Pareto fronts (Agg)."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping, Sequence

os.environ["MPLBACKEND"] = "Agg"

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from constraints.pareto import EvaluatedTheta, nondominated

PathTuple = tuple[Path, ...]
METHOD_TITLES = {
    "grid": "Grid sweep",
    "lhs": "Latin hypercube",
    "nsga2": "NSGA-II",
    "weighted_sum": "Weighted sum",
    "epsilon_constraint": "Epsilon-constraint",
    "safe_bo": "Safe UCB BO",
}


def _title(name: str) -> str:
    return METHOD_TITLES.get(name, name)


def write_method_plots(
    out_dir: Path,
    name: str,
    rows: Sequence[EvaluatedTheta],
) -> PathTuple:
    out_dir.mkdir(parents=True, exist_ok=True)
    theta_path = out_dir / "theta.png"
    obj_path = out_dir / "objectives.png"
    nd = nondominated(tuple(rows))
    label = _title(name)

    fig, ax = plt.subplots(figsize=(7.4, 5.0))
    if rows:
        ax.scatter(
            [r.theta.dmin_m for r in rows],
            [r.theta.vmax_mps for r in rows],
            c=[r.jeff for r in rows],
            cmap="viridis",
            s=18,
            alpha=0.85,
            zorder=2,
            label=f"samples (n={len(rows)})",
        )
        cbar = fig.colorbar(ax.collections[0], ax=ax)
        cbar.set_label("Jeff")
    if nd:
        ax.scatter(
            [r.theta.dmin_m for r in nd],
            [r.theta.vmax_mps for r in nd],
            marker="*",
            s=140,
            color="0.05",
            zorder=4,
            label=f"nondominated (n={len(nd)})",
        )
    ax.set_xlabel("d_min (m)")
    ax.set_ylabel("v_max (m/s)")
    ax.set_xlim(0.30, 1.65)
    ax.set_ylim(0.15, 1.10)
    ax.set_title(f"{label}: parameter space")
    ax.legend(loc="best", frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(theta_path, dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.4, 5.0))
    if rows:
        ax.scatter(
            [r.jsafe for r in rows],
            [r.jeff for r in rows],
            s=18,
            alpha=0.7,
            zorder=2,
            label=f"samples (n={len(rows)})",
        )
    if nd:
        ordered = sorted(nd, key=lambda r: (r.jsafe, -r.jeff))
        ax.plot(
            [r.jsafe for r in ordered],
            [r.jeff for r in ordered],
            color="0.1",
            linewidth=1.7,
            zorder=3,
            label="Pareto front",
        )
        ax.scatter(
            [r.jsafe for r in ordered],
            [r.jeff for r in ordered],
            marker="*",
            s=120,
            color="0.05",
            zorder=4,
        )
    ax.set_xlabel("Jsafe = T_viol / T_ref (lower better)")
    ax.set_ylabel("Jeff = TCR - TT (higher better)")
    ax.set_title(f"{label}: objective space")
    ax.legend(loc="best", frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(obj_path, dpi=140)
    plt.close(fig)
    return (theta_path, obj_path)


def write_front_comparison(
    out_dir: Path,
    methods: Mapping[str, Sequence[EvaluatedTheta]],
) -> PathTuple:
    out_dir.mkdir(parents=True, exist_ok=True)
    obj_path = out_dir / "comparison_objectives.png"
    theta_path = out_dir / "comparison_theta.png"

    fig, ax = plt.subplots(figsize=(7.6, 5.0))
    for name, rows in methods.items():
        if not rows:
            continue
        nd = nondominated(tuple(rows))
        ax.scatter(
            [r.jsafe for r in rows],
            [r.jeff for r in rows],
            s=10,
            alpha=0.25,
        )
        if nd:
            ordered = sorted(nd, key=lambda r: (r.jsafe, -r.jeff))
            ax.plot(
                [r.jsafe for r in ordered],
                [r.jeff for r in ordered],
                linewidth=1.6,
                label=_title(name),
            )
    ax.set_xlabel("Jsafe = T_viol / T_ref (lower better)")
    ax.set_ylabel("Jeff = TCR - TT (higher better)")
    ax.set_title("All methods: Pareto fronts")
    ax.legend(loc="best", frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(obj_path, dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.6, 5.0))
    for name, rows in methods.items():
        nd = nondominated(tuple(rows))
        if not nd:
            continue
        ax.scatter(
            [r.theta.dmin_m for r in nd],
            [r.theta.vmax_mps for r in nd],
            s=46,
            label=_title(name),
        )
    ax.set_xlabel("d_min (m)")
    ax.set_ylabel("v_max (m/s)")
    ax.set_title("All methods: nondominated theta")
    ax.set_xlim(0.30, 1.65)
    ax.set_ylim(0.15, 1.10)
    ax.legend(loc="best", frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(theta_path, dpi=140)
    plt.close(fig)
    return (obj_path, theta_path)
