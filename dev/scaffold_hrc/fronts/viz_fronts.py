"""Plots for discovered 3-objective Pareto fronts (Agg)."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping, Sequence

os.environ["MPLBACKEND"] = "Agg"

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from constraints.pareto import EvaluatedTheta, nondominated
from fronts.space import DMIN_HI, DMIN_LO, VMAX_HI, VMAX_LO

PathTuple = tuple[Path, ...]
METHOD_TITLES = {
    "grid": "Grid sweep",
    "lhs": "Latin hypercube",
    "nsga2": "NSGA-II",
    "weighted_sum": "Weighted sum",
    "epsilon_constraint": "Epsilon-constraint",
    "safe_bo": "SafeOpt",
}


def _title(name: str) -> str:
    return METHOD_TITLES.get(name, name)


def _scatter_pair(
    ax,
    rows: Sequence[EvaluatedTheta],
    nd: Sequence[EvaluatedTheta],
    *,
    x_of,
    y_of,
    xlabel: str,
    ylabel: str,
    panel_title: str,
    color_of=None,
    color_label: str = "",
) -> None:
    if rows:
        kwargs = dict(s=18, alpha=0.7, zorder=2, label=f"samples (n={len(rows)})")
        if color_of is not None:
            kwargs["c"] = [color_of(r) for r in rows]
            kwargs["cmap"] = "viridis"
        ax.scatter([x_of(r) for r in rows], [y_of(r) for r in rows], **kwargs)
        if color_of is not None and ax.collections:
            cbar = ax.figure.colorbar(ax.collections[0], ax=ax, fraction=0.046, pad=0.04)
            cbar.set_label(color_label)
    if nd:
        ax.scatter(
            [x_of(r) for r in nd],
            [y_of(r) for r in nd],
            marker="*",
            s=120,
            color="0.05",
            zorder=4,
            label=f"nondominated (n={len(nd)})",
        )
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(panel_title)
    ax.legend(loc="best", frameon=False, fontsize=7)


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
            [r.theta.vmax_mps for r in rows],
            [r.theta.dmin_m for r in rows],
            c=[r.tt for r in rows],
            cmap="viridis_r",
            s=18,
            alpha=0.85,
            zorder=2,
            label=f"samples (n={len(rows)})",
        )
        cbar = fig.colorbar(ax.collections[0], ax=ax)
        cbar.set_label("TT (s) (lower better)")
    if nd:
        ax.scatter(
            [r.theta.vmax_mps for r in nd],
            [r.theta.dmin_m for r in nd],
            marker="*",
            s=140,
            color="0.05",
            zorder=4,
            label=f"nondominated (n={len(nd)})",
        )
    ax.set_xlabel("v_max (m/s)")
    ax.set_ylabel("d_min (m)")
    ax.set_xlim(VMAX_LO - 0.05, VMAX_HI + 0.10)
    ax.set_ylim(DMIN_LO - 0.10, DMIN_HI + 0.10)
    ax.set_title(f"{label}: parameter space")
    ax.legend(loc="best", frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(theta_path, dpi=140)
    plt.close(fig)

    fig = plt.figure(figsize=(12.4, 10.2))
    ax_tt_v = fig.add_subplot(2, 2, 1)
    ax_tt_s = fig.add_subplot(2, 2, 2)
    ax_v_s = fig.add_subplot(2, 2, 3)
    ax_3d = fig.add_subplot(2, 2, 4, projection="3d")
    _scatter_pair(
        ax_tt_v,
        rows,
        nd,
        x_of=lambda r: r.theta.vmax_mps,
        y_of=lambda r: r.tt,
        xlabel="v_max (m/s) (lower better)",
        ylabel="TT (s) (lower better)",
        panel_title="TT vs v_max",
        color_of=lambda r: r.min_sep_m,
        color_label="min sep (m)",
    )
    _scatter_pair(
        ax_tt_s,
        rows,
        nd,
        x_of=lambda r: r.min_sep_m,
        y_of=lambda r: r.tt,
        xlabel="min sep (m) (higher better)",
        ylabel="TT (s) (lower better)",
        panel_title="TT vs min separation",
        color_of=lambda r: r.theta.vmax_mps,
        color_label="v_max (m/s)",
    )
    _scatter_pair(
        ax_v_s,
        rows,
        nd,
        x_of=lambda r: r.theta.vmax_mps,
        y_of=lambda r: r.min_sep_m,
        xlabel="v_max (m/s) (lower better)",
        ylabel="min sep (m) (higher better)",
        panel_title="v_max vs min separation",
        color_of=lambda r: r.tt,
        color_label="TT (s)",
    )
    if rows:
        ax_3d.scatter(
            [r.tt for r in rows],
            [r.theta.vmax_mps for r in rows],
            [r.min_sep_m for r in rows],
            s=12,
            alpha=0.45,
        )
    if nd:
        ax_3d.scatter(
            [r.tt for r in nd],
            [r.theta.vmax_mps for r in nd],
            [r.min_sep_m for r in nd],
            marker="*",
            s=80,
            color="0.05",
        )
    ax_3d.set_xlabel("TT (s)")
    ax_3d.set_ylabel("v_max (m/s)")
    ax_3d.set_zlabel("min sep (m)")
    ax_3d.set_title("3-objective space")
    fig.suptitle(f"{label}: min TT, min v_max, max min-sep", fontsize=11)
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

    fig, axes = plt.subplots(1, 3, figsize=(14.4, 4.4))
    pairs = (
        (
            axes[0],
            lambda r: r.theta.vmax_mps,
            lambda r: r.tt,
            "v_max (m/s) (lower better)",
            "TT (s) (lower better)",
        ),
        (
            axes[1],
            lambda r: r.min_sep_m,
            lambda r: r.tt,
            "min sep (m) (higher better)",
            "TT (s) (lower better)",
        ),
        (
            axes[2],
            lambda r: r.theta.vmax_mps,
            lambda r: r.min_sep_m,
            "v_max (m/s) (lower better)",
            "min sep (m) (higher better)",
        ),
    )
    for ax, x_of, y_of, xlabel, ylabel in pairs:
        for name, rows in methods.items():
            if not rows:
                continue
            nd = nondominated(tuple(rows))
            ax.scatter(
                [x_of(r) for r in rows],
                [y_of(r) for r in rows],
                s=10,
                alpha=0.18,
            )
            if nd:
                ax.scatter(
                    [x_of(r) for r in nd],
                    [y_of(r) for r in nd],
                    s=36,
                    marker="*",
                    label=_title(name),
                )
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.legend(loc="best", frameon=False, fontsize=7)
    fig.suptitle("All methods: 3-objective nondominated set", fontsize=11)
    fig.tight_layout()
    fig.savefig(obj_path, dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.6, 5.0))
    for name, rows in methods.items():
        nd = nondominated(tuple(rows))
        if not nd:
            continue
        ax.scatter(
            [r.theta.vmax_mps for r in nd],
            [r.theta.dmin_m for r in nd],
            s=46,
            label=_title(name),
        )
    ax.set_xlabel("v_max (m/s)")
    ax.set_ylabel("d_min (m)")
    ax.set_title("All methods: nondominated theta")
    ax.set_xlim(VMAX_LO - 0.05, VMAX_HI + 0.10)
    ax.set_ylim(DMIN_LO - 0.10, DMIN_HI + 0.10)
    ax.legend(loc="best", frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(theta_path, dpi=140)
    plt.close(fig)
    return (obj_path, theta_path)


def write_ab_table_plot(
    out_dir: Path,
    table: Mapping[tuple[float, float], EvaluatedTheta],
) -> Path:
    """5×5 (α, β) weighted-sum lookup as two heatmaps."""
    from fronts.ab_map import LEVELS

    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "ab_table.png"
    tt = [[table[(a, b)].tt for b in LEVELS] for a in LEVELS]
    sep = [[table[(a, b)].min_sep_m for b in LEVELS] for a in LEVELS]
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 4.8))
    for ax, grid, title, cmap in (
        (axes[0], tt, "TT (s)", "viridis_r"),
        (axes[1], sep, "min sep (m)", "viridis"),
    ):
        image = ax.imshow(grid, origin="lower", cmap=cmap)
        ax.set_xticks(range(len(LEVELS)))
        ax.set_yticks(range(len(LEVELS)))
        ax.set_xticklabels([f"{v:.2f}" for v in LEVELS])
        ax.set_yticklabels([f"{v:.2f}" for v in LEVELS])
        ax.set_xlabel("β (0=distance, 1=slow)")
        ax.set_ylabel("α (0=efficiency, 1=safety)")
        ax.set_title(title)
        fig.colorbar(image, ax=ax, fraction=0.046, pad=0.04)
    fig.suptitle("Weighted-sum lookup on ISO front", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path
