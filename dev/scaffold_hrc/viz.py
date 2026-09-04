"""Matplotlib artifacts for a headless oracle run (Agg, no display)."""

from __future__ import annotations

import csv
import os
from pathlib import Path
from typing import Sequence, Tuple

os.environ["MPLBACKEND"] = "Agg"

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

from constraints.pareto import EvaluatedTheta, Theta, nondominated
from oracle.simulate import OracleResult
from scene.geometry import ScaffoldGeom

PathTuple = Tuple[Path, ...]


def write_trace_csv(path: Path, result: OracleResult) -> Path:
    path.write_text("", encoding="utf-8")
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "t_s",
                "spot_x_m",
                "spot_y_m",
                "spot_z_m",
                "human_x_m",
                "human_y_m",
                "human_z_m",
                "sep_m",
                "spot_speed_mps",
                "blocked",
                "in_corridor",
                "n_filled",
                "current_floor",
                "violating",
            ]
        )
        for sample in result.trace:
            writer.writerow(
                [
                    f"{sample.t_s:.3f}",
                    f"{sample.spot[0]:.4f}",
                    f"{sample.spot[1]:.4f}",
                    f"{sample.spot[2]:.4f}",
                    f"{sample.human[0]:.4f}",
                    f"{sample.human[1]:.4f}",
                    f"{sample.human[2]:.4f}",
                    f"{sample.sep_m:.4f}",
                    f"{sample.spot_speed_mps:.4f}",
                    int(sample.blocked),
                    int(sample.in_corridor),
                    sample.n_filled,
                    sample.current_floor,
                    int(sample.violating),
                ]
            )
    return path


def write_pareto_plots(
    out_dir: Path,
    *,
    rows: Sequence[EvaluatedTheta],
    front: Sequence[Theta],
    chosen: Theta,
) -> PathTuple:
    out_dir.mkdir(parents=True, exist_ok=True)
    theta_path = out_dir / "pareto_theta.png"
    obj_path = out_dir / "pareto_objectives.png"

    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    if rows:
        ax.scatter(
            [r.theta.dmin_m for r in rows],
            [r.theta.vmax_mps for r in rows],
            c=[r.jeff for r in rows],
            cmap="viridis",
            s=42,
            zorder=3,
            label="oracle samples",
        )
        cbar = fig.colorbar(ax.collections[0], ax=ax)
        cbar.set_label("Jeff = TCR - TT (dimensionless)")
    ax.plot(
        [p.dmin_m for p in front],
        [p.vmax_mps for p in front],
        color="0.25",
        linewidth=1.4,
        label="design front P (alpha index)",
    )
    ax.scatter(
        [chosen.dmin_m],
        [chosen.vmax_mps],
        marker="*",
        s=180,
        color="0.05",
        zorder=4,
        label="representative theta",
    )
    ax.set_xlabel("d_min (m)")
    ax.set_ylabel("v_max (m/s)")
    ax.set_title("Parameter front: oracle Jeff over (d_min, v_max)")
    ax.set_ylim(0.0, 1.15)
    ax.legend(loc="upper right", frameon=False)
    fig.tight_layout()
    fig.savefig(theta_path, dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    ax.scatter(
        [r.jsafe for r in rows],
        [r.jeff for r in rows],
        s=42,
        label="oracle samples",
    )
    nd = nondominated(tuple(rows))
    if nd:
        ordered = sorted(nd, key=lambda r: r.jsafe)
        ax.plot(
            [r.jsafe for r in ordered],
            [r.jeff for r in ordered],
            color="0.15",
            linewidth=1.5,
            label="nondominated",
        )
    ax.set_xlabel("Jsafe = T_viol / T_ref (penalty, lower is better)")
    ax.set_ylabel("Jeff = w1 TCR - w2 TT (higher is better)")
    ax.set_title("Objective space (maximize Jeff, Jsafe is a penalty)")
    ax.legend(loc="best", frameon=False)
    fig.tight_layout()
    fig.savefig(obj_path, dpi=140)
    plt.close(fig)
    return (theta_path, obj_path)


def write_trajectory_plots(
    out_dir: Path,
    *,
    geom: ScaffoldGeom,
    result: OracleResult,
    theta: Theta | None = None,
    floor: int | None = None,
) -> PathTuple:
    _ = floor
    out_dir.mkdir(parents=True, exist_ok=True)
    xy_path = out_dir / "trajectory_xy.png"
    time_path = out_dir / "trajectory_time.png"
    trace = result.trace
    sx = [s.spot[0] for s in trace]
    sy = [s.spot[1] for s in trace]
    sz = [s.spot[2] for s in trace]
    hx = [s.human[0] for s in trace]
    hy = [s.human[1] for s in trace]
    hz = [s.human[2] for s in trace]
    ts = [s.t_s for s in trace]
    sep = [s.sep_m for s in trace]
    filled = [s.n_filled for s in trace]

    fig, ax = plt.subplots(figsize=(9.2, 3.8))
    store_x, _store_y = geom.storage_xy()
    sx0, sx1, sy0, sy1 = geom.stair_xy_bounds()
    dx0, dx1, dy0, dy1 = geom.deck_xy_bounds()
    ax.add_patch(
        Rectangle(
            (store_x - 0.8, 0.0),
            1.6,
            geom.deck_width_m,
            fill=True,
            facecolor="0.92",
            linewidth=1.0,
            label="storage",
        )
    )
    ax.add_patch(
        Rectangle((sx0, sy0), sx1 - sx0, sy1 - sy0, fill=False, linewidth=1.0, label="stair")
    )
    ax.add_patch(
        Rectangle((dx0, dy0), dx1 - dx0, dy1 - dy0, fill=False, linewidth=1.2, label="deck")
    )
    ax.plot(sx, sy, linewidth=1.6, label="SpotDog")
    ax.plot(hx, hy, linewidth=1.6, linestyle="--", label="Humanoid")
    if sx:
        ax.scatter([sx[0]], [sy[0]], s=28, zorder=3)
        ax.scatter([sx[-1]], [sy[-1]], s=28, marker="s", zorder=3)
    ax.text(store_x, geom.deck_width_m + 0.12, "storage", ha="center", fontsize=8)
    ax.set_xlabel("x (m)  storage ←  stair  → deck")
    ax.set_ylabel("y (m)")
    ax.set_xlim(store_x - 2.0, geom.deck_length_m + 1.0)
    ax.set_ylim(-0.4, geom.deck_width_m + 0.6)
    ax.set_title("Top-down traces, 1F→3F erection (storage is the left box)")
    ax.legend(loc="upper right", ncol=2, frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(xy_path, dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(7.4, 8.2), sharex=True)
    axes[0].plot(ts, sx, label="SpotDog x")
    axes[0].plot(ts, hx, linestyle="--", label="Humanoid x")
    axes[0].axhline(store_x, color="0.7", linewidth=0.7, linestyle=":")
    axes[0].set_ylabel("x (m)")
    axes[0].legend(loc="best", frameon=False, fontsize=8)
    axes[1].plot(ts, sz, label="SpotDog z")
    axes[1].plot(ts, hz, linestyle="--", label="Humanoid z")
    for k, zf in enumerate((0.0, 1.8, 3.6), start=1):
        axes[1].axhline(zf, color="0.85", linewidth=0.6)
        axes[1].text(0.0, zf + 0.05, f"{k}F", fontsize=7, color="0.4")
    axes[1].set_ylabel("z (m)")
    axes[1].legend(loc="best", frameon=False, fontsize=8)
    axes[2].plot(ts, sep, label="separation")
    if theta is not None:
        axes[2].axhline(theta.dmin_m, color="0.3", linestyle="--", linewidth=1.0, label="d_min")
    axes[2].set_ylabel("sep (m)")
    axes[2].legend(loc="best", frameon=False, fontsize=8)
    axes[3].plot(ts, filled, label="boards placed")
    axes[3].set_ylabel("filled")
    axes[3].set_xlabel("t (s)")
    fig.suptitle("Time history: 3F erection (climb after each floor is built)")
    fig.tight_layout()
    fig.savefig(time_path, dpi=140)
    plt.close(fig)
    return (xy_path, time_path)

