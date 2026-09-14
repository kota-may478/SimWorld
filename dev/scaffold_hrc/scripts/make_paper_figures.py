#!/usr/bin/env python3
"""Paper figures: Pareto front + condition-card TT / S_min comparison."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

os.environ["MPLBACKEND"] = "Agg"

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from constraints.pareto import EvaluatedTheta, Theta  # noqa: E402
from fronts.viz_fronts import write_ab_table_plot, write_method_plots  # noqa: E402
from fronts.ab_map import enumerate_table  # noqa: E402


def rows_from_items(items: list[dict]) -> tuple[EvaluatedTheta, ...]:
    return tuple(
        EvaluatedTheta(
            Theta(vmax_mps=i["vmax_mps"], dmin_m=i["dmin_m"]),
            tt=float(i["tt"]),
            t_ssm=float(i.get("t_ssm", i["vmax_mps"])),
            si_min=float(i.get("si_min", 1.0)),
            completed=bool(i.get("completed", True)),
            min_sep_m=float(i.get("min_sep_m", 0.0)),
        )
        for i in items
    )


def _bar_compare(
    out: Path,
    cards: list[dict],
    *,
    metric: str,
    ylabel: str,
    title: str,
    stem: str,
) -> Path:
    # Group by card_id; bars = methods
    order = []
    for row in cards:
        if row["card_id"] not in order:
            order.append(row["card_id"])
    methods = []
    for row in cards:
        if row["method"] not in methods:
            methods.append(row["method"])
    fig, ax = plt.subplots(figsize=(10.5, 4.2))
    width = 0.8 / max(len(methods), 1)
    xs = list(range(len(order)))
    for mi, method in enumerate(methods):
        vals = []
        for cid in order:
            hit = next((r for r in cards if r["card_id"] == cid and r["method"] == method), None)
            vals.append(float("nan") if hit is None else hit[metric])
        offset = (mi - 0.5 * (len(methods) - 1)) * width
        ax.bar([x + offset for x in xs], vals, width=width, label=method)
    ax.set_xticks(xs)
    ax.set_xticklabels(order, rotation=25, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(fontsize=8)
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    path = out / f"{stem}.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def _scatter_theta(out: Path, cards: list[dict], iso: tuple[EvaluatedTheta, ...]) -> Path:
    fig, ax = plt.subplots(figsize=(6.2, 5.0))
    ax.scatter(
        [r.theta.vmax_mps for r in iso],
        [r.theta.dmin_m for r in iso],
        s=18,
        c="#9aa0a6",
        label="ISO Pareto P",
        zorder=1,
    )
    markers = {
        "proposed": "o",
        "b3_keyword": "s",
        "b4_discrete_mode": "^",
        "safeopt": "D",
    }
    for row in cards:
        ax.scatter(
            [row["vmax_mps"]],
            [row["dmin_m"]],
            s=70,
            marker=markers.get(row["method"], "o"),
            label=f"{row['card_id']}/{row['method']}",
            zorder=2,
        )
    # Deduplicate legend entries loosely
    handles, labels = ax.get_legend_handles_labels()
    seen = set()
    uniq_h, uniq_l = [], []
    for h, lab in zip(handles, labels):
        if lab in seen:
            continue
        seen.add(lab)
        uniq_h.append(h)
        uniq_l.append(lab)
    ax.legend(
        uniq_h,
        uniq_l,
        fontsize=7,
        loc="upper left",
        bbox_to_anchor=(1.02, 1.0),
        borderaxespad=0.0,
        frameon=False,
    )
    ax.set_xlabel(r"$v_{\max}$ [m/s]")
    ax.set_ylabel(r"$d_{\min}$ [m]")
    ax.set_title("Condition-card θ on ISO front")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = out / "cards_theta.png"
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fronts", type=Path, required=True)
    parser.add_argument("--cards", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    fronts = json.loads(args.fronts.read_text(encoding="utf-8"))
    cards_payload = json.loads(args.cards.read_text(encoding="utf-8"))
    iso = rows_from_items(fronts["methods"]["nsga2"]["iso_pareto"])
    if not iso:
        iso = rows_from_items(fronts.get("combined_iso_pareto") or [])
    write_method_plots(args.out / "nsga2", "nsga2", iso)
    table = enumerate_table(iso)
    write_ab_table_plot(args.out, table)

    rows = cards_payload["results"]
    lang = [r for r in rows if r["family"] == "language"]
    safe = [r for r in rows if r["family"] == "safety_search"]
    paths = [
        _bar_compare(
            args.out,
            lang,
            metric="tt_s",
            ylabel="TT [s] (lower better)",
            title="Language contrast: TT",
            stem="language_tt",
        ),
        _bar_compare(
            args.out,
            lang,
            metric="smin_m",
            ylabel=r"$S_{\min}$ [m] (higher better)",
            title="Language contrast: realized min separation",
            stem="language_smin",
        ),
        _bar_compare(
            args.out,
            safe,
            metric="tt_s",
            ylabel="TT [s] (lower better)",
            title="Safety-search contrast: TT",
            stem="safety_tt",
        ),
        _bar_compare(
            args.out,
            safe,
            metric="smin_m",
            ylabel=r"$S_{\min}$ [m] (higher better)",
            title="Safety-search contrast: realized min separation",
            stem="safety_smin",
        ),
        _scatter_theta(args.out, rows, iso),
    ]
    meta = {"figures": [str(p) for p in paths]}
    (args.out / "paper_figures.json").write_text(json.dumps(meta, indent=2) + "\n")
    for p in paths:
        print(p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
