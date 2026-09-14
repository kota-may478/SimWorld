#!/usr/bin/env python3
"""Export validation figures for the ICRA paper into Paper/fig/.

Uses the canonical 0908 fronts + condition-card results.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

os.environ["MPLBACKEND"] = "Agg"

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402

from constraints.pareto import EvaluatedTheta, Theta  # noqa: E402
from fronts.ab_map import LEVELS, enumerate_table  # noqa: E402
from fronts.viz_fronts import write_ab_table_plot, write_front_comparison  # noqa: E402
from scripts.make_paper_figures import (  # noqa: E402
    _bar_compare,
    _scatter_theta,
    rows_from_items,
)


PAPER_FIG = (
    PKG
    / "docs"
    / "Paper"
    / "Conference_IEEE_ICRA2027"
    / "Paper"
    / "fig"
)


def load_iso(fronts: dict) -> tuple[EvaluatedTheta, ...]:
    return rows_from_items(fronts["methods"]["nsga2"]["iso_pareto"])


def load_methods(fronts: dict) -> dict[str, tuple[EvaluatedTheta, ...]]:
    out: dict[str, tuple[EvaluatedTheta, ...]] = {}
    for name, block in fronts["methods"].items():
        rows = block.get("rows") or block.get("iso_pareto") or []
        if "rows" in block:
            out[name] = rows_from_items(block["rows"])
        else:
            # fall back: samples may be in method dir; use iso + empty samples
            out[name] = rows_from_items(block.get("iso_pareto") or [])
    return out


def fig7_theta_identity(out: Path, table: dict) -> Path:
    """5x5 map shaded by distinct theta identity with vmax/dmin labels."""
    keys = []
    for a in LEVELS:
        for b in LEVELS:
            row = table[(a, b)]
            key = (round(row.theta.vmax_mps, 4), round(row.theta.dmin_m, 4))
            if key not in keys:
                keys.append(key)
    index = {k: i for i, k in enumerate(keys)}
    grid = []
    for a in LEVELS:
        row_ids = []
        for b in LEVELS:
            row = table[(a, b)]
            key = (round(row.theta.vmax_mps, 4), round(row.theta.dmin_m, 4))
            row_ids.append(index[key])
        grid.append(row_ids)

    cmap = ListedColormap(plt.cm.tab20.colors[: max(len(keys), 1)])
    fig, ax = plt.subplots(figsize=(6.2, 5.4))
    ax.imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=max(len(keys) - 1, 1))
    ax.set_xticks(range(len(LEVELS)))
    ax.set_yticks(range(len(LEVELS)))
    ax.set_xticklabels([f"{v:.2f}" for v in LEVELS])
    ax.set_yticklabels([f"{v:.2f}" for v in LEVELS])
    ax.set_xlabel(r"$\beta$ (0=distance, 1=slow)")
    ax.set_ylabel(r"$\alpha$ (0=efficiency, 1=safety)")
    for ia, a in enumerate(LEVELS):
        for ib, b in enumerate(LEVELS):
            row = table[(a, b)]
            ax.text(
                ib,
                ia,
                f"{row.theta.vmax_mps:.2f}\n{row.theta.dmin_m:.2f}",
                ha="center",
                va="center",
                fontsize=7,
                color="white" if index[(round(row.theta.vmax_mps, 4), round(row.theta.dmin_m, 4))] > len(keys) / 2 else "black",
            )
    ax.set_title(f"5×5 preference map ({len(keys)} distinct $\\theta$)")
    fig.tight_layout()
    path = out / "fig7_abmap.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def _ue_num(value) -> float:
    if value is None:
        return float("nan")
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def oracle_vs_ue_bars(out: Path, cards: list[dict], ue_rows: list[dict] | None) -> list[Path]:
    """Export UE thinned-replay figures.

    Absolute oracle TT (full bare erect) is not scale-comparable to UE max-items=3,
    so bars are UE-only. Oracle card metrics are kept in the export JSON for tables.
    """
    paths: list[Path] = []
    if not ue_rows:
        return paths
    merged = []
    for hit in ue_rows:
        if not hit.get("ok"):
            continue
        o = next(
            (
                c
                for c in cards
                if c.get("card_id") == hit.get("card_id")
                and c.get("method") == hit.get("method")
            ),
            None,
        )
        merged.append(
            {
                "label": f"{hit.get('card_id')}/{hit.get('method')}",
                "tt_ue": _ue_num(hit.get("tt_s")),
                "scaffold_tt_ue": _ue_num(hit.get("scaffold_tt_s")),
                "smin_ue": _ue_num(hit.get("smin_m")),
                "tt_oracle": _ue_num(None if o is None else o.get("tt_s")),
                "smin_oracle": _ue_num(None if o is None else o.get("smin_m")),
            }
        )
    if not merged:
        return paths
    labels = [m["label"] for m in merged]
    xs = list(range(len(labels)))
    for key, ylabel, stem, title in (
        (
            "tt_ue",
            "UE TT [s]",
            "fig_ue_tt",
            "UE5 full three-floor bare erect (174 members): transport time",
        ),
        (
            "scaffold_tt_ue",
            "UE scaffold TT [s]",
            "fig_ue_scaffold_tt",
            "UE5 full three-floor bare erect (174 members): on-scaffold time",
        ),
    ):
        vals = [m[key] for m in merged]
        if not any(v == v for v in vals):
            continue
        fig, ax = plt.subplots(figsize=(11.0, 4.4))
        ax.bar(xs, vals, color="#3B6EA5")
        ax.set_xticks(xs)
        ax.set_xticklabels(labels, rotation=30, ha="right", fontsize=7)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.3)
        fig.tight_layout()
        path = out / f"{stem}.png"
        fig.savefig(path, dpi=200)
        plt.close(fig)
        paths.append(path)
    (out / "ue_card_metrics.json").write_text(
        json.dumps({"rows": merged}, indent=2) + "\n", encoding="utf-8"
    )
    return paths


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--fronts",
        type=Path,
        default=PKG / "out/20260913124333/fronts.json",
    )
    parser.add_argument(
        "--cards",
        type=Path,
        default=PKG / "out/20260913124333/cards/condition_cards.json",
    )
    parser.add_argument("--ue-results", type=Path, default=None)
    parser.add_argument("--out", type=Path, default=PAPER_FIG)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    fronts = json.loads(args.fronts.read_text(encoding="utf-8"))
    cards_payload = json.loads(args.cards.read_text(encoding="utf-8"))
    cards = cards_payload["results"]
    iso = load_iso(fronts)

    # Prefer full sample rows from method folders if present
    methods: dict[str, tuple[EvaluatedTheta, ...]] = {}
    for name in ("grid", "nsga2", "safe_bo"):
        sample = args.fronts.parent / name / "samples.json"
        if sample.is_file():
            payload = json.loads(sample.read_text(encoding="utf-8"))
            items = payload.get("rows") if isinstance(payload, dict) else payload
            methods[name] = rows_from_items(items or [])
        else:
            methods[name] = rows_from_items(fronts["methods"][name].get("iso_pareto") or [])

    write_front_comparison(args.out, methods)
    # Rename comparison plots to paper fig names
    shutil.copy(args.out / "comparison_objectives.png", args.out / "fig5_objectives.png")
    for src, dst in (
        ("comparison_objectives_vmax_tt.png", "fig5a_vmax_tt.png"),
        ("comparison_objectives_smin_tt.png", "fig5b_smin_tt.png"),
        ("comparison_objectives_vmax_smin.png", "fig5c_vmax_smin.png"),
    ):
        shutil.copy(args.out / src, args.out / dst)
    shutil.copy(args.out / "comparison_theta.png", args.out / "fig6_theta.png")

    table = enumerate_table(iso)
    write_ab_table_plot(args.out, table)
    fig7_theta_identity(args.out, table)
    shutil.copy(args.out / "ab_table.png", args.out / "fig7_abmap_heat.png")

    lang = [r for r in cards if r["family"] == "language"]
    safe = [r for r in cards if r["family"] == "safety_search"]
    paths = [
        _bar_compare(
            args.out,
            lang,
            metric="tt_s",
            ylabel="TT [s] (lower better)",
            title="Language condition cards: TT (oracle)",
            stem="fig8_language_tt",
        ),
        _bar_compare(
            args.out,
            lang,
            metric="smin_m",
            ylabel=r"$S_{\min}$ [m] (higher better)",
            title="Language condition cards: $S_{\\min}$ (oracle)",
            stem="fig8_language_smin",
        ),
        _bar_compare(
            args.out,
            safe,
            metric="tt_s",
            ylabel="TT [s] (lower better)",
            title="SafeOpt vs proposed: TT (oracle)",
            stem="fig9_safety_tt",
        ),
        _bar_compare(
            args.out,
            safe,
            metric="smin_m",
            ylabel=r"$S_{\min}$ [m] (higher better)",
            title="SafeOpt vs proposed: $S_{\\min}$ (oracle)",
            stem="fig9_safety_smin",
        ),
        _scatter_theta(args.out, cards, iso),
    ]
    shutil.copy(args.out / "cards_theta.png", args.out / "fig8_cards_theta.png")

    ue_rows = None
    if args.ue_results and args.ue_results.is_file():
        ue_payload = json.loads(args.ue_results.read_text(encoding="utf-8"))
        ue_rows = ue_payload.get("results") or ue_payload
        paths.extend(oracle_vs_ue_bars(args.out, cards, ue_rows))

    meta = {
        "fronts": str(args.fronts),
        "cards": str(args.cards),
        "ue_results": str(args.ue_results) if args.ue_results else None,
        "figures": sorted(p.name for p in args.out.glob("fig*.png")),
        "n_iso": len(iso),
        "n_cards": len(cards),
    }
    (args.out / "export_manifest.json").write_text(
        json.dumps(meta, indent=2) + "\n", encoding="utf-8"
    )
    for p in paths:
        print(p)
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
