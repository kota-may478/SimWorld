#!/usr/bin/env python3
"""Evaluate condition cards on a fronts.json ISO Pareto and write paper tables."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from constraints.pareto import EvaluatedTheta, Theta  # noqa: E402
from fronts.ab_map import enumerate_table, pick_weighted  # noqa: E402
from fronts.baselines_language import discrete_3mode  # noqa: E402
from fronts.condition_cards import condition_cards  # noqa: E402
from fronts.evaluate import OracleEvaluator, measure_t_ref, opt_config  # noqa: E402
from fronts.grounding import b3_keyword, b4_discrete_mode, proposed_from_text, score_grounding  # noqa: E402
from fronts.language import Preference  # noqa: E402
from fronts.space import REF_THETA  # noqa: E402
from oracle.simulate import OracleConfig  # noqa: E402
from paths import make_run_dir  # noqa: E402


def rows_from_items(items: list[dict]) -> tuple[EvaluatedTheta, ...]:
    rows: list[EvaluatedTheta] = []
    for item in items:
        rows.append(
            EvaluatedTheta(
                Theta(vmax_mps=item["vmax_mps"], dmin_m=item["dmin_m"]),
                tt=float(item["tt"]),
                t_ssm=float(item.get("t_ssm", item["vmax_mps"])),
                si_min=float(item.get("si_min", 1.0)),
                completed=bool(item.get("completed", True)),
                min_sep_m=float(item.get("min_sep_m", 0.0)),
                mission_s=float(item.get("mission_s", 0.0)),
            )
        )
    return tuple(rows)


def _dump(row: EvaluatedTheta, *, method: str, card_id: str, extra: dict) -> dict:
    return {
        "card_id": card_id,
        "method": method,
        "vmax_mps": round(row.theta.vmax_mps, 4),
        "dmin_m": round(row.theta.dmin_m, 4),
        "tt_s": round(row.tt, 2),
        "smin_m": round(row.min_sep_m, 4),
        "si_min": round(row.si_min, 4),
        "iso_feasible": bool(row.iso_feasible),
        "completed": bool(row.completed),
        **extra,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Score paper condition cards")
    parser.add_argument("--fronts", type=Path, required=True)
    parser.add_argument("--sockets-per-floor", type=int, default=4)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    payload = json.loads(args.fronts.read_text(encoding="utf-8"))
    nsga = rows_from_items(payload["methods"]["nsga2"]["iso_pareto"])
    if not nsga:
        nsga = rows_from_items(payload.get("combined_iso_pareto") or [])
    if not nsga:
        print("FAIL: empty ISO Pareto in", args.fronts)
        return 1
    safe_inc = payload.get("safeopt_incumbent") or payload["methods"].get("safe_bo", {}).get(
        "iso_pareto", [None]
    )
    if isinstance(safe_inc, list):
        safe_inc = safe_inc[0] if safe_inc else None
    if not safe_inc:
        print("FAIL: missing SafeOpt incumbent")
        return 1

    out_dir = args.out or make_run_dir()
    out_dir.mkdir(parents=True, exist_ok=True)
    base = OracleConfig(
        sockets_per_floor=args.sockets_per_floor,
        record_trace=False,
        timeout_s=36000.0,
    )
    config = opt_config(base)
    t_ref = measure_t_ref(config, REF_THETA, bare_boards_per_floor=None)
    ev = OracleEvaluator(config=config, t_ref_s=t_ref, bare_boards_per_floor=None)
    table = enumerate_table(nsga)

    results: list[dict] = []
    for card in condition_cards():
        state = Preference(card.start_alpha, card.start_beta)
        for method in card.methods:
            if method == "proposed":
                if card.family == "safety_search":
                    picked = pick_weighted(nsga, card.start_alpha, card.start_beta)
                    scored = ev.evaluate(picked.theta)
                    results.append(
                        _dump(
                            scored,
                            method=method,
                            card_id=card.card_id,
                            extra={
                                "family": card.family,
                                "text": card.text,
                                "alpha": card.start_alpha,
                                "beta": card.start_beta,
                                "notes": card.notes,
                            },
                        )
                    )
                else:
                    g = proposed_from_text(card.text, nsga, state=state)
                    scored = score_grounding(g, ev)
                    row = ev.evaluate(scored.theta)
                    results.append(
                        _dump(
                            row,
                            method=method,
                            card_id=card.card_id,
                            extra={
                                "family": card.family,
                                "text": card.text,
                                "alpha": scored.alpha,
                                "beta": scored.beta,
                                "cell": scored.cell,
                                "notes": card.notes,
                            },
                        )
                    )
            elif method == "b3_keyword":
                g = score_grounding(b3_keyword(card.text), ev)
                row = ev.evaluate(g.theta)
                results.append(
                    _dump(
                        row,
                        method=method,
                        card_id=card.card_id,
                        extra={
                            "family": card.family,
                            "text": card.text,
                            "notes": g.notes,
                        },
                    )
                )
            elif method == "b4_discrete_mode":
                pref = discrete_3mode(card.text, state)
                mode = "efficient" if pref.alpha < 0.25 else ("safe" if pref.alpha > 0.75 else "normal")
                g = score_grounding(b4_discrete_mode(mode), ev)
                row = ev.evaluate(g.theta)
                results.append(
                    _dump(
                        row,
                        method=method,
                        card_id=card.card_id,
                        extra={
                            "family": card.family,
                            "text": card.text,
                            "mode": mode,
                            "notes": card.notes,
                        },
                    )
                )
            elif method == "safeopt":
                theta = Theta(vmax_mps=safe_inc["vmax_mps"], dmin_m=safe_inc["dmin_m"])
                row = ev.evaluate(theta)
                results.append(
                    _dump(
                        row,
                        method=method,
                        card_id=card.card_id,
                        extra={
                            "family": card.family,
                            "text": card.text,
                            "notes": "SafeOpt incumbent from fronts run",
                        },
                    )
                )
            else:
                raise ValueError(method)

    summary = {
        "source_fronts": str(args.fronts),
        "sockets_per_floor": args.sockets_per_floor,
        "t_ref_s": t_ref,
        "n_iso_pareto": len(nsga),
        "safeopt_incumbent": safe_inc,
        "ab_table_cells": len(table),
        "results": results,
    }
    out_path = out_dir / "condition_cards.json"
    out_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(out_path)
    for row in results:
        print(
            row["card_id"],
            row["method"],
            f"tt={row['tt_s']}",
            f"smin={row['smin_m']}",
            f"v={row['vmax_mps']}",
            f"d={row['dmin_m']}",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
