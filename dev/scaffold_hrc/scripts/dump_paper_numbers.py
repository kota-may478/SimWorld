#!/usr/bin/env python3
"""Print paper-facing numbers from a fronts.json + condition_cards.json."""

from __future__ import annotations

import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

from constraints.pareto import EvaluatedTheta, Theta  # noqa: E402
from fronts.ab_map import LEVELS, enumerate_table  # noqa: E402


def rows_iso(payload: dict) -> tuple[EvaluatedTheta, ...]:
    items = payload["methods"]["nsga2"]["iso_pareto"]
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


def main() -> int:
    fronts_path = Path(sys.argv[1]) if len(sys.argv) > 1 else PKG / "out/20260913124333/fronts.json"
    cards_path = Path(sys.argv[2]) if len(sys.argv) > 2 else PKG / "out/20260913124333/cards/condition_cards.json"
    d = json.loads(fronts_path.read_text(encoding="utf-8"))
    iso = rows_iso(d)
    table = enumerate_table(iso)
    named = {
        "efficient": (0.0, 0.5),
        "normal_distance": (0.5, 0.0),
        "normal": (0.5, 0.5),
        "normal_slow": (0.5, 1.0),
        "safe_distance": (1.0, 0.0),
        "safe": (1.0, 0.5),
        "safe_slow": (1.0, 1.0),
    }
    keys = {
        (round(row.theta.vmax_mps, 4), round(row.theta.dmin_m, 4))
        for row in table.values()
    }
    tts = [r.tt for r in iso]
    sm = [r.min_sep_m for r in iso]
    vm = [r.theta.vmax_mps for r in iso]
    print("run", fronts_path)
    print("sockets_per_floor", d.get("sockets_per_floor"))
    print("t_ref_s", round(float(d["t_ref_s"]), 1))
    print("n_unique_evals", d.get("n_unique_evals"))
    print("grid", d["methods"]["grid"]["n_rows"], "iso", d["methods"]["grid"]["n_iso_pareto"])
    print("nsga2", d["methods"]["nsga2"]["n_rows"], "iso", d["methods"]["nsga2"]["n_iso_pareto"])
    print("combined_iso", len(d.get("combined_iso_pareto") or []))
    print("TT_range", round(min(tts), 1), round(max(tts), 1))
    print("vmax_range", round(min(vm), 2), round(max(vm), 2))
    print("smin_range", round(min(sm), 2), round(max(sm), 2))
    print("si_min_min", round(min(r.si_min for r in iso), 3))
    print("ab_distinct", len(keys))
    print("alpha0_row:")
    for b in LEVELS:
        row = table[(0.0, b)]
        print(
            " ",
            b,
            round(row.theta.vmax_mps, 4),
            round(row.theta.dmin_m, 4),
            round(row.tt, 1),
            round(row.min_sep_m, 3),
        )
    print("named_cells:")
    for name, key in named.items():
        row = table[key]
        print(
            name,
            key,
            round(row.theta.vmax_mps, 4),
            round(row.theta.dmin_m, 4),
            round(row.tt, 1),
            round(row.min_sep_m, 3),
            round(row.si_min, 3),
        )
    if cards_path.is_file():
        cards = json.loads(cards_path.read_text(encoding="utf-8"))
        print("cards", len(cards["results"]))
        for row in cards["results"]:
            print(
                row["card_id"],
                row["method"],
                round(row["vmax_mps"], 4),
                round(row["dmin_m"], 4),
                round(row["tt_s"], 1),
                round(row["smin_m"], 3),
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
