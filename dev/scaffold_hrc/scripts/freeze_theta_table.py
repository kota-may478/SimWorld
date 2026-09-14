#!/usr/bin/env python3
"""Write the frozen 5x5 ISO-front index used by UE5 validation."""

from __future__ import annotations

import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from constraints.pareto import EvaluatedTheta, Theta
from fronts.ab_map import LEVELS, enumerate_table, theta_key
from fronts.catalog import load_catalog

DEFAULT_FRONTS = PKG / "out" / "20260908153515" / "fronts.json"
OUTPUT = PKG / "data" / "frozen_theta_table.json"
RUN_ID = "20260908153515"


def rows_from_items(items: list[dict]) -> tuple[EvaluatedTheta, ...]:
    rows: list[EvaluatedTheta] = []
    for item in items:
        rows.append(
            EvaluatedTheta(
                Theta(vmax_mps=item["vmax_mps"], dmin_m=item["dmin_m"]),
                tt=item["tt"],
                t_ssm=item.get("t_ssm", item["vmax_mps"]),
                si_min=item.get("si_min", 1.0),
                completed=bool(item.get("completed", True)),
                min_sep_m=item.get("min_sep_m", 0.0),
            )
        )
    return tuple(rows)


def _cell(row: EvaluatedTheta, theta_id: str, alpha: float, beta: float) -> dict:
    return {
        "theta_id": theta_id,
        "alpha": alpha,
        "beta": beta,
        "vmax_mps": round(row.theta.vmax_mps, 4),
        "dmin_m": round(row.theta.dmin_m, 4),
        "tt_s": round(row.tt, 1),
        "smin_m": round(row.min_sep_m, 3),
        "si_min": round(row.si_min, 3),
    }


def main() -> int:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_FRONTS
    payload = json.loads(path.read_text(encoding="utf-8"))
    nsga = rows_from_items(payload["methods"]["nsga2"]["iso_pareto"])
    table = enumerate_table(nsga)
    ids: dict[tuple[float, float], str] = {}
    cells: list[dict] = []
    for alpha in LEVELS:
        for beta in LEVELS:
            row = table[(alpha, beta)]
            key = theta_key(row)
            if key not in ids:
                ids[key] = f"T{len(ids) + 1}"
            cells.append(_cell(row, ids[key], alpha, beta))
    named: list[dict] = []
    for entry in load_catalog():
        if entry.kind != "absolute" or entry.alpha is None or entry.beta is None:
            continue
        row = table[(entry.alpha, entry.beta)]
        named.append(
            {
                "cell": entry.label,
                **_cell(row, ids[theta_key(row)], entry.alpha, entry.beta),
            }
        )
    distinct = [
        {
            "theta_id": theta_id,
            "vmax_mps": key[0],
            "dmin_m": key[1],
        }
        for key, theta_id in ids.items()
    ]
    out = {
        "source_run": RUN_ID,
        "nsga2_iso_points": len(nsga),
        "grid_cells": 25,
        "distinct_theta": len(ids),
        "named_absolute_cells": len(named),
        "cells": cells,
        "named": named,
        "distinct": distinct,
    }
    OUTPUT.write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    print("wrote", OUTPUT)
    print("distinct_theta", len(ids))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
