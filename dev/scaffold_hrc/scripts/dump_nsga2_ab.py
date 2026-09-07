#!/usr/bin/env python3
"""Print NSGA-II ISO 5x5 (alpha, beta) -> theta uniqueness."""

from __future__ import annotations

import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from constraints.pareto import EvaluatedTheta, Theta  # noqa: E402
from fronts.ab_map import LEVELS, enumerate_table  # noqa: E402

DEFAULT = PKG / "out" / "20260904203058" / "fronts.json"


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


def main() -> int:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT
    payload = json.loads(path.read_text(encoding="utf-8"))
    nsga = payload["methods"]["nsga2"]["iso_pareto"]
    combined = payload.get("combined_iso_pareto") or []
    nsga_rows = rows_from_items(nsga)
    table = enumerate_table(nsga_rows)
    print("nsga2_iso", len(nsga_rows))
    print("combined_iso", len(combined))
    print("alpha beta vmax dmin tt smin")
    distinct: set[tuple[float, float]] = set()
    for alpha in LEVELS:
        thetas = []
        for beta in LEVELS:
            picked = table[(alpha, beta)]
            key = (
                round(picked.theta.vmax_mps, 4),
                round(picked.theta.dmin_m, 4),
            )
            distinct.add(key)
            thetas.append(key)
            print(
                f"{alpha:.2f} {beta:.2f} "
                f"{picked.theta.vmax_mps:.4f} {picked.theta.dmin_m:.4f} "
                f"{picked.tt:.1f} {picked.min_sep_m:.3f}"
            )
        print("  unique_theta_for_alpha", alpha, len(set(thetas)))
    print("unique_theta_in_5x5", len(distinct))
    zeros = {distinct_key for distinct_key in (
        (
            round(table[(0.0, b)].theta.vmax_mps, 4),
            round(table[(0.0, b)].theta.dmin_m, 4),
        )
        for b in LEVELS
    )}
    print("alpha0_unique_theta", len(zeros))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
