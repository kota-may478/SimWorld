#!/usr/bin/env python3
"""Group 5x5 cells that share the same NSGA-II theta."""

from __future__ import annotations

import json
import sys
from collections import defaultdict
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from constraints.pareto import EvaluatedTheta, Theta  # noqa: E402
from fronts.ab_map import LEVELS, enumerate_table, mix_weights  # noqa: E402

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
    table = enumerate_table(rows_from_items(payload["methods"]["nsga2"]["iso_pareto"]))
    groups: dict[tuple[float, float], list] = defaultdict(list)
    for alpha in LEVELS:
        for beta in LEVELS:
            picked = table[(alpha, beta)]
            key = (
                round(picked.theta.vmax_mps, 4),
                round(picked.theta.dmin_m, 4),
            )
            w = mix_weights(alpha, beta)
            groups[key].append(
                {
                    "alpha": alpha,
                    "beta": beta,
                    "lambda": w,
                    "tt": round(picked.tt, 1),
                    "smin": round(picked.min_sep_m, 3),
                }
            )
    print("n_groups", len(groups))
    for i, (key, cells) in enumerate(
        sorted(groups.items(), key=lambda kv: (kv[1][0]["tt"], key[0])),
        start=1,
    ):
        print(
            f"T{i} vmax={key[0]:.4f} dmin={key[1]:.4f} "
            f"tt={cells[0]['tt']} smin={cells[0]['smin']} n_cells={len(cells)}"
        )
        for cell in cells:
            w = cell["lambda"]
            print(
                f"  a={cell['alpha']:.2f} b={cell['beta']:.2f} "
                f"lam=({w[0]:.3f},{w[1]:.3f},{w[2]:.3f})"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
