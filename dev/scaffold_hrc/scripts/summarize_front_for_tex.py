#!/usr/bin/env python3
"""Print key Sec.V numbers from a fronts.json + cards for root.tex update."""
from __future__ import annotations

import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

from constraints.pareto import nondominated
from fronts.ab_map import enumerate_table
from scripts.make_paper_figures import rows_from_items

fronts = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
iso = rows_from_items(fronts["methods"]["nsga2"]["iso_pareto"])
grid_iso = rows_from_items(fronts["methods"]["grid"]["iso_pareto"])
nsga_rows = fronts["methods"]["nsga2"]["n_rows"]
print("t_ref", fronts.get("t_ref_s"))
print("nsga_rows", nsga_rows, "nsga_iso", len(iso), "grid_iso", len(grid_iso))
print("combined_iso", len(fronts.get("combined_iso_pareto") or []))
tts = [r.tt for r in iso]
vm = [r.theta.vmax_mps for r in iso]
sm = [r.min_sep_m for r in iso]
si = [r.si_min for r in iso]
print("TT", min(tts), max(tts))
print("vmax", min(vm), max(vm))
print("smin", min(sm), max(sm))
print("si_min_floor", min(si))
table = enumerate_table(iso)
keys = {
    (round(table[(a, b)].theta.vmax_mps, 4), round(table[(a, b)].theta.dmin_m, 4))
    for a in [0.0, 0.25, 0.5, 0.75, 1.0]
    for b in [0.0, 0.25, 0.5, 0.75, 1.0]
}
print("ab_distinct_theta", len(keys))
# alpha=0 row
a0 = [table[(0.0, b)] for b in [0.0, 0.25, 0.5, 0.75, 1.0]]
print(
    "a0_theta",
    a0[0].theta.vmax_mps,
    a0[0].theta.dmin_m,
    "tt",
    a0[0].tt,
    "all_same",
    len({(round(r.theta.vmax_mps, 4), round(r.theta.dmin_m, 4)) for r in a0}) == 1,
)
