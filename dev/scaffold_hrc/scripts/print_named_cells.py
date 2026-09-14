#!/usr/bin/env python3
import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

from fronts.ab_map import enumerate_table
from scripts.make_paper_figures import rows_from_items

f = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
iso = rows_from_items(f["methods"]["nsga2"]["iso_pareto"])
t = enumerate_table(iso)
names = [
    ("efficient", 0.0, 0.5),
    ("normal_distance", 0.5, 0.0),
    ("normal", 0.5, 0.5),
    ("normal_slow", 0.5, 1.0),
    ("safe_distance", 1.0, 0.0),
    ("safe", 1.0, 0.5),
    ("safe_slow", 1.0, 1.0),
]
for name, a, b in names:
    r = t[(a, b)]
    print(
        name,
        round(r.theta.vmax_mps, 4),
        round(r.theta.dmin_m, 4),
        round(r.tt, 1),
        round(r.min_sep_m, 4),
        round(r.si_min, 4),
    )
nd = {(round(t[(a, b)].theta.vmax_mps, 4), round(t[(a, b)].theta.dmin_m, 4)) for a in (0.0, 0.25, 0.5, 0.75, 1.0) for b in (0.0, 0.25, 0.5, 0.75, 1.0)}
print("distinct", len(nd))
print("same_nd_sd", t[(0.5, 0.0)].theta.as_dict() == t[(1.0, 0.0)].theta.as_dict())
