#!/usr/bin/env python3
"""Instrument bare erect until stall; print last reserved item and agent poses."""
from __future__ import annotations

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

# Patch by running a thin wrapper that logs every fill
from constraints.pareto import Theta
from oracle import erect_bare as eb
from scene.erect_plan import build_erect_sequence
from scene.geometry import STAGE1_GEOM

orig = eb.run_bare_erection

# Monkey: wrap run with more timeout and dump by re-implementing via source inspection
# Instead, call with max_items around 52..60
seq = build_erect_sequence(STAGE1_GEOM)
print("around_50", [seq[i].item_id for i in range(48, min(60, len(seq)))])

for n in (50, 52, 53, 55, 58, 60):
    r = eb.run_bare_erection(
        theta=Theta(1.0, 0.35),
        config=eb.BareErectConfig(
            max_items=n, record_trace=False, timeout_s=30000, dt_s=0.25
        ),
    )
    print(f"n={n} filled={r.n_filled}/{r.n_sockets} ok={r.completed} T={r.makespan_s:.0f}")
