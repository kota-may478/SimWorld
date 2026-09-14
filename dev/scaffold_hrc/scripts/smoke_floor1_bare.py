#!/usr/bin/env python3
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

from constraints.pareto import Theta
from oracle.erect_bare import BareErectConfig, run_bare_erection

r = run_bare_erection(
    theta=Theta(1.0, 0.35),
    config=BareErectConfig(
        max_floors=1,
        boards_per_floor=2,
        record_trace=False,
        timeout_s=20000,
        dt_s=0.2,
    ),
)
print(
    {
        "filled": r.n_filled,
        "sockets": r.n_sockets,
        "ok": r.completed,
        "T": round(r.makespan_s, 1),
        "attrs": [a for a in dir(r) if not a.startswith("_")],
    }
)
