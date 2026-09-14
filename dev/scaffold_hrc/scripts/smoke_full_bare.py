#!/usr/bin/env python3
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

from constraints.pareto import Theta
from oracle.erect_bare import BareErectConfig, run_bare_erection
from scene.erect_plan import build_erect_sequence
from scene.geometry import STAGE1_GEOM

full_n = len(build_erect_sequence(STAGE1_GEOM))
print("full_items", full_n)

r = run_bare_erection(
    theta=Theta(1.0, 0.35),
    config=BareErectConfig(record_trace=False, timeout_s=50000, dt_s=0.25),
)
print(
    {
        "filled": r.n_filled,
        "sockets": r.n_sockets,
        "ok": r.completed,
        "T": round(r.makespan_s, 1),
        "ssm_s": round(r.ssm_s, 1),
        "wait_s": round(r.wait_s, 1),
        "smin": None
        if r.min_separation_m is None
        else round(r.min_separation_m, 3),
    }
)
