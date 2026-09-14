#!/usr/bin/env python3
import sys
from pathlib import Path
PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

from constraints.pareto import Theta
from oracle import erect_bare as eb
from scene.erect_plan import build_erect_sequence, limit_erect_sequence
from scene.geometry import STAGE1_GEOM

# Monkeypatch loop with progress prints by wrapping run - simpler: copy minimal state dump
# Instrument by calling internal sequence and a shortened custom loop using public API with max_items increasing
for n in (5, 10, 12, 15, 20):
    r = eb.run_bare_erection(
        theta=Theta(1.0, 0.35),
        config=eb.BareErectConfig(max_items=n, record_trace=False, timeout_s=5000, dt_s=0.2),
    )
    print(f"max_items={n} filled={r.n_filled}/{r.n_sockets} ok={r.completed} T={r.makespan_s:.1f}")

seq = limit_erect_sequence(build_erect_sequence(STAGE1_GEOM), max_items=15)
print("items", [i.item_id for i in seq])
