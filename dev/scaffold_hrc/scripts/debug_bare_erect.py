#!/usr/bin/env python3
from constraints.pareto import Theta
from oracle.erect_bare import BareErectConfig, run_bare_erection

cfg = BareErectConfig(max_items=3, record_trace=True, timeout_s=120.0, dt_s=0.1)
r = run_bare_erection(theta=Theta(1.0, 0.35), config=cfg)
print("completed", r.completed, "filled", r.n_filled, "/", r.n_sockets, "T", r.makespan_s)
if r.trace:
    for s in r.trace[::50][:20]:
        print(
            f"t={s.t_s:.1f} spot=({s.spot[0]:.2f},{s.spot[1]:.2f},{s.spot[2]:.2f}) "
            f"hum=({s.human[0]:.2f},{s.human[1]:.2f},{s.human[2]:.2f}) "
            f"filled={s.n_filled} blocked={s.blocked} corr={s.in_corridor} v={s.spot_speed_mps:.2f}"
        )
    print("last", r.trace[-1])
