#!/usr/bin/env python3
import time
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

from fronts.evaluate import OracleEvaluator, measure_t_ref, opt_config
from fronts.space import REF_THETA
from oracle.simulate import OracleConfig

t0 = time.time()
base = OracleConfig(sockets_per_floor=4, record_trace=False, timeout_s=36000.0)
config = opt_config(base)
t_ref = measure_t_ref(config, REF_THETA, bare_boards_per_floor=4)
print("t_ref", round(t_ref, 1), "wall", round(time.time() - t0, 2))

t1 = time.time()
ev = OracleEvaluator(config=config, t_ref_s=t_ref, bare_boards_per_floor=4)
row = ev.evaluate(REF_THETA)
print(
    "eval",
    row.completed,
    round(row.mission_s, 1),
    "tt",
    round(row.tt, 3),
    "wall",
    round(time.time() - t1, 2),
)
