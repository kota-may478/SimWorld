#!/usr/bin/env python3
"""Discover Pareto fronts with several methods. Does not modify oracle/."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from constraints.pareto import EvaluatedTheta, nondominated  # noqa: E402
from fronts.epsilon_constraint import run_epsilon_constraint  # noqa: E402
from fronts.evaluate import OracleEvaluator, measure_t_ref, opt_config  # noqa: E402
from fronts.grid_sweep import run_grid  # noqa: E402
from fronts.lhs_sample import run_lhs  # noqa: E402
from fronts.nsga2 import run_nsga2  # noqa: E402
from fronts.safe_bo import run_safe_bo  # noqa: E402
from fronts.space import REF_THETA  # noqa: E402
from fronts.viz_fronts import write_front_comparison, write_method_plots  # noqa: E402
from fronts.weighted_sum import run_weighted_sum  # noqa: E402
from oracle.simulate import OracleConfig  # noqa: E402
from paths import make_run_dir  # noqa: E402


def _dump_rows(rows: tuple[EvaluatedTheta, ...]) -> list[dict]:
    return [
        {
            "dmin_m": r.theta.dmin_m,
            "vmax_mps": r.theta.vmax_mps,
            "jeff": r.jeff,
            "jsafe": r.jsafe,
            "completed": r.completed,
        }
        for r in rows
    ]


def _unique(rows: list[EvaluatedTheta]) -> tuple[EvaluatedTheta, ...]:
    seen: set[tuple[float, float]] = set()
    kept: list[EvaluatedTheta] = []
    for row in rows:
        key = (round(row.theta.dmin_m, 5), round(row.theta.vmax_mps, 5))
        if key in seen:
            continue
        seen.add(key)
        kept.append(row)
    return tuple(kept)


def main() -> int:
    parser = argparse.ArgumentParser(description="Discover scaffold_hrc Pareto fronts")
    parser.add_argument("--quick", action="store_true", help="tiny budgets for a smoke run")
    parser.add_argument("--sockets-per-floor", type=int, default=4)
    args = parser.parse_args()

    run_dir = make_run_dir()
    base = OracleConfig(sockets_per_floor=args.sockets_per_floor, record_trace=False)
    config = opt_config(base)
    t_ref = measure_t_ref(config, REF_THETA)

    def fresh() -> OracleEvaluator:
        return OracleEvaluator(config=config, t_ref_s=t_ref)

    if args.quick:
        jobs = {
            "grid": lambda ev: run_grid(ev, n_dmin=3, n_vmax=3),
            "lhs": lambda ev: run_lhs(ev, n_samples=6, seed=3),
            "nsga2": lambda ev: run_nsga2(ev, pop_size=6, n_gen=2, seed=5),
            "weighted_sum": lambda ev: run_weighted_sum(
                ev, weights=(0.0, 1.0, 2.0), restarts=1, steps=3, seed=8
            ),
            "epsilon_constraint": lambda ev: run_epsilon_constraint(
                ev, epsilons=(0.0, 0.05, 0.2), n_try=4, seed=9
            ),
            "safe_bo": lambda ev: run_safe_bo(
                ev, n_iter=4, n_dmin=5, n_vmax=4, d_lim=0.08, densify=True
            ),
        }
    else:
        jobs = {
            "grid": lambda ev: run_grid(ev, n_dmin=16, n_vmax=16),
            "lhs": lambda ev: run_lhs(ev, n_samples=200, seed=7),
            "nsga2": lambda ev: run_nsga2(ev, pop_size=32, n_gen=14, seed=11),
            "weighted_sum": lambda ev: run_weighted_sum(
                ev,
                weights=(0.0, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0),
                restarts=5,
                steps=10,
                seed=19,
            ),
            "epsilon_constraint": lambda ev: run_epsilon_constraint(
                ev,
                epsilons=(0.0, 0.005, 0.01, 0.02, 0.04, 0.06, 0.1, 0.15, 0.25, 0.4),
                n_try=24,
                seed=23,
            ),
            "safe_bo": lambda ev: run_safe_bo(
                ev, n_iter=48, n_dmin=16, n_vmax=12, d_lim=0.08, densify=True
            ),
        }

    packed: dict[str, tuple[EvaluatedTheta, ...]] = {}
    for name, job in jobs.items():
        ev = fresh()
        job(ev)
        packed[name] = _unique(list(ev.cache.values()))
        method_dir = run_dir / name
        write_method_plots(method_dir, name, packed[name])
        (method_dir / "samples.json").write_text(
            json.dumps(
                {
                    "n_rows": len(packed[name]),
                    "n_nondominated": len(nondominated(packed[name])),
                    "rows": _dump_rows(packed[name]),
                    "nondominated": _dump_rows(nondominated(packed[name])),
                },
                indent=2,
            ),
            encoding="utf-8",
        )

    union: list[EvaluatedTheta] = []
    for rows in packed.values():
        union.extend(rows)
    combined = nondominated(tuple(union))
    write_front_comparison(run_dir, packed)

    meta = {
        "run_dir": str(run_dir),
        "t_ref_s": t_ref,
        "ref_theta": {"dmin_m": REF_THETA.dmin_m, "vmax_mps": REF_THETA.vmax_mps},
        "sockets_per_floor": args.sockets_per_floor,
        "n_unique_evals": sum(len(rows) for rows in packed.values()),
        "methods": {
            name: {
                "n_rows": len(rows),
                "n_nondominated": len(nondominated(rows)),
                "nondominated": _dump_rows(nondominated(rows)),
            }
            for name, rows in packed.items()
        },
        "combined_nondominated": _dump_rows(combined),
    }
    (run_dir / "fronts.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(run_dir)
    print(
        json.dumps(
            {
                "n_evals": meta["n_unique_evals"],
                "n_combined_nd": len(combined),
                "per_method": {n: len(r) for n, r in packed.items()},
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
