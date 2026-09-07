#!/usr/bin/env python3
"""Discover Pareto fronts with grid and NSGA-II; SafeOpt is a constrained baseline."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from constraints.pareto import EvaluatedTheta, iso_pareto, nondominated  # noqa: E402
from fronts.ab_map import dump_table, enumerate_table  # noqa: E402
from fronts.evaluate import OracleEvaluator, measure_t_ref, opt_config  # noqa: E402
from fronts.grid_sweep import run_grid  # noqa: E402
from fronts.nsga2 import run_nsga2  # noqa: E402
from fronts.safe_bo import best_safe_incumbent, run_safe_bo  # noqa: E402
from fronts.space import REF_THETA  # noqa: E402
from fronts.viz_fronts import write_ab_table_plot, write_front_comparison, write_method_plots  # noqa: E402
from oracle.simulate import OracleConfig  # noqa: E402
from paths import make_run_dir  # noqa: E402


def _dump_rows(rows: tuple[EvaluatedTheta, ...]) -> list[dict]:
    return [
        {
            **r.theta.as_dict(),
            "tt": r.tt,
            "t_ssm": r.t_ssm,
            "si_min": r.si_min,
            "iso_feasible": r.iso_feasible,
            "completed": r.completed,
            "mission_s": r.mission_s,
            "scaffold_safe_s": r.scaffold_safe_s,
            "scaffold_unsafe_s": r.scaffold_unsafe_s,
            "min_sep_m": r.min_sep_m,
        }
        for r in rows
    ]


def _unique(rows: list[EvaluatedTheta]) -> tuple[EvaluatedTheta, ...]:
    seen: set[tuple[float, float]] = set()
    kept: list[EvaluatedTheta] = []
    for row in rows:
        key = (round(row.theta.vmax_mps, 5), round(row.theta.dmin_m, 5))
        if key in seen:
            continue
        seen.add(key)
        kept.append(row)
    return tuple(kept)


def main() -> int:
    parser = argparse.ArgumentParser(description="Discover scaffold_hrc Pareto fronts")
    parser.add_argument("--quick", action="store_true", help="tiny budgets for a smoke run")
    parser.add_argument("--sockets-per-floor", type=int, default=4)
    parser.add_argument(
        "--methods",
        type=str,
        default="grid,nsga2,safe_bo",
        help="comma-separated subset: grid,nsga2,safe_bo",
    )
    args = parser.parse_args()

    run_dir = make_run_dir()
    base = OracleConfig(sockets_per_floor=args.sockets_per_floor, record_trace=False)
    config = opt_config(base)
    t_ref = measure_t_ref(config, REF_THETA)

    def fresh() -> OracleEvaluator:
        return OracleEvaluator(config=config, t_ref_s=t_ref)

    if args.quick:
        jobs = {
            "grid": lambda ev: run_grid(ev, n_vmax=3, n_dmin=3),
            "nsga2": lambda ev: run_nsga2(ev, pop_size=6, n_gen=2, seed=5),
            "safe_bo": lambda ev: run_safe_bo(
                ev, n_iter=4, n_vmax=5, n_dmin=4, d_lim=0.55, densify=False
            ),
        }
    else:
        jobs = {
            "grid": lambda ev: run_grid(ev, n_vmax=12, n_dmin=9),
            "nsga2": lambda ev: run_nsga2(ev, pop_size=32, n_gen=14, seed=11),
            "safe_bo": lambda ev: run_safe_bo(
                ev, n_iter=48, n_vmax=12, n_dmin=9, d_lim=0.55, densify=False
            ),
        }

    wanted = tuple(name.strip() for name in args.methods.split(",") if name.strip())
    unknown = [name for name in wanted if name not in jobs]
    if unknown:
        parser.error("unknown methods: " + ", ".join(unknown))
    jobs = {name: jobs[name] for name in wanted}

    packed: dict[str, tuple[EvaluatedTheta, ...]] = {}
    incumbents: dict[str, dict] = {}
    for name, job in jobs.items():
        ev = fresh()
        job(ev)
        packed[name] = _unique(list(ev.cache.values()))
        method_dir = run_dir / name
        write_method_plots(method_dir, name, packed[name])
        payload = {
            "n_rows": len(packed[name]),
            "n_nondominated": len(nondominated(packed[name])),
            "n_iso_pareto": len(iso_pareto(packed[name])),
            "rows": _dump_rows(packed[name]),
            "nondominated": _dump_rows(nondominated(packed[name])),
            "iso_pareto": _dump_rows(iso_pareto(packed[name])),
        }
        if name == "safe_bo":
            best = best_safe_incumbent(packed[name], d_lim=0.55)
            incumbents[name] = {
                **best.theta.as_dict(),
                "tt": best.tt,
                "t_ssm": best.t_ssm,
                "si_min": best.si_min,
                "iso_feasible": best.iso_feasible,
            }
            payload["incumbent"] = incumbents[name]
        (method_dir / "samples.json").write_text(
            json.dumps(payload, indent=2),
            encoding="utf-8",
        )

    union: list[EvaluatedTheta] = []
    for name, rows in packed.items():
        if name == "safe_bo":
            continue
        union.extend(rows)
    combined = nondominated(tuple(union))
    combined_iso = iso_pareto(tuple(union))
    write_front_comparison(run_dir, packed)
    ab_table = enumerate_table(combined_iso) if combined_iso else {}
    if ab_table:
        write_ab_table_plot(run_dir, ab_table)

    meta = {
        "run_dir": str(run_dir),
        "t_ref_s": t_ref,
        "ref_theta": REF_THETA.as_dict(),
        "sockets_per_floor": args.sockets_per_floor,
        "n_unique_evals": sum(len(rows) for rows in packed.values()),
        "methods": {
            name: {
                "n_rows": len(rows),
                "n_nondominated": len(nondominated(rows)),
                "n_iso_pareto": len(iso_pareto(rows)),
                "nondominated": _dump_rows(nondominated(rows)),
                "iso_pareto": _dump_rows(iso_pareto(rows)),
            }
            for name, rows in packed.items()
        },
        "combined_nondominated": _dump_rows(combined),
        "combined_iso_pareto": _dump_rows(combined_iso),
        "ab_table": dump_table(ab_table) if ab_table else [],
        "safeopt_incumbent": incumbents.get("safe_bo"),
    }
    (run_dir / "fronts.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(run_dir)
    print(
        json.dumps(
            {
                "n_evals": meta["n_unique_evals"],
                "n_combined_nd": len(combined),
                "per_method": {n: len(r) for n, r in packed.items()},
                "safeopt_incumbent": incumbents.get("safe_bo"),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
