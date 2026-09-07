#!/usr/bin/env python3
"""Compare proposed α-index grounding with B1–B5 and SafeOpt. No LLM."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from constraints.pareto import iso_pareto  # noqa: E402
from fronts.evaluate import OracleEvaluator, measure_t_ref, opt_config  # noqa: E402
from fronts.grid_sweep import run_grid  # noqa: E402
from fronts.grounding import compare_methods, proposed_from_text  # noqa: E402
from fronts.nsga2 import run_nsga2  # noqa: E402
from fronts.space import HALLUCINATED_THETA, REF_THETA  # noqa: E402
from oracle.simulate import OracleConfig  # noqa: E402
from paths import make_run_dir  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare proposed vs B1-B5 and SafeOpt")
    parser.add_argument("--alpha", type=float, default=0.8)
    parser.add_argument("--beta", type=float, default=0.5)
    parser.add_argument("--text", type=str, default="")
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--sockets-per-floor", type=int, default=2)
    parser.add_argument("--preference", type=str, default="please be more cautious")
    parser.add_argument("--mode", type=str, default="conservative")
    args = parser.parse_args()

    run_dir = make_run_dir()
    base = OracleConfig(
        sockets_per_floor=args.sockets_per_floor,
        record_trace=False,
        dt_s=0.25 if args.quick else 0.1,
        timeout_s=480.0 if args.quick else 7200.0,
        erect_s=0.25 if args.quick else 30.0,
        truck_load_s=0.25 if args.quick else 8.0,
        drop_place_s=0.25 if args.quick else 8.0,
    )
    config = opt_config(base)
    t_ref = measure_t_ref(config, REF_THETA)
    evaluator = OracleEvaluator(config=config, t_ref_s=t_ref)
    n_vmax = 3 if args.quick else 8
    n_dmin = 3 if args.quick else 8
    grid_rows = run_grid(evaluator, n_vmax=n_vmax, n_dmin=n_dmin)
    if args.quick:
        nsga_rows = run_nsga2(evaluator, pop_size=6, n_gen=2, seed=5)
    else:
        nsga_rows = run_nsga2(evaluator, pop_size=24, n_gen=8, seed=11)
    union = tuple(grid_rows) + tuple(nsga_rows)
    nd_rows = iso_pareto(union)
    front = tuple(row.theta for row in nd_rows)
    if not front:
        payload = {
            "run_dir": str(run_dir),
            "alpha": args.alpha,
            "beta": args.beta,
            "t_ref_s": t_ref,
            "n_front": 0,
            "error": "empty ISO Pareto front",
        }
        (run_dir / "grounding.json").write_text(
            json.dumps(payload, indent=2), encoding="utf-8"
        )
        print(run_dir)
        print("No ISO-feasible Pareto points; cannot run proposed method.", file=sys.stderr)
        return 1
    compared = compare_methods(
        alpha=args.alpha,
        beta=args.beta,
        front=front,
        rows=nd_rows,
        evaluator=evaluator,
        preference_text=args.preference,
        mode=args.mode,
        score_all=True,
        include_safeopt=True,
        safeopt_n_iter=4 if args.quick else 16,
        safeopt_n_vmax=5 if args.quick else 9,
        safeopt_n_dmin=4 if args.quick else 7,
    )
    payload = {
        "run_dir": str(run_dir),
        "alpha": args.alpha,
        "beta": args.beta,
        "t_ref_s": t_ref,
        "n_front": len(front),
        "front": [t.as_dict() for t in front],
        "methods": [asdict(item) for item in compared],
        "hallucinated": HALLUCINATED_THETA.as_dict(),
    }
    if args.text:
        spoken = proposed_from_text(args.text, nd_rows)
        payload["from_text"] = asdict(spoken)
    (run_dir / "grounding.json").write_text(
        json.dumps(payload, indent=2), encoding="utf-8"
    )
    print(run_dir)
    print(json.dumps(payload["methods"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
