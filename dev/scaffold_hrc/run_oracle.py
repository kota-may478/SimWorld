#!/usr/bin/env python3
"""Run the headless 3F erection oracle, write timestamped plots under out/."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

PKG = Path(__file__).resolve().parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from constraints.pareto import (  # noqa: E402
    EvaluatedTheta,
    Theta,
    nondominated,
    project,
    synthetic_front,
)
from fronts.evaluate import measure_t_ref  # noqa: E402
from fronts.space import REF_THETA  # noqa: E402
from oracle.objectives import W_SAFE, W_TCR, W_TT, score  # noqa: E402
from oracle.simulate import OracleConfig, OracleResult, run_erection  # noqa: E402
from paths import make_run_dir  # noqa: E402
from scene.geometry import STAGE1_GEOM  # noqa: E402
from scene.scaffold_grammar import build_scaffold  # noqa: E402
from viz import write_pareto_plots, write_trace_csv, write_trajectory_plots  # noqa: E402

DEFAULT_ALPHA = 0.8


def _result_payload(
    theta: Theta,
    result: OracleResult,
    *,
    t_ref_s: float,
    alpha: float | None = None,
) -> dict:
    breakdown = score(result, t_ref_s=t_ref_s)
    row = {
        "dmin_m": theta.dmin_m,
        "vmax_mps": theta.vmax_mps,
        "completed": result.completed,
        "makespan_s": result.makespan_s,
        "path_length_m": result.path_length_m,
        "corridor_time_s": result.corridor_time_s,
        "min_separation_m": result.min_separation_m,
        "wait_s": result.wait_s,
        "violation_s": result.violation_s,
        "n_filled": result.n_filled,
        "n_sockets": result.n_sockets,
        "floors_completed": result.floors_completed,
        "tcr": breakdown.tcr,
        "tt": breakdown.tt,
        "jeff": breakdown.jeff,
        "jsafe": breakdown.jsafe,
        "j": breakdown.j,
        "trace_samples": len(result.trace),
    }
    if alpha is not None:
        row["alpha"] = alpha
    return row


def _dump_scaffold(path: Path) -> None:
    spec = build_scaffold(STAGE1_GEOM)
    payload = {
        "geom": asdict(STAGE1_GEOM),
        "n_bays": spec.n_bays,
        "bay_m": spec.bay_m,
        "n_modules": len(spec.modules),
        "n_sockets": len(spec.sockets),
        "modules": [asdict(m) for m in spec.modules],
        "sockets": [asdict(s) for s in spec.sockets],
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="scaffold_hrc 3F kinematic oracle")
    parser.add_argument("--alpha", type=float, default=DEFAULT_ALPHA)
    parser.add_argument("--no-constraint", action="store_true")
    parser.add_argument("--sockets-per-floor", type=int, default=None)
    args = parser.parse_args()

    run_dir = make_run_dir()
    config = OracleConfig(sockets_per_floor=args.sockets_per_floor)
    t_ref_s = measure_t_ref(config, REF_THETA)
    front = synthetic_front()
    chosen = project(Theta(dmin_m=8.0, vmax_mps=3.0), args.alpha, front)
    constraint_active = not args.no_constraint

    rows: list[EvaluatedTheta] = []
    sweep_payload: list[dict] = []
    for sample in front:
        result = run_erection(
            geom=STAGE1_GEOM,
            theta=sample,
            config=config,
            constraint_active=constraint_active,
        )
        breakdown = score(result, t_ref_s=t_ref_s)
        rows.append(EvaluatedTheta(sample, breakdown.jeff, breakdown.jsafe, result.completed))
        sweep_payload.append(_result_payload(sample, result, t_ref_s=t_ref_s))

    nd = nondominated(tuple(rows))
    representative = run_erection(
        geom=STAGE1_GEOM,
        theta=chosen,
        config=config,
        constraint_active=constraint_active,
    )

    _dump_scaffold(run_dir / "scaffold_modules.json")
    write_trace_csv(run_dir / "trajectory.csv", representative)
    write_pareto_plots(run_dir, rows=tuple(rows), front=front, chosen=chosen)
    write_trajectory_plots(
        run_dir,
        geom=STAGE1_GEOM,
        result=representative,
        theta=chosen,
    )

    meta = {
        "run_dir": str(run_dir),
        "alpha": args.alpha,
        "constraint_active": constraint_active,
        "weights": {"w_tcr": W_TCR, "w_tt": W_TT, "w_safe": W_SAFE},
        "t_ref_s": t_ref_s,
        "ref_theta": {"dmin_m": REF_THETA.dmin_m, "vmax_mps": REF_THETA.vmax_mps},
        "chosen_theta": {"dmin_m": chosen.dmin_m, "vmax_mps": chosen.vmax_mps},
        "representative": _result_payload(
            chosen, representative, t_ref_s=t_ref_s, alpha=args.alpha
        ),
        "n_sweep": len(sweep_payload),
        "n_nondominated": len(nd),
        "nondominated": [
            {
                "dmin_m": row.theta.dmin_m,
                "vmax_mps": row.theta.vmax_mps,
                "jeff": row.jeff,
                "jsafe": row.jsafe,
            }
            for row in nd
        ],
        "sweep": sweep_payload,
    }
    (run_dir / "run.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(run_dir)
    print(json.dumps(meta["representative"], indent=2))
    return 0 if representative.completed else 1


if __name__ == "__main__":
    raise SystemExit(main())
