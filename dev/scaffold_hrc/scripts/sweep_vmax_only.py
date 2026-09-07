"""1D sweep: ISO-tight keep-out (δ = 0), vary v_max."""

from __future__ import annotations

from constraints.pareto import Theta, iso_pareto
from fronts.evaluate import OracleEvaluator, measure_t_ref, opt_config
from fronts.space import REF_THETA
from oracle.simulate import OracleConfig
from oracle.ssm import protective_separation_m

VMAXS = (0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00)


def main() -> None:
    config = opt_config(
        OracleConfig(
            dt_s=0.25,
            timeout_s=480.0,
            erect_s=0.25,
            truck_load_s=0.25,
            drop_place_s=0.25,
            sockets_per_floor=2,
            record_trace=False,
        )
    )
    t_ref = measure_t_ref(config, REF_THETA)
    ev = OracleEvaluator(config=config, t_ref_s=t_ref)
    rows = [ev.evaluate(Theta(vmax_mps=v, dmin_m=0.35)) for v in VMAXS]
    print(f"t_ref_s={t_ref:.3f} dmin_fixed=0.35")
    print("vmax  Sp(v)   tt     t_ssm  si_min done")
    for row in rows:
        v = row.theta.vmax_mps
        print(
            f"{v:4.2f}  {protective_separation_m(v):5.3f}  "
            f"{row.tt:5.3f}  {row.t_ssm:5.3f}  "
            f"{row.si_min:5.3f}  {int(row.completed)}"
        )
    front = iso_pareto(rows)
    print("iso_pareto n=", len(front))
    for row in front:
        print(
            f"  vmax={row.theta.vmax_mps:.2f} tt={row.tt:.3f} t_ssm={row.t_ssm:.3f}"
        )


if __name__ == "__main__":
    main()
