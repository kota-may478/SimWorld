"""Audit production-run timing, SSM, and d_min vs Sp. No UE."""

from __future__ import annotations

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from constraints.pareto import Theta
from oracle.objectives import score
from oracle.simulate import OracleConfig, run_erection
from oracle.ssm import protective_separation_m
from scene.geometry import STAGE1_GEOM

CASES = (
    ("ref_fast_small_dmin", Theta(vmax_mps=1.00, dmin_m=0.35)),
    ("fast_mid_dmin", Theta(vmax_mps=1.00, dmin_m=0.80)),
    ("fast_large_dmin", Theta(vmax_mps=1.00, dmin_m=1.50)),
    ("slow_small_dmin", Theta(vmax_mps=0.20, dmin_m=0.35)),
    ("slow_large_dmin", Theta(vmax_mps=0.20, dmin_m=1.50)),
)


def _summarize(name: str, theta: Theta, t_ref_s: float) -> None:
    result = run_erection(
        geom=STAGE1_GEOM,
        theta=theta,
        config=OracleConfig(sockets_per_floor=10, record_trace=True),
    )
    breakdown = score(result, t_ref_s=t_ref_s)
    trace = result.trace
    n = len(trace)
    dt = 0.1
    on_site = 0
    corridor = 0
    stop = 0
    blocked_free = 0
    moving_close_sp = 0
    sep_lt_1 = 0
    sep_lt_dmin = 0
    z_up_spot = 0.0
    prev_z = trace[0].spot[2] if trace else 0.0
    for i, sample in enumerate(trace):
        if sample.in_corridor:
            corridor += 1
        else:
            on_site += 1
        if sample.ssm_mode == "stop":
            stop += 1
        if sample.blocked and sample.ssm_mode == "free":
            blocked_free += 1
        sp_cmd = protective_separation_m(theta.vmax_mps)
        if (not sample.in_corridor) and sample.spot_speed_mps > 0.05 and sample.sep_m < sp_cmd:
            moving_close_sp += 1
        if sample.sep_m < 1.0 and not sample.in_corridor:
            sep_lt_1 += 1
        if sample.sep_m < theta.dmin_m and not sample.in_corridor:
            sep_lt_dmin += 1
        if sample.spot[2] > prev_z + 1e-9:
            z_up_spot += sample.spot[2] - prev_z
        prev_z = sample.spot[2]
    avg_speed = result.path_length_m / result.makespan_s if result.makespan_s > 0 else 0.0
    print("===", name, "===")
    print(
        "theta vmax=%.2f dmin=%.2f  Sp(vmax)=%.3f"
        % (theta.vmax_mps, theta.dmin_m, protective_separation_m(theta.vmax_mps))
    )
    print(
        "completed=%s n_filled=%d/%d makespan_s=%.1f (%.1f min) TT=%.4f"
        % (
            result.completed,
            result.n_filled,
            result.n_sockets,
            result.makespan_s,
            result.makespan_s / 60.0,
            breakdown.tt,
        )
    )
    print(
        "path_m=%.1f corridor_s=%.1f scaffold_safe_s=%.1f scaffold_unsafe_s=%.2f wait_s=%.1f T_SSM=%.5f viol_s=%.2f"
        % (
            result.path_length_m,
            result.corridor_time_s,
            result.scaffold_safe_s,
            result.scaffold_unsafe_s,
            result.wait_s,
            breakdown.t_ssm,
            result.violation_s,
        )
    )
    print(
        "min_sep=%.3f si_min=%.3f avg_path_speed=%.3f m/s"
        % (result.min_separation_m, result.si_min, avg_speed)
    )
    print(
        "ticks n=%d on_scaffold=%.1fs corridor=%.1fs ssm_stop=%.1fs dmin_block_est=%.1fs"
        % (n, on_site * dt, corridor * dt, stop * dt, blocked_free * dt)
    )
    print(
        "moving_with_sep<Sp(vmax)=%.1fs  sep<1m_on_scaffold=%.1fs  sep<dmin_on_scaffold=%.1fs"
        % (moving_close_sp * dt, sep_lt_1 * dt, sep_lt_dmin * dt)
    )
    print("spot_vertical_ascent_m=%.2f (3 floors * 1.8m = 3.60 if climbed fully)" % z_up_spot)
    print()


def main() -> None:
    ref = run_erection(
        geom=STAGE1_GEOM,
        theta=Theta(vmax_mps=1.0, dmin_m=0.35),
        config=OracleConfig(sockets_per_floor=10, record_trace=False),
    )
    t_ref = ref.scaffold_safe_s
    print("T_ref_s=%.3f (scaffold-safe)  mission_s=%.3f  Sp(0.2)=%.3f  Sp(1.0)=%.3f" % (
        t_ref,
        ref.makespan_s,
        protective_separation_m(0.2),
        protective_separation_m(1.0),
    ))
    store_x = STAGE1_GEOM.storage_xy()[0]
    print(
        "storage_x=%.1f drop_x=2.0 one_way_xy=%.1f m"
        % (store_x, 2.0 - store_x)
    )
    print("30 boards * 2 * %.1fm / 1.0m/s = %.0fs if serial ground round-trips only"
          % (2.0 - store_x, 30 * 2 * (2.0 - store_x)))
    print()
    for name, theta in CASES:
        _summarize(name, theta, t_ref)


if __name__ == "__main__":
    main()
