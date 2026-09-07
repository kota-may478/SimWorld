"""ISO/TS 15066 SSM helpers. Sp is a threshold, not an objective."""

from __future__ import annotations

from typing import Tuple

SSM_TR_S = 0.10
SSM_TS_S = 0.30
SSM_C_M = 0.15
SSM_VH_MPS = 1.6
SSM_STOPPED_MPS = 0.05

SsmMode = str


def protective_separation_m(
    vr_mps: float,
    *,
    vh_mps: float = SSM_VH_MPS,
    tr_s: float = SSM_TR_S,
    ts_s: float = SSM_TS_S,
    intrusion_c_m: float = SSM_C_M,
    stopped_mps: float = SSM_STOPPED_MPS,
) -> float:
    """Linearized ISO/TS 15066 Sp. Stopped robots keep only intrusion C."""
    vr = max(0.0, float(vr_mps))
    vh = max(0.0, float(vh_mps))
    if vr <= stopped_mps:
        return max(0.0, float(intrusion_c_m))
    sh = vh * (tr_s + ts_s)
    sr = vr * tr_s
    ss = 0.5 * vr * ts_s
    return sh + sr + ss + max(0.0, float(intrusion_c_m))


def commanded_separation_m(vr_mps: float, extra_margin_m: float = 0.0) -> float:
    """Controller keep-out: ISO Sp plus integrator extra margin δ ≥ 0."""
    return protective_separation_m(vr_mps) + max(0.0, float(extra_margin_m))


def safety_index(sep_m: float, sp_m: float) -> float:
    if sp_m <= 1e-12:
        return float("inf") if sep_m > 0.0 else 0.0
    return sep_m / sp_m


def apply_ssm(
    sep_m: float,
    speed_mps: float,
    *,
    on_site: bool,
    receding: bool = False,
    extra_margin_m: float = 0.0,
) -> Tuple[float, SsmMode]:
    """Return commanded speed and mode. Stop if S < Sp_ISO(v) + δ.

    Receding does not exempt the stop. Spot never reverses; the human retreats.
    """
    _ = receding
    if (not on_site) or speed_mps <= 0.0:
        return max(0.0, speed_mps), "free"
    sp_used = commanded_separation_m(speed_mps, extra_margin_m)
    si = safety_index(sep_m, sp_used)
    if si < 1.0:
        return 0.0, "stop"
    return speed_mps, "free"
