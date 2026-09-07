"""Cached oracle evaluations. Does not change the kinematic simulator."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Dict, Tuple

from constraints.pareto import EvaluatedTheta, Theta
from oracle.objectives import score
from oracle.simulate import OracleConfig, OracleResult, run_erection
from scene.geometry import ScaffoldGeom, STAGE1_GEOM

Key = Tuple[float, float]


def _key(theta: Theta) -> Key:
    return (round(theta.vmax_mps, 5), round(theta.dmin_m, 5))


@dataclass
class OracleEvaluator:
    config: OracleConfig
    t_ref_s: float
    geom: ScaffoldGeom = STAGE1_GEOM
    constraint_active: bool = True
    cache: Dict[Key, EvaluatedTheta] = field(default_factory=dict)

    def evaluate(self, theta: Theta) -> EvaluatedTheta:
        key = _key(theta)
        hit = self.cache.get(key)
        if hit is not None:
            return hit
        result = run_erection(
            geom=self.geom,
            theta=theta,
            config=self.config,
            constraint_active=self.constraint_active,
        )
        breakdown = score(result, vmax_mps=theta.vmax_mps)
        row = EvaluatedTheta(
            theta,
            tt=breakdown.tt,
            t_ssm=theta.vmax_mps,
            si_min=breakdown.si_min,
            completed=result.completed,
            mission_s=result.makespan_s,
            scaffold_safe_s=result.scaffold_safe_s,
            scaffold_unsafe_s=result.scaffold_unsafe_s,
            min_sep_m=result.min_separation_m,
        )
        self.cache[key] = row
        return row

    def reference_time(self, theta: Theta) -> float:
        result = run_erection(
            geom=self.geom,
            theta=theta,
            config=self.config,
            constraint_active=self.constraint_active,
        )
        return result.makespan_s


def opt_config(base: OracleConfig) -> OracleConfig:
    return replace(base, record_trace=False)


def measure_t_ref(config: OracleConfig, theta: Theta, *, geom: ScaffoldGeom = STAGE1_GEOM) -> float:
    quiet = opt_config(config)
    result: OracleResult = run_erection(
        geom=geom,
        theta=theta,
        config=quiet,
        constraint_active=True,
    )
    if result.scaffold_time_s <= 0.0:
        raise ValueError("reference run produced non-positive scaffold time")
    return result.scaffold_time_s
