"""Cached oracle evaluations. Does not change the kinematic simulator."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Dict, Optional, Tuple

from constraints.pareto import EvaluatedTheta, Theta
from oracle.erect_bare import BareErectConfig, run_bare_erection
from oracle.objectives import score
from oracle.simulate import OracleConfig, OracleResult, run_erection
from scene.geometry import ScaffoldGeom, STAGE1_GEOM
from scene.mission_protocol import STAGE1_PROTOCOL

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
    # Bare-ground full member erect (posts/stairs/boards). Default on.
    erect_from_bare: bool = STAGE1_PROTOCOL.erect_from_bare
    bare_max_items: Optional[int] = None
    bare_max_floors: Optional[int] = None
    bare_boards_per_floor: Optional[int] = None

    def _run(self, theta: Theta) -> OracleResult:
        if self.erect_from_bare:
            return run_bare_erection(
                theta=theta,
                geom=self.geom,
                config=BareErectConfig(
                    max_items=self.bare_max_items,
                    max_floors=self.bare_max_floors,
                    boards_per_floor=self.bare_boards_per_floor,
                    record_trace=False,
                    timeout_s=self.config.timeout_s,
                    dt_s=self.config.dt_s,
                ),
                oracle=replace(self.config, record_trace=False),
            )
        return run_erection(
            geom=self.geom,
            theta=theta,
            config=self.config,
            constraint_active=self.constraint_active,
        )

    def evaluate(self, theta: Theta) -> EvaluatedTheta:
        key = _key(theta)
        hit = self.cache.get(key)
        if hit is not None:
            return hit
        result = self._run(theta)
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
        result = self._run(theta)
        return result.makespan_s


def opt_config(base: OracleConfig) -> OracleConfig:
    return replace(base, record_trace=False)


def measure_t_ref(
    config: OracleConfig,
    theta: Theta,
    *,
    geom: ScaffoldGeom = STAGE1_GEOM,
    bare_max_items: Optional[int] = None,
    bare_max_floors: Optional[int] = None,
    bare_boards_per_floor: Optional[int] = None,
) -> float:
    quiet = opt_config(config)
    evaluator = OracleEvaluator(
        config=quiet,
        t_ref_s=1.0,
        geom=geom,
        bare_max_items=bare_max_items,
        bare_max_floors=bare_max_floors,
        bare_boards_per_floor=bare_boards_per_floor,
    )
    result = evaluator._run(theta)
    if result.makespan_s <= 0.0:
        raise ValueError("reference run produced non-positive time")
    return result.makespan_s
