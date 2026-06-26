"""Controller server facade: RPP + closed-loop segment execution."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence, Tuple

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths()

from grid_env_10k_pie_patrol import SegmentCommand  # noqa: E402

from nav_stack.controllers.rpp import RppConfig, compute_rpp_command  # noqa: E402
from nav_stack.controllers.segment_executor import (  # noqa: E402
    ClosedLoopResult,
    SegmentExecutorConfig,
    run_closed_loop,
)
from nav_stack.controllers.velocity_scaler import (  # noqa: E402
    VelocityScaleConfig,
    dynamic_max_move_cm,
)

WorldXY = Tuple[float, float]
PoseFn = Callable[[], Tuple[WorldXY, float]]
ExecuteFn = Callable[[SegmentCommand], None]


@dataclass(frozen=True)
class ControllerServerConfig:
    use_rpp: bool = True
    rpp: RppConfig = RppConfig()
    executor: SegmentExecutorConfig = SegmentExecutorConfig()
    velocity: VelocityScaleConfig = VelocityScaleConfig()
    wp_reach_tolerance_cm: float = 80.0


class ControllerServer:
    """Nav2 controller_server equivalent for SimWorld UE navigation."""

    def __init__(self, config: ControllerServerConfig) -> None:
        self.config = config

    def compute_segment_command(
        self,
        pos_xy: WorldXY,
        yaw_deg: float,
        waypoint_xy: WorldXY,
        waypoints: Sequence[WorldXY],
        wp_index: int,
        *,
        legacy_command_fn: Optional[
            Callable[[WorldXY, float, WorldXY], Optional[SegmentCommand]]
        ] = None,
    ) -> Optional[SegmentCommand]:
        if self.config.use_rpp:
            return compute_rpp_command(
                pos_xy,
                yaw_deg,
                waypoints,
                wp_index,
                config=self.config.rpp,
                max_move_cm=self.config.velocity.max_move_cm,
            )
        if legacy_command_fn is not None:
            return legacy_command_fn(pos_xy, yaw_deg, waypoint_xy)
        return None

    def allowed_move_cm(
        self,
        nearest_dist_cm: float,
        forward_depth_cm: Optional[float],
    ) -> float:
        return dynamic_max_move_cm(
            nearest_dist_cm,
            forward_depth_cm,
            config=self.config.velocity,
        )

    def execute_closed_loop(
        self,
        *,
        target_xy: WorldXY,
        get_pose: PoseFn,
        compute_command: Callable[[WorldXY, float], Optional[SegmentCommand]],
        execute_chunk: ExecuteFn,
    ) -> ClosedLoopResult:
        return run_closed_loop(
            target_xy=target_xy,
            reach_tolerance_cm=self.config.wp_reach_tolerance_cm,
            get_pose=get_pose,
            compute_command=compute_command,
            execute_chunk=execute_chunk,
            config=self.config.executor,
        )
