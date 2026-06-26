"""Closed-loop segment executor with chunked open-loop moves."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Tuple

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths()

from grid_env_10k_pie_patrol import SegmentCommand, dist2d  # noqa: E402

WorldXY = Tuple[float, float]
PoseFn = Callable[[], Tuple[WorldXY, float]]
CommandFn = Callable[[WorldXY, float], Optional[SegmentCommand]]
ExecuteFn = Callable[[SegmentCommand], None]


@dataclass(frozen=True)
class SegmentExecutorConfig:
    chunk_max_move_cm: float = 50.0
    chunk_max_turn_deg: float = 18.0
    progress_epsilon_cm: float = 8.0
    max_chunks_per_segment: int = 8
    open_loop_distance_scale: float = 1.0


@dataclass(frozen=True)
class ClosedLoopResult:
    final_pos: WorldXY
    final_yaw_deg: float
    chunks_executed: int
    arrived: bool
    stalled: bool


def clamp_command_to_chunk(
    command: SegmentCommand,
    *,
    config: SegmentExecutorConfig,
) -> SegmentCommand:
    """Limit one open-loop UE command to a safe chunk size."""
    turn_deg = min(command.turn_deg, config.chunk_max_turn_deg)
    move_cm = min(
        command.move_cm * config.open_loop_distance_scale,
        config.chunk_max_move_cm,
    )
    return SegmentCommand(
        turn_deg=turn_deg,
        turn_clockwise=command.turn_clockwise,
        move_cm=max(0.0, move_cm),
    )


def run_closed_loop(
    *,
    target_xy: WorldXY,
    reach_tolerance_cm: float,
    get_pose: PoseFn,
    compute_command: CommandFn,
    execute_chunk: ExecuteFn,
    config: SegmentExecutorConfig,
) -> ClosedLoopResult:
    """Execute pose-feedback chunks until target reached, stalled, or chunk budget."""
    pos_xy, yaw_deg = get_pose()
    chunks = 0

    while chunks < config.max_chunks_per_segment:
        if dist2d(pos_xy, target_xy) <= reach_tolerance_cm:
            return ClosedLoopResult(
                final_pos=pos_xy,
                final_yaw_deg=yaw_deg,
                chunks_executed=chunks,
                arrived=True,
                stalled=False,
            )

        command = compute_command(pos_xy, yaw_deg)
        if command is None:
            break

        chunk = clamp_command_to_chunk(command, config=config)
        if chunk.turn_deg < 1e-3 and chunk.move_cm < 1e-3:
            break

        before_xy = pos_xy
        execute_chunk(chunk)
        pos_xy, yaw_deg = get_pose()
        progress_cm = dist2d(before_xy, pos_xy)
        chunks += 1

        if chunk.move_cm > 1e-3 and progress_cm < config.progress_epsilon_cm:
            return ClosedLoopResult(
                final_pos=pos_xy,
                final_yaw_deg=yaw_deg,
                chunks_executed=chunks,
                arrived=False,
                stalled=True,
            )

    arrived = dist2d(pos_xy, target_xy) <= reach_tolerance_cm
    return ClosedLoopResult(
        final_pos=pos_xy,
        final_yaw_deg=yaw_deg,
        chunks_executed=chunks,
        arrived=arrived,
        stalled=not arrived,
    )
