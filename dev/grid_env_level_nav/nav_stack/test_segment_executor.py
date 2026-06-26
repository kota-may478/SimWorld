#!/usr/bin/env python3
"""Unit tests for closed-loop segment executor."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import List, Optional, Tuple

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths()

from grid_env_10k_pie_patrol import SegmentCommand  # noqa: E402
from nav_stack.controllers.segment_executor import (  # noqa: E402
    ClosedLoopResult,
    SegmentExecutorConfig,
    clamp_command_to_chunk,
    run_closed_loop,
)


class SegmentExecutorTest(unittest.TestCase):
    def test_clamp_command_to_chunk(self) -> None:
        cmd = SegmentCommand(turn_deg=45.0, turn_clockwise=1, move_cm=120.0)
        cfg = SegmentExecutorConfig(chunk_max_move_cm=50.0, chunk_max_turn_deg=18.0)
        chunk = clamp_command_to_chunk(cmd, config=cfg)
        self.assertAlmostEqual(chunk.turn_deg, 18.0)
        self.assertAlmostEqual(chunk.move_cm, 50.0)

    def test_run_closed_loop_arrives(self) -> None:
        pos = [0.0, 0.0]
        yaw = [0.0]
        executed: List[SegmentCommand] = []

        def get_pose() -> Tuple[Tuple[float, float], float]:
            return (pos[0], pos[1]), yaw[0]

        def compute_command(
            current: Tuple[float, float],
            current_yaw: float,
        ) -> Optional[SegmentCommand]:
            del current_yaw
            dx = 100.0 - current[0]
            if abs(dx) < 1e-3:
                return None
            return SegmentCommand(turn_deg=0.0, turn_clockwise=1, move_cm=min(40.0, dx))

        def execute_chunk(command: SegmentCommand) -> None:
            executed.append(command)
            pos[0] += command.move_cm

        result = run_closed_loop(
            target_xy=(100.0, 0.0),
            reach_tolerance_cm=5.0,
            get_pose=get_pose,
            compute_command=compute_command,
            execute_chunk=execute_chunk,
            config=SegmentExecutorConfig(
                chunk_max_move_cm=50.0,
                max_chunks_per_segment=10,
                progress_epsilon_cm=1.0,
            ),
        )
        self.assertIsInstance(result, ClosedLoopResult)
        self.assertTrue(result.arrived)
        self.assertFalse(result.stalled)
        self.assertGreater(len(executed), 1)

    def test_run_closed_loop_detects_stall(self) -> None:
        pos = (0.0, 0.0)
        yaw = 0.0

        def get_pose() -> Tuple[Tuple[float, float], float]:
            return pos, yaw

        def compute_command(
            current: Tuple[float, float],
            current_yaw: float,
        ) -> Optional[SegmentCommand]:
            del current, current_yaw
            return SegmentCommand(turn_deg=0.0, turn_clockwise=1, move_cm=40.0)

        def execute_chunk(command: SegmentCommand) -> None:
            del command

        result = run_closed_loop(
            target_xy=(200.0, 0.0),
            reach_tolerance_cm=5.0,
            get_pose=get_pose,
            compute_command=compute_command,
            execute_chunk=execute_chunk,
            config=SegmentExecutorConfig(progress_epsilon_cm=8.0, max_chunks_per_segment=3),
        )
        self.assertTrue(result.stalled)
        self.assertFalse(result.arrived)


if __name__ == "__main__":
    unittest.main()
