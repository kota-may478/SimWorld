#!/usr/bin/env python3
"""Unit tests for perception_server."""

from __future__ import annotations

import sys
import unittest
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from nav_stack.perception_server import PerceptionServer, SightPerceptionDeps  # noqa: E402


@dataclass
class _DepthResult:
    total_cells_added: int = 2
    cleared_cells: int = 1


@dataclass
class _RegResult:
    detections: tuple = ({"id": "crate"},)
    visible_actor_names: tuple = ("crate",)
    backend: str = "test"
    entries_added: int = 1
    entries_evicted: int = 0


class PerceptionServerTest(unittest.TestCase):
    def test_perceive_runs_depth_and_registry(self) -> None:
        recorded: list = []

        def _apply_l2(_seen, *, robot_xy=None, robot_yaw=None, skip_depth_fetch=False):
            del robot_xy, robot_yaw, skip_depth_fetch
            return _DepthResult()

        server = PerceptionServer(
            SightPerceptionDeps(
                get_robot_pose=lambda: ((0.0, 0.0), 0.0),
                apply_l2_depth=_apply_l2,
                update_registry=lambda: _RegResult(),
                should_run_registry=lambda n: True,
                record_detection_local=lambda det: recorded.append(det),
            )
        )
        outcome = server.perceive(layers=None, l2_seen_cells=set())
        self.assertTrue(outcome.l2_applied)
        self.assertTrue(outcome.l2_changed)
        self.assertEqual(outcome.cells_added, 2)
        self.assertEqual(outcome.cells_removed, 1)
        self.assertEqual(len(outcome.detections), 1)
        self.assertEqual(len(recorded), 1)

    def test_registry_skipped_on_interval(self) -> None:
        calls = {"n": 0}

        def _registry():
            calls["n"] += 1
            return _RegResult(detections=())

        server = PerceptionServer(
            SightPerceptionDeps(
                get_robot_pose=lambda: ((1.0, 2.0), 90.0),
                apply_l2_depth=lambda *a, **k: _DepthResult(0, 0),
                update_registry=_registry,
                should_run_registry=lambda n: n % 2 == 1,
                record_detection_local=lambda _det: None,
            )
        )
        server.perceive(layers=None, l2_seen_cells=set())
        server.perceive(layers=None, l2_seen_cells=set())
        self.assertEqual(calls["n"], 1)

    def test_skip_depth_when_unchanged(self) -> None:
        depth_calls = {"n": 0}

        def _apply_l2(_seen, *, robot_xy=None, robot_yaw=None, skip_depth_fetch=False):
            del robot_xy, robot_yaw, skip_depth_fetch
            depth_calls["n"] += 1
            return _DepthResult(0, 0)

        server = PerceptionServer(
            SightPerceptionDeps(
                get_robot_pose=lambda: ((0.0, 0.0), 0.0),
                apply_l2_depth=_apply_l2,
                update_registry=lambda: _RegResult(detections=(), entries_added=0, entries_evicted=0),
                should_run_registry=lambda _n: True,
                record_detection_local=lambda _det: None,
                should_skip_depth=lambda _cycle, _reg: True,
            )
        )
        outcome = server.perceive(layers=None, l2_seen_cells=set())
        self.assertFalse(outcome.l2_applied)
        self.assertEqual(depth_calls["n"], 0)

    def test_gate_depth_fetch_consumed(self) -> None:
        skip_flags: list[bool] = []

        def _apply_l2(_seen, *, robot_xy=None, robot_yaw=None, skip_depth_fetch=False):
            del robot_xy, robot_yaw
            skip_flags.append(skip_depth_fetch)
            return _DepthResult(1, 0)

        gate = {"used": False}

        def _consume() -> bool:
            if gate["used"]:
                return False
            gate["used"] = True
            return True

        server = PerceptionServer(
            SightPerceptionDeps(
                get_robot_pose=lambda: ((0.0, 0.0), 0.0),
                apply_l2_depth=_apply_l2,
                update_registry=lambda: _RegResult(detections=()),
                should_run_registry=lambda _n: True,
                record_detection_local=lambda _det: None,
                consume_gate_depth_fetch=_consume,
            )
        )
        server.perceive(layers=None, l2_seen_cells=set())
        self.assertEqual(skip_flags, [True])


if __name__ == "__main__":
    unittest.main()
