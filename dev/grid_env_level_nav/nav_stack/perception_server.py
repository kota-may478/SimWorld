"""Perception server: depth→L2 and sight→registry (Nav2 perception_server equivalent)."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol, Tuple

from nav_stack.nav_types import PerceptionOutcome

WorldXY = Tuple[float, float]


class RegistryUpdateResultLike(Protocol):
    detections: tuple
    visible_actor_names: tuple
    backend: str
    entries_added: int
    entries_evicted: int


@dataclass
class SightPerceptionDeps:
    """Injected UE/scenario hooks for sight-mode perception."""

    get_robot_pose: Callable[[], Tuple[WorldXY, float]]
    apply_l2_depth: Callable[..., Any]
    update_registry: Callable[[], RegistryUpdateResultLike]
    should_run_registry: Callable[[int], bool]
    record_detection_local: Callable[[Any], None]
    should_skip_depth: Optional[Callable[[int, RegistryUpdateResultLike], bool]] = None
    consume_gate_depth_fetch: Optional[Callable[[], bool]] = None
    on_timing_pose_ms: Optional[Callable[[float], None]] = None
    on_timing_registry_ms: Optional[Callable[[float], None]] = None


class PerceptionServer:
    """Nav2 perception_server facade for site_transport sight mode."""

    def __init__(self, deps: SightPerceptionDeps) -> None:
        self._deps = deps
        self._cycle = 0

    def perceive(self, *, layers: Any, l2_seen_cells: set) -> PerceptionOutcome:
        self._cycle += 1
        pose_t0 = time.perf_counter()
        robot_xy, robot_yaw = self._deps.get_robot_pose()
        if self._deps.on_timing_pose_ms is not None:
            self._deps.on_timing_pose_ms((time.perf_counter() - pose_t0) * 1000.0)

        if self._deps.should_run_registry(self._cycle):
            reg_t0 = time.perf_counter()
            reg_result = self._deps.update_registry()
            if self._deps.on_timing_registry_ms is not None:
                self._deps.on_timing_registry_ms((time.perf_counter() - reg_t0) * 1000.0)
        else:
            reg_result = _skipped_registry_result()

        if (
            self._deps.should_skip_depth is not None
            and self._deps.should_skip_depth(self._cycle, reg_result)
        ):
            for det in reg_result.detections:
                self._deps.record_detection_local(det)
            return PerceptionOutcome(
                detections=list(reg_result.detections),
                cells_added=0,
                cells_removed=0,
                l2_applied=False,
            )

        skip_fetch = False
        if self._deps.consume_gate_depth_fetch is not None:
            skip_fetch = self._deps.consume_gate_depth_fetch()

        depth_result = self._deps.apply_l2_depth(
            l2_seen_cells,
            robot_xy=robot_xy,
            robot_yaw=robot_yaw,
            skip_depth_fetch=skip_fetch,
        )
        for det in reg_result.detections:
            self._deps.record_detection_local(det)

        cells_added = int(getattr(depth_result, "total_cells_added", 0))
        cells_removed = int(getattr(depth_result, "cleared_cells", 0))
        return PerceptionOutcome(
            detections=list(reg_result.detections),
            cells_added=cells_added,
            cells_removed=cells_removed,
            l2_applied=True,
        )


def _skipped_registry_result() -> RegistryUpdateResultLike:
    return _SkippedRegistry()


@dataclass
class _SkippedRegistry:
    detections: tuple = ()
    visible_actor_names: tuple = ()
    backend: str = "skipped"
    entries_added: int = 0
    entries_evicted: int = 0
