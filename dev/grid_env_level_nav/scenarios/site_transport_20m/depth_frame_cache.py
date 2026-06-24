#!/usr/bin/env python3
"""Per-nav-iteration depth frame cache — one fetch shared across L2 / standoff / move."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, Optional, Tuple

import numpy as np

WorldXY = Tuple[float, float]
FetchRawFn = Callable[[], Optional[np.ndarray]]
RecordFn = Callable[[np.ndarray, np.ndarray], Optional[float]]


@dataclass
class DepthFrameCache:
    """Cache a single depth frame per nav loop iteration (TTL + pose gate)."""

    ttl_s: float = 0.3
    pose_delta_max_cm: float = 5.0
    stale_max_s: float = 0.5
    move_invalidate_cm: float = 30.0
    hits: int = 0
    misses: int = 0
    _depth_raw: Optional[np.ndarray] = field(default=None, repr=False)
    _depth_m: Optional[np.ndarray] = field(default=None, repr=False)
    min_fwd_cm: Optional[float] = None
    _fetched_at: float = 0.0
    _pose: Optional[WorldXY] = None
    _last_invalidate_reason: str = ""

    def invalidate(self, reason: str = "") -> None:
        self._depth_raw = None
        self._depth_m = None
        self.min_fwd_cm = None
        self._fetched_at = 0.0
        self._pose = None
        self._last_invalidate_reason = reason

    def note_move_cm(self, move_cm: float) -> None:
        if move_cm >= self.move_invalidate_cm:
            self.invalidate(f"move_{move_cm:.0f}cm")

    def note_backoff(self) -> None:
        self.invalidate("standoff_backoff")

    def _pose_delta_cm(self, pose_xy: WorldXY) -> float:
        if self._pose is None:
            return float("inf")
        return math.hypot(pose_xy[0] - self._pose[0], pose_xy[1] - self._pose[1])

    def is_fresh(
        self,
        pose_xy: WorldXY,
        *,
        max_age_s: Optional[float] = None,
    ) -> bool:
        if self._depth_m is None or self._pose is None:
            return False
        age = time.time() - self._fetched_at
        limit = self.ttl_s if max_age_s is None else max_age_s
        if age > limit:
            return False
        return self._pose_delta_cm(pose_xy) <= self.pose_delta_max_cm

    def get_depth_m(self) -> Optional[np.ndarray]:
        return self._depth_m

    def refresh_forward_depth_cm(
        self,
        pose_xy: WorldXY,
        fetch_raw_fn: FetchRawFn,
        record_fn: RecordFn,
        *,
        force: bool = False,
        max_age_s: Optional[float] = None,
    ) -> Optional[float]:
        """Return forward depth cm; fetch only on miss or force."""
        if not force and self.is_fresh(pose_xy, max_age_s=max_age_s):
            self.hits += 1
            return self.min_fwd_cm
        self.misses += 1
        depth_raw = fetch_raw_fn()
        if depth_raw is None:
            return self.min_fwd_cm
        from depth_object_perception import depth_npy_to_meters  # noqa: WPS433

        depth_m = depth_npy_to_meters(depth_raw)
        self._depth_raw = depth_raw
        self._depth_m = depth_m
        self._fetched_at = time.time()
        self._pose = pose_xy
        self.min_fwd_cm = record_fn(depth_raw, depth_m)
        return self.min_fwd_cm
