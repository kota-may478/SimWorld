#!/usr/bin/env python3
"""Per-nav-loop depth UE fetch budget (one fetch per iteration when possible)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional, Tuple

if TYPE_CHECKING:
    from depth_frame_cache import DepthFrameCache

WorldXY = Tuple[float, float]


@dataclass
class DepthNavIterBudget:
    """Track UE depth fetches within one layered_nav while-loop iteration."""

    ue_fetches_this_iter: int = 0
    max_fetches_per_iter: int = 1

    def begin_iter(self) -> None:
        self.ue_fetches_this_iter = 0

    def on_invalidate(self) -> None:
        """Allow a new UE fetch after cache invalidation (e.g. backoff)."""
        self.ue_fetches_this_iter = 0

    def can_reuse_in_iter(self, depth_frame: "DepthFrameCache") -> bool:
        return (
            self.ue_fetches_this_iter >= self.max_fetches_per_iter
            and depth_frame.get_depth_m() is not None
        )

    def note_ue_fetch(self) -> None:
        self.ue_fetches_this_iter += 1

    def should_fetch_ue(
        self,
        depth_frame: "DepthFrameCache",
        pose_xy: WorldXY,
        *,
        max_age_s: float,
        force: bool = False,
    ) -> bool:
        if force:
            return True
        if depth_frame.is_fresh(pose_xy, max_age_s=max_age_s):
            return False
        if self.can_reuse_in_iter(depth_frame):
            return False
        return True
