"""Navigation KPI accumulator (roadmap §6.3)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

WorldXY = Tuple[float, float]


def cross_track_error_cm(
    pos_xy: WorldXY,
    segment_start: WorldXY,
    segment_end: WorldXY,
) -> float:
    """Perpendicular distance from pos to the line segment (cm)."""
    sx, sy = segment_start
    ex, ey = segment_end
    px, py = pos_xy
    dx = ex - sx
    dy = ey - sy
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-6:
        return math.hypot(px - sx, py - sy)
    t = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / seg_len_sq))
    proj_x = sx + t * dx
    proj_y = sy + t * dy
    return math.hypot(px - proj_x, py - proj_y)


@dataclass
class NavKpiTracker:
    stuck_events: int = 0
    replan_attempts: int = 0
    replan_successes: int = 0
    cross_track_errors_cm: List[float] = field(default_factory=list)
    local_costmap_updates: int = 0
    open_loop_scale_ema: float = 1.0

    def record_replan(self, *, success: bool) -> None:
        self.replan_attempts += 1
        if success:
            self.replan_successes += 1

    def record_stuck(self) -> None:
        self.stuck_events += 1

    def record_cross_track(self, error_cm: float) -> None:
        if math.isfinite(error_cm):
            self.cross_track_errors_cm.append(error_cm)

    def record_local_costmap_update(self) -> None:
        self.local_costmap_updates += 1

    def update_open_loop_scale(self, commanded_cm: float, actual_cm: float) -> None:
        if commanded_cm < 1e-3 or actual_cm < 0.0:
            return
        measured = actual_cm / commanded_cm
        alpha = 0.2
        self.open_loop_scale_ema = (1.0 - alpha) * self.open_loop_scale_ema + alpha * measured

    @property
    def replan_success_rate(self) -> float:
        if self.replan_attempts <= 0:
            return 1.0
        return self.replan_successes / float(self.replan_attempts)

    @property
    def mean_cross_track_error_cm(self) -> float:
        if not self.cross_track_errors_cm:
            return 0.0
        return sum(self.cross_track_errors_cm) / float(len(self.cross_track_errors_cm))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "stuck_events": self.stuck_events,
            "replan_attempts": self.replan_attempts,
            "replan_successes": self.replan_successes,
            "replan_success_rate": round(self.replan_success_rate, 4),
            "mean_cross_track_error_cm": round(self.mean_cross_track_error_cm, 2),
            "local_costmap_updates": self.local_costmap_updates,
            "open_loop_scale_ema": round(self.open_loop_scale_ema, 4),
            "cross_track_sample_count": len(self.cross_track_errors_cm),
        }
