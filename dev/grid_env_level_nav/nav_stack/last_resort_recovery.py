"""LAST RESORT: aggressive L2 flush + L0+L1-only replan."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, List, Optional, Set, Tuple

WorldXY = Tuple[float, float]
GridCell = Tuple[int, int]

MAX_L2_FLUSH_COUNT = 3
LAST_RESORT_PERCEIVE_PAUSE_STEPS = 40


@dataclass(frozen=True)
class LastResortOutcome:
    success: bool
    waypoints: List[WorldXY]
    wp_index: int
    l2_flush_count: int
    perceive_pause_steps: int


def try_last_resort_recovery(
    *,
    layers: Any,
    pos_xy: WorldXY,
    goal_xy: WorldXY,
    l2_seen_cells: Set[GridCell],
    l2_flush_count: int,
    stuck_xy: WorldXY,
    soft_reset_fn: Optional[Callable[..., None]],
    replan_fn: Callable[[WorldXY, WorldXY], Any],
    nearest_wp_index_fn: Callable[[WorldXY, list, int], int],
    max_flush_count: int = MAX_L2_FLUSH_COUNT,
    perceive_pause_steps: int = LAST_RESORT_PERCEIVE_PAUSE_STEPS,
) -> Optional[LastResortOutcome]:
    """Flush L2 and replan on L0+L1. Returns None when flush budget is exhausted."""
    if l2_flush_count >= max_flush_count:
        return None

    attempt = l2_flush_count + 1
    print(
        f"  [SiteNav] LAST RESORT #{attempt}:"
        " flush all L2 cells, replan on L0+L1 only"
    )
    if soft_reset_fn is not None:
        soft_reset_fn(l2_seen_cells, stuck_xy, aggressive=True)
    else:
        layers.l2[:, :] = 0
        l2_seen_cells.clear()

    new_flush_count = l2_flush_count + 1
    try:
        flush_plan = replan_fn(pos_xy, goal_xy)
        waypoints = list(flush_plan.waypoints_xy)
        wp_index = nearest_wp_index_fn(pos_xy, waypoints, 0)
        print(
            f"  [SiteNav] L2 flush replan → {len(waypoints)} WP on L0+L1"
            f" (perception paused {perceive_pause_steps} steps)"
        )
        return LastResortOutcome(
            success=True,
            waypoints=waypoints,
            wp_index=wp_index,
            l2_flush_count=new_flush_count,
            perceive_pause_steps=perceive_pause_steps,
        )
    except (ValueError, RuntimeError) as exc:
        print(f"  [SiteNav] L2 flush replan also failed: {exc}")
        return LastResortOutcome(
            success=False,
            waypoints=[],
            wp_index=0,
            l2_flush_count=new_flush_count,
            perceive_pause_steps=perceive_pause_steps,
        )
