"""Shared navigation state (Nav2-style context object)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Optional, Set, Tuple

from nav_stack.nav_kpi import NavKpiTracker
from nav_stack.nav_stack_config import NavStackConfig

WorldXY = Tuple[float, float]
GridCell = Tuple[int, int]


def build_nav_context(
    *,
    ucv: Any,
    layers: Any,
    profile: Any,
    trace: Any = None,
    timing: Any = None,
    object_registry: Any = None,
    robot_name: str = "",
) -> NavContext:
    """Construct production NavContext from a NavProfile."""
    return NavContext(
        ucv=ucv,
        layers=layers,
        local_costmap=None,
        object_registry=object_registry,
        profile=profile,
        trace=trace,
        timing=timing,
        stack_config=NavStackConfig.from_profile(profile),
        kpi=NavKpiTracker(),
        robot_name=robot_name,
    )


@dataclass
class NavContext:
    """Explicit container replacing layered_nav module globals (Phase 2)."""

    ucv: Any
    layers: Any
    local_costmap: Any
    object_registry: Any = None
    profile: Any = None
    trace: Any = None
    timing: Any = None
    depth_cache: Any = None
    stack_config: Optional[NavStackConfig] = None
    kpi: Optional[NavKpiTracker] = None
    l2_seen_cells: Set[GridCell] = field(default_factory=set)
    l2_flush_count: int = 0
    robot_name: str = ""
    carry_motion_cb: Optional[Callable[[], None]] = None
    extra_obstacle_positions_fn: Optional[Callable[[], Tuple[WorldXY, ...]]] = None
    soft_reset_fn: Optional[Callable[..., None]] = None
