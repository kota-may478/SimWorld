"""Site transport stuck recovery orchestrated via behavior_server."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Set, Tuple

from nav_stack.behavior_server import BehaviorServer, RecoveryActionSpec, RecoveryResult
from nav_stack.nav_context import NavContext
from nav_stack.planner_server import ReplanResult, replan_on_merged_layers

WorldXY = Tuple[float, float]
GridCell = Tuple[int, int]


@dataclass
class StuckRecoverySession:
    """Mutable state for one stuck-recovery transaction."""

    ucv: Any
    layers: Any
    robot_name: str
    goal_xy: WorldXY
    stuck_xy: WorldXY
    waypoints: list
    wp_index: int
    waypoint_xy: WorldXY
    stuck_hotspots: List[WorldXY] = field(default_factory=list)
    unstuck_attempts: int = 0
    l2_seen_cells: Set[GridCell] = field(default_factory=set)
    trace: Any = None
    carry_motion_cb: Optional[Callable[[], None]] = None
    planning_clearance_cm: float = 100.0
    planning_clearance_cost: float = 300.0
    backup_cm: float = 100.0
    stuck_move_threshold_cm: float = 14.0
    escape_min_displacement_cm: float = 35.0
    max_unstuck_attempts: int = 16
    mission_failed: bool = False
    backup_xy: WorldXY = (0.0, 0.0)
    mark_counts: Tuple[int, int, int] = (0, 0, 0)
    replan_stage: str = ""
    nearest_wp_index_fn: Optional[Callable[[WorldXY, list, int], int]] = None


@dataclass(frozen=True)
class StuckRecoveryOutcome:
    ucv: Any
    pos_xy: WorldXY
    waypoints: list
    wp_index: int
    steps_on_wp: int
    unstuck_attempts: int
    stuck_hotspots: List[WorldXY]
    mission_failed: bool
    replan_stage: str = ""


@dataclass(frozen=True)
class StuckRecoveryCallbacks:
    """UE / layered_nav hooks injected to keep nav_stack free of UnrealCV imports."""

    mark_stuck_cells: Callable[[StuckRecoverySession], Tuple[int, int, int]]
    unstuck_backup: Callable[[StuckRecoverySession], None]
    safe_get_pos2d: Callable[[StuckRecoverySession], Tuple[WorldXY, Any]]
    execute_escape_step: Callable[[StuckRecoverySession, WorldXY], Optional[WorldXY]]
    world_to_local: Callable[[WorldXY], Tuple[float, float]]
    record_plan: Callable[[StuckRecoverySession, list, str], None]
    spin_backup: Optional[Callable[[StuckRecoverySession], None]] = None
    clear_local_l2: Optional[Callable[[StuckRecoverySession], int]] = None
    wait_settle: Optional[Callable[[StuckRecoverySession], None]] = None


def _stage_log_message(stage: str) -> str:
    messages = {
        "tight_merged": "replan using tight merged L0+L1+L2 clearance",
        "l0_l1": "replan using L0+L1 (L2 ignored)",
        "l0_only": "escape replan on L0 only",
        "failed": "tight merged replan failed",
    }
    return messages.get(stage, f"replan stage={stage}")


def run_site_stuck_recovery(
    session: StuckRecoverySession,
    *,
    callbacks: StuckRecoveryCallbacks,
) -> StuckRecoveryOutcome:
    """Run backup + optional escape + replan using behavior_server action chain."""
    session.stuck_hotspots = list(session.stuck_hotspots)
    session.stuck_hotspots.append(session.stuck_xy)
    pending_attempt = session.unstuck_attempts + 1
    ctx = NavContext(
        ucv=session.ucv,
        layers=session.layers,
        local_costmap=None,
        trace=session.trace,
        l2_seen_cells=session.l2_seen_cells,
        robot_name=session.robot_name,
        carry_motion_cb=session.carry_motion_cb,
    )

    def _mark(_ctx: NavContext) -> RecoveryResult:
        del _ctx
        session.mark_counts = callbacks.mark_stuck_cells(session)
        n_marked, n_hotspot, n_corridor = session.mark_counts
        print(
            f"  [SiteNav] STUCK @ local={callbacks.world_to_local(session.stuck_xy)} "
            f"mark_l2={n_marked} hotspot={n_hotspot} corridor={n_corridor} "
            f"→ backup {session.backup_cm:.0f}cm + replan "
            f"(attempt {pending_attempt}/{session.max_unstuck_attempts})"
        )
        return RecoveryResult(success=True, message="mark_l2")

    def _backup(_ctx: NavContext) -> RecoveryResult:
        del _ctx
        callbacks.unstuck_backup(session)
        if session.carry_motion_cb is not None:
            session.carry_motion_cb()
        session.backup_xy, session.ucv = callbacks.safe_get_pos2d(session)
        return RecoveryResult(success=True, message="backup")

    def _escape(_ctx: NavContext) -> RecoveryResult:
        del _ctx
        if dist2d(session.backup_xy, session.stuck_xy) < session.stuck_move_threshold_cm * 2.0:
            escape_xy = callbacks.execute_escape_step(session, session.backup_xy)
            if escape_xy is not None:
                session.backup_xy = escape_xy
                if session.carry_motion_cb is not None:
                    session.carry_motion_cb()
        return RecoveryResult(success=True, message="escape")

    def _evaluate(_ctx: NavContext) -> RecoveryResult:
        del _ctx
        displacement = dist2d(session.backup_xy, session.stuck_xy)
        if displacement >= session.escape_min_displacement_cm:
            print(f"  [SiteNav] unstuck displacement={displacement:.0f}cm")
            session.unstuck_attempts = 0
            return RecoveryResult(success=True, message="displaced")
        session.unstuck_attempts = pending_attempt
        if displacement < 5.0 and pending_attempt >= 3:
            print(
                f"  [SiteNav] zero-displacement for {pending_attempt} attempts"
                " — fast-track to LAST RESORT"
            )
            session.unstuck_attempts = session.max_unstuck_attempts
        if session.unstuck_attempts >= session.max_unstuck_attempts:
            print(
                f"  [SiteNav] FAIL: stuck at local={callbacks.world_to_local(session.backup_xy)} "
                f"after {session.max_unstuck_attempts} backup+replan attempts"
            )
            if session.trace is not None:
                session.trace.l2_cell_count = len(session.l2_seen_cells)
            session.mission_failed = True
            return RecoveryResult(success=False, message="mission_failed")
        return RecoveryResult(success=True, message="continue")

    def _tiered_recovery(_ctx: NavContext) -> RecoveryResult:
        del _ctx
        if session.mission_failed:
            return RecoveryResult(success=False, message="skipped")
        if pending_attempt >= 4 and callbacks.clear_local_l2 is not None:
            removed = callbacks.clear_local_l2(session)
            print(f"  [SiteNav] clear_local_l2 evicted {removed} cells (attempt {pending_attempt})")
        elif pending_attempt >= 2 and callbacks.spin_backup is not None:
            callbacks.spin_backup(session)
            if callbacks.wait_settle is not None:
                callbacks.wait_settle(session)
            print(f"  [SiteNav] spin+backup recovery (attempt {pending_attempt})")
        return RecoveryResult(success=True, message="tiered")

    def _replan(_ctx: NavContext) -> RecoveryResult:
        del _ctx
        if session.mission_failed:
            return RecoveryResult(success=False, message="skipped")
        result: ReplanResult = replan_on_merged_layers(
            session.layers,
            session.backup_xy,
            session.goal_xy,
            planning_clearance_cm=session.planning_clearance_cm,
            planning_clearance_cost=session.planning_clearance_cost,
            l2_seen_cells=session.l2_seen_cells,
        )
        session.replan_stage = result.stage
        if result.stage in {"tight_merged", "l0_l1", "l0_only"}:
            print(f"  [SiteNav] {_stage_log_message(result.stage)}")
        if result.waypoints:
            callbacks.record_plan(session, result.waypoints, "unstuck_replan")
            session.waypoints = result.waypoints
            if session.nearest_wp_index_fn is not None:
                session.wp_index = session.nearest_wp_index_fn(
                    session.backup_xy,
                    session.waypoints,
                    session.wp_index,
                )
            print(
                f"  [SiteNav] unstuck replan → {len(session.waypoints)} WP (merged L0+L1+L2)"
            )
            return RecoveryResult(success=True, message=result.stage)
        print(
            f"  [SiteNav] unstuck replan failed at "
            f"local={callbacks.world_to_local(session.backup_xy)}"
        )
        return RecoveryResult(success=False, message="replan_failed")

    server = BehaviorServer(
        [
            RecoveryActionSpec("mark_l2", _mark),
            RecoveryActionSpec("backup", _backup),
            RecoveryActionSpec("escape", _escape),
            RecoveryActionSpec("evaluate_displacement", _evaluate),
            RecoveryActionSpec("tiered_recovery", _tiered_recovery),
            RecoveryActionSpec("replan", _replan),
        ]
    )
    server.run_chain(ctx, stop_on_success=False)

    return StuckRecoveryOutcome(
        ucv=session.ucv,
        pos_xy=session.backup_xy,
        waypoints=session.waypoints,
        wp_index=session.wp_index,
        steps_on_wp=0,
        unstuck_attempts=session.unstuck_attempts,
        stuck_hotspots=session.stuck_hotspots,
        mission_failed=session.mission_failed,
        replan_stage=session.replan_stage,
    )


def dist2d(a: WorldXY, b: WorldXY) -> float:
    import math

    return math.hypot(b[0] - a[0], b[1] - a[1])
