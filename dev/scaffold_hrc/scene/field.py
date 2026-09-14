"""Shared Stage-1 field: oracle kinematics and UE placement stay aligned.

UE Recast stairs (AABB treads) and the kei-truck yard after the 2026-09-08
layout pass are the source of truth. Oracle hops follow the same run length
and south/north zigzag so TT reflects the same walk.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

# Landing sill / Recast stair run (matches ue.placement).
LANDING_SILL_DEPTH_M = 0.90
RAMP_DECK_X_M = -LANDING_SILL_DEPTH_M + 0.10
RAMP_STEPS = 6
RAMP_STEP_DEPTH_M = 1.80
RAMP_STEP_OVERLAP_M = 0.80
RAMP_STEP_SPACING_M = RAMP_STEP_DEPTH_M - RAMP_STEP_OVERLAP_M
RAMP_RUN_M = RAMP_STEP_DEPTH_M + (RAMP_STEPS - 1) * RAMP_STEP_SPACING_M
RAMP_WIDTH_M = 1.10
RAMP_YARD_PAD_M = 1.20
# Walkable slab thickness (deck / AABB treads). Same as ue.placement.
DECK_THICKNESS_M = 0.12

# UE STAGING_LOCAL_XY_CM = (150, 280) with ENTRANCE at (0, 1400):
# scaffold x = (280-1400)/100 = -11.2, y = 150/100 = 1.5.
STORAGE_XY_M: Tuple[float, float] = (-11.2, 1.5)


@dataclass(frozen=True)
class StairTreadPose:
    lift: int
    step: int
    x_m: float
    y_m: float
    z_bottom_m: float
    sx_m: float
    sy_m: float
    sz_m: float


@dataclass(frozen=True)
class StairPostPose:
    lift: int
    lane: int
    side: str
    station: int
    x_m: float
    y_m: float
    z_m: float
    sz_m: float


def ramp_yard_x_m() -> float:
    return RAMP_DECK_X_M - RAMP_RUN_M


def ramp_step_x_m(lift: int, step: int) -> float:
    """Center x of one AABB tread. L0 yard→deck, L1 deck→yard."""
    if lift == 0:
        return ramp_yard_x_m() + 0.5 * RAMP_STEP_DEPTH_M + step * RAMP_STEP_SPACING_M
    return RAMP_DECK_X_M - 0.5 * RAMP_STEP_DEPTH_M - step * RAMP_STEP_SPACING_M


def stair_tread_poses(
    *,
    lift_m: float,
    deck_width_m: float,
    n_lifts: int = 2,
) -> Tuple[StairTreadPose, ...]:
    """Spot-climbable zigzag AABB treads (same geometry as ue.placement.spot_ramps)."""
    rise = lift_m / float(RAMP_STEPS)
    poses: list[StairTreadPose] = []
    for lift in range(n_lifts):
        y = stair_lane_y_m(lift, deck_width_m)
        z_low = lift * lift_m + DECK_THICKNESS_M
        for step in range(RAMP_STEPS):
            z_top = z_low + (step + 1) * rise
            poses.append(
                StairTreadPose(
                    lift=lift,
                    step=step,
                    x_m=ramp_step_x_m(lift, step),
                    y_m=y,
                    z_bottom_m=z_top - DECK_THICKNESS_M,
                    sx_m=RAMP_STEP_DEPTH_M,
                    sy_m=RAMP_WIDTH_M,
                    sz_m=DECK_THICKNESS_M,
                )
            )
    return tuple(poses)


def stair_tower_x_stations() -> Tuple[float, ...]:
    """Three 建地 lines along the Recast run: yard, mid, deck."""
    yard = ramp_yard_x_m()
    deck = RAMP_DECK_X_M
    return (yard, 0.5 * (yard + deck), deck)


def stair_post_poses(
    *,
    lift_m: float,
    deck_width_m: float,
    n_lifts: int = 2,
) -> Tuple[StairPostPose, ...]:
    """Full-height posts on both stringers of each zigzag ramp, every lift.

    Without these, AABB treads span the 6.8 m run with no 建地 under them.
    """
    xs = stair_tower_x_stations()
    poses: list[StairPostPose] = []
    for lift in range(n_lifts):
        z0 = lift * lift_m
        for lane in range(n_lifts):
            y_c = stair_lane_y_m(lane, deck_width_m)
            ys = (y_c - 0.5 * RAMP_WIDTH_M, y_c + 0.5 * RAMP_WIDTH_M)
            for station, x in enumerate(xs):
                for side, y in zip(("s", "n"), ys):
                    poses.append(
                        StairPostPose(
                            lift=lift,
                            lane=lane,
                            side=side,
                            station=station,
                            x_m=x,
                            y_m=y,
                            z_m=z0,
                            sz_m=lift_m,
                        )
                    )
    return tuple(poses)


def scaffold_edge_x_m() -> float:
    """Most negative x still under θ / SSM (stair yard, not the truck)."""
    return ramp_yard_x_m()


def stair_lane_y_m(lift: int, deck_width_m: float) -> float:
    """L0 south 1F→2F, L1 north 2F→3F (same as UE spot_ramps)."""
    if lift % 2 == 0:
        return 0.5 * RAMP_WIDTH_M
    return deck_width_m - 0.5 * RAMP_WIDTH_M


def stair_run_ends(lift: int) -> Tuple[float, float]:
    """(start_x, end_x) walking up one lift along the Recast run."""
    yard = ramp_yard_x_m()
    deck = RAMP_DECK_X_M
    if lift % 2 == 0:
        return yard, deck
    return deck, yard
