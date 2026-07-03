#!/usr/bin/env python3
"""Dynamic NavMesh hybrid navigation constants for site_transport_20m.

Planning constraint (user-specified):
    distance(obstacle_surface, agent_center) >= PROXIMITY_CENTER_FROM_SURFACE_CM

Achieved by:
    1. Nav obstacle boundary at actor AABB surface (GetActorBounds, no extra inflation)
    2. NavFindPath AgentRadius = PROXIMITY_CENTER_FROM_SURFACE_CM

Violation metrics (Phase 3) use true 2D surface distance (bounds-based), not center distance.
"""

from __future__ import annotations

# --- Planning (NavFindPath) -------------------------------------------------
# Center-to-obstacle-surface clearance enforced by UE agent radius on nav boundaries.
PROXIMITY_CENTER_FROM_SURFACE_CM = 100.0

# SpotDog approximate body radius (L2 self-exclude / metrics). Refine via GetActorBounds.
SPOTDOG_BODY_RADIUS_CM = 70.0

# Humanoid horizontal radius fallback until bounds are cached at spawn.
HUMANOID_BODY_RADIUS_CM = 45.0

# Leg2: replan when humanoid moves more than this since last NavFindPath.
HUMANOID_REPLAN_DELTA_CM = 75.0

# Stuck detection: replan NavFindPath after this many steps without WP progress.
NAVMESH_REPLAN_STUCK_STEPS = 10

# Path following (VBP execution until Phase 5 MoveTo).
NAVMESH_GOAL_TOLERANCE_CM = 130.0
NAVMESH_WP_REACH_TOLERANCE_CM = 80.0
NAVMESH_WAYPOINT_SPACING_CM = 80.0
NAVMESH_MAX_OPEN_LOOP_MOVE_CM = 180.0
NAVMESH_STUCK_MOVE_THRESHOLD_CM = 8.0
NAVMESH_STUCK_UNCHANGED_STEPS = 3
NAVMESH_MAX_TURN_DEG_PER_STEP = 22.0
NAVMESH_ROTATE_THRESHOLD_DEG = 6.0

# Phase 4: faster navmesh profile (no L2 depth cycles).
NAVMESH_PERCEPTION_INTERVAL_S = 5.0
NAVMESH_MOVES_PER_CYCLE = 3
NAVMESH_POST_MOTION_SETTLE_S = 0.08
NAVMESH_PRE_LEG1_SETTLE_S = 2.0
NAVMESH_NAV_WARMUP_SETTLE_S = 1.0

# Nav obstacle volume height (Z extent for modifier boxes) [cm].
NAV_OBSTACLE_HALF_HEIGHT_CM = 120.0
NAV_REBUILD_SETTLE_S = 1.5

# Actor names for dynamic humanoid nav obstacle volume (spawned by NavQueryService).
HUMANOID_NAV_OBSTACLE_ID = "site20_humanoid_nav_obs"
