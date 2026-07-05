#!/usr/bin/env python3
"""Dynamic NavMesh hybrid navigation constants for site_transport_20m.

Planning constraint (user-specified):
    distance(obstacle_surface, SpotDog body outer edge) >= PROXIMITY_EDGE_FROM_SURFACE_CM

Achieved by:
    1. Nav obstacle boundary at actor AABB surface (GetActorBounds, no extra inflation)
    2. NavFindPath AgentRadius = NAV_PLANNING_AGENT_RADIUS_CM
       (= PROXIMITY_EDGE_FROM_SURFACE_CM + SPOTDOG_BODY_RADIUS_CM)

Violation metrics (navmesh): center-to-AABB-surface distance with the same center threshold.
"""

from __future__ import annotations

# --- Planning (NavFindPath) -------------------------------------------------
# Desired clearance from SpotDog body outer edge to obstacle AABB surface [cm].
PROXIMITY_EDGE_FROM_SURFACE_CM = 100.0

# SpotDog body radius from pawn center (conservative for legs/mesh beyond capsule).
SPOTDOG_BODY_RADIUS_CM = 80.0

# NavFindPath agent radius: center must stay this far from obstacle surfaces.
NAV_PLANNING_AGENT_RADIUS_CM = (
    PROXIMITY_EDGE_FROM_SURFACE_CM + SPOTDOG_BODY_RADIUS_CM
)

# Alias used by path planning / chord-clearance densify call sites.
PROXIMITY_CENTER_FROM_SURFACE_CM = NAV_PLANNING_AGENT_RADIUS_CM

# Humanoid horizontal radius fallback until bounds are cached at spawn.
HUMANOID_BODY_RADIUS_CM = 45.0

# Leg2: replan when humanoid moves more than this since last NavFindPath.
HUMANOID_REPLAN_DELTA_CM = 75.0

# Stuck detection: replan NavFindPath after this many steps without WP progress.
NAVMESH_REPLAN_STUCK_STEPS = 10

# Path following (VBP execution until Phase 5 MoveTo).
NAVMESH_GOAL_TOLERANCE_CM = 130.0
NAVMESH_WP_REACH_TOLERANCE_CM = 12.0
NAVMESH_WAYPOINT_SPACING_CM = 20.0
NAVMESH_MAX_OPEN_LOOP_MOVE_CM = 25.0
NAVMESH_MIN_COMMAND_DURATION_S = 0.06
NAVMESH_STUCK_MOVE_THRESHOLD_CM = 8.0
NAVMESH_STUCK_UNCHANGED_STEPS = 3
NAVMESH_MAX_TURN_DEG_PER_STEP = 22.0
NAVMESH_ROTATE_THRESHOLD_DEG = 45.0
NAVMESH_COLLINEAR_PRUNE_MAX_TURN_DEG = 8.0

# Nav obstacle half-extent padding (mesh/collision hull > AABB).
NAV_PROP_OBSTACLE_PADDING_CM = 15.0
NAV_ROADBLOCK_OBSTACLE_PADDING_CM = 35.0

# Open-loop chord clearance: sample segments and insert midpoints when below planning radius.
NAVMESH_CHORD_SAMPLE_SPACING_CM = 8.0

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
