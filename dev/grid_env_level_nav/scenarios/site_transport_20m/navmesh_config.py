#!/usr/bin/env python3
"""Dynamic NavMesh hybrid navigation constants for site_transport_20m.

Planning constraint (user-specified):
    For *unspecified* props only:
        distance(prop AABB surface, SpotDog body outer edge) >= PROXIMITY_EDGE_FROM_SURFACE_CM
    Exempt from this rule (may approach within 1 m):
        - material storage transport target (registry.material_actor_name)
        - final destination humanoid (registry.humanoid_actor_name)

Validation uses >= (at least) the configured distance, not "exactly" 1 m.
Set NAV_MODIFIER_CLEARANCE_MARGIN_CM and NAV_PLANNING_GOAL_PUSH_MARGIN_CM to 0.0
for the tightest planning clearance (subject to NavMesh discretization).

Achieved by:
    1. NavModifier box half-extents = actor AABB + NAV_PROP_OBSTACLE_PADDING_CM
       (standoff + optional margin — UE FindPath ignores AgentRadius in this project)
    2. NavFindPath AgentRadius passed for API compatibility (modifier carries standoff)
    3. NavFindPathValidated resamples with NavMesh projection; rejects paths whose waypoints violate clearance
    4. Python corridor gate: transit waypoints >= 1 m on unpadded prop AABBs (goal exempt; chords not checked)
    5. Python chord densify remains a secondary safety net on unpadded AABBs

Violation metrics: center-to-AABB-surface and body-edge-to-surface thresholds.
"""

from __future__ import annotations

# --- Planning (NavFindPath) -------------------------------------------------
# Primary rule: body outer edge to *unspecified prop* AABB surface [cm] (>=, not ==).
PROXIMITY_EDGE_FROM_SURFACE_CM = 100.0

# SpotDog approximate body radius from pawn center (conservative for legs/mesh).
SPOTDOG_BODY_RADIUS_CM = 70.0

# Center must stay this far from unpadded obstacle AABB surfaces (metrics + validation).
NAV_PLANNING_CENTER_CLEARANCE_CM = (
    PROXIMITY_EDGE_FROM_SURFACE_CM + SPOTDOG_BODY_RADIUS_CM
)

# Back-compat alias for metrics and analysis scripts.
NAV_PLANNING_AGENT_RADIUS_CM = NAV_PLANNING_CENTER_CLEARANCE_CM
PROXIMITY_CENTER_FROM_SURFACE_CM = NAV_PLANNING_CENTER_CLEARANCE_CM

# Nav obstacle half-extent padding on modifier volumes.
# Full center standoff on modifiers — dynamic modifiers carry clearance because UE 5.3
# FindPathSync returns identical paths for AgentRadius 170–220 cm in PIE.
NAV_MESH_HULL_PADDING_CM = 0.0
NAV_MODIFIER_STANDOFF_CM = NAV_PLANNING_CENTER_CLEARANCE_CM
# Extra margin on modifiers for Recast cell discretization and corner grazing on carved holes.
NAV_MODIFIER_CLEARANCE_MARGIN_CM = 0.0
NAV_PROP_OBSTACLE_PADDING_CM = (
    NAV_MESH_HULL_PADDING_CM + NAV_MODIFIER_STANDOFF_CM + NAV_MODIFIER_CLEARANCE_MARGIN_CM
)
# Extra center clearance when nudging planning goals away from *props* (0 = strict).
NAV_PLANNING_GOAL_PUSH_MARGIN_CM = 0.0
# Positioning safety so SpotDog center stays >= 170 cm after nav projection / resample.
NAV_PLANNING_GOAL_POSITION_EPSILON_CM = 1.0

# Leg2: carve dynamic humanoid into NavMesh (legacy; prefer dynamic obstacle trackers).
NAV_REGISTER_HUMANOID_NAV_MODIFIER = False

# Mission runtime: register NavModifiers for tracked moving actors (humanoid, etc.).
NAV_DYNAMIC_OBSTACLE_MODIFIER_ENABLED = True

# Replan + local NavRebuild when a tracked actor moves at least this far [cm].
DYNAMIC_OBSTACLE_REPLAN_DELTA_CM = 75.0
HUMANOID_REPLAN_DELTA_CM = DYNAMIC_OBSTACLE_REPLAN_DELTA_CM

# NavFindPath agent radius (secondary; modifier standoff is primary).
NAV_FINDPATH_AGENT_RADIUS_CM = NAV_MESH_HULL_PADDING_CM

# Humanoid horizontal radius fallback until bounds are cached at spawn.
HUMANOID_BODY_RADIUS_CM = 45.0

# Stuck detection: replan NavFindPath after this many steps without WP progress.
NAVMESH_REPLAN_STUCK_STEPS = 20

# Path following (VBP execution until Phase 5 MoveTo).
NAVMESH_GOAL_TOLERANCE_CM = 130.0
NAVMESH_WP_REACH_TOLERANCE_CM = 50.0
NAVMESH_WAYPOINT_SPACING_CM = 40.0
# Open-loop translate per VBP command on straight segments [cm].
NAVMESH_MAX_OPEN_LOOP_MOVE_CM = 100.0
NAVMESH_MIN_COMMAND_DURATION_S = 0.06
NAVMESH_STUCK_MOVE_THRESHOLD_CM = 8.0
NAVMESH_STUCK_UNCHANGED_STEPS = 3
NAVMESH_MAX_TURN_DEG_PER_STEP = 22.0
NAVMESH_ROTATE_THRESHOLD_DEG = 6.0

NAV_ROADBLOCK_OBSTACLE_EXTRA_PADDING_CM = 0.0
# Back-compat alias (analysis scripts).
NAV_ROADBLOCK_OBSTACLE_PADDING_CM = NAV_ROADBLOCK_OBSTACLE_EXTRA_PADDING_CM

# NavRebuild: expand dirty bounds beyond modifier boxes [cm] (full/static setup).
NAV_REBUILD_DIRTY_MARGIN_CM = 200.0
# Local dirty-region rebuild for moving obstacles [cm].
NAV_REBUILD_LOCAL_DIRTY_MARGIN_CM = 50.0

# Prefer 170cm center clearance when possible; otherwise shortest NavFindPath (metrics track violations).
NAV_PLANNING_STRICT_CLEARANCE = False

# NavFindPathValidated: 0 = corner polyline only (no resample along straights).
NAV_PATH_RESAMPLE_SPACING_CM = 0.0

# Clearance grid A* — mission fallback; penalizes skating along prop outer perimeters.
CLEARANCE_GRID_RESOLUTION_CM = 40.0
CLEARANCE_GRID_BLOCK_MARGIN_CM = 2.0
# Extra A* cost [cm-equivalent] when a step uses the minimum-clearance band near props.
NAV_CLEARANCE_HUG_PENALTY_WEIGHT = 2000.0
# Margin above minimum clearance before hug penalty drops to zero.
NAV_CLEARANCE_OPEN_MARGIN_CM = 250.0

# Open-loop chord clearance fallback (Python) when UE validated API unavailable.
NAVMESH_CHORD_SAMPLE_SPACING_CM = 16.0
NAVMESH_CHORD_MAX_INSERTIONS = 2048

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


def planning_center_clearance_required_cm() -> float:
    """Center-to-prop-surface minimum for path validation (unspecified props only)."""
    return NAV_PLANNING_CENTER_CLEARANCE_CM


def planning_goal_center_clearance_required_cm() -> float:
    """Center clearance target when pushing planning goals away from props."""
    return NAV_PLANNING_CENTER_CLEARANCE_CM + NAV_PLANNING_GOAL_PUSH_MARGIN_CM


def planning_goal_position_center_clearance_cm() -> float:
    """SpotDog center-to-prop-surface target at approach/deliver goals (>= 170 cm)."""
    return (
        NAV_PLANNING_CENTER_CLEARANCE_CM
        + NAV_PLANNING_GOAL_PUSH_MARGIN_CM
        + NAV_PLANNING_GOAL_POSITION_EPSILON_CM
    )


def planning_body_edge_clearance_required_cm() -> float:
    """Body-edge-to-prop-surface minimum (= PROXIMITY_EDGE_FROM_SURFACE_CM)."""
    return PROXIMITY_EDGE_FROM_SURFACE_CM
