"""Actor list for the parametric 3F scaffold on Level."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from scene.geometry import STAGE1_GEOM, ScaffoldGeom
from scene.field import (
    DECK_THICKNESS_M,
    LANDING_SILL_DEPTH_M,
    RAMP_DECK_X_M,
    RAMP_RUN_M,
    RAMP_STEP_DEPTH_M,
    RAMP_STEP_OVERLAP_M,
    RAMP_STEP_SPACING_M,
    RAMP_STEPS,
    RAMP_WIDTH_M,
    RAMP_YARD_PAD_M,
    ramp_step_x_m,
    ramp_yard_x_m,
    stair_tread_poses,
)
from scene.scaffold_grammar import Module, N_WORKING_BAYS, build_scaffold
from ue.layout import (
    STAGING_LOCAL_XY_CM,
    cube_scale_level_world,
    scaffold_xyz_to_local_cm,
)

ACTOR_PREFIX = "ScaffoldHrc"
REFUGE_XY_M: Tuple[float, float] = (9.0, 1.2)
DROP_XY_M: Tuple[float, float] = (2.0, 1.2)
# Lined-up floor staging: slot 0 is DROP_XY_M, then +X (along the deck).
DROP_SLOT_COUNT = 6
DROP_SLOT_PITCH_M = 1.2
LANDING_SOUTH_WIDTH_M = 1.20
LANDING_CONNECTOR_OVERLAP_M = 0.08
DECK_NAV_OVERLAP_M = 0.80
STRINGER_THICK_M = 0.08
# Longest axis Spot can take from the kei bed (one 建枠 bay / 横架材).
# 交差筋違 is hypot(bay, lift) ≈ 2.69 m — one catalog diagonal, not a slab.
CARRY_MAX_M = 2.4
BOARD_SEAM_OVERLAP_M = 0.08
WALKABLE_KINDS = frozenset({"deck", "landing", "ramp"})
PIPE_KINDS = frozenset({"post", "ledger", "transom", "brace", "rail", "stringer"})
# Import FBX via ue/import_kei_truck_editor.py (Editor, not PIE).
TRUCK_BP_CANDIDATES: tuple[str, ...] = (
    "/Game/SimWorld/Props/KeiTruck/BP_KeiTruck.BP_KeiTruck_C",
)
# FBX imported at 1433 cm longest axis; a Carry is ~330 cm.
TRUCK_UNIFORM_SCALE = 330.0 / 1433.4
TRUCK_ACTOR = f"{ACTOR_PREFIX}_truck"
YARD_HUMAN_ACTOR = f"{ACTOR_PREFIX}_yard_human"
ASSEMBLER_ACTOR = f"{ACTOR_PREFIX}_assembler"
# Destroy→respawn of these BPs crashes UE 5.3.2 (same class of failure as KeiTruck).
# Bare/full resets must reuse + reposition only; never UnrealCV-destroy them.
PERSISTENT_ACTORS: tuple[str, ...] = (
    TRUCK_ACTOR,
    YARD_HUMAN_ACTOR,
    ASSEMBLER_ACTOR,
)
# Mesh +X is cab-forward. UnrealCV yaw 0 = +world X = +local Y (scaffold).
# 180° puts the bed toward the scaffold.
TRUCK_YAW_DEG = 180.0
TRUCK_LENGTH_CM = 330.0
# Stand beside the bed, past the lip, off the centerline so the head stays out.
SPOT_TRUCK_SIDE_CM = 150.0
SPOT_TRUCK_ALONG_CM = 0.5 * TRUCK_LENGTH_CM + 70.0
YARD_HUMAN_SIDE_CM = 180.0
YARD_HUMAN_ALONG_CM = 120.0
YARD_HUMAN_Z_CM = 95.0
# Base_User_Agent ≈ 180 cm with pelvis-near root. Clear height under the next
# deck is lift_m - deck thickness (~168 cm); full scale clips through 1F ceiling.
NATIVE_HUMANOID_HEIGHT_CM = 180.0
HUMANOID_STORY_CLEARANCE_MARGIN_CM = 12.0
YARD_HUMAN_YAW_DEG = -90.0
# Next to the kei bed so Spot is visible after ``--bare`` (not only after hrc_mission).
SPOT_YARD_LOCAL_XY_CM: Tuple[float, float] = (
    STAGING_LOCAL_XY_CM[0] + SPOT_TRUCK_SIDE_CM,
    STAGING_LOCAL_XY_CM[1] + SPOT_TRUCK_ALONG_CM,
)
STALE_ACTORS: tuple[str, ...] = (
    f"{ACTOR_PREFIX}_run_L0",
    f"{ACTOR_PREFIX}_run_L1",
    f"{ACTOR_PREFIX}_truck_bed",
    f"{ACTOR_PREFIX}_truck_cab",
    f"{ACTOR_PREFIX}_landing_f2",
    f"{ACTOR_PREFIX}_landing_f3",
    f"{ACTOR_PREFIX}_landing_f3_north",
    f"{ACTOR_PREFIX}_rail_f2_0",
    f"{ACTOR_PREFIX}_rail_f2_1",
    f"{ACTOR_PREFIX}_rail_f3_0",
    f"{ACTOR_PREFIX}_rail_f3_1",
    f"{ACTOR_PREFIX}_transom_f2_0",
    f"{ACTOR_PREFIX}_transom_f3_0",
    *(f"{ACTOR_PREFIX}_post_{i}_{j}" for i in range(6) for j in range(2)),
    f"{ACTOR_PREFIX}_post_stair_0",
    f"{ACTOR_PREFIX}_post_stair_1",
    f"{ACTOR_PREFIX}_ramp_L0",
    f"{ACTOR_PREFIX}_ramp_L1",
    f"{ACTOR_PREFIX}_ramp_L0_a",
    f"{ACTOR_PREFIX}_ramp_L0_b",
    f"{ACTOR_PREFIX}_ramp_L1_a",
    f"{ACTOR_PREFIX}_ramp_L1_b",
    f"{ACTOR_PREFIX}_landing_mid_L0",
    f"{ACTOR_PREFIX}_landing_mid_L1",
    *(f"{ACTOR_PREFIX}_tread_L0_{i}" for i in range(10)),
    *(f"{ACTOR_PREFIX}_tread_L1_{i}" for i in range(10)),
    f"{ACTOR_PREFIX}_stringer_L0_inner",
    f"{ACTOR_PREFIX}_stringer_L1_inner",
    f"{ACTOR_PREFIX}_stringer_L0_outer",
    f"{ACTOR_PREFIX}_stringer_L1_outer",
    *(f"{ACTOR_PREFIX}_cheek_L0_{i}" for i in range(6)),
    *(f"{ACTOR_PREFIX}_cheek_L1_{i}" for i in range(6)),
    f"{ACTOR_PREFIX}_deck_f1",
    f"{ACTOR_PREFIX}_deck_f2",
    f"{ACTOR_PREFIX}_deck_f3",
    *(f"{ACTOR_PREFIX}_ledger_f{f}_{j}" for f in range(1, 4) for j in range(2)),
    f"{ACTOR_PREFIX}_landing_f1",
    f"{ACTOR_PREFIX}_landing_f3_south",
    f"{ACTOR_PREFIX}_landing_f2_sill",
    f"{ACTOR_PREFIX}_landing_f3_yard",
)


@dataclass(frozen=True)
class SpawnBox:
    actor: str
    kind: str
    local_cm: Tuple[float, float, float]
    scale: Tuple[float, float, float]
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0


def skip_board_module(module: Module) -> bool:
    return module.kind == "board"


def floor_walk_z_m(floor: int, geom: ScaffoldGeom = STAGE1_GEOM) -> float:
    """Walkable top of a deck/landing on this floor (metres)."""
    return geom.floor_z_m(floor) + DECK_THICKNESS_M


def skip_spawn_module(module: Module) -> bool:
    return module.kind in {"board", "stair_tread", "rail"}


def _slab(
    suffix: str,
    kind: str,
    x_m: float,
    y_m: float,
    z_m: float,
    sx_m: float,
    sy_m: float,
    geom: ScaffoldGeom = STAGE1_GEOM,
) -> SpawnBox:
    return SpawnBox(
        actor=f"{ACTOR_PREFIX}_{suffix}",
        kind=kind,
        local_cm=scaffold_xyz_to_local_cm(x_m, y_m, z_m, geom=geom),
        scale=cube_scale_level_world(sx_m, sy_m, DECK_THICKNESS_M),
    )


def deck_slabs(geom: ScaffoldGeom = STAGE1_GEOM) -> tuple[SpawnBox, ...]:
    """Working decks as アンチ boards: one bay × half-width, Spot-carryable."""
    bay_m = geom.deck_length_m / N_WORKING_BAYS
    row_w = geom.deck_width_m / 2.0
    sx = bay_m + BOARD_SEAM_OVERLAP_M
    sy = row_w + BOARD_SEAM_OVERLAP_M
    boxes: list[SpawnBox] = []
    for floor in range(1, geom.n_floors + 1):
        z = geom.floor_z_m(floor)
        for i in range(N_WORKING_BAYS):
            x = (i + 0.5) * bay_m
            for row in range(2):
                y = (row + 0.5) * row_w
                boxes.append(
                    _slab(
                        f"deck_f{floor}_b{i}_r{row}",
                        "deck",
                        x,
                        y,
                        z,
                        sx,
                        sy,
                        geom=geom,
                    )
                )
    return tuple(boxes)


def _slabs_along_x(
    suffix: str,
    kind: str,
    x0: float,
    x1: float,
    y_m: float,
    z_m: float,
    sy_m: float,
    geom: ScaffoldGeom,
) -> tuple[SpawnBox, ...]:
    span = x1 - x0
    n_piece = max(1, int(math.ceil(span / CARRY_MAX_M - 1e-9)))
    piece = span / n_piece
    sx = piece + BOARD_SEAM_OVERLAP_M
    boxes: list[SpawnBox] = []
    for i in range(n_piece):
        xa = x0 + i * piece
        boxes.append(
            _slab(
                f"{suffix}_{i}",
                kind,
                xa + 0.5 * piece,
                y_m,
                z_m,
                sx,
                sy_m,
                geom=geom,
            )
        )
    return tuple(boxes)


def _slabs_across_y(
    suffix: str,
    kind: str,
    x_m: float,
    z_m: float,
    sx_m: float,
    geom: ScaffoldGeom,
) -> tuple[SpawnBox, ...]:
    row_w = geom.deck_width_m / 2.0
    sy = row_w + BOARD_SEAM_OVERLAP_M
    return tuple(
        _slab(
            f"{suffix}_r{row}",
            kind,
            x_m,
            (row + 0.5) * row_w,
            z_m,
            sx_m,
            sy,
            geom=geom,
        )
        for row in range(2)
    )


def stair_landings(geom: ScaffoldGeom = STAGE1_GEOM) -> tuple[SpawnBox, ...]:
    """Split 踊場: 1F boards, 2F/3F pads that do not cap the stair shafts.

    3F south walk covers the whole stair bay plus overlap onto the working
    deck so a Recast agent can leave L1 (yard end) onto 3F. North of that
    strip stays open so L1 is not a ceiling.
    """
    y_south = 0.5 * LANDING_SOUTH_WIDTH_M
    x_mid = -0.5 * geom.stair_bay_m
    f2_x0 = -LANDING_SILL_DEPTH_M
    f2_x1 = DECK_NAV_OVERLAP_M
    f2_sx = f2_x1 - f2_x0
    f2_x = 0.5 * (f2_x0 + f2_x1)
    f3_x0 = ramp_yard_x_m() - 0.50
    f3_x1 = DECK_NAV_OVERLAP_M
    exit_x = -geom.stair_bay_m + 0.5 * LANDING_SILL_DEPTH_M
    sill_x = 0.5 * (f2_x0 + f2_x1)
    yard_x = ramp_yard_x_m()
    z1 = geom.floor_z_m(1)
    z2 = geom.floor_z_m(2)
    z3 = geom.floor_z_m(3)
    return (
        *_slabs_across_y(
            "landing_f1",
            "landing",
            x_mid,
            z1,
            geom.stair_bay_m,
            geom,
        ),
        *_slabs_across_y(
            "landing_f2_sill",
            "landing",
            f2_x,
            z2,
            f2_sx,
            geom,
        ),
        _slab(
            "landing_f3_exit",
            "landing",
            exit_x,
            y_south,
            z3,
            LANDING_SILL_DEPTH_M,
            LANDING_SOUTH_WIDTH_M,
            geom=geom,
        ),
        *_slabs_along_x(
            "landing_f3_south",
            "landing",
            f3_x0,
            f3_x1,
            y_south,
            z3,
            LANDING_SOUTH_WIDTH_M,
            geom,
        ),
        _slab(
            "landing_f3_sill",
            "landing",
            sill_x,
            y_south,
            z3,
            f2_sx,
            LANDING_SOUTH_WIDTH_M,
            geom=geom,
        ),
        *_slabs_across_y(
            "landing_f3_yard",
            "landing",
            yard_x,
            z3,
            RAMP_YARD_PAD_M,
            geom,
        ),
    )


def ramp_step_rise_m(geom: ScaffoldGeom = STAGE1_GEOM) -> float:
    return geom.lift_m / float(RAMP_STEPS)


def ramp_pitch_deg(geom: ScaffoldGeom = STAGE1_GEOM) -> float:
    _ = geom
    return 0.0


def _ramp_step_x_m(lift: int, step: int, geom: ScaffoldGeom) -> float:
    _ = geom
    return ramp_step_x_m(lift, step)


def spot_ramps(geom: ScaffoldGeom = STAGE1_GEOM) -> tuple[SpawnBox, ...]:
    """AABB treads: L0 south 1F→2F, L1 north 2F→3F."""
    n_lifts = geom.n_floors - 1
    boxes: list[SpawnBox] = []
    for pose in stair_tread_poses(
        lift_m=geom.lift_m,
        deck_width_m=geom.deck_width_m,
        n_lifts=n_lifts,
    ):
        boxes.append(
            SpawnBox(
                actor=f"{ACTOR_PREFIX}_ramp_L{pose.lift}_{pose.step}",
                kind="ramp",
                local_cm=scaffold_xyz_to_local_cm(
                    pose.x_m, pose.y_m, pose.z_bottom_m, geom=geom
                ),
                scale=cube_scale_level_world(pose.sx_m, pose.sy_m, pose.sz_m),
            )
        )
    return tuple(boxes)


def spot_ramp_cheeks(geom: ScaffoldGeom = STAGE1_GEOM) -> tuple[SpawnBox, ...]:
    """Side plates on both faces of each overlap gap (not a full-height wall)."""
    rise = ramp_step_rise_m(geom)
    gap = rise - DECK_THICKNESS_M
    half = 0.5 * RAMP_STEP_DEPTH_M
    half_t = 0.5 * STRINGER_THICK_M
    sides = (
        (0, "s", -half_t, floor_walk_z_m(1, geom)),
        (0, "n", RAMP_WIDTH_M + half_t, floor_walk_z_m(1, geom)),
        (1, "s", geom.deck_width_m - RAMP_WIDTH_M - half_t, floor_walk_z_m(2, geom)),
        (1, "n", geom.deck_width_m + half_t, floor_walk_z_m(2, geom)),
    )
    boxes: list[SpawnBox] = []
    for lift, side, y, z_low in sides:
        for step in range(RAMP_STEPS - 1):
            xa = _ramp_step_x_m(lift, step, geom)
            xb = _ramp_step_x_m(lift, step + 1, geom)
            o0 = max(xa - half, xb - half)
            o1 = min(xa + half, xb + half)
            z_bottom = z_low + (step + 1) * rise
            boxes.append(
                SpawnBox(
                    actor=f"{ACTOR_PREFIX}_cheek_L{lift}_{side}_{step}",
                    kind="stringer",
                    local_cm=scaffold_xyz_to_local_cm(
                        0.5 * (o0 + o1), y, z_bottom, geom=geom
                    ),
                    scale=cube_scale_level_world(
                        o1 - o0, STRINGER_THICK_M, gap
                    ),
                )
            )
    return tuple(boxes)


def boxes_from_modules(
    geom: ScaffoldGeom = STAGE1_GEOM,
) -> tuple[SpawnBox, ...]:
    spec = build_scaffold(geom)
    boxes: list[SpawnBox] = []
    for module in spec.modules:
        if skip_spawn_module(module):
            continue
        boxes.append(module_to_spawn_box(module, geom))
    return tuple(boxes)


def _marker_local(
    suffix: str,
    local_xy: Tuple[float, float],
    size_m: float,
    kind: str = "marker",
) -> SpawnBox:
    lx, ly = local_xy
    hz = size_m * 50.0
    return SpawnBox(
        actor=f"{ACTOR_PREFIX}_{suffix}",
        kind=kind,
        local_cm=(lx, ly, hz),
        scale=cube_scale_level_world(size_m, size_m, size_m),
    )


def landmark_boxes(geom: ScaffoldGeom = STAGE1_GEOM) -> tuple[SpawnBox, ...]:
    return (
        _marker_local("storage", STAGING_LOCAL_XY_CM, 0.50),
        SpawnBox(
            actor=f"{ACTOR_PREFIX}_drop_f1",
            kind="marker",
            local_cm=scaffold_xyz_to_local_cm(
                DROP_XY_M[0], DROP_XY_M[1], 0.15, geom=geom
            ),
            scale=cube_scale_level_world(0.30, 0.30, 0.30),
        ),
        SpawnBox(
            actor=f"{ACTOR_PREFIX}_refuge_f1",
            kind="marker",
            local_cm=scaffold_xyz_to_local_cm(
                REFUGE_XY_M[0], REFUGE_XY_M[1], 0.15, geom=geom
            ),
            scale=cube_scale_level_world(0.30, 0.30, 0.30),
        ),
    )


def module_to_spawn_box(module: Module, geom: ScaffoldGeom = STAGE1_GEOM) -> SpawnBox:
    """Map a grammar Module to a PIE cube at the finished pose (same as boxes_from_modules)."""
    z = module.z_m
    pitch = module.pitch_deg
    roll = module.roll_deg
    # Long axis is +X. Tilt in the bay–lift plane so 交差筋違 is diagonal, not a header.
    if module.kind == "brace":
        pitch = module.roll_deg
        roll = 0.0
    kind = module.kind
    if kind == "board":
        kind = "deck"
    elif kind == "stair_tread":
        kind = "ramp"
    return SpawnBox(
        actor=f"{ACTOR_PREFIX}_{module.module_id}",
        kind=kind,
        local_cm=scaffold_xyz_to_local_cm(module.x_m, module.y_m, z, geom=geom),
        scale=cube_scale_level_world(module.sx_m, module.sy_m, max(module.sz_m, 0.05)),
        yaw_deg=module.yaw_deg,
        pitch_deg=pitch,
        roll_deg=roll,
    )


def staging_props() -> tuple[SpawnBox, ...]:
    """Yard markers only. The truck is a TrafficSystem vehicle BP, not cubes."""
    return ()


def all_spawn_boxes(geom: ScaffoldGeom = STAGE1_GEOM) -> tuple[SpawnBox, ...]:
    return (
        boxes_from_modules(geom)
        + deck_slabs(geom)
        + stair_landings(geom)
        + spot_ramps(geom)
        + spot_ramp_cheeks(geom)
        + landmark_boxes(geom)
        + staging_props()
    )
