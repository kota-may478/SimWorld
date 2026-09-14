"""Drop / refuge / socket poses for UE HRC missions.

Spot delivers to a per-floor staging drop. The Assembler picks up there and
walks to the member socket. Do not send Spot to the socket.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional, Tuple

from oracle.simulate import _floor_of_z, _next_hop
from scene.erect_plan import ErectItem
from scene.field import DECK_THICKNESS_M, stair_lane_y_m, stair_tread_poses
from scene.geometry import STAGE1_GEOM, ScaffoldGeom
from ue.layout import local_cm_to_scaffold_xyz, scaffold_xyz_to_local_cm
from ue.placement import (
    DROP_SLOT_COUNT,
    DROP_SLOT_PITCH_M,
    DROP_XY_M,
    HUMANOID_STORY_CLEARANCE_MARGIN_CM,
    NATIVE_HUMANOID_HEIGHT_CM,
    REFUGE_XY_M,
    YARD_HUMAN_Z_CM,
)

LocalCm = Tuple[float, float, float]
ASSEMBLER_HIP_Z_CM = 90.0


def story_clearance_cm(
    lift_m: float,
    deck_thickness_m: float = DECK_THICKNESS_M,
    *,
    margin_cm: float = HUMANOID_STORY_CLEARANCE_MARGIN_CM,
) -> float:
    """Walkable clear height under the next deck, minus a safety margin."""
    return max(0.0, float(lift_m) * 100.0 - float(deck_thickness_m) * 100.0 - float(margin_cm))


def humanoid_fit_scale(
    *,
    lift_m: float,
    deck_thickness_m: float = DECK_THICKNESS_M,
    native_height_cm: float = NATIVE_HUMANOID_HEIGHT_CM,
    margin_cm: float = HUMANOID_STORY_CLEARANCE_MARGIN_CM,
) -> float:
    """Uniform scale so a native-height humanoid fits under one storey."""
    native = max(1.0, float(native_height_cm))
    return min(1.0, story_clearance_cm(lift_m, deck_thickness_m, margin_cm=margin_cm) / native)


def humanoid_root_z_cm(
    *,
    scale: float,
    native_root_z_cm: float = YARD_HUMAN_Z_CM,
) -> float:
    """Pelvis/root height above the walk surface after ``humanoid_fit_scale``."""
    return float(native_root_z_cm) * max(0.05, float(scale))


# Same root Z as the yard humanoid (capsule origin is not the feet).
_ASSEMBLER_FIT_SCALE = humanoid_fit_scale(lift_m=STAGE1_GEOM.lift_m)
ASSEMBLER_SCALE = _ASSEMBLER_FIT_SCALE
ASSEMBLER_FOOT_Z_CM = humanoid_root_z_cm(scale=_ASSEMBLER_FIT_SCALE)
# BP_SpotRobot origin is the body, not the paws — same class of bug as a
# humanoid placed on the walk surface. Match Level's ROBOT_FOOT_Z_OFFSET_CM.
SPOT_STAND_Z_CM = 50.0
# Stand this far in front of the spawned member so the capsule is not inside it.
ASSEMBLER_INSTALL_STANDOFF_M = 0.70
HUMANOID_NAME_MARKERS = (
    "User_Agent",
    "Pedestrian",
    "BP_Mannequin",
    "ScaffoldHrc_assembler",
)


def reachable_floor(completed_stair_lifts: Iterable[int]) -> int:
    floor = 1
    lift = 0
    done = set(completed_stair_lifts)
    while lift in done:
        floor += 1
        lift += 1
    return min(floor, STAGE1_GEOM.n_floors)


def _floor_z_m(floor: int, geom: ScaffoldGeom = STAGE1_GEOM) -> float:
    return geom.floor_z_m(max(1, min(floor, geom.n_floors)))


def drop_slot_xy_m(slot: int) -> Tuple[float, float]:
    index = max(0, min(int(slot), DROP_SLOT_COUNT - 1))
    return (DROP_XY_M[0] + index * DROP_SLOT_PITCH_M, DROP_XY_M[1])


def first_free_drop_slot(occupied: Iterable[int]) -> Optional[int]:
    taken = {int(i) for i in occupied}
    for slot in range(DROP_SLOT_COUNT):
        if slot not in taken:
            return slot
    return None


def floor_drop_slot_local_cm(
    floor: int, slot: int = 0, geom: ScaffoldGeom = STAGE1_GEOM
) -> LocalCm:
    x_m, y_m = drop_slot_xy_m(slot)
    return scaffold_xyz_to_local_cm(
        x_m, y_m, _floor_z_m(floor, geom), geom=geom
    )


def floor_drop_local_cm(
    floor: int, geom: ScaffoldGeom = STAGE1_GEOM
) -> LocalCm:
    """Spot staging drop on this floor (scaffold metres → Level local cm)."""
    return floor_drop_slot_local_cm(floor, 0, geom=geom)


def floor_refuge_local_cm(
    floor: int, geom: ScaffoldGeom = STAGE1_GEOM
) -> LocalCm:
    return scaffold_xyz_to_local_cm(
        REFUGE_XY_M[0], REFUGE_XY_M[1], _floor_z_m(floor, geom), geom=geom
    )


def socket_local_cm(
    item: ErectItem,
    geom: ScaffoldGeom = STAGE1_GEOM,
    walk_floor: Optional[int] = None,
) -> LocalCm:
    """Walkable work pose at the member socket (not mid-air brace center).

    walk_floor is the deck Spot/Assembler may occupy now (1F until the
    2F stair exists). The member still spawns at its true z.
    """
    module = item.module
    floor = item.floor if walk_floor is None else min(item.floor, walk_floor)
    floor = max(1, min(floor, geom.n_floors))
    return scaffold_xyz_to_local_cm(
        module.x_m, module.y_m, geom.floor_z_m(floor), geom=geom
    )


def assembler_install_xy_m(
    item: ErectItem,
    geom: ScaffoldGeom = STAGE1_GEOM,
    *,
    standoff_m: float = ASSEMBLER_INSTALL_STANDOFF_M,
) -> Tuple[float, float]:
    """XY in front of the member, toward the floor drop (not on the pipe).

    Always clamp onto the walkable deck slab. Stair/ramp members sit at x<0; an
    unclamped standoff lands on AABB tread tops, ``standing_local_cm`` then lifts
    the goal to ~2F, and the Assembler deadlocks on a 15m stair hop.
    """
    mx, my = float(item.module.x_m), float(item.module.y_m)
    dx = DROP_XY_M[0] - mx
    dy = DROP_XY_M[1] - my
    n = math.hypot(dx, dy)
    if n < 0.15:
        dx, dy, n = -1.0, 0.0, 1.0
    x = mx + float(standoff_m) * dx / n
    y = my + float(standoff_m) * dy / n
    x = min(max(x, 0.20), geom.deck_length_m - 0.20)
    y = min(max(y, 0.20), geom.deck_width_m - 0.20)
    return x, y


def assembler_install_local_cm(
    item: ErectItem,
    geom: ScaffoldGeom = STAGE1_GEOM,
    walk_floor: Optional[int] = None,
) -> LocalCm:
    """Assembler hip-plane feet pose used while installing (offset from spawn)."""
    x_m, y_m = assembler_install_xy_m(item, geom)
    floor = item.floor if walk_floor is None else min(item.floor, walk_floor)
    floor = max(1, min(floor, geom.n_floors))
    # Deck top for the walk floor — never member z / tread snap (see install_xy).
    return scaffold_xyz_to_local_cm(
        x_m, y_m, floor_walk_z_m(floor, geom), geom=geom
    )


def next_hop_local_cm(
    src_local_cm: LocalCm,
    dest_local_cm: LocalCm,
    geom: ScaffoldGeom = STAGE1_GEOM,
) -> LocalCm:
    """UE local-cm waypoint matching the kinematic stair/corridor hop."""
    src = local_cm_to_scaffold_xyz(*src_local_cm)
    dest = local_cm_to_scaffold_xyz(*dest_local_cm)
    hop = _next_hop(geom, src, dest)
    return scaffold_xyz_to_local_cm(hop[0], hop[1], hop[2], geom=geom)


def floor_walk_z_m(floor: int, geom: ScaffoldGeom = STAGE1_GEOM) -> float:
    return geom.floor_z_m(max(1, min(floor, geom.n_floors))) + DECK_THICKNESS_M


def walk_surface_z_m(
    x_m: float,
    y_m: float,
    z_hint_m: float,
    geom: ScaffoldGeom = STAGE1_GEOM,
) -> float:
    """Top of the board under (x, y): AABB tread, deck, or yard ground."""
    tread = _nearest_tread(x_m, y_m, z_hint_m, geom)
    if tread is not None:
        return float(tread.z_bottom_m + tread.sz_m)
    if 0.0 <= x_m <= geom.deck_length_m and 0.0 <= y_m <= geom.deck_width_m:
        return floor_walk_z_m(_floor_of_z(geom, z_hint_m), geom)
    return 0.0


def _nearest_tread(
    x_m: float,
    y_m: float,
    z_hint_m: float,
    geom: ScaffoldGeom,
):
    best = None
    best_d = 1e9
    for pose in stair_tread_poses(
        lift_m=geom.lift_m, deck_width_m=geom.deck_width_m, n_lifts=geom.n_floors - 1
    ):
        half_x = 0.5 * pose.sx_m + 0.12
        half_y = 0.5 * pose.sy_m + 0.12
        if abs(x_m - pose.x_m) > half_x or abs(y_m - pose.y_m) > half_y:
            continue
        z_top = pose.z_bottom_m + pose.sz_m
        dist = math.hypot(x_m - pose.x_m, y_m - pose.y_m) + 0.35 * abs(z_hint_m - z_top)
        if dist < best_d:
            best_d = dist
            best = pose
    return best


def next_surface_hop_xyz(
    src: Tuple[float, float, float],
    dest: Tuple[float, float, float],
    geom: ScaffoldGeom = STAGE1_GEOM,
) -> Tuple[float, float, float]:
    """One walk step on tread tops (or the deck/yard), not a chord through the flight."""
    src_f = _floor_of_z(geom, src[2])
    dest_f = _floor_of_z(geom, dest[2])
    if src_f != dest_f:
        going_up = dest_f > src_f
        lift = (src_f - 1) if going_up else (src_f - 2)
        lift = max(0, min(geom.n_floors - 2, lift))
        lane_y = stair_lane_y_m(lift, geom.deck_width_m)
        treads = [
            pose
            for pose in stair_tread_poses(
                lift_m=geom.lift_m,
                deck_width_m=geom.deck_width_m,
                n_lifts=geom.n_floors - 1,
            )
            if pose.lift == lift
        ]
        treads.sort(key=lambda pose: pose.step)
        if treads:
            on_lane = abs(src[1] - lane_y) <= 0.50
            near = _nearest_tread(src[0], src[1], src[2], geom)
            if near is None or near.lift != lift or not on_lane:
                first = treads[0] if going_up else treads[-1]
                return (first.x_m, first.y_m, first.z_bottom_m + first.sz_m)
            if going_up:
                nxt = [pose for pose in treads if pose.step > near.step]
                if nxt:
                    pose = min(nxt, key=lambda item: item.step)
                    return (pose.x_m, pose.y_m, pose.z_bottom_m + pose.sz_m)
            else:
                nxt = [pose for pose in treads if pose.step < near.step]
                if nxt:
                    pose = max(nxt, key=lambda item: item.step)
                    return (pose.x_m, pose.y_m, pose.z_bottom_m + pose.sz_m)
    hop = _next_hop(geom, src, dest)
    z_hint = hop[2] if abs(hop[2] - src[2]) > 1e-6 else dest[2]
    return (hop[0], hop[1], walk_surface_z_m(hop[0], hop[1], z_hint, geom))


def standing_local_cm(
    local_cm: LocalCm,
    clearance_cm: float,
    geom: ScaffoldGeom = STAGE1_GEOM,
) -> LocalCm:
    """Actor root just above the board/ground under this XY."""
    x_m, y_m, z_m = local_cm_to_scaffold_xyz(*local_cm)
    feet_z_m = z_m - float(clearance_cm) / 100.0
    surf_m = walk_surface_z_m(x_m, y_m, max(feet_z_m, 0.0), geom)
    lx, ly, lz = scaffold_xyz_to_local_cm(x_m, y_m, surf_m, geom=geom)
    return (lx, ly, lz + float(clearance_cm))


def next_hop_standing_local_cm(
    src_local_cm: LocalCm,
    dest_local_cm: LocalCm,
    clearance_cm: float,
    geom: ScaffoldGeom = STAGE1_GEOM,
) -> LocalCm:
    """Next tread or deck pose, then raise the actor origin."""
    src = local_cm_to_scaffold_xyz(*_feet_local_cm(src_local_cm, clearance_cm))
    dest = local_cm_to_scaffold_xyz(*_feet_local_cm(dest_local_cm, clearance_cm))
    hop = next_surface_hop_xyz(src, dest, geom)
    lx, ly, lz = scaffold_xyz_to_local_cm(hop[0], hop[1], hop[2], geom=geom)
    return (lx, ly, lz + float(clearance_cm))


def _feet_local_cm(local_cm: LocalCm, clearance_cm: float) -> LocalCm:
    return (local_cm[0], local_cm[1], local_cm[2] - float(clearance_cm))


def snap_hover_local_cm(
    src_local_cm: LocalCm,
    dest_local_cm: LocalCm,
    *,
    slop_cm: float = 25.0,
) -> LocalCm:
    """If the pawn was launched above the goal walk plane, drop back onto it."""
    if src_local_cm[2] > dest_local_cm[2] + float(slop_cm):
        return (src_local_cm[0], src_local_cm[1], dest_local_cm[2])
    return src_local_cm


def assembler_hip_local_cm(local_cm: LocalCm) -> LocalCm:
    return (local_cm[0], local_cm[1], local_cm[2] + ASSEMBLER_HIP_Z_CM)


def assembler_stand_local_cm(local_cm: LocalCm) -> LocalCm:
    """Walk pose: same actor-root height as the grounded yard humanoid."""
    return standing_local_cm(local_cm, ASSEMBLER_FOOT_Z_CM)


def lift_sunk_local_cm(
    local_cm: LocalCm,
    clearance_cm: float,
    geom: ScaffoldGeom = STAGE1_GEOM,
    *,
    slop_cm: float = 15.0,
) -> LocalCm:
    """If the pawn has fallen through the walk plane, snap back onto it."""
    stood = standing_local_cm(local_cm, clearance_cm, geom=geom)
    if local_cm[2] < stood[2] - float(slop_cm):
        return stood
    return local_cm


def extra_humanoid_actor_names(
    names: Iterable[str],
    keep: Iterable[str],
) -> list[str]:
    """Leftover pedestrians from prior PIE runs (do not destroy; park them)."""
    kept = {str(n) for n in keep}
    extras: list[str] = []
    for name in names:
        if name in kept:
            continue
        if any(marker in name for marker in HUMANOID_NAME_MARKERS):
            extras.append(name)
    return extras
