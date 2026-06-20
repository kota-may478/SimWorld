#!/usr/bin/env python3
"""Carry visual for site transport mission (crate → humanoid feet)."""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import grid_env_hri_simulation as geh
from level_coords import FLOOR_REF_Z_CM, foot_world_xyz_from_local_xy
from placement import SiteTransportRegistry
from pie_safety import tick_settle  # noqa: E402
from pie_spawn_safety import spawn_bp_resilient

WorldXYZ = Tuple[float, float, float]
WorldXY = Tuple[float, float]

# Keep the carried prop on SpotDog's back side so it does not fill the forward
# FusionCam view during L2 depth/object-mask capture.
CARRY_FORWARD_CM = -20.0
CARRY_Z_OFFSET_CM = 88.0
PICKUP_STANDOFF_CM = 140.0
PICKUP_ATTACH_STEPS = 8
PICKUP_ATTACH_SLEEP_S = 0.05
DELIVERY_ATTACH_STEPS = 10
DELIVERY_ATTACH_SLEEP_S = 0.04
ACTOR_SETTLE_S = 0.35
ROBOT_ACTOR = geh.ROBOT_ACTOR_NAME
DELIVERY_TOLERANCE_CM = 150.0


def get_robot_pose(ucv, robot_name: str = ROBOT_ACTOR) -> Tuple[WorldXY, float]:
    loc = ucv.get_location(robot_name)
    rot = ucv.get_orientation(robot_name)
    return (float(loc[0]), float(loc[1])), float(rot[1])


def get_robot_carry_pose(ucv, robot_name: str = ROBOT_ACTOR) -> Tuple[WorldXYZ, Tuple[float, float, float]]:
    (rx, ry), yaw_deg = get_robot_pose(ucv, robot_name)
    yaw_rad = math.radians(yaw_deg)
    carry_x = rx + CARRY_FORWARD_CM * math.cos(yaw_rad)
    carry_y = ry + CARRY_FORWARD_CM * math.sin(yaw_rad)
    carry_z = FLOOR_REF_Z_CM + CARRY_Z_OFFSET_CM
    return (carry_x, carry_y, carry_z), (0.0, yaw_deg, 0.0)


def pickup_standoff_xy(
    target_xy: WorldXY,
    from_xy: WorldXY,
    standoff_cm: float = PICKUP_STANDOFF_CM,
) -> WorldXY:
    dx = from_xy[0] - target_xy[0]
    dy = from_xy[1] - target_xy[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        return (target_xy[0] - standoff_cm, target_xy[1])
    scale = standoff_cm / dist
    return (target_xy[0] + dx * scale, target_xy[1] + dy * scale)


def _force_carry_no_collision(ucv, carry_name: str) -> None:
    """Carry mesh must not block CharacterMovement (PropMesh penetration stalls SpotDog)."""
    ucv.set_physics(carry_name, False)
    ucv.set_collision(carry_name, False)
    ucv.set_movable(carry_name, True)
    for cmd in (
        f"vset /object/{carry_name}/collision 0",
        f"vset /object/{carry_name}/physics 0",
    ):
        try:
            geh._ue_request(ucv, cmd, timeout_s=10.0)  # noqa: SLF001
        except Exception:
            pass


def sync_carry_pose(ucv, carry_name: str, robot_name: str = ROBOT_ACTOR) -> None:
    if not geh.actor_exists(ucv, carry_name):
        return
    carry_loc, carry_rot = get_robot_carry_pose(ucv, robot_name)
    ucv.set_location(list(carry_loc), carry_name)
    ucv.set_orientation(carry_rot, carry_name)
    _force_carry_no_collision(ucv, carry_name)


ACTOR_HIDE_STASH_Z_CM = FLOOR_REF_Z_CM - 50_000.0


def _hide_actor_offmap(ucv, actor_name: str) -> None:
    """Move actor far below floor instead of destroy (destroy often crashes Level PIE)."""
    if not geh.actor_exists(ucv, actor_name):
        return
    loc = ucv.get_location(actor_name)
    stash = (float(loc[0]), float(loc[1]), ACTOR_HIDE_STASH_Z_CM)
    ucv.set_physics(actor_name, False)
    ucv.set_collision(actor_name, False)
    ucv.set_location(list(stash), actor_name)
    tick_settle(ucv, settle_s=0.15, ticks=1)


def begin_carry_from_material(
    ucv,
    registry: SiteTransportRegistry,
    *,
    robot_name: str = ROBOT_ACTOR,
) -> Optional[str]:
    transport = registry.transport_slot()
    if transport is None:
        return None
    material_name = registry.material_actor_name
    carry_name = registry.carry_actor_name
    pickup_xyz = None
    if geh.actor_exists(ucv, material_name):
        loc = ucv.get_location(material_name)
        pickup_xyz = (float(loc[0]), float(loc[1]), float(loc[2]))
        _hide_actor_offmap(ucv, material_name)
        time.sleep(ACTOR_SETTLE_S * 0.5)
    elif transport.world_xyz_cm is not None:
        pickup_xyz = transport.world_xyz_cm
    else:
        return None
    if not geh.actor_exists(ucv, carry_name):
        ok, ucv = spawn_bp_resilient(ucv, transport.bp_path, carry_name, timeout_s=120.0)
        if not ok:
            return None
    ucv.set_location(list(pickup_xyz), carry_name)
    ucv.set_orientation((0.0, transport.yaw_deg, 0.0), carry_name)
    _force_carry_no_collision(ucv, carry_name)
    time.sleep(ACTOR_SETTLE_S)
    start = pickup_xyz
    carry_loc, carry_rot = get_robot_carry_pose(ucv, robot_name)
    for step in range(1, PICKUP_ATTACH_STEPS + 1):
        t = step / PICKUP_ATTACH_STEPS
        loc = tuple(start[i] + (carry_loc[i] - start[i]) * t for i in range(3))
        ucv.set_location(loc, carry_name)
        ucv.set_orientation(carry_rot, carry_name)
        time.sleep(PICKUP_ATTACH_SLEEP_S)
    sync_carry_pose(ucv, carry_name, robot_name)
    return carry_name


def deliver_carry_at_humanoid(
    ucv,
    registry: SiteTransportRegistry,
    *,
    robot_name: str = ROBOT_ACTOR,
) -> bool:
    carry_name = registry.carry_actor_name
    if not geh.actor_exists(ucv, carry_name):
        return False
    human_xyz = foot_world_xyz_from_local_xy(*registry.humanoid_local_cm)
    delivery = (human_xyz[0], human_xyz[1], human_xyz[2] + 20.0)
    start = tuple(float(v) for v in ucv.get_location(carry_name))
    for step in range(1, DELIVERY_ATTACH_STEPS + 1):
        t = step / DELIVERY_ATTACH_STEPS
        loc = tuple(start[i] + (delivery[i] - start[i]) * t for i in range(3))
        ucv.set_location(loc, carry_name)
        ucv.set_orientation((0.0, 0.0, 0.0), carry_name)
        time.sleep(DELIVERY_ATTACH_SLEEP_S)
    _hide_actor_offmap(ucv, carry_name)
    time.sleep(ACTOR_SETTLE_S * 0.5)
    robot_xy, _ = get_robot_pose(ucv, robot_name)
    dist = math.hypot(robot_xy[0] - human_xyz[0], robot_xy[1] - human_xyz[1])
    return dist <= DELIVERY_TOLERANCE_CM * 2.0
