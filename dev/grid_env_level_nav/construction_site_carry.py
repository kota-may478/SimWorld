#!/usr/bin/env python3
"""Construction-site carry visual using the transport-target construction prop BP."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

_THIS_DIR = Path(__file__).resolve().parent
_ROOT = _THIS_DIR.parent.parent
_GEH = _ROOT / "dev" / "grid_env_hri"
for _p in (_GEH, _THIS_DIR, _ROOT / "dev" / "grid_env_10k"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import grid_env_hri_simulation as geh  # noqa: E402
from construction_site_placement import ConstructionSiteRegistry  # noqa: E402
from level_coords import FLOOR_REF_Z_CM  # noqa: E402

WorldXYZ = Tuple[float, float, float]
WorldXY = Tuple[float, float]

CARRY_FORWARD_CM = 58.0
CARRY_SIDE_CM = 0.0
CARRY_Z_OFFSET_CM = 72.0
PICKUP_STANDOFF_CM = 140.0
PICKUP_ATTACH_STEPS = 8
PICKUP_ATTACH_SLEEP_S = 0.05
DELIVERY_ATTACH_STEPS = 10
DELIVERY_ATTACH_SLEEP_S = 0.04
ACTOR_SETTLE_S = 0.35
ROBOT_ACTOR = geh.ROBOT_ACTOR_NAME


def _actor_exists(ucv, name: str) -> bool:
    return geh.actor_exists(ucv, name)


def get_robot_pose(ucv, robot_name: str = ROBOT_ACTOR) -> Tuple[WorldXY, float]:
    loc = ucv.get_location(robot_name)
    rot = ucv.get_orientation(robot_name)
    return (float(loc[0]), float(loc[1])), float(rot[1])


def get_robot_carry_pose(ucv, robot_name: str = ROBOT_ACTOR) -> Tuple[WorldXYZ, Tuple[float, float, float]]:
    (rx, ry), yaw_deg = get_robot_pose(ucv, robot_name)
    yaw_rad = math.radians(yaw_deg)
    carry_x = rx + CARRY_FORWARD_CM * math.cos(yaw_rad) - CARRY_SIDE_CM * math.sin(yaw_rad)
    carry_y = ry + CARRY_FORWARD_CM * math.sin(yaw_rad) + CARRY_SIDE_CM * math.cos(yaw_rad)
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


def sync_carry_pose(ucv, carry_name: str, robot_name: str = ROBOT_ACTOR) -> None:
    if not _actor_exists(ucv, carry_name):
        return
    carry_loc, carry_rot = get_robot_carry_pose(ucv, robot_name)
    ucv.set_location(list(carry_loc), carry_name)
    ucv.set_orientation(carry_rot, carry_name)
    ucv.set_physics(carry_name, False)
    ucv.set_collision(carry_name, False)
    ucv.set_movable(carry_name, True)


def begin_carry_from_material(
    ucv,
    registry: ConstructionSiteRegistry,
    *,
    robot_name: str = ROBOT_ACTOR,
) -> Optional[str]:
    """Hide ground transport prop and attach a carry clone to the robot."""
    transport = registry.transport_slot()
    if transport is None:
        return None
    material_name = registry.material_actor_name
    carry_name = registry.carry_actor_name

    pickup_xyz = None
    if _actor_exists(ucv, material_name):
        loc = ucv.get_location(material_name)
        pickup_xyz = (float(loc[0]), float(loc[1]), float(loc[2]))
        geh._ue_request(ucv, f"vset /object/{material_name}/destroy", timeout_s=30.0)  # noqa: SLF001
        geh.wait_until_actor_gone(ucv, material_name, timeout_s=4.0)
        time.sleep(ACTOR_SETTLE_S)
    elif transport.world_xyz_cm is not None:
        pickup_xyz = transport.world_xyz_cm
    else:
        return None

    if not geh.spawn_bp(ucv, transport.bp_path, carry_name, timeout_s=120.0):
        return None
    ucv.set_location(list(pickup_xyz), carry_name)
    ucv.set_orientation((0.0, transport.yaw_deg, 0.0), carry_name)
    ucv.set_physics(carry_name, False)
    ucv.set_collision(carry_name, False)
    ucv.set_movable(carry_name, True)
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


def deliver_carry_at_home(
    ucv,
    registry: ConstructionSiteRegistry,
    *,
    robot_name: str = ROBOT_ACTOR,
) -> bool:
    carry_name = registry.carry_actor_name
    if not _actor_exists(ucv, carry_name):
        return False
    from level_coords import foot_world_xyz_from_local_xy  # noqa: WPS433

    home_xyz = foot_world_xyz_from_local_xy(*registry.home_local_cm)
    start = tuple(float(v) for v in ucv.get_location(carry_name))
    delivery = (home_xyz[0], home_xyz[1], home_xyz[2] + 20.0)
    for step in range(1, DELIVERY_ATTACH_STEPS + 1):
        t = step / DELIVERY_ATTACH_STEPS
        loc = tuple(start[i] + (delivery[i] - start[i]) * t for i in range(3))
        ucv.set_location(loc, carry_name)
        ucv.set_orientation((0.0, 0.0, 0.0), carry_name)
        time.sleep(DELIVERY_ATTACH_SLEEP_S)
    geh._ue_request(ucv, f"vset /object/{carry_name}/destroy", timeout_s=30.0)  # noqa: SLF001
    geh.wait_until_actor_gone(ucv, carry_name, timeout_s=4.0)
    time.sleep(ACTOR_SETTLE_S)
    return True
