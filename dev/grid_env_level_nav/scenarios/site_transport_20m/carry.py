#!/usr/bin/env python3
"""Carry visual for site transport mission (crate → humanoid feet)."""

from __future__ import annotations

import json
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

# Keep the carried prop on SpotDog's back (slightly aft). Collision is disabled on
# attach — original floor-relative height is sufficient for the carry visual.
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
CARRY_SOCKET_NAME = "CarrySocket"
VBP_ATTACH_CARRY = "AttachCarryActor"
VBP_DETACH_CARRY = "DetachCarryActor"
VBP_PROBE_ATTACH = "ProbeCarryAttach"

_carry_ue_attached = False
_attach_vbp_available: Optional[bool] = None


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


def is_carry_ue_attached() -> bool:
    return _carry_ue_attached


def reset_carry_attach_state() -> None:
    global _carry_ue_attached, _attach_vbp_available
    _carry_ue_attached = False
    _attach_vbp_available = None


def _vbp_success(raw: object) -> bool:
    if raw is None:
        return False
    text = str(raw).strip().lower()
    if not text or text.startswith("error"):
        return False
    if text in {"true", "ok", "1"}:
        return True
    if text in {"false", "0"}:
        return False
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        return True
    if isinstance(payload, dict):
        if payload.get("ok") is False or payload.get("error"):
            return False
        return bool(payload.get("ok", True))
    return True


def probe_carry_attach_vbp(ucv, robot_name: str = ROBOT_ACTOR) -> bool:
    """True when BP_SpotRobot exposes AttachCarryActor (see CARRY_ATTACH_UE_SETUP.md)."""
    global _attach_vbp_available
    if _attach_vbp_available is not None:
        return _attach_vbp_available
    for cmd in (
        f"vbp {robot_name} {VBP_PROBE_ATTACH}",
        f"vbp {robot_name} {VBP_ATTACH_CARRY} __probe__",
    ):
        try:
            raw = geh._ue_request(ucv, cmd, timeout_s=10.0)  # noqa: SLF001
        except (ConnectionError, OSError, RuntimeError, ValueError):
            continue
        text = str(raw).strip().lower() if raw is not None else ""
        if not text or "not found" in text or "unknown" in text or text.startswith("error"):
            continue
        _attach_vbp_available = True
        return True
    _attach_vbp_available = False
    return False


def attach_carry_to_robot_bone(
    ucv,
    carry_name: str,
    robot_name: str = ROBOT_ACTOR,
) -> bool:
    """Attach carry actor to robot skeletal socket via vbp (falls back to Python sync)."""
    global _carry_ue_attached
    if not geh.actor_exists(ucv, carry_name):
        return False
    if not probe_carry_attach_vbp(ucv, robot_name):
        return False
    _force_carry_no_collision(ucv, carry_name)
    try:
        raw = geh._ue_request(  # noqa: SLF001
            ucv,
            f"vbp {robot_name} {VBP_ATTACH_CARRY} {carry_name}",
            timeout_s=15.0,
        )
    except (ConnectionError, OSError, RuntimeError, ValueError):
        return False
    if not _vbp_success(raw):
        return False
    _carry_ue_attached = True
    print(
        f"[Site20Carry] UE bone attach {carry_name!r} → socket {CARRY_SOCKET_NAME!r} "
        "(no Python sync during leg2)"
    )
    return True


def detach_carry_from_robot_bone(
    ucv,
    carry_name: str,
    robot_name: str = ROBOT_ACTOR,
    *,
    force: bool = False,
) -> bool:
    """Detach carry from robot socket before delivery animation."""
    global _carry_ue_attached
    if not force and not _carry_ue_attached:
        return False
    if not probe_carry_attach_vbp(ucv, robot_name):
        _carry_ue_attached = False
        return False
    if not geh.actor_exists(ucv, carry_name):
        _carry_ue_attached = False
        return False
    try:
        raw = geh._ue_request(  # noqa: SLF001
            ucv,
            f"vbp {robot_name} {VBP_DETACH_CARRY} {carry_name}",
            timeout_s=15.0,
        )
    except (ConnectionError, OSError, RuntimeError, ValueError):
        return False
    _carry_ue_attached = False
    if not _vbp_success(raw):
        print(f"[Site20Carry] WARN: DetachCarryActor returned {raw!r}")
        return False
    print(f"[Site20Carry] UE bone detach {carry_name!r}")
    return True


def reset_carry_from_previous_mission(
    ucv,
    carry_name: str,
    robot_name: str = ROBOT_ACTOR,
) -> None:
    """Detach UE carry + stash crate after aborted runs (new Python process)."""
    reset_carry_attach_state()
    if not geh.actor_exists(ucv, carry_name):
        return
    try:
        detach_carry_from_robot_bone(ucv, carry_name, robot_name, force=True)
    except (ConnectionError, OSError, RuntimeError, ValueError) as exc:
        print(f"[Site20Carry] detach skip ({exc})")
    tick_settle(ucv, settle_s=0.25, ticks=1)
    _hide_actor_offmap(ucv, carry_name)


def _force_carry_no_collision(ucv, carry_name: str) -> None:
    """Carry mesh must not block CharacterMovement (PropMesh penetration stalls SpotDog)."""
    ucv.set_physics(carry_name, False)
    ucv.set_collision(carry_name, False)
    ucv.set_movable(carry_name, False)
    for cmd in (
        f"vset /object/{carry_name}/collision 0",
        f"vset /object/{carry_name}/physics 0",
        f"vset /object/{carry_name}/objectapicollision false",
        f"vbp {carry_name} SetActorEnableCollision false",
    ):
        try:
            geh._ue_request(ucv, cmd, timeout_s=10.0)  # noqa: SLF001
        except Exception:
            pass


def sync_carry_pose(
    ucv,
    carry_name: str,
    robot_name: str = ROBOT_ACTOR,
    *,
    refresh_collision: bool = False,
) -> None:
    if not geh.actor_exists(ucv, carry_name):
        return
    carry_loc, carry_rot = get_robot_carry_pose(ucv, robot_name)
    ucv.set_location(list(carry_loc), carry_name)
    ucv.set_orientation(carry_rot, carry_name)
    if refresh_collision:
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
    sync_carry_pose(ucv, carry_name, robot_name, refresh_collision=True)
    if not attach_carry_to_robot_bone(ucv, carry_name, robot_name):
        carry_loc, _ = get_robot_carry_pose(ucv, robot_name)
        print(
            f"[Site20Carry] visual ready {carry_name!r} @ z={carry_loc[2]:.1f} "
            f"(floor+{CARRY_Z_OFFSET_CM:.0f}cm, Python sync — "
            f"see CARRY_ATTACH_UE_SETUP.md for bone attach)"
        )
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
    detach_carry_from_robot_bone(ucv, carry_name, robot_name)
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
