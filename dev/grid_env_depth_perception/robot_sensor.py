#!/usr/bin/env python3
"""SpotDog-mounted camera helpers."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from simworld.communicator.communicator import Communicator
from simworld.communicator.unrealcv import UnrealCV

SENSOR_CAMERA_ID_PREFERRED = 1
SENSOR_RESOLUTION = (640, 384)
SENSOR_FOV_DEG = 90.0
SENSOR_CAM_HEIGHT_OFFSET_CM = 45.0
SENSOR_CAM_FORWARD_OFFSET_CM = 22.0
SENSOR_CAM_PITCH_DEG = -5.0
FUSION_CAM_NAME_SUBSTR = "FusionCam"
MASK_BG_BGR = (76, 76, 76)
MASK_BG_TOLERANCE = 8
MASK_MIN_NON_BG_FRACTION = 0.005


def list_camera_names(ucv: UnrealCV) -> List[str]:
    raw = ucv.get_cameras()
    return [t for t in str(raw).replace(",", " ").split() if t]


def resolve_sensor_camera_id(ucv: UnrealCV, preferred_id: int = SENSOR_CAMERA_ID_PREFERRED) -> int:
    names = list_camera_names(ucv)
    for idx, name in enumerate(names):
        if FUSION_CAM_NAME_SUBSTR in name:
            return idx
    for token in names:
        try:
            cam_id = int(token)
            if cam_id == preferred_id:
                return cam_id
        except ValueError:
            continue
    if names:
        return 0
    return preferred_id


def uses_robot_mounted_fusion_cam(ucv: UnrealCV, camera_id: int) -> bool:
    names = list_camera_names(ucv)
    if camera_id < 0 or camera_id >= len(names):
        return False
    return FUSION_CAM_NAME_SUBSTR in names[camera_id]


def uses_engine_follow_camera(ucv: UnrealCV, camera_id: int) -> bool:
    """Cameras that follow the pawn — do not manually teleport."""
    names = list_camera_names(ucv)
    if camera_id < 0 or camera_id >= len(names):
        return False
    name = names[camera_id]
    markers = (FUSION_CAM_NAME_SUBSTR, "ThirdPerson", "PawnSensor")
    return any(marker in name for marker in markers)


def mask_segmentation_active(mask_bgr) -> bool:
    """True when object_mask shows labeled pixels (not flat UE gray background)."""
    if mask_bgr is None or getattr(mask_bgr, "size", 0) == 0:
        return False
    import numpy as np

    bg = np.array(MASK_BG_BGR, dtype=np.int16)
    diff = np.abs(mask_bgr[..., :3].astype(np.int16) - bg)
    near_bg = (diff <= MASK_BG_TOLERANCE).all(axis=-1)
    non_bg = int((~near_bg).sum())
    return non_bg > mask_bgr.shape[0] * mask_bgr.shape[1] * MASK_MIN_NON_BG_FRACTION


def restore_editor_viewmode_lit(ucv: UnrealCV) -> None:
    """Restore PIE/editor viewport to Lit after object_mask or vset viewmode calls."""
    try:
        with ucv.lock:
            ucv.client.request("vset /viewmode lit")
    except Exception:
        pass


def resolve_mask_camera_id(
    communicator: Communicator,
    ucv: UnrealCV,
    fusion_camera_id: int,
) -> int:
    """Pick camera for object_mask; prefer head FusionCam when segmentation is active."""
    names = list_camera_names(ucv)
    order: List[int] = []
    if fusion_camera_id not in order:
        order.append(fusion_camera_id)
    for preferred_name in (FUSION_CAM_NAME_SUBSTR, "ThirdPerson", "PawnSensor"):
        for idx, name in enumerate(names):
            if preferred_name in name and idx not in order:
                order.append(idx)
    for idx in range(len(names)):
        if idx not in order:
            order.append(idx)

    for cam_id in order:
        mask = fetch_mask_rgb(communicator, cam_id)
        if mask_segmentation_active(mask):
            return cam_id
    return fusion_camera_id


def get_yaw_deg(ucv: UnrealCV, actor_name: str) -> float:
    ori = ucv.get_orientation(actor_name)
    return float(ori[1])


def get_pos3d(ucv: UnrealCV, actor_name: str) -> Tuple[float, float, float]:
    loc = ucv.get_location(actor_name)
    return float(loc[0]), float(loc[1]), float(loc[2])


def get_pos2d(ucv: UnrealCV, actor_name: str) -> Tuple[float, float]:
    x, y, _ = get_pos3d(ucv, actor_name)
    return x, y


def yaw_to_unit_vec(yaw_deg: float) -> Tuple[float, float]:
    rad = math.radians(yaw_deg)
    return math.cos(rad), math.sin(rad)


def configure_sensor_camera(ucv: UnrealCV, camera_id: int) -> None:
    """Set resolution/FOV only on external cameras (not pawn-mounted FusionCam)."""
    if uses_engine_follow_camera(ucv, camera_id):
        return
    ucv.set_camera_resolution(camera_id, SENSOR_RESOLUTION)
    ucv.set_camera_fov(camera_id, SENSOR_FOV_DEG)


def update_sensor_camera_pose(ucv: UnrealCV, robot_name: str, camera_id: int) -> None:
    """Sync external camera to robot. Skip pawn-attached / follow cameras."""
    if uses_engine_follow_camera(ucv, camera_id):
        return
    robot_pos = get_pos3d(ucv, robot_name)
    robot_yaw = get_yaw_deg(ucv, robot_name)
    fx, fy = yaw_to_unit_vec(robot_yaw)
    cam_loc = (
        robot_pos[0] + fx * SENSOR_CAM_FORWARD_OFFSET_CM,
        robot_pos[1] + fy * SENSOR_CAM_FORWARD_OFFSET_CM,
        robot_pos[2] + SENSOR_CAM_HEIGHT_OFFSET_CM,
    )
    ucv.set_camera_location(camera_id, cam_loc)
    ucv.set_camera_rotation(camera_id, (SENSOR_CAM_PITCH_DEG, robot_yaw, 0.0))


def fetch_lit_bgr(communicator: Communicator, camera_id: int):
    return communicator.get_camera_observation(camera_id, "lit", mode="direct")


def fetch_mask_rgb(communicator: Communicator, camera_id: int, mode: str = "direct"):
    return communicator.get_camera_observation(camera_id, "object_mask", mode=mode)
