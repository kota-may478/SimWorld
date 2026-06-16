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
    ucv.set_camera_resolution(camera_id, SENSOR_RESOLUTION)
    ucv.set_camera_fov(camera_id, SENSOR_FOV_DEG)


def update_sensor_camera_pose(ucv: UnrealCV, robot_name: str, camera_id: int) -> None:
    """Sync external camera to robot. No-op for BP-attached FusionCamSensor."""
    if uses_robot_mounted_fusion_cam(ucv, camera_id):
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


def fetch_mask_rgb(communicator: Communicator, camera_id: int):
    return communicator.get_camera_observation(camera_id, "object_mask", mode="direct")
