# ---
# jupyter:
#   jupytext:
#     cell_metadata_filter: -all
#     formats: ipynb,py:percent
#     notebook_metadata_filter: -all,kernelspec,jupytext
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.19.1
#   kernelspec:
#     display_name: simworld
#     language: python
#     name: python3
# ---

# %% [markdown]
# # SpotDog Human Following in a 10m x 10m Room
#
# This project reuses the same square-room setup from `dev/hri_agv`.
#
# Human:
# - Walks straight continuously.
# - If blocked, turns and keeps walking.
#
# SpotDog:
# - Tracks the person and follows at about 1m.
# - Uses camera detection for heading control.
# - Uses depth sensing for distance control.
# - If person is lost, spins in place to search.

# %%
import importlib
import math
import sys
import threading
import time
from collections import deque
from io import BytesIO
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import pandas as pd

# Resolve repo root robustly and add it to sys.path.
if "__file__" in globals():
    script_path = Path(__file__).resolve()
    root_candidates = [script_path.parents[2], script_path.parents[1], script_path.parents[0]]
else:
    cwd = Path().resolve()
    root_candidates = [cwd, cwd.parent, cwd.parent.parent]

for candidate in root_candidates:
    if (candidate / "simworld").exists():
        sys.path.append(str(candidate))
        break

from simworld.agent.humanoid import Humanoid
from simworld.communicator.communicator import Communicator
from simworld.communicator.unrealcv import UnrealCV
from simworld.utils.vector import Vector


# %%
# ---------------------------------------------------------------------------
# UE connection
# ---------------------------------------------------------------------------
ucv = UnrealCV()
communicator = Communicator(ucv)


# %%
# ---------------------------------------------------------------------------
# Simulation configuration
# ---------------------------------------------------------------------------

# Room in Unreal units (cm)
ROOM_CM = 1000
WALL_MARGIN_CM = 80

WALL_BP_PATH = "/Game/InteractableAsset/Box/BP_Interactable_Box.BP_Interactable_Box_C"
WALL_THICK_CM = 20
WALL_H_CM = 300
WALL_Z_CM = 0
WALL_ASSET_SIZE_CM = 100
WALL_SEGMENT_LEN_CM = 100
WALL_SEGMENT_OVERLAP_NS_CM = 40
WALL_SEGMENT_OVERLAP_EW_CM = 25

HUMAN_BP_PATH = "/Game/TrafficSystem/Pedestrian/Base_User_Agent.Base_User_Agent_C"
ROBOT_BP_PATH = "/Game/Robot_Dog/Blueprint/BP_SpotRobot.BP_SpotRobot_C"
ROBOT_NAME = "SpotDog_Follower"

HUMAN_SPAWN = (750, 200, 100)
ROBOT_SPAWN = (640, 200, 20)

# Human movement
HUMAN_SPEED = 180
HUMAN_STEP_DUR = 0.35
HUMAN_COLLISION_MOVE_EPS_CM = 6.0
HUMAN_TURN_MIN_DEG = 55.0
HUMAN_TURN_MAX_DEG = 130.0
HUMAN_TURN_COOLDOWN_S = 0.45

# Wall handling
WALL_CONTACT_MARGIN_CM = 45.0
WALL_ESCAPE_JITTER_DEG = 20.0
WALL_TURN_COOLDOWN_S = 0.6

# SpotDog follow parameters
FOLLOW_DISTANCE_CM = 100.0
FOLLOW_DISTANCE_TOL_CM = 12.0
FOLLOW_DISTANCE_KP = 1.25
FOLLOW_REAR_BLEND_GAIN = 0.8
FOLLOW_MIN_BEHIND_CM = 65.0

ROBOT_SPEED_MAX_FWD = 180.0
ROBOT_SPEED_MAX_REV = 80.0
ROBOT_MIN_MOVE_SPEED = 35.0
ROBOT_MOVE_SLICE_S = 0.18
ROBOT_STOP_PULSE_S = 0.05
ROBOT_ROTATE_SLICE_S = 0.18
ROBOT_MAX_TURN_DEG_PER_STEP = 18.0
ROBOT_HEADING_KP = 0.95
ROBOT_HEADING_DEADBAND_DEG = 2.8

ROBOT_WALL_STOP_MARGIN_CM = 70.0
ROBOT_MIN_INWARD_DOT = 0.15

# Sensor setup (camera mounted on SpotDog)
SENSOR_CAMERA_ID_PREFERRED = 1
SENSOR_CAMERA_ID = SENSOR_CAMERA_ID_PREFERRED
SENSOR_RESOLUTION = (640, 384)
SENSOR_FOV_DEG = 90.0
SENSOR_CAM_HEIGHT_OFFSET_CM = 45.0
SENSOR_CAM_FORWARD_OFFSET_CM = 22.0
SENSOR_CAM_PITCH_DEG = -5.0

# Vision backend configuration
VISION_USE_YOLO = False
VISION_YOLO_MODEL_PATH = None
VISION_HOG_SCALE = 0.65

# Far-range robust detection settings (general methods)
VISION_ENABLE_CLAHE = True
VISION_ENABLE_TILED_SEARCH = True
VISION_TILE_OVERLAP = 0.30
VISION_FAR_UPSAMPLE = 1.8
VISION_TEMPORAL_HOLD_S = 0.25

# Search behavior when visual target is lost
SEARCH_LOST_GRACE_S = 0.40
SEARCH_SPIN_PERIOD_S = 2.0
SEARCH_SPIN_CLOCKWISE = True
SEARCH_ROTATE_SLICE_S = 0.05

# Real-time monitor
ENABLE_REALTIME_MONITOR = True
MONITOR_FPS = 15.0
MONITOR_RANGE_WINDOW_S = 20.0
MONITOR_RANGE_MIN_CM = 0.0
MONITOR_RANGE_MAX_CM = 400.0
MONITOR_CLOSE_KEY_HINT = "Press q or ESC to close monitor windows."

# Simulation runtime
SIM_DURATION = 60.0
RECORDER_DT_S = 0.2

# Output
OUTPUT_CSV = Path("dev/hri_spotdog_follow/spotdog_follow_log.csv")

rng = np.random.default_rng(20260413)


# %%
# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def get_pos2d(actor_name: str) -> Tuple[float, float]:
    loc = ucv.get_location(actor_name)
    return float(loc[0]), float(loc[1])


def get_pos3d(actor_name: str) -> Tuple[float, float, float]:
    loc = ucv.get_location(actor_name)
    return float(loc[0]), float(loc[1]), float(loc[2])


def get_yaw(actor_name: str) -> float:
    ori = ucv.get_orientation(actor_name)
    return float(ori[1])


def yaw_to_unit_vec(yaw_deg: float) -> Tuple[float, float]:
    rad = math.radians(yaw_deg)
    return math.cos(rad), math.sin(rad)


def yaw_to_target(from_xy: Tuple[float, float], to_xy: Tuple[float, float]) -> float:
    dx = to_xy[0] - from_xy[0]
    dy = to_xy[1] - from_xy[1]
    return math.degrees(math.atan2(dy, dx))


def wall_inward_normal(
    pos_xy: Tuple[float, float], margin_cm: float = WALL_CONTACT_MARGIN_CM
) -> Tuple[float, float]:
    nx, ny = 0.0, 0.0
    if pos_xy[0] <= margin_cm:
        nx += 1.0
    if pos_xy[0] >= ROOM_CM - margin_cm:
        nx -= 1.0
    if pos_xy[1] <= margin_cm:
        ny += 1.0
    if pos_xy[1] >= ROOM_CM - margin_cm:
        ny -= 1.0
    return nx, ny


def is_near_wall(pos_xy: Tuple[float, float], margin_cm: float = WALL_CONTACT_MARGIN_CM) -> bool:
    nx, ny = wall_inward_normal(pos_xy, margin_cm)
    return (nx != 0.0) or (ny != 0.0)


def should_stop_for_wall(pos_xy: Tuple[float, float], yaw_deg: float) -> bool:
    nx, ny = wall_inward_normal(pos_xy, ROBOT_WALL_STOP_MARGIN_CM)
    if nx == 0.0 and ny == 0.0:
        return False

    norm = math.hypot(nx, ny)
    nx, ny = nx / norm, ny / norm
    hx, hy = yaw_to_unit_vec(yaw_deg)
    inward_dot = hx * nx + hy * ny
    return inward_dot < ROBOT_MIN_INWARD_DOT


def escape_yaw_from_wall(pos_xy: Tuple[float, float]) -> float:
    nx, ny = wall_inward_normal(pos_xy)
    if nx == 0.0 and ny == 0.0:
        return float(rng.uniform(-180.0, 180.0))

    base_yaw = math.degrees(math.atan2(ny, nx))
    jitter = float(rng.uniform(-WALL_ESCAPE_JITTER_DEG, WALL_ESCAPE_JITTER_DEG))
    return normalize_angle(base_yaw + jitter)


def choose_human_collision_turn(curr_pos: Tuple[float, float], curr_yaw: float) -> float:
    nx, ny = wall_inward_normal(curr_pos)
    if nx != 0.0 or ny != 0.0:
        wall_normal_yaw = math.degrees(math.atan2(ny, nx))
        tangent_left = normalize_angle(wall_normal_yaw + 90.0)
        tangent_right = normalize_angle(wall_normal_yaw - 90.0)
        tangent_target = tangent_left if rng.random() < 0.5 else tangent_right
        jitter = float(rng.uniform(-15.0, 15.0))
        return normalize_angle(tangent_target + jitter)

    sign = 1.0 if rng.random() < 0.5 else -1.0
    angle = float(rng.uniform(HUMAN_TURN_MIN_DEG, HUMAN_TURN_MAX_DEG))
    return normalize_angle(curr_yaw + sign * angle)


# %%
# ---------------------------------------------------------------------------
# Sensor helpers
# ---------------------------------------------------------------------------
def resolve_sensor_camera_id(preferred_id: int) -> int:
    raw = ucv.get_cameras()
    tokens = str(raw).replace(",", " ").split()
    ids: List[int] = []
    for token in tokens:
        try:
            ids.append(int(token))
        except ValueError:
            continue

    if preferred_id in ids:
        return preferred_id
    if ids:
        return ids[0]
    return preferred_id


def update_sensor_camera_pose() -> None:
    robot_pos = get_pos3d(ROBOT_NAME)
    robot_yaw = get_yaw(ROBOT_NAME)
    fx, fy = yaw_to_unit_vec(robot_yaw)

    cam_loc = (
        robot_pos[0] + fx * SENSOR_CAM_FORWARD_OFFSET_CM,
        robot_pos[1] + fy * SENSOR_CAM_FORWARD_OFFSET_CM,
        robot_pos[2] + SENSOR_CAM_HEIGHT_OFFSET_CM,
    )
    ucv.set_camera_location(SENSOR_CAMERA_ID, cam_loc)
    ucv.set_camera_rotation(SENSOR_CAMERA_ID, (SENSOR_CAM_PITCH_DEG, robot_yaw, 0.0))


def get_raw_depth_map(camera_id: int) -> Optional[np.ndarray]:
    cmd = f"vget /camera/{camera_id}/depth npy"
    try:
        with ucv.lock:
            payload = ucv.client.request(cmd)
        depth = np.load(BytesIO(payload))
        if not isinstance(depth, np.ndarray):
            return None
        if depth.ndim != 2:
            return None
        return depth
    except Exception:
        return None


def estimate_distance_cm_from_depth(depth_map: np.ndarray, bbox: Tuple[int, int, int, int]) -> Optional[float]:
    x, y, w, h = bbox
    if w <= 0 or h <= 0:
        return None

    cx = int(x + 0.5 * w)
    cy = int(y + 0.60 * h)
    rx = max(2, int(0.14 * w))
    ry = max(2, int(0.14 * h))

    x0 = max(0, cx - rx)
    x1 = min(depth_map.shape[1], cx + rx)
    y0 = max(0, cy - ry)
    y1 = min(depth_map.shape[0], cy + ry)

    roi = depth_map[y0:y1, x0:x1]
    if roi.size == 0:
        return None

    valid = roi[np.isfinite(roi)]
    valid = valid[(valid > 0.001) & (valid < 10000.0)]
    if valid.size < 10:
        return None

    depth_value = float(np.percentile(valid, 35))
    if depth_value < 20.0:
        return depth_value * 100.0
    return depth_value


def bbox_heading_error_deg(bbox: Tuple[int, int, int, int], frame_w: int) -> float:
    x, _, w, _ = bbox
    center_x = x + 0.5 * w
    norm = (center_x - 0.5 * frame_w) / (0.5 * frame_w)
    return float(norm * (SENSOR_FOV_DEG * 0.5))


def draw_sensor_overlay(
    frame_bgr: Optional[np.ndarray],
    det: Optional[Dict[str, object]],
    range_cm: float,
    backend: str,
    searching: bool,
) -> np.ndarray:
    if frame_bgr is None or frame_bgr.size == 0:
        canvas = np.zeros((SENSOR_RESOLUTION[1], SENSOR_RESOLUTION[0], 3), dtype=np.uint8)
    else:
        canvas = frame_bgr.copy()

    if det is not None:
        x, y, w, h = det["bbox"]
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 255, 0), 2)

    H, W = canvas.shape[:2]
    cx = W // 2
    cy = H // 2
    cv2.line(canvas, (cx - 10, cy), (cx + 10, cy), (255, 255, 0), 1)
    cv2.line(canvas, (cx, cy - 10), (cx, cy + 10), (255, 255, 0), 1)

    status_text = "SEARCHING" if searching else "TRACKING"
    status_color = (0, 180, 255) if searching else (0, 255, 0)

    range_text = "nan" if not np.isfinite(range_cm) else f"{range_cm:.1f} cm"
    cv2.putText(canvas, f"state: {status_text}", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, status_color, 2)
    cv2.putText(canvas, f"range: {range_text}", (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)
    cv2.putText(canvas, f"backend: {backend}", (12, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (210, 210, 210), 1)
    return canvas


def render_range_waveform(history: List[Tuple[float, float]], now_ts: float) -> np.ndarray:
    width, height = 760, 260
    canvas = np.full((height, width, 3), 245, dtype=np.uint8)

    left, right, top, bottom = 52, 16, 14, 36
    plot_w = width - left - right
    plot_h = height - top - bottom

    cv2.rectangle(canvas, (left, top), (left + plot_w, top + plot_h), (30, 30, 30), 1)
    cv2.putText(canvas, "Distance [cm]", (8, top + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (70, 70, 70), 1)
    cv2.putText(canvas, "Time [s]", (left + plot_w - 72, top + plot_h + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (70, 70, 70), 1)

    recent: List[Tuple[float, float]] = []
    for ts, val in history:
        if now_ts - ts <= MONITOR_RANGE_WINDOW_S and np.isfinite(val):
            recent.append((ts, val))

    if len(recent) < 2:
        cv2.putText(canvas, "Waiting for range samples...", (left + 10, top + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (100, 100, 100), 1)
        return canvas

    y_max = max(MONITOR_RANGE_MAX_CM, max(v for _, v in recent) * 1.05)
    y_min = min(MONITOR_RANGE_MIN_CM, min(v for _, v in recent) * 0.95)
    if y_max - y_min < 1.0:
        y_max = y_min + 1.0

    nx = 5
    ny = 5
    for i in range(nx + 1):
        x = int(left + (i / nx) * plot_w)
        cv2.line(canvas, (x, top), (x, top + plot_h), (210, 210, 210), 1)
        t_tick = -MONITOR_RANGE_WINDOW_S + (i / nx) * MONITOR_RANGE_WINDOW_S
        cv2.putText(canvas, f"{t_tick:.0f}", (x - 12, top + plot_h + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (95, 95, 95), 1)

    for j in range(ny + 1):
        y = int(top + (j / ny) * plot_h)
        cv2.line(canvas, (left, y), (left + plot_w, y), (210, 210, 210), 1)
        y_val = y_max - (j / ny) * (y_max - y_min)
        cv2.putText(canvas, f"{y_val:.0f}", (8, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (95, 95, 95), 1)

    def to_px(ts: float, val: float) -> Tuple[int, int]:
        x_norm = 1.0 - (now_ts - ts) / MONITOR_RANGE_WINDOW_S
        x = int(left + clamp(x_norm, 0.0, 1.0) * plot_w)
        y_norm = (val - y_min) / (y_max - y_min)
        y = int(top + plot_h - clamp(y_norm, 0.0, 1.0) * plot_h)
        return x, y

    points = [to_px(ts, val) for ts, val in recent]
    for i in range(1, len(points)):
        cv2.line(canvas, points[i - 1], points[i], (40, 120, 255), 2)

    latest_val = recent[-1][1]
    cv2.putText(canvas, f"latest: {latest_val:.1f} cm", (left + 10, top + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (30, 30, 30), 1)
    cv2.putText(canvas, f"window: {MONITOR_RANGE_WINDOW_S:.0f}s", (left + 220, top + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (30, 30, 30), 1)

    return canvas


# %%
# ---------------------------------------------------------------------------
# Robust person detector
# ---------------------------------------------------------------------------
class PersonDetector:
    def __init__(self, use_yolo: bool = False, yolo_model_path: Optional[str] = None, hog_scale: float = 0.65):
        self.backend = "hog"
        self.yolo_model = None
        self.hog_scale = clamp(hog_scale, 0.4, 1.0)

        self.enable_clahe = VISION_ENABLE_CLAHE
        self.enable_tiled_search = VISION_ENABLE_TILED_SEARCH
        self.tile_overlap = clamp(VISION_TILE_OVERLAP, 0.0, 0.45)
        self.far_upsample = max(1.0, VISION_FAR_UPSAMPLE)
        self.temporal_hold_s = max(0.0, VISION_TEMPORAL_HOLD_S)

        self.last_det: Optional[Dict[str, object]] = None
        self.last_det_ts = 0.0

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        if use_yolo:
            try:
                YOLO = getattr(importlib.import_module("ultralytics"), "YOLO")
                self.yolo_model = YOLO(yolo_model_path or "yolov8n.pt")
                self.backend = "yolo"
                print("[Vision] YOLO backend enabled.")
            except Exception as exc:
                print(f"[Vision] YOLO unavailable ({exc}). Falling back to HOG.")
                self.backend = "hog"

    def detect(self, frame_bgr: np.ndarray) -> Optional[Dict[str, object]]:
        if frame_bgr is None or frame_bgr.size == 0:
            return None

        processed = self._preprocess_for_far(frame_bgr)
        candidates: List[Dict[str, object]] = []

        if self.backend == "yolo" and self.yolo_model is not None:
            candidates.extend(self._detect_yolo_candidates(processed, 0, 0, 1.0, "yolo_full"))
            if self.enable_tiled_search:
                candidates.extend(self._detect_yolo_tiled(processed))

        candidates.extend(self._detect_hog_candidates(processed, 0, 0, 1.0, "hog_full"))

        if self.far_upsample > 1.01:
            up = cv2.resize(
                processed,
                dsize=None,
                fx=self.far_upsample,
                fy=self.far_upsample,
                interpolation=cv2.INTER_CUBIC,
            )
            candidates.extend(self._detect_hog_candidates(up, 0, 0, 1.0 / self.far_upsample, "hog_up"))

        if self.enable_tiled_search:
            candidates.extend(self._detect_hog_tiled(processed))

        det = self._select_best_detection(candidates, frame_bgr.shape[1], frame_bgr.shape[0])
        if det is not None:
            self.last_det = det
            self.last_det_ts = time.time()
            return det

        if self.last_det is not None and (time.time() - self.last_det_ts) <= self.temporal_hold_s:
            hold = dict(self.last_det)
            hold["confidence"] = float(hold.get("confidence", 0.0)) * 0.88
            hold["backend"] = str(hold.get("backend", "detector")) + "+temporal_hold"
            return hold

        return None

    def _preprocess_for_far(self, frame_bgr: np.ndarray) -> np.ndarray:
        if not self.enable_clahe:
            return frame_bgr

        lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l_channel = clahe.apply(l_channel)
        merged = cv2.merge((l_channel, a_channel, b_channel))
        enhanced = cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)

        sharpen_kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.float32)
        return cv2.filter2D(enhanced, -1, sharpen_kernel)

    def _iter_tiles(self, width: int, height: int) -> List[Tuple[int, int, int, int]]:
        tile_w = max(120, int(width * 0.58))
        tile_h = max(120, int(height * 0.65))

        step_x = max(40, int(tile_w * (1.0 - self.tile_overlap)))
        step_y = max(40, int(tile_h * (1.0 - self.tile_overlap)))

        x_starts = [0, max(0, (width - tile_w) // 2), max(0, width - tile_w)]
        y_starts = [0, max(0, (height - tile_h) // 2), max(0, height - tile_h)]

        x = 0
        while x + tile_w < width:
            x_starts.append(x)
            x += step_x
        x_starts.append(max(0, width - tile_w))

        y = 0
        while y + tile_h < height:
            y_starts.append(y)
            y += step_y
        y_starts.append(max(0, height - tile_h))

        x_starts = sorted(set(int(clamp(v, 0, max(0, width - tile_w))) for v in x_starts))
        y_starts = sorted(set(int(clamp(v, 0, max(0, height - tile_h))) for v in y_starts))

        tiles: List[Tuple[int, int, int, int]] = []
        for y0 in y_starts:
            for x0 in x_starts:
                x1 = min(width, x0 + tile_w)
                y1 = min(height, y0 + tile_h)
                if (x1 - x0) >= 100 and (y1 - y0) >= 100:
                    tiles.append((x0, y0, x1, y1))

        return tiles[:16]

    def _detect_hog_candidates(
        self,
        frame_bgr: np.ndarray,
        offset_x: int,
        offset_y: int,
        scale_to_original: float,
        tag: str,
    ) -> List[Dict[str, object]]:
        if self.hog_scale < 0.999:
            small = cv2.resize(frame_bgr, dsize=None, fx=self.hog_scale, fy=self.hog_scale)
            restore_scale = 1.0 / self.hog_scale
        else:
            small = frame_bgr
            restore_scale = 1.0

        rects, weights = self.hog.detectMultiScale(
            small,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
        )

        candidates: List[Dict[str, object]] = []
        for idx, (x, y, w, h) in enumerate(rects):
            conf = float(weights[idx]) if len(weights) > idx else 0.0
            x_img = x * restore_scale
            y_img = y * restore_scale
            w_img = w * restore_scale
            h_img = h * restore_scale

            x_ori = int(offset_x + x_img * scale_to_original)
            y_ori = int(offset_y + y_img * scale_to_original)
            w_ori = int(max(1.0, w_img * scale_to_original))
            h_ori = int(max(1.0, h_img * scale_to_original))

            candidates.append(
                {
                    "bbox": (x_ori, y_ori, w_ori, h_ori),
                    "confidence": conf,
                    "backend": tag,
                }
            )

        return candidates

    def _detect_hog_tiled(self, frame_bgr: np.ndarray) -> List[Dict[str, object]]:
        H, W = frame_bgr.shape[:2]
        candidates: List[Dict[str, object]] = []

        for x0, y0, x1, y1 in self._iter_tiles(W, H):
            tile = frame_bgr[y0:y1, x0:x1]
            candidates.extend(self._detect_hog_candidates(tile, x0, y0, 1.0, "hog_tile"))

            if self.far_upsample > 1.01:
                tile_up = cv2.resize(
                    tile,
                    dsize=None,
                    fx=self.far_upsample,
                    fy=self.far_upsample,
                    interpolation=cv2.INTER_CUBIC,
                )
                candidates.extend(
                    self._detect_hog_candidates(
                        tile_up,
                        x0,
                        y0,
                        1.0 / self.far_upsample,
                        "hog_tile_up",
                    )
                )

        return candidates

    def _detect_yolo_candidates(
        self,
        frame_bgr: np.ndarray,
        offset_x: int,
        offset_y: int,
        scale_to_original: float,
        tag: str,
    ) -> List[Dict[str, object]]:
        if self.yolo_model is None:
            return []

        try:
            results = self.yolo_model(frame_bgr, verbose=False, classes=[0], conf=0.18)
            if not results:
                return []

            boxes = results[0].boxes
            if boxes is None or len(boxes) == 0:
                return []

            xyxy_all = boxes.xyxy.cpu().numpy()
            conf_all = boxes.conf.cpu().numpy()

            candidates: List[Dict[str, object]] = []
            for i in range(len(xyxy_all)):
                x1, y1, x2, y2 = [float(v) for v in xyxy_all[i].tolist()]
                x = min(x1, x2)
                y = min(y1, y2)
                w = max(1.0, abs(x2 - x1))
                h = max(1.0, abs(y2 - y1))

                x_ori = int(offset_x + x * scale_to_original)
                y_ori = int(offset_y + y * scale_to_original)
                w_ori = int(max(1.0, w * scale_to_original))
                h_ori = int(max(1.0, h * scale_to_original))

                candidates.append(
                    {
                        "bbox": (x_ori, y_ori, w_ori, h_ori),
                        "confidence": float(conf_all[i]),
                        "backend": tag,
                    }
                )

            return candidates
        except Exception:
            return []

    def _detect_yolo_tiled(self, frame_bgr: np.ndarray) -> List[Dict[str, object]]:
        if self.yolo_model is None:
            return []

        H, W = frame_bgr.shape[:2]
        candidates: List[Dict[str, object]] = []
        for x0, y0, x1, y1 in self._iter_tiles(W, H):
            tile = frame_bgr[y0:y1, x0:x1]
            candidates.extend(self._detect_yolo_candidates(tile, x0, y0, 1.0, "yolo_tile"))

        return candidates

    def _bbox_iou(self, a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
        ax, ay, aw, ah = a
        bx, by, bw, bh = b

        ax2, ay2 = ax + aw, ay + ah
        bx2, by2 = bx + bw, by + bh

        ix1 = max(ax, bx)
        iy1 = max(ay, by)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)

        iw = max(0, ix2 - ix1)
        ih = max(0, iy2 - iy1)
        inter = iw * ih
        if inter <= 0:
            return 0.0

        union = aw * ah + bw * bh - inter
        if union <= 0:
            return 0.0

        return float(inter / union)

    def _select_best_detection(
        self,
        candidates: List[Dict[str, object]],
        frame_w: int,
        frame_h: int,
    ) -> Optional[Dict[str, object]]:
        best: Optional[Dict[str, object]] = None
        best_score = -1e9

        for cand in candidates:
            x, y, w, h = cand["bbox"]
            x = int(clamp(x, 0, frame_w - 1))
            y = int(clamp(y, 0, frame_h - 1))
            w = int(clamp(w, 1, frame_w - x))
            h = int(clamp(h, 1, frame_h - y))

            area = float(w * h)
            if area < 120.0:
                continue

            conf = float(cand.get("confidence", 0.0))
            cx = x + 0.5 * w
            center_bias = 1.0 - min(1.0, abs(cx - 0.5 * frame_w) / (0.5 * frame_w))

            ratio = w / max(1.0, float(h))
            aspect_score = 1.0 - min(1.0, abs(ratio - 0.42) / 0.42)

            far_bonus = 0.22 if area < 9000.0 else 0.0

            temporal_bonus = 0.0
            if self.last_det is not None:
                temporal_bonus = 0.25 * self._bbox_iou((x, y, w, h), self.last_det["bbox"])

            score = conf + 0.35 * center_bias + 0.20 * aspect_score + far_bonus + temporal_bonus
            if score > best_score:
                best_score = score
                best = {
                    "bbox": (x, y, w, h),
                    "confidence": conf,
                    "backend": str(cand.get("backend", "detector")),
                }

        return best


# %%
# ---------------------------------------------------------------------------
# World spawning
# ---------------------------------------------------------------------------
def spawn_walls() -> None:
    R = ROOM_CM
    T = WALL_THICK_CM
    H = WALL_H_CM
    S = WALL_ASSET_SIZE_CM
    z_center = WALL_Z_CM

    for obj in ucv.get_objects():
        if str(obj).startswith("WALL_"):
            ucv.destroy(str(obj))

    def build_edge_centers(edge_len_cm: float, seg_len_cm: float, overlap_cm: float):
        seg_len_cm = min(seg_len_cm, edge_len_cm)
        overlap_cm = min(overlap_cm, seg_len_cm - 1.0)

        if edge_len_cm <= seg_len_cm:
            return [edge_len_cm / 2], 0.0

        advance_target = max(1.0, seg_len_cm - overlap_cm)
        n_segments = int(math.ceil((edge_len_cm - seg_len_cm) / advance_target)) + 1
        step = (edge_len_cm - seg_len_cm) / (n_segments - 1)
        centers = [seg_len_cm / 2 + i * step for i in range(n_segments)]
        actual_overlap = seg_len_cm - step
        return centers, actual_overlap

    seg_len = float(WALL_SEGMENT_LEN_CM)
    ns_centers, _ = build_edge_centers(float(R), seg_len, float(WALL_SEGMENT_OVERLAP_NS_CM))
    ew_centers, _ = build_edge_centers(float(R), seg_len, float(WALL_SEGMENT_OVERLAP_EW_CM))

    walls = []
    for i, c in enumerate(ns_centers):
        walls.append((f"WALL_South_{i:02d}", (c, -T / 2, z_center), (T / S, seg_len / S, H / S), (0, 90, 0)))
        walls.append((f"WALL_North_{i:02d}", (c, R + T / 2, z_center), (T / S, seg_len / S, H / S), (0, 90, 0)))

    for i, c in enumerate(ew_centers):
        walls.append((f"WALL_West_{i:02d}", (-T / 2, c, z_center), (T / S, seg_len / S, H / S), (0, 0, 0)))
        walls.append((f"WALL_East_{i:02d}", (R + T / 2, c, z_center), (T / S, seg_len / S, H / S), (0, 0, 0)))

    corner_scale = (T / S, T / S, H / S)
    walls.extend(
        [
            ("WALL_Corner_SW", (-T / 2, -T / 2, z_center), corner_scale, (0, 0, 0)),
            ("WALL_Corner_SE", (R + T / 2, -T / 2, z_center), corner_scale, (0, 0, 0)),
            ("WALL_Corner_NW", (-T / 2, R + T / 2, z_center), corner_scale, (0, 0, 0)),
            ("WALL_Corner_NE", (R + T / 2, R + T / 2, z_center), corner_scale, (0, 0, 0)),
        ]
    )

    for name, loc, scale, orient in walls:
        ucv.spawn_bp_asset(WALL_BP_PATH, name)
        ucv.set_location(loc, name)
        ucv.set_orientation(orient, name)
        ucv.set_scale(scale, name)
        ucv.set_collision(name, True)
        ucv.set_movable(name, False)


def spawn_human() -> Humanoid:
    h = Humanoid(position=Vector(HUMAN_SPAWN[0], HUMAN_SPAWN[1]), direction=Vector(1, 0))
    communicator.spawn_agent(
        agent=h,
        name=None,
        position=HUMAN_SPAWN,
        model_path=HUMAN_BP_PATH,
        type="humanoid",
    )
    communicator.humanoid_set_speed(h.id, HUMAN_SPEED)
    return h


def spawn_robot(name: str) -> str:
    ucv.spawn_bp_asset(ROBOT_BP_PATH, name)
    ucv.set_location(ROBOT_SPAWN, name)
    ucv.set_orientation((0, 0, 0), name)
    ucv.set_collision(name, True)
    ucv.set_movable(name, True)
    ucv.enable_controller(name, True)
    return name


# %%
print("=== Spawning world for SpotDog follow project ===")
spawn_walls()
human = spawn_human()
HUMAN_NAME = communicator.get_humanoid_name(human.id)
spawn_robot(ROBOT_NAME)
time.sleep(2.0)
print(f"Spawned human={HUMAN_NAME}, robot={ROBOT_NAME}")


# %%
# ---------------------------------------------------------------------------
# Sensor initialization
# ---------------------------------------------------------------------------
SENSOR_CAMERA_ID = resolve_sensor_camera_id(SENSOR_CAMERA_ID_PREFERRED)
print(f"Using camera_id={SENSOR_CAMERA_ID} for SpotDog sensing.")

try:
    ucv.set_camera_resolution(SENSOR_CAMERA_ID, SENSOR_RESOLUTION)
    ucv.set_camera_fov(SENSOR_CAMERA_ID, SENSOR_FOV_DEG)
except Exception as exc:
    print(f"[Sensor] Camera parameter setup warning: {exc}")

vision_detector = PersonDetector(
    use_yolo=VISION_USE_YOLO,
    yolo_model_path=VISION_YOLO_MODEL_PATH,
    hog_scale=VISION_HOG_SCALE,
)


# %%
# ---------------------------------------------------------------------------
# Threaded control loops
# ---------------------------------------------------------------------------
sim_data: List[Dict[str, float]] = []
stop_event = threading.Event()
monitor_stop_event = threading.Event()
simulation_done_event = threading.Event()

MONITOR_CAMERA_WINDOW_NAME = "SpotDog Camera"
MONITOR_RANGE_WINDOW_NAME = "Range Sensor Timeline"

latest_sensor_state: Dict[str, float] = {
    "detected": 0.0,
    "range_cm": float("nan"),
    "yaw_delta_deg": float("nan"),
    "confidence": float("nan"),
    "backend": "none",
}

sensor_lock = threading.Lock()
latest_camera_frame: Optional[np.ndarray] = None
sensor_range_history: deque = deque(maxlen=6000)


def set_latest_sensor_state(state: Dict[str, object]) -> None:
    range_cm = float(state.get("range_cm", float("nan")))
    with sensor_lock:
        latest_sensor_state["detected"] = 1.0 if state.get("detected", False) else 0.0
        latest_sensor_state["range_cm"] = range_cm
        latest_sensor_state["yaw_delta_deg"] = float(state.get("yaw_delta_deg", float("nan")))
        latest_sensor_state["confidence"] = float(state.get("confidence", float("nan")))
        latest_sensor_state["backend"] = str(state.get("backend", "none"))
        if np.isfinite(range_cm):
            sensor_range_history.append((time.time(), range_cm))


def set_latest_camera_frame(frame: Optional[np.ndarray]) -> None:
    if frame is None:
        return
    with sensor_lock:
        global latest_camera_frame
        latest_camera_frame = frame.copy()


def get_latest_camera_frame() -> Optional[np.ndarray]:
    with sensor_lock:
        if latest_camera_frame is None:
            return None
        return latest_camera_frame.copy()


def get_sensor_range_history_snapshot() -> List[Tuple[float, float]]:
    with sensor_lock:
        return list(sensor_range_history)


def get_latest_sensor_state_snapshot() -> Dict[str, object]:
    with sensor_lock:
        return {
            "detected": latest_sensor_state["detected"],
            "range_cm": latest_sensor_state["range_cm"],
            "yaw_delta_deg": latest_sensor_state["yaw_delta_deg"],
            "confidence": latest_sensor_state["confidence"],
            "backend": latest_sensor_state["backend"],
        }


def human_control_loop() -> None:
    last_turn_ts = 0.0

    while not stop_event.is_set():
        prev_pos = get_pos2d(HUMAN_NAME)
        communicator.humanoid_step_forward(human.id, HUMAN_STEP_DUR)

        if stop_event.is_set():
            break

        curr_pos = get_pos2d(HUMAN_NAME)
        moved_cm = math.hypot(curr_pos[0] - prev_pos[0], curr_pos[1] - prev_pos[1])
        near_wall = is_near_wall(curr_pos)
        now_ts = time.time()

        if (moved_cm < HUMAN_COLLISION_MOVE_EPS_CM or near_wall) and (now_ts - last_turn_ts >= HUMAN_TURN_COOLDOWN_S):
            current_yaw = get_yaw(HUMAN_NAME)
            target_yaw = choose_human_collision_turn(curr_pos, current_yaw)
            angle_diff = normalize_angle(target_yaw - current_yaw)
            if abs(angle_diff) < 8.0:
                angle_diff = 10.0 if rng.random() < 0.5 else -10.0

            direction = "left" if angle_diff > 0 else "right"
            communicator.humanoid_rotate(human.id, abs(angle_diff), direction)
            last_turn_ts = now_ts


def build_follow_sensor_state() -> Dict[str, object]:
    robot_pos = get_pos2d(ROBOT_NAME)
    robot_yaw = get_yaw(ROBOT_NAME)
    human_pos = get_pos2d(HUMAN_NAME)

    geom_yaw_delta = normalize_angle(yaw_to_target(robot_pos, human_pos) - robot_yaw)
    geom_range_cm = math.hypot(human_pos[0] - robot_pos[0], human_pos[1] - robot_pos[1])

    sensor_state: Dict[str, object] = {
        "detected": False,
        "yaw_delta_deg": geom_yaw_delta,
        "range_cm": geom_range_cm,
        "confidence": 0.0,
        "backend": "geometry",
    }

    try:
        update_sensor_camera_pose()
        rgb = communicator.get_camera_observation(SENSOR_CAMERA_ID, "lit", mode="direct")
        det = vision_detector.detect(rgb)

        if det is None:
            overlay = draw_sensor_overlay(
                rgb,
                None,
                float(sensor_state["range_cm"]),
                str(sensor_state["backend"]),
                searching=True,
            )
            set_latest_camera_frame(overlay)
            return sensor_state

        bbox = det["bbox"]
        heading_error = bbox_heading_error_deg(bbox, rgb.shape[1])
        yaw_delta_from_vision = -heading_error

        depth_map = get_raw_depth_map(SENSOR_CAMERA_ID)
        depth_range_cm = None
        if depth_map is not None:
            depth_range_cm = estimate_distance_cm_from_depth(depth_map, bbox)

        sensor_state["detected"] = True
        sensor_state["yaw_delta_deg"] = yaw_delta_from_vision
        sensor_state["confidence"] = float(det["confidence"])
        sensor_state["backend"] = str(det["backend"])

        if depth_range_cm is not None:
            sensor_state["range_cm"] = depth_range_cm
            sensor_state["backend"] = sensor_state["backend"] + "+depth"

        overlay = draw_sensor_overlay(
            rgb,
            det,
            float(sensor_state["range_cm"]),
            str(sensor_state["backend"]),
            searching=False,
        )
        set_latest_camera_frame(overlay)

        return sensor_state
    except Exception as exc:
        sensor_state["backend"] = str(sensor_state.get("backend", "geometry")) + "+sensor_error"
        fallback = draw_sensor_overlay(
            None,
            None,
            float(sensor_state["range_cm"]),
            str(sensor_state["backend"]),
            searching=True,
        )
        cv2.putText(
            fallback,
            f"sensor exception: {str(exc)[:52]}",
            (10, 104),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 120, 255),
            1,
        )
        set_latest_camera_frame(fallback)
        return sensor_state


def monitor_loop() -> None:
    camera_win = MONITOR_CAMERA_WINDOW_NAME
    range_win = MONITOR_RANGE_WINDOW_NAME
    dt = 1.0 / max(1.0, MONITOR_FPS)

    try:
        while not monitor_stop_event.is_set():
            frame = get_latest_camera_frame()
            if frame is not None:
                if simulation_done_event.is_set():
                    cv2.putText(
                        frame,
                        MONITOR_CLOSE_KEY_HINT,
                        (10, max(20, frame.shape[0] - 12)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.52,
                        (0, 220, 255),
                        2,
                    )
                cv2.imshow(camera_win, frame)

            waveform = render_range_waveform(get_sensor_range_history_snapshot(), time.time())
            if simulation_done_event.is_set():
                cv2.putText(
                    waveform,
                    MONITOR_CLOSE_KEY_HINT,
                    (10, waveform.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (0, 220, 255),
                    1,
                )
            cv2.imshow(range_win, waveform)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                monitor_stop_event.set()
                break

            time.sleep(dt)
    except Exception as exc:
        print(f"[Monitor] Realtime monitor disabled: {exc}")
        monitor_stop_event.set()
    finally:
        try:
            cv2.destroyWindow(camera_win)
        except Exception:
            pass
        try:
            cv2.destroyWindow(range_win)
        except Exception:
            pass
        monitor_stop_event.set()


def prepare_monitor_windows() -> None:
    if not ENABLE_REALTIME_MONITOR:
        return

    try:
        frame = np.zeros((SENSOR_RESOLUTION[1], SENSOR_RESOLUTION[0], 3), dtype=np.uint8)
        cv2.putText(frame, "SpotDog Camera Monitor", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2)
        cv2.putText(frame, "Waiting for simulation start...", (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (160, 255, 160), 1)
        cv2.putText(frame, MONITOR_CLOSE_KEY_HINT, (12, frame.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 220, 255), 1)
        set_latest_camera_frame(frame)

        wave = render_range_waveform([], time.time())
        cv2.putText(wave, "Waiting for simulation start...", (58, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (110, 110, 110), 1)
        cv2.putText(wave, MONITOR_CLOSE_KEY_HINT, (10, wave.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 220, 255), 1)

        cv2.imshow(MONITOR_CAMERA_WINDOW_NAME, frame)
        cv2.imshow(MONITOR_RANGE_WINDOW_NAME, wave)
        cv2.waitKey(1)
    except Exception as exc:
        print(f"[Monitor] Failed to pre-open windows: {exc}")


def reset_runtime_state() -> None:
    stop_event.clear()
    monitor_stop_event.clear()
    simulation_done_event.clear()

    with sensor_lock:
        latest_sensor_state["detected"] = 0.0
        latest_sensor_state["range_cm"] = float("nan")
        latest_sensor_state["yaw_delta_deg"] = float("nan")
        latest_sensor_state["confidence"] = float("nan")
        latest_sensor_state["backend"] = "none"
        sensor_range_history.clear()

        global latest_camera_frame
        latest_camera_frame = None


def agv_follow_loop() -> None:
    last_wall_turn_ts = 0.0
    last_seen_ts = time.time()
    search_spin_angle_deg = 360.0 * SEARCH_ROTATE_SLICE_S / SEARCH_SPIN_PERIOD_S
    search_spin_clockwise = 1 if SEARCH_SPIN_CLOCKWISE else -1

    while not stop_event.is_set():
        try:
            robot_pos = get_pos2d(ROBOT_NAME)
            robot_yaw = get_yaw(ROBOT_NAME)

            sensor_state = build_follow_sensor_state()
            now_ts = time.time()

            if bool(sensor_state["detected"]):
                last_seen_ts = now_ts
            elif now_ts - last_seen_ts >= SEARCH_LOST_GRACE_S:
                sensor_state["backend"] = str(sensor_state["backend"]) + "+search_spin_cont"
                set_latest_sensor_state(sensor_state)
                ucv.dog_rotate(ROBOT_NAME, [SEARCH_ROTATE_SLICE_S, search_spin_angle_deg, search_spin_clockwise])
                continue

            set_latest_sensor_state(sensor_state)

            if should_stop_for_wall(robot_pos, robot_yaw):
                ucv.dog_move(ROBOT_NAME, [0.0, ROBOT_STOP_PULSE_S, 0])
                target_yaw = escape_yaw_from_wall(robot_pos)
                angle_diff = normalize_angle(target_yaw - robot_yaw)
                angle_diff = clamp(angle_diff, -ROBOT_MAX_TURN_DEG_PER_STEP, ROBOT_MAX_TURN_DEG_PER_STEP)
                if abs(angle_diff) < 6.0:
                    angle_diff = 8.0 if rng.random() < 0.5 else -8.0
                clockwise = 1 if angle_diff < 0.0 else -1
                ucv.dog_rotate(ROBOT_NAME, [ROBOT_ROTATE_SLICE_S, abs(angle_diff), clockwise])
                last_wall_turn_ts = time.time()
                continue

            human_pos = get_pos2d(HUMAN_NAME)
            human_yaw = get_yaw(HUMAN_NAME)
            hx, hy = yaw_to_unit_vec(human_yaw)

            rear_target = (
                human_pos[0] - hx * FOLLOW_DISTANCE_CM,
                human_pos[1] - hy * FOLLOW_DISTANCE_CM,
            )
            rear_yaw_delta = normalize_angle(yaw_to_target(robot_pos, rear_target) - robot_yaw)
            rel_x = robot_pos[0] - human_pos[0]
            rel_y = robot_pos[1] - human_pos[1]
            behind_component_cm = rel_x * hx + rel_y * hy

            yaw_delta_cmd = float(sensor_state["yaw_delta_deg"])
            if behind_component_cm > -FOLLOW_MIN_BEHIND_CM:
                blend = clamp(
                    FOLLOW_REAR_BLEND_GAIN * (behind_component_cm + FOLLOW_MIN_BEHIND_CM) / FOLLOW_DISTANCE_CM,
                    0.0,
                    1.0,
                )
                yaw_delta_cmd = (1.0 - blend) * yaw_delta_cmd + blend * rear_yaw_delta

            yaw_turn = clamp(
                ROBOT_HEADING_KP * yaw_delta_cmd,
                -ROBOT_MAX_TURN_DEG_PER_STEP,
                ROBOT_MAX_TURN_DEG_PER_STEP,
            )

            if abs(yaw_turn) > ROBOT_HEADING_DEADBAND_DEG:
                clockwise = 1 if yaw_turn < 0.0 else -1
                ucv.dog_rotate(ROBOT_NAME, [ROBOT_ROTATE_SLICE_S, abs(yaw_turn), clockwise])
                robot_pos = get_pos2d(ROBOT_NAME)
                robot_yaw = get_yaw(ROBOT_NAME)

            range_cm = float(sensor_state["range_cm"])
            range_error_cm = range_cm - FOLLOW_DISTANCE_CM

            speed_cmd = FOLLOW_DISTANCE_KP * range_error_cm
            speed_cmd = clamp(speed_cmd, -ROBOT_SPEED_MAX_REV, ROBOT_SPEED_MAX_FWD)

            if abs(range_error_cm) <= FOLLOW_DISTANCE_TOL_CM:
                speed_cmd = 0.0
            elif abs(speed_cmd) < ROBOT_MIN_MOVE_SPEED:
                speed_cmd = math.copysign(ROBOT_MIN_MOVE_SPEED, speed_cmd)

            if is_near_wall(robot_pos, ROBOT_WALL_STOP_MARGIN_CM):
                now_ts = time.time()
                if now_ts - last_wall_turn_ts >= WALL_TURN_COOLDOWN_S:
                    target_yaw = escape_yaw_from_wall(robot_pos)
                    angle_diff = normalize_angle(target_yaw - robot_yaw)
                    angle_diff = clamp(angle_diff, -ROBOT_MAX_TURN_DEG_PER_STEP, ROBOT_MAX_TURN_DEG_PER_STEP)
                    clockwise = 1 if angle_diff < 0.0 else -1
                    ucv.dog_rotate(ROBOT_NAME, [ROBOT_ROTATE_SLICE_S, abs(angle_diff), clockwise])
                    last_wall_turn_ts = now_ts

            ucv.dog_move(ROBOT_NAME, [speed_cmd, ROBOT_MOVE_SLICE_S, 0])

        except Exception as exc:
            print(f"[AGV] loop exception: {exc}")
            error_frame = draw_sensor_overlay(
                None,
                None,
                float("nan"),
                "agv_loop_exception",
                searching=True,
            )
            cv2.putText(
                error_frame,
                f"AGV loop exception: {str(exc)[:52]}",
                (10, 104),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 120, 255),
                1,
            )
            set_latest_camera_frame(error_frame)
            time.sleep(0.10)


def recorder_loop() -> None:
    t_start = time.time()

    while not stop_event.is_set():
        t = time.time() - t_start
        human_pos = get_pos2d(HUMAN_NAME)
        robot_pos = get_pos2d(ROBOT_NAME)
        dist_cm = math.hypot(human_pos[0] - robot_pos[0], human_pos[1] - robot_pos[1])

        sensor_snapshot = get_latest_sensor_state_snapshot()

        sim_data.append(
            {
                "t": t,
                "human_x": human_pos[0],
                "human_y": human_pos[1],
                "robot_x": robot_pos[0],
                "robot_y": robot_pos[1],
                "human_robot_distance_cm": dist_cm,
                "follow_error_cm": dist_cm - FOLLOW_DISTANCE_CM,
                "sensor_detected": sensor_snapshot["detected"],
                "sensor_range_cm": sensor_snapshot["range_cm"],
                "sensor_yaw_delta_deg": sensor_snapshot["yaw_delta_deg"],
                "sensor_confidence": sensor_snapshot["confidence"],
                "sensor_backend": sensor_snapshot["backend"],
            }
        )

        time.sleep(RECORDER_DT_S)


# %%
# ---------------------------------------------------------------------------
# Run simulation
# ---------------------------------------------------------------------------
print(f"=== Starting SpotDog follow simulation for {SIM_DURATION:.0f}s ===")

reset_runtime_state()

thread_human = threading.Thread(target=human_control_loop, daemon=True)
thread_robot = threading.Thread(target=agv_follow_loop, daemon=True)
thread_rec = threading.Thread(target=recorder_loop, daemon=True)
thread_monitor = None

if ENABLE_REALTIME_MONITOR:
    prepare_monitor_windows()
    thread_monitor = threading.Thread(target=monitor_loop, daemon=True)
    thread_monitor.start()

thread_human.start()
thread_robot.start()
thread_rec.start()

time.sleep(SIM_DURATION)
stop_event.set()
simulation_done_event.set()

for t in [thread_human, thread_robot, thread_rec]:
    t.join(timeout=5.0)

print(f"=== Simulation finished. samples={len(sim_data)} ===")

if thread_monitor is not None and thread_monitor.is_alive():
    print("Simulation ended. Monitor windows remain open.")
    print(MONITOR_CLOSE_KEY_HINT)
    while not monitor_stop_event.is_set():
        time.sleep(0.05)
    thread_monitor.join(timeout=2.0)


# %%
# ---------------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------------
try:
    communicator.humanoid_stop(human.id)
except Exception:
    pass

communicator.disconnect()
print("Disconnected from UE.")


# %%
# ---------------------------------------------------------------------------
# Summary and export
# ---------------------------------------------------------------------------
df = pd.DataFrame(sim_data)

if df.empty:
    print("No data recorded.")
else:
    df["human_x_m"] = df["human_x"] / 100.0
    df["human_y_m"] = df["human_y"] / 100.0
    df["robot_x_m"] = df["robot_x"] / 100.0
    df["robot_y_m"] = df["robot_y"] / 100.0
    df["distance_m"] = df["human_robot_distance_cm"] / 100.0

    within_tol = np.mean(np.abs(df["follow_error_cm"]) <= FOLLOW_DISTANCE_TOL_CM) * 100.0
    sensor_detect_rate = np.mean(df["sensor_detected"] > 0.5) * 100.0

    metrics = {
        "duration_s": float(df["t"].max()),
        "samples": int(len(df)),
        "mean_distance_m": float(df["distance_m"].mean()),
        "std_distance_m": float(df["distance_m"].std(ddof=0)),
        "min_distance_m": float(df["distance_m"].min()),
        "max_distance_m": float(df["distance_m"].max()),
        "mean_abs_follow_error_cm": float(np.mean(np.abs(df["follow_error_cm"]))),
        "within_tolerance_ratio_percent": float(within_tol),
        "sensor_detection_ratio_percent": float(sensor_detect_rate),
    }

    summary_df = pd.DataFrame(list(metrics.items()), columns=["metric", "value"])
    print(summary_df)

    OUTPUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(OUTPUT_CSV, index=False)
    print(f"Saved log CSV to: {OUTPUT_CSV}")
    print(df.tail(10))

# %% [markdown]
# ## Optional YOLO backend
#
# To use YOLO for camera detection:
# 1. Install `ultralytics` in your python environment.
# 2. Set `VISION_USE_YOLO = True`.
# 3. Optionally set `VISION_YOLO_MODEL_PATH` to a local weight file.
#
# Default still works without extra dependencies (HOG + far-range robust strategy).
