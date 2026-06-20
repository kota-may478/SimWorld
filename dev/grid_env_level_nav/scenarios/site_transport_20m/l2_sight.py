#!/usr/bin/env python3
"""AI Perception Sight bridge: visible actors → L2 with static memory / dynamic eviction."""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

import level_coords as lc  # noqa: E402
from costmap_layers import LayeredCostmap  # noqa: E402
from depth_object_perception import ObjectEstimate  # noqa: E402
from grid_env_10k_pie_patrol import dist2d, get_pos2d, get_yaw  # noqa: E402
from l2_fusion import (  # noqa: E402
    L2_PROP_RADIUS_CM,
    estimate_world_xy_from_detection,
    l2_cells_for_world_disk,
)
from l2_geom import GeomPerceptionConfig, _bearing_deg_robot_frame, _in_fov_cone, _prop_world_xy  # noqa: E402
from perception_layer import EgocentricPerceptionConfig, apply_l2_obstacle_cells  # noqa: E402
from prop_placement import PlacementRegistry  # noqa: E402
from robot_sensor import SENSOR_CAM_FORWARD_OFFSET_CM, SENSOR_FOV_DEG  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

WorldXY = Tuple[float, float]
GridCell = Tuple[int, int]

VBP_SIGHT_COMMANDS = (
    "GetVisibleSightTargetsJson",
    "GetSightPerceptionJson",
)

HUMAN_PROP_TYPE_ID = "human_worker"
DYNAMIC_PROP_TYPE_IDS = frozenset({HUMAN_PROP_TYPE_ID, "pedestrian", "robot_agent"})


@dataclass(frozen=True)
class SightConfig:
    fov_deg: float = SENSOR_FOV_DEG
    max_range_cm: float = 650.0
    sensor_forward_cm: float = SENSOR_CAM_FORWARD_OFFSET_CM
    prop_radius_cm: float = L2_PROP_RADIUS_CM


@dataclass(frozen=True)
class VisibleTarget:
    actor_name: str
    prop_type_id: str
    slot_id: str
    is_dynamic: bool


@dataclass
class SightMemory:
    """Static props: last seen world XY persists. Dynamic: only while visible."""

    static_last_seen_xy: Dict[str, WorldXY] = field(default_factory=dict)
    last_visible_dynamic: Set[str] = field(default_factory=set)


@dataclass
class L2SlotCellTracker:
    slot_to_cells: Dict[str, Set[GridCell]] = field(default_factory=dict)

    def cells_for(self, slot_id: str) -> Set[GridCell]:
        return set(self.slot_to_cells.get(slot_id, set()))

    def set_cells(self, slot_id: str, cells: Set[GridCell]) -> None:
        self.slot_to_cells[slot_id] = set(cells)

    def pop_cells(self, slot_id: str) -> Set[GridCell]:
        return set(self.slot_to_cells.pop(slot_id, set()))


@dataclass
class SightUpdateResult:
    detections: List[ObjectEstimate]
    cells_added: int
    cells_removed: int
    visible_actor_names: Tuple[str, ...]
    backend: str

    @property
    def l2_changed(self) -> bool:
        return self.cells_added > 0 or self.cells_removed > 0


def build_actor_maps(
    placement_reg: PlacementRegistry,
    *,
    humanoid_actor_name: str,
    material_actor_name: str,
    extra_dynamic_actors: Optional[Sequence[str]] = None,
) -> Tuple[Dict[str, str], Dict[str, str], Set[str]]:
    """actor/slot name → prop_type_id; slot_id → prop_type_id; dynamic slot ids."""
    actor_to_type: Dict[str, str] = {}
    slot_to_type: Dict[str, str] = {}
    dynamic_slots: Set[str] = set()

    for prop in placement_reg.props:
        slot_to_type[prop.slot_id] = prop.prop_type_id
        actor_to_type[prop.slot_id] = prop.prop_type_id

    actor_to_type[material_actor_name] = slot_to_type.get(material_actor_name, "shipping_crate")
    actor_to_type[humanoid_actor_name] = HUMAN_PROP_TYPE_ID
    dynamic_slots.add(humanoid_actor_name)

    for name in extra_dynamic_actors or ():
        dynamic_slots.add(name)
        actor_to_type.setdefault(name, "robot_agent")

    return actor_to_type, slot_to_type, dynamic_slots


def is_dynamic_slot(
    slot_id: str,
    *,
    dynamic_slots: Set[str],
    prop_type_id: str,
) -> bool:
    if slot_id in dynamic_slots:
        return True
    return prop_type_id in DYNAMIC_PROP_TYPE_IDS


def _unwrap_vbp_payload(payload: dict) -> object:
    """UnrealCV vbp wraps Blueprint return values as ``{"ReturnValue": ...}``."""
    if "ReturnValue" not in payload:
        return payload
    inner = payload["ReturnValue"]
    if isinstance(inner, dict):
        return inner
    if isinstance(inner, str):
        text = inner.strip()
        if not text:
            return payload
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return {"error": text}
    return payload


def _parse_sight_payload(raw: object) -> Optional[List[VisibleTarget]]:
    if raw is None:
        return None
    text = str(raw).strip()
    if not text or text.lower().startswith("error"):
        return None
    try:
        payload: object = json.loads(text)
    except json.JSONDecodeError:
        return None

    if isinstance(payload, dict):
        payload = _unwrap_vbp_payload(payload)

    entries: List[object]
    if isinstance(payload, dict):
        entries = payload.get("targets") or payload.get("actors") or []
    elif isinstance(payload, list):
        entries = payload
    else:
        return None

    out: List[VisibleTarget] = []
    for item in entries:
        if isinstance(item, str):
            out.append(
                VisibleTarget(
                    actor_name=item,
                    prop_type_id="",
                    slot_id=item,
                    is_dynamic=False,
                )
            )
            continue
        if not isinstance(item, dict):
            continue
        actor = str(item.get("actor") or item.get("actor_name") or item.get("name") or "")
        if not actor:
            continue
        prop_type = str(item.get("prop_type_id") or item.get("type") or "")
        slot_id = str(item.get("slot_id") or actor)
        is_dyn = bool(item.get("is_dynamic", False))
        out.append(
            VisibleTarget(
                actor_name=actor,
                prop_type_id=prop_type,
                slot_id=slot_id,
                is_dynamic=is_dyn,
            )
        )
    return out


def _sight_controller_names(ucv: UnrealCV) -> List[str]:
    try:
        raw = ucv.client.request("vget /objects")
    except (ConnectionError, OSError, ValueError, RuntimeError):
        return []
    names = [name.strip() for name in str(raw).split() if name.strip()]
    return sorted(name for name in names if "SpotDogAIController" in name)


def _fetch_sight_targets_from_actor(
    ucv: UnrealCV,
    actor_name: str,
) -> Optional[List[VisibleTarget]]:
    for cmd in VBP_SIGHT_COMMANDS:
        try:
            raw = ucv.client.request(f"vbp {actor_name} {cmd}")
        except (ConnectionError, OSError, ValueError, RuntimeError):
            continue
        parsed = _parse_sight_payload(raw)
        if parsed is not None:
            return parsed
    return None


def fetch_ue_sight_targets(ucv: UnrealCV, robot_name: str) -> Optional[List[VisibleTarget]]:
    pawn_targets = _fetch_sight_targets_from_actor(ucv, robot_name)
    if pawn_targets is not None:
        return pawn_targets

    controller_names = _sight_controller_names(ucv)
    if len(controller_names) != 1:
        return pawn_targets

    for controller_name in controller_names:
        controller_targets = _fetch_sight_targets_from_actor(ucv, controller_name)
        if controller_targets is not None:
            return controller_targets

    return pawn_targets


def _resolve_target(
    target: VisibleTarget,
    *,
    actor_to_type: Dict[str, str],
    dynamic_slots: Set[str],
) -> VisibleTarget:
    prop_type = target.prop_type_id or actor_to_type.get(target.actor_name, "")
    slot_id = target.slot_id or target.actor_name
    is_dyn = target.is_dynamic or is_dynamic_slot(
        slot_id, dynamic_slots=dynamic_slots, prop_type_id=prop_type
    )
    return VisibleTarget(
        actor_name=target.actor_name,
        prop_type_id=prop_type,
        slot_id=slot_id,
        is_dynamic=is_dyn,
    )


def _fallback_visible_targets(
    ucv: UnrealCV,
    robot_name: str,
    placement_reg: PlacementRegistry,
    *,
    config: SightConfig,
    humanoid_actor_name: str,
    actor_to_type: Dict[str, str],
    dynamic_slots: Set[str],
) -> List[VisibleTarget]:
    """Interim backend when UE vbp Sight API is not yet wired (geom FOV + humanoid pose)."""
    from l2_geom import geom_detections  # noqa: WPS433

    robot_xy = get_pos2d(ucv, robot_name)
    robot_yaw = get_yaw(ucv, robot_name)
    geom_cfg = GeomPerceptionConfig(
        fov_deg=config.fov_deg,
        max_range_cm=config.max_range_cm,
        sensor_forward_cm=config.sensor_forward_cm,
    )
    visible: List[VisibleTarget] = []
    for det in geom_detections(robot_xy, robot_yaw, placement_reg, config=geom_cfg):
        visible.append(
            VisibleTarget(
                actor_name=det.slot_id,
                prop_type_id=det.prop_type_id,
                slot_id=det.slot_id,
                is_dynamic=False,
            )
        )

    try:
        human_xy = get_pos2d(ucv, humanoid_actor_name)
    except (ConnectionError, OSError, ValueError, RuntimeError):
        return visible

    dist_cm = dist2d(robot_xy, human_xy)
    if dist_cm > config.max_range_cm:
        return visible
    bearing = _bearing_deg_robot_frame(
        robot_xy,
        robot_yaw,
        human_xy,
        sensor_forward_cm=config.sensor_forward_cm,
    )
    if not _in_fov_cone(bearing, config.fov_deg):
        return visible

    visible.append(
        VisibleTarget(
            actor_name=humanoid_actor_name,
            prop_type_id=actor_to_type.get(humanoid_actor_name, HUMAN_PROP_TYPE_ID),
            slot_id=humanoid_actor_name,
            is_dynamic=True,
        )
    )
    return visible


def _actor_world_xy(ucv: UnrealCV, actor_name: str) -> Optional[WorldXY]:
    try:
        return get_pos2d(ucv, actor_name)
    except (ConnectionError, OSError, ValueError, RuntimeError):
        return None


def _placement_xy_for_slot(
    placement_reg: PlacementRegistry,
    slot_id: str,
) -> Optional[WorldXY]:
    for prop in placement_reg.props:
        if prop.slot_id == slot_id:
            return _prop_world_xy(prop)
    return None


def _target_world_xy(
    ucv: UnrealCV,
    target: VisibleTarget,
    placement_reg: PlacementRegistry,
    *,
    humanoid_actor_name: str,
) -> Optional[WorldXY]:
    live = _actor_world_xy(ucv, target.actor_name)
    if live is not None:
        return live
    if target.actor_name == humanoid_actor_name or target.slot_id == humanoid_actor_name:
        return _actor_world_xy(ucv, humanoid_actor_name)
    return _placement_xy_for_slot(placement_reg, target.slot_id)


def detection_from_world_pose(
    *,
    slot_id: str,
    prop_type_id: str,
    target_xy: WorldXY,
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    config: SightConfig,
) -> ObjectEstimate:
    dist_m = dist2d(robot_xy, target_xy) / 100.0
    bearing = _bearing_deg_robot_frame(
        robot_xy,
        robot_yaw_deg,
        target_xy,
        sensor_forward_cm=config.sensor_forward_cm,
    )
    confidence = max(0.3, min(1.0, 1.0 - dist_m * 100.0 / max(config.max_range_cm, 1.0)))
    return ObjectEstimate(
        prop_type_id=prop_type_id,
        slot_id=slot_id,
        distance_m=dist_m,
        bearing_deg=bearing,
        mask_pixels=100,
        confidence=confidence,
    )


def _world_xy_for_slot(
    slot_id: str,
    memory: SightMemory,
    *,
    live_xy: Optional[WorldXY],
    is_dynamic: bool,
) -> Optional[WorldXY]:
    if is_dynamic:
        return live_xy
    if live_xy is not None:
        memory.static_last_seen_xy[slot_id] = live_xy
        return live_xy
    return memory.static_last_seen_xy.get(slot_id)


def _apply_slot_cells(
    layers: LayeredCostmap,
    slot_id: str,
    center_xy: WorldXY,
    *,
    config: SightConfig,
    tracker: L2SlotCellTracker,
    l2_seen_cells: Set[GridCell],
) -> Tuple[int, int]:
    desired = set(l2_cells_for_world_disk(layers, center_xy, radius_cm=config.prop_radius_cm))
    previous = tracker.cells_for(slot_id)
    to_remove = previous - desired
    to_add = desired - previous
    for gx, gy in to_remove:
        layers.clear_l2_cell(gx, gy)
        l2_seen_cells.discard((gx, gy))
    if to_add:
        apply_l2_obstacle_cells(
            layers,
            list(to_add),
            config=EgocentricPerceptionConfig(use_lethal=True),
        )
        for cell in to_add:
            l2_seen_cells.add(cell)
    tracker.set_cells(slot_id, desired)
    return len(to_add), len(to_remove)


def _remove_slot_cells(
    layers: LayeredCostmap,
    slot_id: str,
    *,
    tracker: L2SlotCellTracker,
    l2_seen_cells: Set[GridCell],
) -> int:
    removed = tracker.pop_cells(slot_id)
    for gx, gy in removed:
        layers.clear_l2_cell(gx, gy)
        l2_seen_cells.discard((gx, gy))
    return len(removed)


def update_l2_from_sight(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    *,
    robot_name: str,
    placement_reg: PlacementRegistry,
    humanoid_actor_name: str,
    material_actor_name: str,
    memory: SightMemory,
    tracker: L2SlotCellTracker,
    l2_seen_cells: Set[GridCell],
    config: Optional[SightConfig] = None,
    extra_dynamic_actors: Optional[Sequence[str]] = None,
) -> SightUpdateResult:
    cfg = config or SightConfig()
    actor_to_type, _slot_to_type, dynamic_slots = build_actor_maps(
        placement_reg,
        humanoid_actor_name=humanoid_actor_name,
        material_actor_name=material_actor_name,
        extra_dynamic_actors=extra_dynamic_actors,
    )

    backend = "ue_sight"
    raw_targets = fetch_ue_sight_targets(ucv, robot_name)
    if raw_targets is None:
        backend = "geom_fallback"
        raw_targets = _fallback_visible_targets(
            ucv,
            robot_name,
            placement_reg,
            config=cfg,
            humanoid_actor_name=humanoid_actor_name,
            actor_to_type=actor_to_type,
            dynamic_slots=dynamic_slots,
        )

    robot_xy = get_pos2d(ucv, robot_name)
    robot_yaw = get_yaw(ucv, robot_name)

    resolved: List[VisibleTarget] = []
    for target in raw_targets:
        resolved.append(
            _resolve_target(
                target,
                actor_to_type=actor_to_type,
                dynamic_slots=dynamic_slots,
            )
        )

    visible_now: Dict[str, VisibleTarget] = {}
    for target in resolved:
        prop_type = target.prop_type_id or actor_to_type.get(target.actor_name, "unknown")
        slot_id = target.slot_id
        visible_now[slot_id] = VisibleTarget(
            actor_name=target.actor_name,
            prop_type_id=prop_type,
            slot_id=slot_id,
            is_dynamic=is_dynamic_slot(
                slot_id, dynamic_slots=dynamic_slots, prop_type_id=prop_type
            ),
        )

    cells_added = 0
    cells_removed = 0
    detections: List[ObjectEstimate] = []

    # Dynamic: drop slots that left FOV.
    for slot_id in list(memory.last_visible_dynamic):
        if slot_id not in visible_now:
            cells_removed += _remove_slot_cells(
                layers, slot_id, tracker=tracker, l2_seen_cells=l2_seen_cells
            )
    memory.last_visible_dynamic = {
        slot_id
        for slot_id, target in visible_now.items()
        if target.is_dynamic
    }

    # Visible targets: update L2 at current (dynamic) or last-seen (static) position.
    for slot_id, target in visible_now.items():
        live_xy = _target_world_xy(
            ucv,
            target,
            placement_reg,
            humanoid_actor_name=humanoid_actor_name,
        )
        center_xy = _world_xy_for_slot(
            slot_id,
            memory,
            live_xy=live_xy,
            is_dynamic=target.is_dynamic,
        )
        if center_xy is None:
            continue
        added, removed = _apply_slot_cells(
            layers,
            slot_id,
            center_xy,
            config=cfg,
            tracker=tracker,
            l2_seen_cells=l2_seen_cells,
        )
        cells_added += added
        cells_removed += removed
        detections.append(
            detection_from_world_pose(
                slot_id=slot_id,
                prop_type_id=target.prop_type_id,
                target_xy=center_xy,
                robot_xy=robot_xy,
                robot_yaw_deg=robot_yaw,
                config=cfg,
            )
        )

    # Static memory: keep L2 at last seen even when not currently visible.
    for slot_id, center_xy in memory.static_last_seen_xy.items():
        if slot_id in visible_now:
            continue
        prop_type = actor_to_type.get(slot_id, "static_prop")
        if is_dynamic_slot(slot_id, dynamic_slots=dynamic_slots, prop_type_id=prop_type):
            continue
        added, removed = _apply_slot_cells(
            layers,
            slot_id,
            center_xy,
            config=cfg,
            tracker=tracker,
            l2_seen_cells=l2_seen_cells,
        )
        cells_added += added
        cells_removed += removed
        detections.append(
            detection_from_world_pose(
                slot_id=slot_id,
                prop_type_id=prop_type,
                target_xy=center_xy,
                robot_xy=robot_xy,
                robot_yaw_deg=robot_yaw,
                config=cfg,
            )
        )

    return SightUpdateResult(
        detections=detections,
        cells_added=cells_added,
        cells_removed=cells_removed,
        visible_actor_names=tuple(t.actor_name for t in visible_now.values()),
        backend=backend,
    )


def estimate_local_xy_from_detection(
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    detection: ObjectEstimate,
    *,
    sensor_forward_cm: float = SENSOR_CAM_FORWARD_OFFSET_CM,
) -> Tuple[float, float]:
    wx, wy = estimate_world_xy_from_detection(
        robot_xy,
        robot_yaw_deg,
        distance_m=float(detection.distance_m),
        bearing_deg=float(detection.bearing_deg),
        camera_offset_forward_cm=sensor_forward_cm,
    )
    return lc.world_xy_to_local(wx, wy)
