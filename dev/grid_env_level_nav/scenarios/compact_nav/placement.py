#!/usr/bin/env python3
"""Fixed 3-prop placement for 30 m × 30 m compact navigation test."""

from __future__ import annotations

import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

THIS_DIR = Path(__file__).resolve().parent
from paths import COMPACT_NAV_REGISTRY, REGISTRY_DIR  # noqa: E402

CACHE_DIR = REGISTRY_DIR
REGISTRY_PATH = COMPACT_NAV_REGISTRY

from region import GOAL_LOCAL_CM, ROBOT_START_LOCAL_CM  # noqa: E402

PROP_ACTOR_PREFIX = "compact_prop"
DEFAULT_SEED = 20260618
PROP_COUNT = 3

# Along diagonal between robot (1m,1m) and goal (25m,25m).
_PROP_LAYOUT: List[Tuple[str, Tuple[float, float], float]] = [
    ("BP_Boxes_03a", (1500.0, 1500.0), 10.0),
    ("BP_ConstructionPylons_01a", (1300.0, 1700.0), 0.0),
    ("BP_Barrel_01", (1700.0, 1300.0), 25.0),
]


@dataclass(frozen=True)
class CompactPropSlot:
    slot_id: str
    bp_name: str
    bp_path: str
    prop_type_id: str
    mask_color_rgb: Tuple[int, int, int]
    local_xy_cm: Tuple[float, float]
    yaw_deg: float = 0.0
    world_xyz_cm: Optional[Tuple[float, float, float]] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["local_xy_cm"] = list(self.local_xy_cm)
        d["mask_color_rgb"] = list(self.mask_color_rgb)
        if self.world_xyz_cm is not None:
            d["world_xyz_cm"] = list(self.world_xyz_cm)
        return d

    @classmethod
    def from_dict(cls, raw: Dict[str, Any]) -> "CompactPropSlot":
        world = raw.get("world_xyz_cm")
        return cls(
            slot_id=str(raw["slot_id"]),
            bp_name=str(raw["bp_name"]),
            bp_path=str(raw["bp_path"]),
            prop_type_id=str(raw["prop_type_id"]),
            mask_color_rgb=tuple(int(v) for v in raw["mask_color_rgb"]),  # type: ignore[arg-type]
            local_xy_cm=(float(raw["local_xy_cm"][0]), float(raw["local_xy_cm"][1])),
            yaw_deg=float(raw.get("yaw_deg", 0.0)),
            world_xyz_cm=(
                (float(world[0]), float(world[1]), float(world[2])) if world is not None else None
            ),
        )

    def detection_bgr(self) -> Tuple[int, int, int]:
        r, g, b = self.mask_color_rgb
        return (b, g, r)


@dataclass(frozen=True)
class CompactNavRegistry:
    version: int
    seed: int
    region_size_cm: float
    goal_local_cm: Tuple[float, float]
    robot_start_local_cm: Tuple[float, float]
    props: Tuple[CompactPropSlot, ...]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": self.version,
            "seed": self.seed,
            "region_size_cm": self.region_size_cm,
            "goal_local_cm": list(self.goal_local_cm),
            "robot_start_local_cm": list(self.robot_start_local_cm),
            "props": [p.to_dict() for p in self.props],
        }

    @classmethod
    def from_dict(cls, raw: Dict[str, Any]) -> CompactNavRegistry:
        props = tuple(CompactPropSlot.from_dict(p) for p in raw["props"])
        return cls(
            version=int(raw.get("version", 1)),
            seed=int(raw["seed"]),
            region_size_cm=float(raw.get("region_size_cm", 3000.0)),
            goal_local_cm=(
                float(raw["goal_local_cm"][0]),
                float(raw["goal_local_cm"][1]),
            ),
            robot_start_local_cm=(
                float(raw["robot_start_local_cm"][0]),
                float(raw["robot_start_local_cm"][1]),
            ),
            props=props,
        )


def mask_color_for_slot(slot_index: int) -> Tuple[int, int, int]:
    base = slot_index + 3
    return (
        (base * 41 + 13) % 190 + 35,
        (base * 59 + 19) % 190 + 35,
        (base * 67 + 23) % 190 + 35,
    )


def _catalog_by_bp_name():
    from prop_catalog import ensure_catalog  # noqa: WPS433

    return {e.bp_name: e for e in ensure_catalog()}


def build_compact_nav_registry(*, seed: int = DEFAULT_SEED) -> CompactNavRegistry:
    catalog = _catalog_by_bp_name()
    props: List[CompactPropSlot] = []
    for idx, (bp_name, local_xy, yaw_deg) in enumerate(_PROP_LAYOUT):
        entry = catalog[bp_name]
        props.append(
            CompactPropSlot(
                slot_id=f"{PROP_ACTOR_PREFIX}_{idx:03d}",
                bp_name=bp_name,
                bp_path=entry.bp_path,
                prop_type_id=entry.prop_type_id,
                mask_color_rgb=mask_color_for_slot(idx),
                local_xy_cm=local_xy,
                yaw_deg=yaw_deg,
            )
        )
    _assert_between_start_and_goal(props)
    return CompactNavRegistry(
        version=1,
        seed=seed,
        region_size_cm=3000.0,
        goal_local_cm=GOAL_LOCAL_CM,
        robot_start_local_cm=ROBOT_START_LOCAL_CM,
        props=tuple(props),
    )


def _assert_between_start_and_goal(props: List[CompactPropSlot]) -> None:
    sx, sy = ROBOT_START_LOCAL_CM
    gx, gy = GOAL_LOCAL_CM
    for prop in props:
        lx, ly = prop.local_xy_cm
        if not (sx < lx < gx and sy < ly < gy):
            raise ValueError(f"{prop.slot_id} @ {prop.local_xy_cm} not between start and goal")


def _dist_to_segment_cm(
    p: Tuple[float, float],
    a: Tuple[float, float],
    b: Tuple[float, float],
) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab_len_sq = abx * abx + aby * aby
    if ab_len_sq < 1e-6:
        return math.hypot(apx, apy)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_len_sq))
    cx, cy = ax + t * abx, ay + t * aby
    return math.hypot(px - cx, py - cy)


def save_registry(registry: CompactNavRegistry, path: Path = REGISTRY_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(registry.to_dict(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def load_registry(path: Path = REGISTRY_PATH) -> CompactNavRegistry:
    return CompactNavRegistry.from_dict(json.loads(path.read_text(encoding="utf-8")))


def ensure_registry(*, seed: int = DEFAULT_SEED, force_rebuild: bool = False) -> CompactNavRegistry:
    if REGISTRY_PATH.is_file() and not force_rebuild:
        return load_registry()
    registry = build_compact_nav_registry(seed=seed)
    save_registry(registry)
    return registry


def to_placement_registry(registry: CompactNavRegistry):
    """Adapter for depth_object_perception.detect_objects."""
    from prop_placement import PlacementRegistry, PropPlacement  # noqa: WPS433

    props: List[PropPlacement] = []
    for idx, slot in enumerate(registry.props):
        props.append(
            PropPlacement(
                slot_id=slot.slot_id,
                catalog_index=idx,
                prop_type_id=slot.prop_type_id,
                bp_name=slot.bp_name,
                bp_path=slot.bp_path,
                mask_color_rgb=slot.mask_color_rgb,
                local_xy_cm=slot.local_xy_cm,
                world_xyz_cm=slot.world_xyz_cm,
                mask_color_canonical_rgb=slot.mask_color_rgb,
            )
        )
    return PlacementRegistry(
        version=1,
        seed=registry.seed,
        prop_count=len(props),
        region_x_max_cm=registry.region_size_cm,
        region_y_max_cm=registry.region_size_cm,
        exclusion_cm=0.0,
        spotdog_spawn_local_cm=registry.robot_start_local_cm,
        props=tuple(props),
    )


def update_slot_pose(
    registry: CompactNavRegistry,
    slot_id: str,
    world_xyz_cm: Tuple[float, float, float],
    *,
    local_xy_cm: Optional[Tuple[float, float]] = None,
) -> CompactNavRegistry:
    updated: List[CompactPropSlot] = []
    for prop in registry.props:
        if prop.slot_id != slot_id:
            updated.append(prop)
            continue
        updated.append(
            CompactPropSlot(
                slot_id=prop.slot_id,
                bp_name=prop.bp_name,
                bp_path=prop.bp_path,
                prop_type_id=prop.prop_type_id,
                mask_color_rgb=prop.mask_color_rgb,
                local_xy_cm=local_xy_cm or prop.local_xy_cm,
                yaw_deg=prop.yaw_deg,
                world_xyz_cm=world_xyz_cm,
            )
        )
    return CompactNavRegistry(
        version=registry.version,
        seed=registry.seed,
        region_size_cm=registry.region_size_cm,
        goal_local_cm=registry.goal_local_cm,
        robot_start_local_cm=registry.robot_start_local_cm,
        props=tuple(updated),
    )
