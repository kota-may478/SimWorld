#!/usr/bin/env python3
"""Curated construction-site prop placement (fixed seed, cluster layout)."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

THIS_DIR = Path(__file__).resolve().parent
from paths import CONSTRUCTION_SITE_REGISTRY, REGISTRY_DIR  # noqa: E402

CACHE_DIR = REGISTRY_DIR
REGISTRY_PATH = CONSTRUCTION_SITE_REGISTRY

PROP_ACTOR_PREFIX = "csite_prop"
MATERIAL_ACTOR_NAME = "csite_material"
CARRY_ACTOR_NAME = "csite_carry"

DEFAULT_SEED = 20260617
MIN_CORRIDOR_WIDTH_CM = 200.0
ROBOT_START_LOCAL_CM = (1200.0, 1200.0)
HOME_LOCAL_CM = ROBOT_START_LOCAL_CM
MATERIAL_PICKUP_LOCAL_CM = (6100.0, 5300.0)

# Debug / calibration meshes — not suitable for a construction site scene.
EXCLUDED_BP_NAMES = frozenset(
    {
        "BP_ZBackdrop_01",
        "BP_ZPlane_01a",
        "BP_ZSphere",
    }
)


@dataclass(frozen=True)
class SitePropSlot:
    slot_id: str
    bp_name: str
    cluster_id: str
    role: str
    local_xy_cm: Tuple[float, float]
    yaw_deg: float = 0.0
    is_transport_target: bool = False
    bp_path: str = ""
    prop_type_id: str = ""
    world_xyz_cm: Optional[Tuple[float, float, float]] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["local_xy_cm"] = list(self.local_xy_cm)
        if self.world_xyz_cm is not None:
            d["world_xyz_cm"] = list(self.world_xyz_cm)
        return d

    @classmethod
    def from_dict(cls, raw: Dict[str, Any]) -> SitePropSlot:
        world = raw.get("world_xyz_cm")
        return cls(
            slot_id=str(raw["slot_id"]),
            bp_name=str(raw["bp_name"]),
            cluster_id=str(raw["cluster_id"]),
            role=str(raw["role"]),
            local_xy_cm=(float(raw["local_xy_cm"][0]), float(raw["local_xy_cm"][1])),
            yaw_deg=float(raw.get("yaw_deg", 0.0)),
            is_transport_target=bool(raw.get("is_transport_target", False)),
            bp_path=str(raw.get("bp_path", "")),
            prop_type_id=str(raw.get("prop_type_id", "")),
            world_xyz_cm=(
                (float(world[0]), float(world[1]), float(world[2])) if world is not None else None
            ),
        )


@dataclass(frozen=True)
class ConstructionSiteRegistry:
    version: int
    seed: int
    min_corridor_width_cm: float
    robot_start_local_cm: Tuple[float, float]
    home_local_cm: Tuple[float, float]
    material_pickup_local_cm: Tuple[float, float]
    material_actor_name: str
    carry_actor_name: str
    props: Tuple[SitePropSlot, ...]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": self.version,
            "seed": self.seed,
            "min_corridor_width_cm": self.min_corridor_width_cm,
            "robot_start_local_cm": list(self.robot_start_local_cm),
            "home_local_cm": list(self.home_local_cm),
            "material_pickup_local_cm": list(self.material_pickup_local_cm),
            "material_actor_name": self.material_actor_name,
            "carry_actor_name": self.carry_actor_name,
            "props": [p.to_dict() for p in self.props],
        }

    @classmethod
    def from_dict(cls, raw: Dict[str, Any]) -> ConstructionSiteRegistry:
        props = tuple(SitePropSlot.from_dict(p) for p in raw["props"])
        return cls(
            version=int(raw.get("version", 1)),
            seed=int(raw["seed"]),
            min_corridor_width_cm=float(raw.get("min_corridor_width_cm", MIN_CORRIDOR_WIDTH_CM)),
            robot_start_local_cm=(
                float(raw["robot_start_local_cm"][0]),
                float(raw["robot_start_local_cm"][1]),
            ),
            home_local_cm=(
                float(raw["home_local_cm"][0]),
                float(raw["home_local_cm"][1]),
            ),
            material_pickup_local_cm=(
                float(raw["material_pickup_local_cm"][0]),
                float(raw["material_pickup_local_cm"][1]),
            ),
            material_actor_name=str(raw.get("material_actor_name", MATERIAL_ACTOR_NAME)),
            carry_actor_name=str(raw.get("carry_actor_name", CARRY_ACTOR_NAME)),
            props=props,
        )

    def transport_slot(self) -> Optional[SitePropSlot]:
        for prop in self.props:
            if prop.is_transport_target:
                return prop
        return None

    def obstacle_slots(self) -> Tuple[SitePropSlot, ...]:
        return tuple(p for p in self.props if not p.is_transport_target)


def _catalog_by_bp_name():
    from prop_catalog import ensure_catalog  # noqa: WPS433

    entries = ensure_catalog()
    return {e.bp_name: e for e in entries}


def _dist_point_to_segment_cm(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    ab_len_sq = abx * abx + aby * aby
    if ab_len_sq < 1e-6:
        return math.hypot(apx, apy)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_len_sq))
    cx = ax + t * abx
    cy = ay + t * aby
    return math.hypot(px - cx, py - cy)


def corridor_clearance_cm(
    local_xy: Tuple[float, float],
    *,
    start_local: Tuple[float, float] = ROBOT_START_LOCAL_CM,
    goal_local: Tuple[float, float] = MATERIAL_PICKUP_LOCAL_CM,
) -> float:
    return _dist_point_to_segment_cm(
        local_xy[0],
        local_xy[1],
        start_local[0],
        start_local[1],
        goal_local[0],
        goal_local[1],
    )


def assert_corridor_clear(
    slots: Sequence[SitePropSlot],
    *,
    min_width_cm: float = MIN_CORRIDOR_WIDTH_CM,
    start_local: Tuple[float, float] = ROBOT_START_LOCAL_CM,
    goal_local: Tuple[float, float] = MATERIAL_PICKUP_LOCAL_CM,
) -> None:
    half = min_width_cm * 0.5
    for slot in slots:
        if slot.is_transport_target:
            continue
        dist = corridor_clearance_cm(slot.local_xy_cm, start_local=start_local, goal_local=goal_local)
        if dist < half:
            raise ValueError(
                f"{slot.slot_id} @ {slot.local_xy_cm} is only {dist:.0f}cm from transport corridor "
                f"(need >= {half:.0f}cm half-width)"
            )


def _slot(
    index: int,
    bp_name: str,
    cluster_id: str,
    role: str,
    local_xy_cm: Tuple[float, float],
    *,
    yaw_deg: float = 0.0,
    is_transport_target: bool = False,
    catalog: Dict[str, object],
) -> SitePropSlot:
    entry = catalog[bp_name]
    return SitePropSlot(
        slot_id=f"{PROP_ACTOR_PREFIX}_{index:03d}",
        bp_name=bp_name,
        cluster_id=cluster_id,
        role=role,
        local_xy_cm=local_xy_cm,
        yaw_deg=yaw_deg,
        is_transport_target=is_transport_target,
        bp_path=entry.bp_path,
        prop_type_id=entry.prop_type_id,
    )


def build_construction_site_registry(
    *,
    seed: int = DEFAULT_SEED,
    min_corridor_width_cm: float = MIN_CORRIDOR_WIDTH_CM,
) -> ConstructionSiteRegistry:
    """20 curated prop types in thematic clusters; fixed layout for seed reproducibility."""
    catalog = _catalog_by_bp_name()
    required_names = {bp_name for bp_name, *_ in _CURATED_LAYOUT if bp_name not in EXCLUDED_BP_NAMES}
    missing = sorted(name for name in required_names if name not in catalog)
    if missing:
        raise RuntimeError(f"missing catalog entries: {missing}")

    slots: List[SitePropSlot] = []
    index = 0
    for bp_name, cluster_id, role, local_xy, yaw_deg, is_target in _CURATED_LAYOUT:
        if bp_name in EXCLUDED_BP_NAMES:
            continue
        slots.append(
            _slot(
                index,
                bp_name,
                cluster_id,
                role,
                local_xy,
                yaw_deg=yaw_deg,
                is_transport_target=is_target,
                catalog=catalog,
            )
        )
        index += 1

    assert_corridor_clear(slots, min_width_cm=min_corridor_width_cm)
    unique_types = {s.bp_name for s in slots}
    if len(unique_types) != 20:
        raise RuntimeError(f"expected 20 unique prop types, got {len(unique_types)}")

    return ConstructionSiteRegistry(
        version=1,
        seed=seed,
        min_corridor_width_cm=min_corridor_width_cm,
        robot_start_local_cm=ROBOT_START_LOCAL_CM,
        home_local_cm=HOME_LOCAL_CM,
        material_pickup_local_cm=MATERIAL_PICKUP_LOCAL_CM,
        material_actor_name=MATERIAL_ACTOR_NAME,
        carry_actor_name=CARRY_ACTOR_NAME,
        props=tuple(slots),
    )


# (bp_name, cluster_id, role, local_xy_cm, yaw_deg, is_transport_target)
_CURATED_LAYOUT: List[Tuple[str, str, str, Tuple[float, float], float, bool]] = [
    # SW staging — pallets and loose boxes away from the main corridor
    ("BP_woodenpalette_01", "staging_sw", "wooden_pallet", (750.0, 850.0), 15.0, False),
    ("BP_Boxes_03a", "staging_sw", "cardboard_boxes", (950.0, 650.0), -10.0, False),
    # West equipment yard
    ("BP_Dumpster", "equipment_west", "waste_dumpster", (1350.0, 4300.0), 0.0, False),
    ("BP_LightGenerator_01a", "equipment_west", "portable_light_tower", (1050.0, 4850.0), 25.0, False),
    ("BP_CableReel", "equipment_west", "cable_spool", (1650.0, 4550.0), 40.0, False),
    ("BP_Drywall_01a", "equipment_west", "drywall_sheets", (1950.0, 5150.0), -5.0, False),
    # Mid-site traffic control (well west of the diagonal transport corridor)
    ("BP_ConstructionPylons_01a", "traffic_mid", "safety_pylon", (1750.0, 3950.0), 0.0, False),
    ("BP_ConstructionPylons_01d", "traffic_mid", "safety_pylon", (1950.0, 4150.0), 0.0, False),
    ("BP_Roadblock_01a", "traffic_mid", "road_barrier", (2150.0, 3750.0), 90.0, False),
    ("BP_Trafficbarrier_01", "traffic_mid", "traffic_barrier", (2350.0, 3550.0), 0.0, False),
    # Cinder block wall segment (west of mid-corridor)
    ("BP_CinderStack_01a", "cinder_wall", "cinder_blocks", (2500.0, 4300.0), 0.0, False),
    ("BP_CinderStack_01a", "cinder_wall", "cinder_blocks", (2700.0, 4300.0), 0.0, False),
    ("BP_CinderStack_01a", "cinder_wall", "cinder_blocks", (2900.0, 4300.0), 0.0, False),
    # North perimeter fence line
    ("BP_construction_fence_01a", "fence_north", "site_fence_panel", (3100.0, 7050.0), 0.0, False),
    ("BP_construction_fence_01a", "fence_north", "site_fence_panel", (3600.0, 7050.0), 0.0, False),
    ("BP_construction_fence_connectors_01a", "fence_north", "fence_connector", (3350.0, 7050.0), 0.0, False),
    ("BP_construction_fence_support_01a", "fence_north", "fence_support", (3850.0, 7050.0), 0.0, False),
    # NE material yard — transport target crate at the diagonal goal
    ("BP_Crate_01a", "material_yard", "shipping_crate", MATERIAL_PICKUP_LOCAL_CM, 20.0, True),
    ("BP_BrickPaletteStack_01a", "material_yard", "brick_pallet", (5800.0, 5450.0), 0.0, False),
    ("BP_ConcreteBag_01a", "material_yard", "bagged_concrete", (6350.0, 5480.0), 0.0, False),
    ("BP_Rebar_01a", "material_yard", "rebar_bundle", (6450.0, 5200.0), 45.0, False),
    # SE site facilities
    ("BP_Portapotty_01", "facilities_se", "portable_toilet", (5950.0, 3950.0), -30.0, False),
    ("BP_WaterTank_01a", "facilities_se", "water_tank", (5750.0, 4250.0), 0.0, False),
]


def save_registry(registry: ConstructionSiteRegistry, path: Path = REGISTRY_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(registry.to_dict(), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def load_registry(path: Path = REGISTRY_PATH) -> ConstructionSiteRegistry:
    if not path.is_file():
        raise FileNotFoundError(f"construction site registry not found: {path}")
    return ConstructionSiteRegistry.from_dict(json.loads(path.read_text(encoding="utf-8")))


def ensure_registry(
    path: Path = REGISTRY_PATH,
    *,
    seed: int = DEFAULT_SEED,
    force_rebuild: bool = False,
) -> ConstructionSiteRegistry:
    if path.is_file() and not force_rebuild:
        return load_registry(path)
    registry = build_construction_site_registry(seed=seed)
    save_registry(registry, path)
    return registry


def update_slot_pose(
    registry: ConstructionSiteRegistry,
    slot_id: str,
    world_xyz_cm: Tuple[float, float, float],
    *,
    local_xy_cm: Optional[Tuple[float, float]] = None,
) -> ConstructionSiteRegistry:
    updated: List[SitePropSlot] = []
    for prop in registry.props:
        if prop.slot_id != slot_id:
            updated.append(prop)
            continue
        updated.append(
            SitePropSlot(
                slot_id=prop.slot_id,
                bp_name=prop.bp_name,
                cluster_id=prop.cluster_id,
                role=prop.role,
                local_xy_cm=local_xy_cm or prop.local_xy_cm,
                yaw_deg=prop.yaw_deg,
                is_transport_target=prop.is_transport_target,
                bp_path=prop.bp_path,
                prop_type_id=prop.prop_type_id,
                world_xyz_cm=world_xyz_cm,
            )
        )
    return ConstructionSiteRegistry(
        version=registry.version,
        seed=registry.seed,
        min_corridor_width_cm=registry.min_corridor_width_cm,
        robot_start_local_cm=registry.robot_start_local_cm,
        home_local_cm=registry.home_local_cm,
        material_pickup_local_cm=registry.material_pickup_local_cm,
        material_actor_name=registry.material_actor_name,
        carry_actor_name=registry.carry_actor_name,
        props=tuple(updated),
    )
