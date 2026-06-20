#!/usr/bin/env python3
"""Curated 20 m construction-site layout with material yard at NE corner."""

from __future__ import annotations

import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from paths import SITE_TRANSPORT_20M_REGISTRY, REGISTRY_DIR  # noqa: E402
from region import (  # noqa: E402
    HUMANOID_LOCAL_CM,
    LAYOUT_ID,
    MATERIAL_YARD_CORNER_CM,
    REGION_SIZE_CM,
    ROBOT_START_LOCAL_CM,
)
from zones import (  # noqa: E402
    FORBIDDEN_ZONES_LAYOUT_01,
    ROADBLOCK_03B_WIDTH_CM,
    ROADBLOCK_BP_NAME,
    ROADBLOCKS_PER_SIDE,
    ForbiddenZone,
)

REGISTRY_PATH = SITE_TRANSPORT_20M_REGISTRY
PROP_ACTOR_PREFIX = "site20_prop"
MATERIAL_ACTOR_NAME = "site20_material"
CARRY_ACTOR_NAME = "site20_carry"
HUMANOID_ACTOR_NAME = "site20_humanoid"
DEFAULT_SEED = 20260619
MIN_CORRIDOR_WIDTH_CM = 160.0

# Transport crate near (20m, 20m) corner inside material yard cluster.
TRANSPORT_LOCAL_CM = (1850.0, 1850.0)

LayoutEntry = Tuple[str, str, str, Tuple[float, float], float, bool]

# (bp_name, cluster, role, local_xy_cm, yaw_deg, is_transport_target)
_SITE_PROPS_LAYOUT: List[LayoutEntry] = [
    # SW facilities / equipment
    ("BP_Dumpster", "facilities_sw", "waste_dumpster", (400.0, 450.0), 0.0, False),
    ("BP_LightGenerator_01a", "equipment_sw", "light_tower", (550.0, 600.0), 20.0, False),
    ("BP_CableReel", "equipment_sw", "cable_spool", (650.0, 400.0), 35.0, False),
    ("BP_Portapotty_01", "facilities_sw", "portable_toilet", (350.0, 750.0), -15.0, False),
    ("BP_WaterTank_01a", "facilities_sw", "water_tank", (500.0, 850.0), 0.0, False),
    # Mid-site clutter (off main diagonal)
    ("BP_Barrel_01", "mid_site", "barrel", (900.0, 700.0), 25.0, False),
    ("BP_Drywall_01a", "mid_site", "drywall", (1100.0, 550.0), -5.0, False),
    ("BP_CinderStack_01a", "mid_site", "cinder_blocks", (750.0, 1100.0), 0.0, False),
    ("BP_CinderStack_01a", "mid_site", "cinder_blocks", (850.0, 1200.0), 0.0, False),
    ("BP_woodenpalette_01", "mid_site", "pallet", (1200.0, 300.0), 10.0, False),
    ("BP_ConstructionPylons_01d", "mid_site", "safety_pylon", (1000.0, 950.0), 0.0, False),
    # NE material yard (corner ~20m × 20m)
    ("BP_woodenpalette_01", "material_yard", "pallet", (1720.0, 1780.0), 15.0, False),
    ("BP_Boxes_03a", "material_yard", "cardboard_boxes", (1780.0, 1920.0), -10.0, False),
    ("BP_BrickPaletteStack_01a", "material_yard", "brick_pallet", (1920.0, 1720.0), 0.0, False),
    ("BP_ConcreteBag_01a", "material_yard", "bagged_concrete", (1680.0, 1880.0), 0.0, False),
    ("BP_Rebar_01a", "material_yard", "rebar_bundle", (1900.0, 1900.0), 45.0, False),
    ("BP_Crate_01a", "material_yard", "shipping_crate", TRANSPORT_LOCAL_CM, 20.0, True),
]


def _roadblock_perimeter_layout(
    rect_local_cm: Tuple[float, float, float, float],
) -> List[LayoutEntry]:
    """Place 3 roadblocks per side around the L1 forbidden rect."""
    x0, y0, x1, y1 = rect_local_cm
    x_min, x_max = min(x0, x1), max(x0, x1)
    y_min, y_max = min(y0, y1), max(y0, y1)
    entries: List[LayoutEntry] = []
    width = ROADBLOCK_03B_WIDTH_CM
    half = width * 0.5

    for index in range(ROADBLOCKS_PER_SIDE):
        x = x_min + half + index * width
        y = y_min + half + index * width
        entries.append(
            (ROADBLOCK_BP_NAME, "no_entry_roadblock", "roadblock_south", (x, y_min), 0.0, False)
        )
        entries.append(
            (ROADBLOCK_BP_NAME, "no_entry_roadblock", "roadblock_north", (x, y_max), 180.0, False)
        )
        entries.append(
            (ROADBLOCK_BP_NAME, "no_entry_roadblock", "roadblock_west", (x_min, y), 90.0, False)
        )
        entries.append(
            (ROADBLOCK_BP_NAME, "no_entry_roadblock", "roadblock_east", (x_max, y), -90.0, False)
        )
    return entries


_CURATED_LAYOUT: List[LayoutEntry] = _SITE_PROPS_LAYOUT + _roadblock_perimeter_layout(
    FORBIDDEN_ZONES_LAYOUT_01[0].rect_local_cm
)


@dataclass(frozen=True)
class SitePropSlot:
    slot_id: str
    bp_name: str
    bp_path: str
    prop_type_id: str
    mask_color_rgb: Tuple[int, int, int]
    cluster_id: str
    role: str
    local_xy_cm: Tuple[float, float]
    yaw_deg: float = 0.0
    is_transport_target: bool = False
    world_xyz_cm: Optional[Tuple[float, float, float]] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["local_xy_cm"] = list(self.local_xy_cm)
        d["mask_color_rgb"] = list(self.mask_color_rgb)
        if self.world_xyz_cm is not None:
            d["world_xyz_cm"] = list(self.world_xyz_cm)
        return d

    @classmethod
    def from_dict(cls, raw: Dict[str, Any]) -> SitePropSlot:
        world = raw.get("world_xyz_cm")
        return cls(
            slot_id=str(raw["slot_id"]),
            bp_name=str(raw["bp_name"]),
            bp_path=str(raw["bp_path"]),
            prop_type_id=str(raw["prop_type_id"]),
            mask_color_rgb=tuple(int(v) for v in raw["mask_color_rgb"]),  # type: ignore[arg-type]
            cluster_id=str(raw.get("cluster_id", "")),
            role=str(raw.get("role", "")),
            local_xy_cm=(float(raw["local_xy_cm"][0]), float(raw["local_xy_cm"][1])),
            yaw_deg=float(raw.get("yaw_deg", 0.0)),
            is_transport_target=bool(raw.get("is_transport_target", False)),
            world_xyz_cm=(
                (float(world[0]), float(world[1]), float(world[2])) if world is not None else None
            ),
        )


@dataclass(frozen=True)
class SiteTransportRegistry:
    version: int
    layout_id: str
    seed: int
    region_size_cm: float
    robot_start_local_cm: Tuple[float, float]
    humanoid_local_cm: Tuple[float, float]
    material_yard_corner_cm: Tuple[float, float]
    material_pickup_local_cm: Tuple[float, float]
    material_actor_name: str
    carry_actor_name: str
    humanoid_actor_name: str
    forbidden_zones: Tuple[ForbiddenZone, ...]
    props: Tuple[SitePropSlot, ...]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": self.version,
            "layout_id": self.layout_id,
            "seed": self.seed,
            "region_size_cm": self.region_size_cm,
            "robot_start_local_cm": list(self.robot_start_local_cm),
            "humanoid_local_cm": list(self.humanoid_local_cm),
            "material_yard_corner_cm": list(self.material_yard_corner_cm),
            "material_pickup_local_cm": list(self.material_pickup_local_cm),
            "material_actor_name": self.material_actor_name,
            "carry_actor_name": self.carry_actor_name,
            "humanoid_actor_name": self.humanoid_actor_name,
            "forbidden_zones": [
                {
                    "zone_id": z.zone_id,
                    "rect_local_cm": list(z.rect_local_cm),
                    "note": z.note,
                }
                for z in self.forbidden_zones
            ],
            "props": [p.to_dict() for p in self.props],
        }

    @classmethod
    def from_dict(cls, raw: Dict[str, Any]) -> SiteTransportRegistry:
        props = tuple(SitePropSlot.from_dict(p) for p in raw["props"])
        zones = tuple(
            ForbiddenZone(
                zone_id=str(z["zone_id"]),
                rect_local_cm=tuple(float(v) for v in z["rect_local_cm"]),  # type: ignore[arg-type]
                note=str(z.get("note", "")),
            )
            for z in raw.get("forbidden_zones", [])
        )
        return cls(
            version=int(raw.get("version", 1)),
            layout_id=str(raw.get("layout_id", LAYOUT_ID)),
            seed=int(raw["seed"]),
            region_size_cm=float(raw.get("region_size_cm", REGION_SIZE_CM)),
            robot_start_local_cm=tuple(raw["robot_start_local_cm"]),  # type: ignore[arg-type]
            humanoid_local_cm=tuple(raw["humanoid_local_cm"]),  # type: ignore[arg-type]
            material_yard_corner_cm=tuple(raw.get("material_yard_corner_cm", MATERIAL_YARD_CORNER_CM)),  # type: ignore[arg-type]
            material_pickup_local_cm=tuple(raw["material_pickup_local_cm"]),  # type: ignore[arg-type]
            material_actor_name=str(raw.get("material_actor_name", MATERIAL_ACTOR_NAME)),
            carry_actor_name=str(raw.get("carry_actor_name", CARRY_ACTOR_NAME)),
            humanoid_actor_name=str(raw.get("humanoid_actor_name", HUMANOID_ACTOR_NAME)),
            forbidden_zones=zones,
            props=props,
        )

    def transport_slot(self) -> Optional[SitePropSlot]:
        for prop in self.props:
            if prop.is_transport_target:
                return prop
        return None


def mask_color_for_slot(slot_index: int) -> Tuple[int, int, int]:
    base = slot_index + 11
    return (
        (base * 43 + 17) % 190 + 35,
        (base * 61 + 29) % 190 + 35,
        (base * 71 + 31) % 190 + 35,
    )


def _catalog_by_bp_name():
    from prop_catalog import ensure_catalog  # noqa: WPS433

    return {e.bp_name: e for e in ensure_catalog()}


def build_registry(*, seed: int = DEFAULT_SEED, layout_id: str = LAYOUT_ID) -> SiteTransportRegistry:
    catalog = _catalog_by_bp_name()
    props: List[SitePropSlot] = []
    for idx, (bp_name, cluster, role, local_xy, yaw_deg, is_target) in enumerate(_CURATED_LAYOUT):
        if bp_name not in catalog:
            raise RuntimeError(f"missing catalog entry: {bp_name}")
        entry = catalog[bp_name]
        props.append(
            SitePropSlot(
                slot_id=f"{PROP_ACTOR_PREFIX}_{idx:03d}",
                bp_name=bp_name,
                bp_path=entry.bp_path,
                prop_type_id=entry.prop_type_id,
                mask_color_rgb=mask_color_for_slot(idx),
                cluster_id=cluster,
                role=role,
                local_xy_cm=local_xy,
                yaw_deg=yaw_deg,
                is_transport_target=is_target,
            )
        )
    _assert_in_region(props)
    transport = next(p for p in props if p.is_transport_target)
    return SiteTransportRegistry(
        version=1,
        layout_id=layout_id,
        seed=seed,
        region_size_cm=REGION_SIZE_CM,
        robot_start_local_cm=ROBOT_START_LOCAL_CM,
        humanoid_local_cm=HUMANOID_LOCAL_CM,
        material_yard_corner_cm=MATERIAL_YARD_CORNER_CM,
        material_pickup_local_cm=transport.local_xy_cm,
        material_actor_name=MATERIAL_ACTOR_NAME,
        carry_actor_name=CARRY_ACTOR_NAME,
        humanoid_actor_name=HUMANOID_ACTOR_NAME,
        forbidden_zones=FORBIDDEN_ZONES_LAYOUT_01,
        props=tuple(props),
    )


def _assert_in_region(props: Sequence[SitePropSlot]) -> None:
    margin = 80.0
    for prop in props:
        lx, ly = prop.local_xy_cm
        if not (margin <= lx <= REGION_SIZE_CM - margin and margin <= ly <= REGION_SIZE_CM - margin):
            raise ValueError(f"{prop.slot_id} @ {prop.local_xy_cm} outside {REGION_SIZE_CM}cm region")


def save_registry(registry: SiteTransportRegistry, path: Path = REGISTRY_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(registry.to_dict(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def load_registry(path: Path = REGISTRY_PATH) -> SiteTransportRegistry:
    return SiteTransportRegistry.from_dict(json.loads(path.read_text(encoding="utf-8")))


def ensure_registry(*, seed: int = DEFAULT_SEED, force_rebuild: bool = False) -> SiteTransportRegistry:
    if REGISTRY_PATH.is_file() and not force_rebuild:
        return load_registry()
    registry = build_registry(seed=seed)
    save_registry(registry)
    return registry


def to_placement_registry(registry: SiteTransportRegistry):
    from prop_placement import PlacementRegistry, PropPlacement  # noqa: WPS433

    props: List[PropPlacement] = []
    for idx, slot in enumerate(registry.props):
        actor_slot_id = (
            registry.material_actor_name if slot.is_transport_target else slot.slot_id
        )
        props.append(
            PropPlacement(
                slot_id=actor_slot_id,
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
    registry: SiteTransportRegistry,
    slot_id: str,
    world_xyz_cm: Tuple[float, float, float],
    *,
    local_xy_cm: Optional[Tuple[float, float]] = None,
) -> SiteTransportRegistry:
    updated: List[SitePropSlot] = []
    for prop in registry.props:
        if prop.slot_id != slot_id:
            updated.append(prop)
            continue
        updated.append(
            SitePropSlot(
                slot_id=prop.slot_id,
                bp_name=prop.bp_name,
                bp_path=prop.bp_path,
                prop_type_id=prop.prop_type_id,
                mask_color_rgb=prop.mask_color_rgb,
                cluster_id=prop.cluster_id,
                role=prop.role,
                local_xy_cm=local_xy_cm or prop.local_xy_cm,
                yaw_deg=prop.yaw_deg,
                is_transport_target=prop.is_transport_target,
                world_xyz_cm=world_xyz_cm,
            )
        )
    return SiteTransportRegistry(
        version=registry.version,
        layout_id=registry.layout_id,
        seed=registry.seed,
        region_size_cm=registry.region_size_cm,
        robot_start_local_cm=registry.robot_start_local_cm,
        humanoid_local_cm=registry.humanoid_local_cm,
        material_yard_corner_cm=registry.material_yard_corner_cm,
        material_pickup_local_cm=registry.material_pickup_local_cm,
        material_actor_name=registry.material_actor_name,
        carry_actor_name=registry.carry_actor_name,
        humanoid_actor_name=registry.humanoid_actor_name,
        forbidden_zones=registry.forbidden_zones,
        props=tuple(updated),
    )


def apply_mask_colors_from_placement(
    registry: SiteTransportRegistry,
    placement: "PlacementRegistry",
) -> SiteTransportRegistry:
    """Copy canonical mask RGB from synced placement registry into site slots."""
    from prop_placement import PlacementRegistry  # noqa: WPS433

    if not isinstance(placement, PlacementRegistry):
        raise TypeError("placement must be PlacementRegistry")
    by_slot = {p.slot_id: p for p in placement.props}
    merged: List[SitePropSlot] = []
    for prop in registry.props:
        synced = by_slot.get(prop.slot_id)
        rgb = prop.mask_color_rgb
        if synced is not None and synced.mask_color_canonical_rgb is not None:
            rgb = synced.mask_color_canonical_rgb
        merged.append(
            SitePropSlot(
                slot_id=prop.slot_id,
                bp_name=prop.bp_name,
                bp_path=prop.bp_path,
                prop_type_id=prop.prop_type_id,
                mask_color_rgb=rgb,
                cluster_id=prop.cluster_id,
                role=prop.role,
                local_xy_cm=prop.local_xy_cm,
                yaw_deg=prop.yaw_deg,
                is_transport_target=prop.is_transport_target,
                world_xyz_cm=prop.world_xyz_cm,
            )
        )
    return SiteTransportRegistry(
        version=registry.version,
        layout_id=registry.layout_id,
        seed=registry.seed,
        region_size_cm=registry.region_size_cm,
        robot_start_local_cm=registry.robot_start_local_cm,
        humanoid_local_cm=registry.humanoid_local_cm,
        material_yard_corner_cm=registry.material_yard_corner_cm,
        material_pickup_local_cm=registry.material_pickup_local_cm,
        material_actor_name=registry.material_actor_name,
        carry_actor_name=registry.carry_actor_name,
        humanoid_actor_name=registry.humanoid_actor_name,
        forbidden_zones=registry.forbidden_zones,
        props=tuple(merged),
    )
