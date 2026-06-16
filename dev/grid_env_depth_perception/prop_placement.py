#!/usr/bin/env python3
"""Fixed prop selection and placement registry for depth perception tests."""

from __future__ import annotations

import json
import math
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

THIS_DIR = Path(__file__).resolve().parent
NAV_DIR = THIS_DIR.parent / "grid_env_level_nav"
CACHE_DIR = THIS_DIR / "cache"
REGISTRY_PATH = CACHE_DIR / "prop_placement_registry.json"

REGION_X_MAX_CM = 3000.0
REGION_Y_MAX_CM = 3000.0
EXCLUSION_CM = 500.0
SPOTDOG_SPAWN_LOCAL_CM = (100.0, 100.0)
DEFAULT_SEED = 42
DEFAULT_PROP_COUNT = 5
PROP_ACTOR_PREFIX = "depth_test_prop"
NAV_XY_TOLERANCE_CM = 120.0


@dataclass(frozen=True)
class PropPlacement:
    slot_id: str
    catalog_index: int
    prop_type_id: str
    bp_name: str
    bp_path: str
    mask_color_rgb: Tuple[int, int, int]
    local_xy_cm: Tuple[float, float]
    world_xyz_cm: Optional[Tuple[float, float, float]] = None
    visit_order: Optional[int] = None
    # Colors assigned at spawn via vset /object/.../color (RGB).
    mask_color_set_rgb: Optional[Tuple[int, int, int]] = None
    # Canonical RGB from vget /object/{name}/color after spawn (Approach C).
    mask_color_canonical_rgb: Optional[Tuple[int, int, int]] = None
    # Deprecated: one-pose calibration — do not use for detection.
    mask_color_observed_bgr: Optional[Tuple[int, int, int]] = None
    # Lit appearance at standoff (depth-gated); used when object_mask is inactive.
    lit_color_observed_bgr: Optional[Tuple[int, int, int]] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["local_xy_cm"] = list(self.local_xy_cm)
        d["mask_color_rgb"] = list(self.mask_color_rgb)
        if self.world_xyz_cm is not None:
            d["world_xyz_cm"] = list(self.world_xyz_cm)
        if self.mask_color_set_rgb is not None:
            d["mask_color_set_rgb"] = list(self.mask_color_set_rgb)
        if self.mask_color_canonical_rgb is not None:
            d["mask_color_canonical_rgb"] = list(self.mask_color_canonical_rgb)
        if self.mask_color_observed_bgr is not None:
            d["mask_color_observed_bgr"] = list(self.mask_color_observed_bgr)
        if self.lit_color_observed_bgr is not None:
            d["lit_color_observed_bgr"] = list(self.lit_color_observed_bgr)
        return d

    @classmethod
    def from_dict(cls, raw: Dict[str, Any]) -> "PropPlacement":
        world = raw.get("world_xyz_cm")
        set_rgb = raw.get("mask_color_set_rgb")
        canonical = raw.get("mask_color_canonical_rgb")
        obs_bgr = raw.get("mask_color_observed_bgr")
        lit_bgr = raw.get("lit_color_observed_bgr")
        return cls(
            slot_id=str(raw["slot_id"]),
            catalog_index=int(raw["catalog_index"]),
            prop_type_id=str(raw["prop_type_id"]),
            bp_name=str(raw["bp_name"]),
            bp_path=str(raw["bp_path"]),
            mask_color_rgb=tuple(int(v) for v in raw["mask_color_rgb"]),  # type: ignore[arg-type]
            local_xy_cm=(float(raw["local_xy_cm"][0]), float(raw["local_xy_cm"][1])),
            world_xyz_cm=(
                (float(world[0]), float(world[1]), float(world[2])) if world is not None else None
            ),
            visit_order=int(raw["visit_order"]) if raw.get("visit_order") is not None else None,
            mask_color_set_rgb=(
                tuple(int(v) for v in set_rgb) if set_rgb is not None else None  # type: ignore[arg-type]
            ),
            mask_color_canonical_rgb=(
                tuple(int(v) for v in canonical) if canonical is not None else None  # type: ignore[arg-type]
            ),
            mask_color_observed_bgr=(
                tuple(int(v) for v in obs_bgr) if obs_bgr is not None else None  # type: ignore[arg-type]
            ),
            lit_color_observed_bgr=(
                tuple(int(v) for v in lit_bgr) if lit_bgr is not None else None  # type: ignore[arg-type]
            ),
        )

    def detection_bgr(self) -> Tuple[int, int, int]:
        """BGR tuple used to match object_mask pixels."""
        if self.mask_color_observed_bgr is not None:
            return self.mask_color_observed_bgr
        if self.mask_color_canonical_rgb is not None:
            r, g, b = self.mask_color_canonical_rgb
            return (b, g, r)
        if self.mask_color_set_rgb is not None:
            r, g, b = self.mask_color_set_rgb
            return (b, g, r)
        rgb = self.mask_color_rgb
        return (rgb[2], rgb[1], rgb[0])

    def detection_lit_bgr(self) -> Optional[Tuple[int, int, int]]:
        return self.lit_color_observed_bgr


def _copy_prop(prop: PropPlacement, **kwargs) -> PropPlacement:
    return PropPlacement(
        slot_id=kwargs.get("slot_id", prop.slot_id),
        catalog_index=kwargs.get("catalog_index", prop.catalog_index),
        prop_type_id=kwargs.get("prop_type_id", prop.prop_type_id),
        bp_name=kwargs.get("bp_name", prop.bp_name),
        bp_path=kwargs.get("bp_path", prop.bp_path),
        mask_color_rgb=kwargs.get("mask_color_rgb", prop.mask_color_rgb),
        local_xy_cm=kwargs.get("local_xy_cm", prop.local_xy_cm),
        world_xyz_cm=kwargs.get("world_xyz_cm", prop.world_xyz_cm),
        visit_order=kwargs.get("visit_order", prop.visit_order),
        mask_color_set_rgb=kwargs.get("mask_color_set_rgb", prop.mask_color_set_rgb),
        mask_color_canonical_rgb=kwargs.get("mask_color_canonical_rgb", prop.mask_color_canonical_rgb),
        mask_color_observed_bgr=kwargs.get("mask_color_observed_bgr", prop.mask_color_observed_bgr),
        lit_color_observed_bgr=kwargs.get("lit_color_observed_bgr", prop.lit_color_observed_bgr),
    )


@dataclass(frozen=True)
class PlacementRegistry:
    version: int
    seed: int
    prop_count: int
    region_x_max_cm: float
    region_y_max_cm: float
    exclusion_cm: float
    spotdog_spawn_local_cm: Tuple[float, float]
    props: Tuple[PropPlacement, ...]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": self.version,
            "seed": self.seed,
            "prop_count": self.prop_count,
            "region_x_max_cm": self.region_x_max_cm,
            "region_y_max_cm": self.region_y_max_cm,
            "exclusion_cm": self.exclusion_cm,
            "spotdog_spawn_local_cm": list(self.spotdog_spawn_local_cm),
            "props": [p.to_dict() for p in self.props],
        }

    @classmethod
    def from_dict(cls, raw: Dict[str, Any]) -> "PlacementRegistry":
        props = tuple(PropPlacement.from_dict(p) for p in raw["props"])
        return cls(
            version=int(raw.get("version", 1)),
            seed=int(raw["seed"]),
            prop_count=int(raw.get("prop_count", len(props))),
            region_x_max_cm=float(raw.get("region_x_max_cm", REGION_X_MAX_CM)),
            region_y_max_cm=float(raw.get("region_y_max_cm", REGION_Y_MAX_CM)),
            exclusion_cm=float(raw.get("exclusion_cm", EXCLUSION_CM)),
            spotdog_spawn_local_cm=(
                float(raw["spotdog_spawn_local_cm"][0]),
                float(raw["spotdog_spawn_local_cm"][1]),
            ),
            props=props,
        )

    def visit_order_props(self) -> Tuple[PropPlacement, ...]:
        return tuple(sorted(self.props, key=lambda p: (p.visit_order or 999, p.slot_id)))

    def prop_by_type_id(self, prop_type_id: str) -> Optional[PropPlacement]:
        for prop in self.props:
            if prop.prop_type_id == prop_type_id:
                return prop
        return None


def _load_catalog_entries():
    import sys

    nav_path = str(NAV_DIR)
    if nav_path not in sys.path:
        sys.path.insert(0, nav_path)
    from prop_catalog import ensure_catalog  # noqa: WPS433

    return ensure_catalog()


def mask_color_for_slot(slot_index: int) -> Tuple[int, int, int]:
    """Distinct non-black RGB for object_mask segmentation."""
    base = slot_index + 1
    return (
        (base * 37 + 11) % 200 + 30,
        (base * 53 + 17) % 200 + 30,
        (base * 71 + 23) % 200 + 30,
    )


def _in_exclusion_zone(lx: float, ly: float, exclusion_cm: float) -> bool:
    return lx < exclusion_cm and ly < exclusion_cm


def _candidate_local_positions(
    count: int,
    *,
    seed: int,
    region_x_max_cm: float,
    region_y_max_cm: float,
    exclusion_cm: float,
) -> List[Tuple[float, float]]:
    rng = random.Random(seed)
    margin = 250.0
    xs = [
        margin + (region_x_max_cm - 2.0 * margin) * i / max(count - 1, 1)
        for i in range(count)
    ]
    rng.shuffle(xs)
    positions: List[Tuple[float, float]] = []
    for i in range(count):
        for attempt in range(80):
            jitter_x = rng.uniform(-350.0, 350.0)
            jitter_y = rng.uniform(-350.0, 350.0)
            lx = min(max(xs[i] + jitter_x, margin), region_x_max_cm - margin)
            ly = min(
                max(margin + rng.uniform(0.0, region_y_max_cm - 2.0 * margin) + jitter_y, margin),
                region_y_max_cm - margin,
            )
            if _in_exclusion_zone(lx, ly, exclusion_cm):
                continue
            positions.append((lx, ly))
            break
        else:
            lx = exclusion_cm + 200.0 + i * 420.0
            ly = exclusion_cm + 300.0 + (i % 3) * 520.0
            positions.append((lx, ly))
    return positions


def build_placement_registry(
    *,
    seed: int = DEFAULT_SEED,
    prop_count: int = DEFAULT_PROP_COUNT,
    region_x_max_cm: float = REGION_X_MAX_CM,
    region_y_max_cm: float = REGION_Y_MAX_CM,
    exclusion_cm: float = EXCLUSION_CM,
    spotdog_spawn_local_cm: Tuple[float, float] = SPOTDOG_SPAWN_LOCAL_CM,
) -> PlacementRegistry:
    catalog = _load_catalog_entries()
    if len(catalog) < prop_count:
        raise ValueError(f"catalog has {len(catalog)} entries, need {prop_count}")

    rng = random.Random(seed)
    indices = sorted(rng.sample(range(len(catalog)), prop_count))
    local_positions = _candidate_local_positions(
        prop_count,
        seed=seed + 1000,
        region_x_max_cm=region_x_max_cm,
        region_y_max_cm=region_y_max_cm,
        exclusion_cm=exclusion_cm,
    )

    props: List[PropPlacement] = []
    for slot, (cat_idx, (lx, ly)) in enumerate(zip(indices, local_positions), start=1):
        entry = catalog[cat_idx]
        props.append(
            PropPlacement(
                slot_id=f"{PROP_ACTOR_PREFIX}_{slot:03d}",
                catalog_index=cat_idx,
                prop_type_id=entry.prop_type_id,
                bp_name=entry.bp_name,
                bp_path=entry.bp_path,
                mask_color_rgb=mask_color_for_slot(slot),
                local_xy_cm=(lx, ly),
                mask_color_set_rgb=mask_color_for_slot(slot),
            )
        )

    ordered = _assign_visit_order(props, spotdog_spawn_local_cm)
    return PlacementRegistry(
        version=1,
        seed=seed,
        prop_count=prop_count,
        region_x_max_cm=region_x_max_cm,
        region_y_max_cm=region_y_max_cm,
        exclusion_cm=exclusion_cm,
        spotdog_spawn_local_cm=spotdog_spawn_local_cm,
        props=tuple(ordered),
    )


def _assign_visit_order(
    props: Sequence[PropPlacement],
    start_local_cm: Tuple[float, float],
) -> Tuple[PropPlacement, ...]:
    sx, sy = start_local_cm
    ranked = sorted(
        props,
        key=lambda p: (math.hypot(p.local_xy_cm[0] - sx, p.local_xy_cm[1] - sy), p.slot_id),
    )
    out: List[PropPlacement] = []
    for order, prop in enumerate(ranked, start=1):
        out.append(_copy_prop(prop, visit_order=order))
    return tuple(out)


def save_registry(registry: PlacementRegistry, path: Path = REGISTRY_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(registry.to_dict(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def load_registry(path: Path = REGISTRY_PATH) -> PlacementRegistry:
    if not path.is_file():
        raise FileNotFoundError(f"placement registry not found: {path}")
    return PlacementRegistry.from_dict(json.loads(path.read_text(encoding="utf-8")))


def ensure_registry(
    path: Path = REGISTRY_PATH,
    *,
    seed: int = DEFAULT_SEED,
    force_rebuild: bool = False,
) -> PlacementRegistry:
    if path.is_file() and not force_rebuild:
        return load_registry(path)
    registry = build_placement_registry(seed=seed)
    save_registry(registry, path)
    return registry


def update_prop_world_pose(
    registry: PlacementRegistry,
    slot_id: str,
    world_xyz_cm: Tuple[float, float, float],
    *,
    local_xy_cm: Optional[Tuple[float, float]] = None,
) -> PlacementRegistry:
    updated: List[PropPlacement] = []
    for prop in registry.props:
        if prop.slot_id != slot_id:
            updated.append(prop)
            continue
        updated.append(
            _copy_prop(
                prop,
                local_xy_cm=local_xy_cm or prop.local_xy_cm,
                world_xyz_cm=world_xyz_cm,
            )
        )
    return PlacementRegistry(
        version=registry.version,
        seed=registry.seed,
        prop_count=registry.prop_count,
        region_x_max_cm=registry.region_x_max_cm,
        region_y_max_cm=registry.region_y_max_cm,
        exclusion_cm=registry.exclusion_cm,
        spotdog_spawn_local_cm=registry.spotdog_spawn_local_cm,
        props=tuple(updated),
    )


def finalize_registry_after_spawn(registry: PlacementRegistry) -> PlacementRegistry:
    ordered = _assign_visit_order(registry.props, registry.spotdog_spawn_local_cm)
    return PlacementRegistry(
        version=registry.version,
        seed=registry.seed,
        prop_count=registry.prop_count,
        region_x_max_cm=registry.region_x_max_cm,
        region_y_max_cm=registry.region_y_max_cm,
        exclusion_cm=registry.exclusion_cm,
        spotdog_spawn_local_cm=registry.spotdog_spawn_local_cm,
        props=ordered,
    )
