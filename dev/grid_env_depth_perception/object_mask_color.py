#!/usr/bin/env python3
"""UnrealCV object_mask color IDs — canonical vget /object/{name}/color (Approach C)."""

from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import List, Optional, Tuple

_THIS_DIR = Path(__file__).resolve().parent
_GEH_DIR = _THIS_DIR.parent / "grid_env_hri"
if str(_GEH_DIR) not in sys.path:
    sys.path.insert(0, str(_GEH_DIR))

import grid_env_hri_simulation as geh  # noqa: E402
from prop_placement import (  # noqa: E402
    PlacementRegistry,
    PropPlacement,
    _copy_prop,
    save_registry,
)
from pie_safety import require_live_ucv, tick_settle  # noqa: E402

RGB = Tuple[int, int, int]
_COLOR_RE = re.compile(
    r"R\s*=\s*(\d+)\s*,\s*G\s*=\s*(\d+)\s*,\s*B\s*=\s*(\d+)",
    re.IGNORECASE,
)


def parse_unreal_color_response(raw: str) -> Optional[RGB]:
    """Parse UnrealCV color e.g. '(R=152,G=206,B=66,A=255)'."""
    if not raw:
        return None
    match = _COLOR_RE.search(str(raw).strip())
    if not match:
        parts = str(raw).replace(",", " ").split()
        ints: List[int] = []
        for token in parts:
            try:
                ints.append(int(token))
            except ValueError:
                continue
        if len(ints) >= 3:
            return ints[0], ints[1], ints[2]
        return None
    return int(match.group(1)), int(match.group(2)), int(match.group(3))


def get_object_mask_color_rgb(ucv, actor_name: str) -> Optional[RGB]:
    """Query canonical segmentation color assigned to actor."""
    require_live_ucv(ucv, context=f"vget color {actor_name}")
    raw = geh._ue_request(ucv, f"vget /object/{actor_name}/color", timeout_s=15.0)  # noqa: SLF001
    if raw is None:
        return None
    return parse_unreal_color_response(str(raw))


def set_object_mask_color_rgb(ucv, actor_name: str, rgb: RGB) -> None:
    require_live_ucv(ucv, context=f"vset color {actor_name}")
    ucv.set_color(actor_name, list(rgb))
    tick_settle(ucv, settle_s=0.2, ticks=1)


def ensure_prop_mask_color(
    ucv,
    prop: PropPlacement,
    *,
    reapply_if_missing: bool = True,
) -> Optional[RGB]:
    """Return canonical mask RGB from UE; optionally vset intended color first."""
    intended = prop.mask_color_set_rgb or prop.mask_color_rgb
    if not geh.actor_exists(ucv, prop.slot_id):
        return None
    canonical = get_object_mask_color_rgb(ucv, prop.slot_id)
    if canonical is None and reapply_if_missing:
        set_object_mask_color_rgb(ucv, prop.slot_id, intended)
        canonical = get_object_mask_color_rgb(ucv, prop.slot_id)
    return canonical


def sync_registry_mask_colors(
    ucv,
    registry: PlacementRegistry,
    *,
    reapply_colors: bool = False,
) -> PlacementRegistry:
    """Refresh mask_color_canonical_rgb from UE for all props."""
    updated: List[PropPlacement] = []
    for prop in registry.props:
        if reapply_colors and geh.actor_exists(ucv, prop.slot_id):
            set_object_mask_color_rgb(
                ucv,
                prop.slot_id,
                prop.mask_color_set_rgb or prop.mask_color_rgb,
            )
        canonical = ensure_prop_mask_color(ucv, prop, reapply_if_missing=reapply_colors)
        if canonical is None:
            print(f"[MaskColor] WARN: no color for {prop.slot_id}")
            updated.append(prop)
            continue
        print(f"[MaskColor] {prop.slot_id} canonical RGB={canonical}")
        updated.append(
            _copy_prop(
                prop,
                mask_color_canonical_rgb=canonical,
                mask_color_rgb=canonical,
            )
        )
    out = PlacementRegistry(
        version=registry.version,
        seed=registry.seed,
        prop_count=registry.prop_count,
        region_x_max_cm=registry.region_x_max_cm,
        region_y_max_cm=registry.region_y_max_cm,
        exclusion_cm=registry.exclusion_cm,
        spotdog_spawn_local_cm=registry.spotdog_spawn_local_cm,
        props=tuple(updated),
    )
    save_registry(out)
    return out
