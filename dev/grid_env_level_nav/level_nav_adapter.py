#!/usr/bin/env python3
"""Adapter: material transport FSM ↔ LayeredCostmap navigation on Level map."""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple

_THIS_DIR = Path(__file__).resolve().parent
_ROOT = _THIS_DIR.parent.parent
_MT = _ROOT / "dev" / "llm_material_transport"
for _p in (_THIS_DIR, _ROOT / "dev" / "grid_env_hri", _ROOT / "dev" / "grid_env_10k", _MT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from costmap_layers import LayeredCostmap  # noqa: E402
from level_coords import REGION_ORIGIN_WORLD_XY, local_xy_to_world  # noqa: E402
from path_planning_costmap import Costmap2D  # noqa: E402
from spotdog_nav_follower import navigate_world_xy  # noqa: E402
from zone_catalog import ZoneCatalog, catalog_to_zone_registry  # noqa: E402
from zone_registry import ZoneRegistry  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

# Level map spawn frame (see #320 §7-1)
LEVEL_MAP_ORIGIN_XY = REGION_ORIGIN_WORLD_XY
LEVEL_GRID_SPAWN_Z_CM = 6490.0

WorldXY = Tuple[float, float]
LocalXY = Tuple[float, float]


@dataclass
class LevelNavSession:
    """Holds layered costmap + zone registry for one material-transport run."""

    layers: LayeredCostmap
    registry: Optional[ZoneRegistry] = None
    catalog: Optional[ZoneCatalog] = None
    _closed: set[str] = field(default_factory=set)

    @classmethod
    def from_cache(
        cls,
        l0_path: Path | str,
        zone_catalog_path: Optional[Path | str] = None,
        *,
        zone_registry_path: Optional[Path | str] = None,
    ) -> LevelNavSession:
        layers = LayeredCostmap.from_l0_cache(l0_path)
        catalog = ZoneCatalog.load(zone_catalog_path) if zone_catalog_path else None
        if catalog is not None:
            registry = catalog_to_zone_registry(catalog, layers.resolution_cm)
        elif zone_registry_path:
            registry = ZoneRegistry.load(zone_registry_path)
        else:
            registry = None
        return cls(layers=layers, registry=registry, catalog=catalog)

    def close_zone(self, zone_id: str) -> int:
        if self.registry is None:
            raise RuntimeError("zone registry not loaded")
        n = self.layers.close_zone(zone_id, self.registry)
        self._closed.add(zone_id)
        return n

    def open_zone(self, zone_id: str) -> int:
        if self.registry is None:
            raise RuntimeError("zone registry not loaded")
        n = self.layers.open_zone(zone_id, self.registry)
        self._closed.discard(zone_id)
        return n

    def plan_local(self, start_local: LocalXY, goal_local: LocalXY):
        return self.layers.plan_astar_local(start_local, goal_local)

    def merged_costmap(self) -> Costmap2D:
        return self.layers.to_costmap2d()

    def navigate_robot_local(
        self,
        ucv: UnrealCV,
        goal_local: LocalXY,
        *,
        label: str = "",
    ) -> bool:
        goal_xy = local_xy_to_world(*goal_local)
        return navigate_world_xy(ucv, self.layers, goal_xy, label=label)

    def navigate_robot_world(
        self,
        ucv: UnrealCV,
        goal_xy: WorldXY,
        *,
        label: str = "",
    ) -> bool:
        return navigate_world_xy(ucv, self.layers, goal_xy, label=label)


def local_m_to_world_xy(lx_m: float, ly_m: float) -> WorldXY:
    return local_xy_to_world(lx_m * 100.0, ly_m * 100.0)


def apply_instruction_to_zones(session: LevelNavSession, instruction: str) -> Optional[str]:
    """
    Minimal keyword mapping for HRC instructions (Phase 7 MVP).

    Returns zone_id if handled, else None.
    """
    text = instruction.strip().lower()
    if "room d" in text and ("封鎖" in instruction or "通行" in instruction or "closed" in text):
        if "解除" in instruction or "open" in text:
            session.open_zone("RoomD")
            return "RoomD:opened"
        session.close_zone("RoomD")
        return "RoomD:closed"
    return None
