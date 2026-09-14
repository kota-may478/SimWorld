"""UE5 Level-map helpers for the Stage-1 3F scaffold."""

from ue.layout import (
    ANCHOR_LOCAL_XY_CM,
    ENTRANCE_LOCAL_XY_CM,
    STAGING_LOCAL_XY_CM,
    cube_scale,
    footprint_clears_site20,
    footprint_inside_level_work,
    scaffold_xyz_to_local_cm,
)
from ue.placement import all_spawn_boxes

__all__ = [
    "ANCHOR_LOCAL_XY_CM",
    "ENTRANCE_LOCAL_XY_CM",
    "STAGING_LOCAL_XY_CM",
    "all_spawn_boxes",
    "cube_scale",
    "footprint_clears_site20",
    "footprint_inside_level_work",
    "scaffold_xyz_to_local_cm",
]
