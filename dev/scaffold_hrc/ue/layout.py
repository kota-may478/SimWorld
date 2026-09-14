"""Map Stage-1 scaffold metres onto /Game/Maps/Level local centimetres.

Logistics story (paper HRC):
  - Yard (truck + humanoid handover) near local (0, 0)
  - Scaffold entrance near local (0, +10 m)
  - Deck runs along +local Y so Spot carries along Y then climbs

Scaffold x (length) → local Y; scaffold y (width) → local X.
"""

from __future__ import annotations

from typing import Tuple

from scene.geometry import STAGE1_GEOM, ScaffoldGeom

LEVEL_WORK_SIZE_CM: Tuple[float, float] = (7000.0, 7900.0)
SITE20_SIZE_CM = 2000.0
CUBE_NATIVE_CM = 30.0
# Entrance of the 10 m deck (scaffold x=0, y=0) in Level local cm.
ENTRANCE_LOCAL_XY_CM: Tuple[float, float] = (0.0, 1400.0)
# Yard toward the stair run (local Y). Stair yard is ~640 cm.
STAGING_LOCAL_XY_CM: Tuple[float, float] = (150.0, 280.0)
# Kept as a name for older tests / plots: local origin of scaffold x=0, y=0.
ANCHOR_LOCAL_XY_CM = ENTRANCE_LOCAL_XY_CM


def scaffold_xy_to_local_cm(
    x_m: float,
    y_m: float,
    *,
    geom: ScaffoldGeom = STAGE1_GEOM,
    entrance_xy_cm: Tuple[float, float] = ENTRANCE_LOCAL_XY_CM,
) -> Tuple[float, float]:
    _ = geom
    ex, ey = entrance_xy_cm
    return ex + y_m * 100.0, ey + x_m * 100.0


def scaffold_z_to_local_cm(z_m: float) -> float:
    return z_m * 100.0


def local_cm_to_scaffold_xyz(
    lx: float,
    ly: float,
    lz: float,
    *,
    entrance_xy_cm: Tuple[float, float] = ENTRANCE_LOCAL_XY_CM,
) -> Tuple[float, float, float]:
    """Inverse of scaffold_xyz_to_local_cm (scaffold metres)."""
    ex, ey = entrance_xy_cm
    y_m = (lx - ex) / 100.0
    x_m = (ly - ey) / 100.0
    z_m = lz / 100.0
    return x_m, y_m, z_m


def scaffold_xyz_to_local_cm(
    x_m: float,
    y_m: float,
    z_m: float,
    *,
    geom: ScaffoldGeom = STAGE1_GEOM,
    entrance_xy_cm: Tuple[float, float] = ENTRANCE_LOCAL_XY_CM,
) -> Tuple[float, float, float]:
    lx, ly = scaffold_xy_to_local_cm(
        x_m, y_m, geom=geom, entrance_xy_cm=entrance_xy_cm
    )
    return lx, ly, scaffold_z_to_local_cm(z_m)


def cube_scale(sx_m: float, sy_m: float, sz_m: float) -> Tuple[float, float, float]:
    native_m = CUBE_NATIVE_CM / 100.0
    return (sx_m / native_m, sy_m / native_m, sz_m / native_m)


def cube_scale_level_world(sx_m: float, sy_m: float, sz_m: float) -> Tuple[float, float, float]:
    """Actor XYZ scale on Level after 90° map: world X follows scaffold X."""
    return cube_scale(sx_m, sy_m, sz_m)


def footprint_local_cm(
    *,
    geom: ScaffoldGeom = STAGE1_GEOM,
    entrance_xy_cm: Tuple[float, float] = ENTRANCE_LOCAL_XY_CM,
) -> Tuple[float, float, float, float]:
    """Inclusive local XY covering stair run, deck, and staging."""
    x0 = geom.stair_xy_bounds()[0] - 0.5
    x1 = geom.deck_length_m
    y0 = 0.0
    y1 = geom.deck_width_m
    sx, sy = geom.storage_xy()
    corners = [
        scaffold_xy_to_local_cm(x0, y0, geom=geom, entrance_xy_cm=entrance_xy_cm),
        scaffold_xy_to_local_cm(x0, y1, geom=geom, entrance_xy_cm=entrance_xy_cm),
        scaffold_xy_to_local_cm(x1, y0, geom=geom, entrance_xy_cm=entrance_xy_cm),
        scaffold_xy_to_local_cm(x1, y1, geom=geom, entrance_xy_cm=entrance_xy_cm),
        STAGING_LOCAL_XY_CM,
        scaffold_xy_to_local_cm(sx, sy, geom=geom, entrance_xy_cm=entrance_xy_cm),
    ]
    lxs = [c[0] for c in corners]
    lys = [c[1] for c in corners]
    return (min(lxs), min(lys), max(lxs), max(lys))


def footprint_clears_site20(
    *,
    geom: ScaffoldGeom = STAGE1_GEOM,
    entrance_xy_cm: Tuple[float, float] = ENTRANCE_LOCAL_XY_CM,
) -> bool:
    """False by design: yard and entrance sit on the site20 strip near local X=0."""
    _ = geom
    _ = entrance_xy_cm
    return False


def footprint_inside_level_work(
    *,
    geom: ScaffoldGeom = STAGE1_GEOM,
    entrance_xy_cm: Tuple[float, float] = ENTRANCE_LOCAL_XY_CM,
) -> bool:
    x0, y0, x1, y1 = footprint_local_cm(geom=geom, entrance_xy_cm=entrance_xy_cm)
    wx, wy = LEVEL_WORK_SIZE_CM
    return 0.0 <= x0 and 0.0 <= y0 and x1 <= wx and y1 <= wy
