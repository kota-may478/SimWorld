#!/usr/bin/env python3
"""Measure SpotDog horizontal body radius (PIE + NavQueryService GetActorBoundsJson).

Uses colliding AABB (half_*) and CapsuleComponent radius when available.
All-components bounds (half_*_all) often include Sight / debug meshes and are ignored.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import nav_query as nq  # noqa: E402

ROBOT = geh.ROBOT_ACTOR_NAME
_CONFIG_PATH = THIS_DIR / "scenarios" / "site_transport_20m" / "navmesh_config.py"
_MAX_TRUSTED_COLLIDING_HALF_CM = 120.0


def _pawn_xy(ucv, actor: str) -> tuple[float, float]:
    loc = ucv.get_location(actor)
    return float(loc[0]), float(loc[1])


def _body_radius_from_bounds(
    pawn_xy: tuple[float, float],
    *,
    cx: float,
    cy: float,
    half_x: float,
    half_y: float,
) -> dict[str, float]:
    px, py = pawn_xy
    corners = (
        (cx - half_x, cy - half_y),
        (cx - half_x, cy + half_y),
        (cx + half_x, cy - half_y),
        (cx + half_x, cy + half_y),
    )
    corner_dist = max(math.hypot(px - x, py - y) for x, y in corners)
    max_half = max(half_x, half_y)
    hypot_half = math.hypot(half_x, half_y)
    offset = math.hypot(px - cx, py - cy)
    offset_inflated = max(half_x + abs(px - cx), half_y + abs(py - cy))
    return {
        "corner_max_cm": corner_dist,
        "max_half_cm": max_half,
        "hypot_half_cm": hypot_half,
        "offset_from_bounds_center_cm": offset,
        "offset_inflated_max_half_cm": offset_inflated,
    }


def _recommend_body_radius_cm(
    *,
    capsule_radius_cm: float | None,
    colliding_radii: dict[str, float] | None,
) -> tuple[float, str]:
    """Return (radius_cm, method_label)."""
    if capsule_radius_cm is not None and capsule_radius_cm > 0.0:
        # Quadruped legs extend beyond the capsule; 2× matches prior L2 exclude (~70 cm).
        doubled = capsule_radius_cm * 2.0
        return round(doubled, 1), f"2x_capsule({capsule_radius_cm:.1f}cm)"

    if colliding_radii is not None:
        max_half = colliding_radii["max_half_cm"]
        if max_half <= _MAX_TRUSTED_COLLIDING_HALF_CM:
            return (
                round(colliding_radii["offset_inflated_max_half_cm"], 1),
                "colliding_aabb_offset_inflated",
            )

    raise RuntimeError(
        "Could not derive body radius: rebuild NavQueryService (colliding + capsule fields) "
        "or fix SpotDog collision primitives."
    )


def _apply_to_navmesh_config(radius_cm: float) -> None:
    text = _CONFIG_PATH.read_text(encoding="utf-8")
    new_text, n = re.subn(
        r"^SPOTDOG_BODY_RADIUS_CM = [0-9.]+$",
        f"SPOTDOG_BODY_RADIUS_CM = {radius_cm:.1f}",
        text,
        count=1,
        flags=re.MULTILINE,
    )
    if n != 1:
        raise RuntimeError(f"failed to patch SPOTDOG_BODY_RADIUS_CM in {_CONFIG_PATH}")
    _CONFIG_PATH.write_text(new_text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Write SPOTDOG_BODY_RADIUS_CM into navmesh_config.py",
    )
    args = parser.parse_args()

    ucv, _ = g10k.ensure_connection()
    try:
        nav_actor = nq.find_nav_query_actor(ucv)
        if nav_actor is None:
            print("FAIL: NavQueryService not found")
            return 1
        if ROBOT not in geh.actor_names(ucv):
            print(f"FAIL: robot {ROBOT!r} not in level")
            return 1

        raw = nq.get_actor_bounds(ucv, nav_actor, ROBOT)
        if not raw.get("ok"):
            print(f"FAIL: GetActorBoundsJson: {raw}")
            return 1

        pawn_xy = _pawn_xy(ucv, ROBOT)
        cx = float(raw["cx"])
        cy = float(raw["cy"])
        half_x = float(raw["half_x"])
        half_y = float(raw["half_y"])

        colliding_radii = _body_radius_from_bounds(
            pawn_xy,
            cx=cx,
            cy=cy,
            half_x=half_x,
            half_y=half_y,
        )

        capsule_raw = raw.get("capsule_radius_cm")
        capsule_radius_cm = (
            float(capsule_raw) if capsule_raw is not None and float(capsule_raw) > 0.0 else None
        )

        recommended, method = _recommend_body_radius_cm(
            capsule_radius_cm=capsule_radius_cm,
            colliding_radii=colliding_radii,
        )

        all_bounds = None
        if "half_x_all" in raw:
            all_bounds = {
                "cx": float(raw["cx_all"]),
                "cy": float(raw["cy_all"]),
                "half_x": float(raw["half_x_all"]),
                "half_y": float(raw["half_y_all"]),
                "half_z": float(raw.get("half_z_all", 0.0)),
            }

        out = {
            "robot": ROBOT,
            "pawn_xy": list(pawn_xy),
            "bounds_colliding": {
                "cx": cx,
                "cy": cy,
                "half_x": half_x,
                "half_y": half_y,
                "half_z": float(raw.get("half_z", 0.0)),
            },
            "bounds_all_components": all_bounds,
            "capsule_radius_cm": capsule_radius_cm,
            "capsule_half_height_cm": float(raw.get("capsule_half_height_cm", -1.0)),
            "radii_colliding": colliding_radii,
            "recommended_body_radius_cm": recommended,
            "recommendation_method": method,
            "planning_agent_radius_cm": round(100.0 + recommended, 1),
        }
        print(json.dumps(out, indent=2))

        if args.apply:
            _apply_to_navmesh_config(recommended)
            print(f"Updated {_CONFIG_PATH}: SPOTDOG_BODY_RADIUS_CM={recommended:.1f}")
    finally:
        geh.release_connection(ucv)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
