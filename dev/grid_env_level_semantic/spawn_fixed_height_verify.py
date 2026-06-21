#!/usr/bin/env python3
"""Spawn verification cubes at a fixed world XY + block-bottom Z (no height adjust).

Default: camera sample (1943.07, 3093.64) cm, bottom Z = 6500 cm (65 m).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Iterator, List, Tuple

THIS_DIR = Path(__file__).resolve().parent
GEH_DIR = THIS_DIR.parent / "grid_env_hri"
G10K_DIR = THIS_DIR.parent / "grid_env_10k"
for p in (THIS_DIR, GEH_DIR, G10K_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_camera_probe as lcp  # noqa: E402
from level_region import BLOCK_BOTTOM_Z_CM, CELL_SIZE_CM  # noqa: E402

# Latest camera-based check point (user sample).
VERIFY_CENTER_XY_CM = (1943.07, 3093.64)
VERIFY_BOTTOM_Z_CM = BLOCK_BOTTOM_Z_CM  # 6500 = 65 m
VERIFY_PREFIX = "level_sem_verify"
POST_SPAWN_S = 0.25


def _block_bottom_to_actor_z(bottom_z_cm: float) -> float:
    if geh.CUBE_PIVOT_AT_CENTER:
        return bottom_z_cm + geh.CUBE_HALF_CM
    return bottom_z_cm


def iter_verify_names(grid_n: int) -> Iterator[str]:
    half = grid_n // 2
    for ix in range(-half, half + 1):
        for iy in range(-half, half + 1):
            yield f"{VERIFY_PREFIX}_{ix:+03d}_{iy:+03d}"


def cell_xy_cm(center_x: float, center_y: float, ix: int, iy: int) -> Tuple[float, float]:
    x = center_x + ix * CELL_SIZE_CM
    y = center_y + iy * CELL_SIZE_CM
    return x, y


def cleanup_verify_actors(ucv) -> None:
    names = list(iter_verify_names(9))  # wide enough for grid_n <= 9
    for name in sorted(set(names), reverse=True):
        if geh.actor_exists(ucv, name):
            geh.destroy_actor_safely(ucv, name, max_attempts=3, timeout_s=10.0)
            geh.wait_until_actor_gone(ucv, name, timeout_s=5.0)
            time.sleep(0.1)


def spawn_verify_grid(
    ucv,
    *,
    center_xy_cm: Tuple[float, float],
    bottom_z_cm: float,
    grid_n: int = 3,
) -> List[Tuple[str, Tuple[float, float, float]]]:
    if grid_n % 2 == 0:
        raise ValueError("grid_n must be odd (3, 5, ...)")
    half = grid_n // 2
    actor_z = _block_bottom_to_actor_z(bottom_z_cm)
    placed: List[Tuple[str, Tuple[float, float, float]]] = []

    for ix in range(-half, half + 1):
        for iy in range(-half, half + 1):
            name = f"{VERIFY_PREFIX}_{ix:+03d}_{iy:+03d}"
            x, y = cell_xy_cm(center_xy_cm[0], center_xy_cm[1], ix, iy)
            loc = (x, y, actor_z)

            if geh.actor_exists(ucv, name):
                geh.destroy_actor_safely(ucv, name, max_attempts=3, timeout_s=10.0)
                geh.wait_until_actor_gone(ucv, name, timeout_s=5.0)

            if not geh.spawn_bp(ucv, geh.CUBE_BP, name, timeout_s=90.0):
                print(f"[VerifySpawn] FAIL spawn {name}")
                continue

            ucv.set_physics(name, False)
            ucv.set_movable(name, False)
            ucv.set_location(list(loc), name)
            ucv.set_orientation((0.0, 0.0, 0.0), name)
            ucv.set_collision(name, True)
            geh.set_cube_blocking_mode(ucv, name, blocking=True, apply_tint=False)
            time.sleep(POST_SPAWN_S)

            actual = tuple(float(v) for v in ucv.get_location(name))
            placed.append((name, actual))
            print(
                f"[VerifySpawn] {name} target_bottom_z={bottom_z_cm:.1f}cm "
                f"actor_z={actor_z:.1f}cm loc={geh._fmt_xyz(actual)}"
            )
    return placed


def main() -> int:
    parser = argparse.ArgumentParser(description="Fixed-height verification spawn")
    parser.add_argument("--cx", type=float, default=VERIFY_CENTER_XY_CM[0])
    parser.add_argument("--cy", type=float, default=VERIFY_CENTER_XY_CM[1])
    parser.add_argument("--bottom-z-cm", type=float, default=VERIFY_BOTTOM_Z_CM)
    parser.add_argument("--grid-n", type=int, default=3, help="odd: 3=3x3, 5=5x5")
    parser.add_argument("--wait-ue", type=float, default=30.0)
    args = parser.parse_args()

    if not lcp.wait_for_ue_port(args.wait_ue):
        print("ERROR: UnrealCV not reachable — start Level PIE", file=sys.stderr)
        return 2

    ucv, _ = g10k.ensure_connection()
    geh._prepare_ue_spawn(ucv)
    cleanup_verify_actors(ucv)

    center = (args.cx, args.cy)
    print(
        f"[VerifySpawn] center_xy_cm={center} bottom_z={args.bottom_z_cm:.1f}cm "
        f"({args.bottom_z_cm/100:.3f}m) grid={args.grid_n}x{args.grid_n}"
    )
    placed = spawn_verify_grid(
        ucv,
        center_xy_cm=center,
        bottom_z_cm=args.bottom_z_cm,
        grid_n=args.grid_n,
    )
    print(f"[VerifySpawn] DONE placed={len(placed)} — look for {VERIFY_PREFIX}_* in Outliner")
    print(
        f"[VerifySpawn] Fly camera to approx ({center[0]/100:.2f}m, {center[1]/100:.2f}m, "
        f"{args.bottom_z_cm/100:.1f}m)"
    )
    return 0 if placed else 1


if __name__ == "__main__":
    raise SystemExit(main())
