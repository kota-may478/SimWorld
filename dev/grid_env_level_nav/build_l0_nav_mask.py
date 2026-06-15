#!/usr/bin/env python3
"""Build L0 NavMesh mask via NavProjectPoint (PIE required)."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
_ROOT = _THIS_DIR.parent.parent
for _p in (_THIS_DIR, _ROOT / "dev" / "grid_env_hri", _ROOT / "dev" / "grid_env_10k"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import nav_query as nq  # noqa: E402
from ue_client_guard import ensure_exclusive_ue_session  # noqa: E402
from level_coords import NAV_PROJECT_PROBE_Z_CM  # noqa: E402
from l0_nav_mask import (  # noqa: E402
    build_l0_mask_from_project_fn,
    load_l0_mask_npz,
    save_l0_mask_npz,
)
from work_region import (  # noqa: E402
    DEFAULT_RESOLUTION_CM,
    DEFAULT_XY_TOLERANCE_CM,
    DEFAULT_Z_TOLERANCE_CM,
)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Build L0 NavMesh cost mask (PIE + NavQueryService).")
    p.add_argument(
        "--resolution-cm",
        type=float,
        default=DEFAULT_RESOLUTION_CM,
        help="Cell size in cm (default 10).",
    )
    p.add_argument(
        "--output",
        type=Path,
        default=_THIS_DIR / "cache" / "l0_mask_10cm.npz",
    )
    p.add_argument(
        "--z-cm",
        type=float,
        default=NAV_PROJECT_PROBE_Z_CM,
        help="Probe Z for NavProjectPoint (must be within ProjectExtentCm of NavMesh).",
    )
    p.add_argument(
        "--xy-tolerance-cm",
        type=float,
        default=DEFAULT_XY_TOLERANCE_CM,
        help="Max horizontal snap from cell center to projected NavMesh point.",
    )
    p.add_argument(
        "--z-tolerance-cm",
        type=float,
        default=DEFAULT_Z_TOLERANCE_CM,
        help="Max |pz - floor_z| for walkable cells.",
    )
    p.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Sample every N cells; gaps filled by nearest-neighbor (faster coarse pass).",
    )
    p.add_argument(
        "--quick",
        action="store_true",
        help="Shorthand: --resolution-cm 30 --stride 1 (~5h at ~300ms/cell).",
    )
    p.add_argument(
        "--resume",
        action="store_true",
        help="Resume from --output if it exists (re-samples all stride cells).",
    )
    p.add_argument(
        "--checkpoint-interval",
        type=int,
        default=1000,
    )
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    if args.quick:
        args.resolution_cm = 30.0
        if args.output == _THIS_DIR / "cache" / "l0_mask_10cm.npz":
            args.output = _THIS_DIR / "cache" / "l0_mask_30cm_quick.npz"

    ucv, _ = ensure_exclusive_ue_session(force_new=True)
    ok, actor = nq.ensure_nav_query_service(ucv)
    if not ok:
        print("[L0] ERROR: NavQueryService not available. Start PIE on Level first.")
        return 1

    resume_costs = None
    if args.resume and args.output.is_file():
        costs, res, _, _ = load_l0_mask_npz(args.output)
        if abs(res - args.resolution_cm) > 0.01:
            print(f"[L0] WARN: resume file resolution {res} != {args.resolution_cm}")
        resume_costs = costs
        print(f"[L0] resuming from {args.output}")

    def project_fn(wx: float, wy: float, wz: float) -> dict:
        return nq.nav_project_point(ucv, actor, wx, wy, wz)

    w_cells = int(__import__("math").ceil(7000.0 / args.resolution_cm))
    h_cells = int(__import__("math").ceil(7900.0 / args.resolution_cm))
    sample_count = ((w_cells + args.stride - 1) // args.stride) * (
        (h_cells + args.stride - 1) // args.stride
    )
    est_s = sample_count * 0.31
    print(
        f"[L0] grid≈{w_cells}x{h_cells} stride={args.stride} "
        f"xy_tol={args.xy_tolerance_cm}cm z_tol={args.z_tolerance_cm}cm "
        f"sample≈{sample_count} est≈{est_s/3600:.1f} h @300ms/cell"
    )

    t0 = time.time()
    costs = build_l0_mask_from_project_fn(
        project_fn,
        resolution_cm=args.resolution_cm,
        z_cm=args.z_cm,
        xy_tolerance_cm=args.xy_tolerance_cm,
        z_tolerance_cm=args.z_tolerance_cm,
        stride=args.stride,
        checkpoint_path=args.output,
        checkpoint_interval=args.checkpoint_interval,
        resume_costs=resume_costs,
    )
    save_l0_mask_npz(
        args.output,
        costs,
        resolution_cm=args.resolution_cm,
        partial=False,
        xy_tolerance_cm=args.xy_tolerance_cm,
    )
    print(f"[L0] done in {time.time() - t0:.1f}s → {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
