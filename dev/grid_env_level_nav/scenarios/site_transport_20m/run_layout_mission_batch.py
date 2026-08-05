#!/usr/bin/env python3
"""Spawn each layout and run the transport mission (material → humanoid).

Outputs are grouped under one batch directory:

  out/layout_batch_<UTC>/
    batch_transcript.log
    batch_summary.json
    layout_01_test/
      timing_layout_01_test.json
      metricsSummary_layout_01_test.json
      costMap_layout_01_test.png
      site_transport_trajectory_layout_01_test.json
      site_transport_costmap_layout_01_test.npz
      console.log
    layout_02_test/
      ...
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

_PKG = Path(__file__).resolve().parents[2]
_REPO = _PKG.parent.parent
_SCENARIO_OUT = _PKG / "scenarios" / "site_transport_20m" / "out"
_RUNNER = _PKG / "run_site_transport_20m_test.py"

if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

from layout_variants import layout_id_for_index  # noqa: E402


def _utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _artifact_suffix(index: int) -> str:
    return f"layout_{index:02d}_test"


def _parse_leg_times(log_text: str) -> Dict[str, float | None]:
    leg1 = leg2 = None
    m1 = re.search(r"\[Site20\] leg1_time_s=([\d.]+)", log_text)
    m2 = re.search(r"\[Site20\] leg2_time_s=([\d.]+)", log_text)
    if m1:
        leg1 = float(m1.group(1))
    if m2:
        leg2 = float(m2.group(1))
    return {"leg1_time_s": leg1, "leg2_time_s": leg2}


def _artifact_paths(artifact_dir: Path, suffix: str) -> Dict[str, Path]:
    return {
        "timing_json": artifact_dir / f"timing_{suffix}.json",
        "metrics_json": artifact_dir / f"metricsSummary_{suffix}.json",
        "costmap_png": artifact_dir / f"costMap_{suffix}.png",
        "traj_json": artifact_dir / f"site_transport_trajectory_{suffix}.json",
        "summary_png": artifact_dir / f"metricsSummary_{suffix}.png",
        "costmap_npz": artifact_dir / f"site_transport_costmap_{suffix}.npz",
    }


def _run_one(
    index: int,
    *,
    batch_dir: Path,
    profile: str,
    l2_mode: str,
    nav_mode: str,
    nav_exec: str,
    skip_spawn: bool,
    no_l1: bool,
    log_path: Path,
) -> Dict[str, Any]:
    layout_id = layout_id_for_index(index)
    suffix = _artifact_suffix(index)
    artifact_dir = batch_dir / suffix
    artifact_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        "conda",
        "run",
        "--no-capture-output",
        "-n",
        "simworld",
        "python",
        "-u",
        str(_RUNNER),
        "--profile",
        profile,
        "--l2-mode",
        l2_mode,
        "--layout-id",
        layout_id,
        "--artifact-suffix",
        suffix,
        "--artifact-dir",
        str(artifact_dir),
        "--nav-mode",
        nav_mode,
    ]
    if nav_mode == "navmesh":
        cmd.extend(["--nav-exec", nav_exec])
    if not skip_spawn:
        cmd.append("--force-respawn")
    else:
        cmd.append("--skip-spawn")
    if no_l1:
        cmd.append("--no-l1")

    header = (
        f"\n{'=' * 72}\n"
        f"[Batch] START {layout_id} artifact_dir={artifact_dir} at {_utc_stamp()}\n"
        f"[Batch] cmd: {' '.join(cmd)}\n"
        f"{'=' * 72}\n"
    )
    log_path.parent.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "PYTHONUNBUFFERED": "1"}
    with log_path.open("a", encoding="utf-8") as log_f:
        log_f.write(header)
        log_f.flush()
        proc = subprocess.run(
            cmd,
            cwd=str(_REPO),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
            env=env,
        )
        log_f.write(proc.stdout)
        log_f.write(
            f"\n[Batch] END {layout_id} exit_code={proc.returncode} at {_utc_stamp()}\n"
        )

    (artifact_dir / "console.log").write_text(proc.stdout, encoding="utf-8")
    paths = _artifact_paths(artifact_dir, suffix)
    legs = _parse_leg_times(proc.stdout)
    passed = proc.returncode == 0 and "[Site20] PASS" in proc.stdout

    return {
        "index": index,
        "layout_id": layout_id,
        "artifact_suffix": suffix,
        "artifact_dir": str(artifact_dir),
        "exit_code": proc.returncode,
        "pass": passed,
        **legs,
        "artifacts": {key: str(p) if p.is_file() else None for key, p in paths.items()},
        "log_path": str(log_path),
    }


def main() -> int:
    p = argparse.ArgumentParser(
        description="Batch: spawn each layout + run material→humanoid mission"
    )
    p.add_argument("--start", type=int, default=1, help="First layout index (1..10)")
    p.add_argument("--end", type=int, default=10, help="Last layout index (1..10)")
    p.add_argument("--profile", default="default")
    p.add_argument("--l2-mode", default="sight", choices=("sight", "geom", "camera", "off"))
    p.add_argument(
        "--skip-spawn",
        action="store_true",
        help="Skip spawn (mission only; scene must already match layout)",
    )
    p.add_argument(
        "--no-l1",
        action="store_true",
        help="Disable L1 forbidden zones (L0+L2 navigation only)",
    )
    p.add_argument(
        "--nav-mode",
        default="navmesh",
        choices=("costmap", "navmesh"),
        help="Navigation backend (default: navmesh)",
    )
    p.add_argument(
        "--nav-exec",
        default="moveto",
        choices=("vbp", "moveto"),
        help="navmesh execution: moveto (UE controller) or vbp",
    )
    p.add_argument(
        "--batch-dir",
        type=Path,
        default=None,
        help="Output root (default: out/layout_batch_<UTC>)",
    )
    p.add_argument(
        "--pause-between-s",
        type=float,
        default=2.0,
        help="Seconds between layouts (default 2.0)",
    )
    args = p.parse_args()

    if args.start < 1 or args.end < args.start:
        print("[Batch] FAIL: invalid layout index range")
        return 1

    stamp = _utc_stamp()
    batch_dir = args.batch_dir or (_SCENARIO_OUT / f"layout_batch_{stamp}")
    batch_dir.mkdir(parents=True, exist_ok=True)
    master_log = batch_dir / "batch_transcript.log"
    summary_path = batch_dir / "batch_summary.json"

    results: List[Dict[str, Any]] = []
    t0 = time.time()
    print(f"[Batch] output dir: {batch_dir}")
    l1_label = "off" if args.no_l1 else "on"
    print(
        f"[Batch] layouts {args.start}..{args.end} profile={args.profile} "
        f"l2={args.l2_mode} l1={l1_label} nav_mode={args.nav_mode} "
        f"nav_exec={args.nav_exec} spawn={'skip' if args.skip_spawn else 'force-respawn'}"
    )

    for index in range(args.start, args.end + 1):
        row = _run_one(
            index,
            batch_dir=batch_dir,
            profile=args.profile,
            l2_mode=args.l2_mode,
            nav_mode=args.nav_mode,
            nav_exec=args.nav_exec,
            skip_spawn=args.skip_spawn,
            no_l1=args.no_l1,
            log_path=master_log,
        )
        results.append(row)
        summary = {
            "batch_stamp": stamp,
            "batch_dir": str(batch_dir),
            "profile": args.profile,
            "l2_mode": args.l2_mode,
            "nav_mode": args.nav_mode,
            "nav_exec": args.nav_exec,
            "l1_enabled": not args.no_l1,
            "skip_spawn": args.skip_spawn,
            "started_at": stamp,
            "elapsed_s": round(time.time() - t0, 1),
            "results": results,
        }
        summary_path.write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        status = "PASS" if row["pass"] else "FAIL"
        print(
            f"[Batch] {row['layout_id']} {status} "
            f"leg1={row.get('leg1_time_s')}s leg2={row.get('leg2_time_s')}s "
            f"exit={row['exit_code']} → {row['artifact_dir']}"
        )
        if index < args.end and args.pause_between_s > 0:
            time.sleep(args.pause_between_s)

    failed = [r for r in results if not r["pass"]]
    print(f"[Batch] done elapsed={time.time() - t0:.0f}s failed={len(failed)}")
    print(f"[Batch] summary: {summary_path}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
