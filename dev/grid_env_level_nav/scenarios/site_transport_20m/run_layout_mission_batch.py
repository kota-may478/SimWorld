#!/usr/bin/env python3
"""Run site_transport missions for layout_01..layout_10 sequentially (PIE)."""

from __future__ import annotations

import argparse
import json
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


def _utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _layout_id(index: int) -> str:
    return f"layout_{index:02d}"


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


def _run_one(
    index: int,
    *,
    profile: str,
    l2_mode: str,
    skip_spawn: bool,
    log_path: Path,
) -> Dict[str, Any]:
    layout_id = _layout_id(index)
    suffix = _artifact_suffix(index)
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
    ]
    if not skip_spawn:
        cmd.append("--force-respawn")
    else:
        cmd.append("--skip-spawn")

    header = (
        f"\n{'=' * 72}\n"
        f"[Batch] START {layout_id} artifact_suffix={suffix} at {_utc_stamp()}\n"
        f"[Batch] cmd: {' '.join(cmd)}\n"
        f"{'=' * 72}\n"
    )
    log_path.parent.mkdir(parents=True, exist_ok=True)
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
        )
        log_f.write(proc.stdout)
        log_f.write(
            f"\n[Batch] END {layout_id} exit_code={proc.returncode} at {_utc_stamp()}\n"
        )

    timing_path = _SCENARIO_OUT / f"timing_{suffix}.json"
    metrics_path = _SCENARIO_OUT / f"metricsSummary_{suffix}.json"
    costmap_path = _SCENARIO_OUT / f"costMap_{suffix}.png"
    traj_path = _SCENARIO_OUT / f"site_transport_trajectory_{suffix}.json"
    summary_png = _SCENARIO_OUT / f"metricsSummary_{suffix}.png"
    legs = _parse_leg_times(proc.stdout)
    passed = proc.returncode == 0 and "[Site20] PASS" in proc.stdout

    record_dir = log_path.parent / suffix
    record_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "console.log").write_text(proc.stdout, encoding="utf-8")
    for src in (timing_path, metrics_path, costmap_path, traj_path, summary_png):
        if src.is_file():
            (record_dir / src.name).write_bytes(src.read_bytes())

    return {
        "index": index,
        "layout_id": layout_id,
        "artifact_suffix": suffix,
        "exit_code": proc.returncode,
        "pass": passed,
        **legs,
        "timing_json": str(timing_path) if timing_path.is_file() else None,
        "metrics_json": str(metrics_path) if metrics_path.is_file() else None,
        "costmap_png": str(costmap_path) if costmap_path.is_file() else None,
        "record_dir": str(record_dir),
        "log_path": str(log_path),
    }


def main() -> int:
    p = argparse.ArgumentParser(description="Batch run layout_01..10 missions")
    p.add_argument("--start", type=int, default=1, help="First layout index (1..10)")
    p.add_argument("--end", type=int, default=10, help="Last layout index (1..10)")
    p.add_argument("--profile", default="default")
    p.add_argument("--l2-mode", default="sight", choices=("sight", "geom", "camera", "off"))
    p.add_argument("--skip-spawn", action="store_true")
    p.add_argument(
        "--batch-dir",
        type=Path,
        default=None,
        help="Directory for batch transcript + per-layout records",
    )
    args = p.parse_args()

    stamp = _utc_stamp()
    batch_dir = args.batch_dir or (_SCENARIO_OUT / f"layout_batch_{stamp}")
    batch_dir.mkdir(parents=True, exist_ok=True)
    master_log = batch_dir / "batch_transcript.log"
    summary_path = batch_dir / "batch_summary.json"

    results: List[Dict[str, Any]] = []
    t0 = time.time()
    print(f"[Batch] output dir: {batch_dir}")
    print(f"[Batch] layouts {args.start}..{args.end} profile={args.profile} l2={args.l2_mode}")

    for index in range(args.start, args.end + 1):
        row = _run_one(
            index,
            profile=args.profile,
            l2_mode=args.l2_mode,
            skip_spawn=args.skip_spawn,
            log_path=master_log,
        )
        results.append(row)
        summary = {
            "batch_stamp": stamp,
            "profile": args.profile,
            "l2_mode": args.l2_mode,
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
            f"exit={row['exit_code']}"
        )

    failed = [r for r in results if not r["pass"]]
    print(f"[Batch] done elapsed={time.time() - t0:.0f}s failed={len(failed)}")
    print(f"[Batch] summary: {summary_path}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
