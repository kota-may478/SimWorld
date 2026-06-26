#!/usr/bin/env python3
"""Run N consecutive L0+L2 (sight/SLAM) trials with labeled artifacts.

Requires UE PIE Play before each trial. On any failure, restarts from trial 1.
Default: N=10 golden trials with default profile (roadmap §6.4).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

_SCENARIO = Path(__file__).resolve().parent
_RUN_TEST = _SCENARIO / "run_test.py"
_DEFAULT_LABEL = "L0andL2withSLAM"
_OUT_DIR = _SCENARIO / "out"


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Batch L0+L2 sight trials with labeled outputs")
    p.add_argument("--trials", type=int, default=10, help="Consecutive PASS trials required")
    p.add_argument("--run-label", default=_DEFAULT_LABEL)
    p.add_argument("--start-index", type=int, default=1, help="First trial index (default 1)")
    p.add_argument("--profile", default="default", help="Nav profile passed to run_test.py")
    p.add_argument(
        "--skip-spawn",
        action="store_true",
        help="Reuse existing PIE scene (passes --skip-spawn to run_test.py)",
    )
    return p.parse_args()


def _run_one(trial_index: int, run_label: str, profile: str, skip_spawn: bool) -> tuple[int, float, str]:
    cmd = [
        sys.executable,
        str(_RUN_TEST),
        "--l2-mode",
        "sight",
        "--no-l1",
        "--profile",
        profile,
        "--run-label",
        run_label,
        "--trial-index",
        str(trial_index),
    ]
    if skip_spawn:
        cmd.append("--skip-spawn")
    start = time.time()
    proc = subprocess.run(cmd, cwd=str(_SCENARIO), capture_output=True, text=True)
    duration = time.time() - start
    output = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode, duration, output


def _write_summary(
    *,
    results: list[dict[str, object]],
    run_label: str,
    profile: str,
    trials_required: int,
) -> Path:
    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    path = _OUT_DIR / f"golden_trials_{run_label}_{profile}_{stamp}.json"
    payload = {
        "run_label": run_label,
        "profile": profile,
        "trials_required": trials_required,
        "trial_rows": results,
        "pass_count": sum(1 for row in results if row.get("pass")),
        "fail_count": sum(1 for row in results if not row.get("pass")),
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return path


def main() -> int:
    args = _parse_args()
    results: list[dict[str, object]] = []
    trial = args.start_index
    passed = 0

    while passed < args.trials:
        print(
            f"[Batch] trial {trial} profile={args.profile} "
            f"(need {args.trials} consecutive PASS, at {passed})"
        )
        code, duration, output = _run_one(
            trial, args.run_label, args.profile, args.skip_spawn
        )
        print(output, end="")
        ok = code == 0 and "[Site20] PASS" in output
        results.append(
            {
                "trial_index": trial,
                "exit_code": code,
                "duration_s": round(duration, 1),
                "pass": ok,
                "profile": args.profile,
            }
        )
        if ok:
            passed += 1
            trial += 1
        else:
            print(f"[Batch] FAIL at trial {trial}; restarting consecutive count from 0")
            passed = 0
            trial = args.start_index

    summary_path = _write_summary(
        results=results,
        run_label=args.run_label,
        profile=args.profile,
        trials_required=args.trials,
    )
    print("[Batch] all trials PASS")
    print(f"[Batch] summary: {summary_path}")
    for row in results:
        print(row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
