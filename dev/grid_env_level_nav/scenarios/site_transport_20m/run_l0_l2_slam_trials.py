#!/usr/bin/env python3
"""Run N consecutive L0+L2 (sight/SLAM) trials with labeled artifacts.

Requires UE PIE Play before each trial. On any failure, restarts from trial 1.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

_SCENARIO = Path(__file__).resolve().parent
_RUN_TEST = _SCENARIO / "run_test.py"
_DEFAULT_LABEL = "L0andL2withSLAM"


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Batch L0+L2 sight trials with labeled outputs")
    p.add_argument("--trials", type=int, default=5, help="Consecutive PASS trials required")
    p.add_argument("--run-label", default=_DEFAULT_LABEL)
    p.add_argument("--start-index", type=int, default=1, help="First trial index (default 1)")
    return p.parse_args()


def _run_one(trial_index: int, run_label: str) -> tuple[int, float, str]:
    cmd = [
        sys.executable,
        str(_RUN_TEST),
        "--l2-mode",
        "sight",
        "--no-l1",
        "--run-label",
        run_label,
        "--trial-index",
        str(trial_index),
    ]
    start = time.time()
    proc = subprocess.run(cmd, cwd=str(_SCENARIO), capture_output=True, text=True)
    duration = time.time() - start
    output = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode, duration, output


def main() -> int:
    args = _parse_args()
    results: list[dict[str, object]] = []
    trial = args.start_index
    passed = 0

    while passed < args.trials:
        print(f"[Batch] trial {trial} (need {args.trials} consecutive PASS, at {passed})")
        code, duration, output = _run_one(trial, args.run_label)
        print(output, end="")
        ok = code == 0 and "[Site20] PASS" in output
        results.append(
            {
                "trial_index": trial,
                "exit_code": code,
                "duration_s": round(duration, 1),
                "pass": ok,
            }
        )
        if ok:
            passed += 1
            trial += 1
        else:
            print(f"[Batch] FAIL at trial {trial}; restarting consecutive count from 0")
            passed = 0
            trial = args.start_index

    print("[Batch] all trials PASS")
    for row in results:
        print(row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
