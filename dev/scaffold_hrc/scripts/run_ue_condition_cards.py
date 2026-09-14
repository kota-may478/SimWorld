#!/usr/bin/env python3
"""Run ue/hrc_mission.py for each unique theta in a condition_cards.json."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent


def unique_thetas(cards: list[dict]) -> list[tuple[float, float, dict]]:
    seen = set()
    out = []
    for row in cards:
        key = (round(row["vmax_mps"], 4), round(row["dmin_m"], 4))
        if key in seen:
            continue
        seen.add(key)
        out.append((key[0], key[1], row))
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cards", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--sockets-per-floor", type=int, default=1)
    parser.add_argument("--floors", type=int, default=1)
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    payload = json.loads(args.cards.read_text(encoding="utf-8"))
    jobs = unique_thetas(payload["results"])
    if args.limit > 0:
        jobs = jobs[: args.limit]

    results = []
    for i, (vmax, dmin, seed) in enumerate(jobs):
        out = args.out_dir / f"theta_{i:02d}_v{vmax:.3f}_d{dmin:.3f}.json"
        print("JOB", i + 1, "/", len(jobs), vmax, dmin, "->", out)
        cmd = [
            sys.executable,
            str(PKG / "ue" / "hrc_mission.py"),
            "--vmax",
            str(vmax),
            "--dmin",
            str(dmin),
            "--sockets-per-floor",
            str(args.sockets_per_floor),
            "--floors",
            str(args.floors),
            "--out",
            str(out),
        ]
        proc = subprocess.run(cmd, cwd=str(PKG))
        row = {
            "seed_card_id": seed["card_id"],
            "seed_method": seed["method"],
            "vmax_mps": vmax,
            "dmin_m": dmin,
            "exit_code": proc.returncode,
            "out": str(out),
        }
        if out.is_file():
            row["result"] = json.loads(out.read_text(encoding="utf-8"))
        results.append(row)

    summary = args.out_dir / "batch_summary.json"
    summary.write_text(json.dumps({"jobs": results}, indent=2) + "\n", encoding="utf-8")
    print(summary)
    n_ok = sum(1 for r in results if (r.get("result") or {}).get("ok"))
    print(f"done ok={n_ok}/{len(results)}")
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
