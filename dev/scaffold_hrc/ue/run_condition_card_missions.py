#!/usr/bin/env python3
"""Replay unique condition-card θ in UE via hrc_mission (bare progressive erect).

Pareto P stays offline. This only freezes θ from cards and measures UE metrics.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from scene.erect_plan import ue_protocol_limits  # noqa: E402


def _unique_thetas(cards: list[dict]) -> list[dict]:
    seen: set[tuple[float, float]] = set()
    out: list[dict] = []
    for row in cards:
        key = (round(float(row["vmax_mps"]), 4), round(float(row["dmin_m"]), 4))
        if key in seen:
            continue
        seen.add(key)
        out.append(
            {
                "card_id": row["card_id"],
                "method": row["method"],
                "vmax_mps": float(row["vmax_mps"]),
                "dmin_m": float(row["dmin_m"]),
                "family": row.get("family"),
            }
        )
    return out


def _run(cmd: list[str]) -> int:
    print("+", " ".join(cmd), flush=True)
    return subprocess.call(cmd, cwd=str(PKG))


def _mission_ok(path: Path) -> bool:
    return mission_matches_protocol(path, "smoke")


def mission_matches_protocol(path: Path, protocol: str) -> bool:
    if not path.is_file():
        return False
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    if not payload.get("ok"):
        return False
    recorded = payload.get("protocol")
    if recorded is not None and recorded != protocol:
        return False
    limits = ue_protocol_limits(protocol)
    n_items = payload.get("n_items")
    if limits.max_items is not None and n_items != limits.max_items:
        return False
    if limits.max_items is None:
        from scene.erect_plan import build_erect_sequence
        from scene.geometry import STAGE1_GEOM

        expected = len(build_erect_sequence(STAGE1_GEOM))
        if protocol == "full" and n_items != expected:
            return False
    if limits.require_smin and payload.get("smin_m") is None:
        return False
    return True


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--cards", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument(
        "--protocol",
        choices=("smoke", "smin", "full"),
        default="smoke",
        help="smoke=3 posts; smin=first keep-out leg; full=all members",
    )
    p.add_argument("--max-items", type=int, default=None)
    p.add_argument("--max-floors", type=int, default=None)
    p.add_argument("--boards-per-floor", type=int, default=None)
    p.add_argument("--skip-bare", action="store_true")
    p.add_argument(
        "--skip-existing",
        action="store_true",
        help="skip θ whose mission_*.json already matches this protocol and is ok",
    )
    p.add_argument("--limit", type=int, default=0, help="0=all unique theta")
    p.add_argument(
        "--sim-rate",
        type=float,
        default=4.0,
        help="UE playback rate passed to hrc_mission (slomo). 4 is the sync ceiling.",
    )
    args = p.parse_args()

    limits = ue_protocol_limits(args.protocol)
    max_items = args.max_items if args.max_items is not None else limits.max_items
    max_floors = args.max_floors if args.max_floors is not None else limits.max_floors
    boards_per_floor = (
        args.boards_per_floor
        if args.boards_per_floor is not None
        else limits.boards_per_floor
    )

    payload = json.loads(args.cards.read_text(encoding="utf-8"))
    cards = payload["results"] if isinstance(payload, dict) else payload
    uniques = _unique_thetas(cards)
    if args.limit > 0:
        uniques = uniques[: args.limit]
    args.out.mkdir(parents=True, exist_ok=True)

    results: list[dict] = []
    py = [sys.executable, "-u"]
    for i, row in enumerate(uniques, start=1):
        tag = f"{row['card_id']}__{row['method']}"
        out_json = args.out / f"mission_{tag}.json"
        print(
            f"=== [{i}/{len(uniques)}] {tag} vmax={row['vmax_mps']} dmin={row['dmin_m']}",
            flush=True,
        )
        if args.skip_existing and mission_matches_protocol(out_json, args.protocol):
            mission = json.loads(out_json.read_text(encoding="utf-8"))
            results.append(
                {**row, **mission, "tag": tag, "rc": 0, "skipped_existing": True}
            )
            print(f"[skip-existing] reuse {out_json.name}", flush=True)
            continue
        if not args.skip_bare:
            rc = _run(py + ["ue/spawn_scaffold_pie.py", "--bare"])
            if rc != 0:
                results.append(
                    {**row, "ok": False, "error": "bare_spawn_failed", "rc": rc}
                )
                continue
            # Destroy→new-process spawn_bp crashes UE 5.3.2 if the TCP session
            # is reopened immediately (especially under slomo 4).
            print("[UE] settle 12s after bare before mission spawn_bp", flush=True)
            time.sleep(12.0)
        cmd = py + [
            "ue/hrc_mission.py",
            "--protocol",
            args.protocol,
            "--vmax",
            str(row["vmax_mps"]),
            "--dmin",
            str(row["dmin_m"]),
            "--out",
            str(out_json),
        ]
        if max_items is not None:
            cmd.extend(["--max-items", str(max_items)])
        if max_floors is not None:
            cmd.extend(["--max-floors", str(max_floors)])
        if boards_per_floor is not None:
            cmd.extend(["--boards-per-floor", str(boards_per_floor)])
        if limits.require_smin:
            cmd.append("--require-smin")
        if args.sim_rate != 1.0:
            cmd.extend(["--sim-rate", str(args.sim_rate)])
        rc = _run(cmd)
        if out_json.is_file():
            mission = json.loads(out_json.read_text(encoding="utf-8"))
            results.append({**row, **mission, "tag": tag, "rc": rc})
        else:
            results.append(
                {
                    **row,
                    "ok": False,
                    "error": "missing_mission_json",
                    "rc": rc,
                    "tag": tag,
                }
            )

    summary = {
        "cards": str(args.cards),
        "protocol": args.protocol,
        "n_unique_theta": len(uniques),
        "max_items": max_items,
        "max_floors": max_floors,
        "boards_per_floor": boards_per_floor,
        "n_ok": sum(1 for r in results if r.get("ok")),
        "results": results,
    }
    out_path = args.out / "ue_mission_results.json"
    out_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {"n_ok": summary["n_ok"], "n": len(results), "out": str(out_path)},
            indent=2,
        )
    )
    return 0 if summary["n_ok"] == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
