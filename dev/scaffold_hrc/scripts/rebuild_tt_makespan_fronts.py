#!/usr/bin/env python3
"""Rebuild ISO Pareto with full-mission TT, then condition cards and paper figs."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
PY = sys.executable


def _run(cmd: list[str]) -> None:
    print("+", " ".join(cmd), flush=True)
    code = subprocess.call(cmd, cwd=str(PKG))
    if code != 0:
        raise SystemExit(code)


def _latest_run_dir(before: set[str]) -> Path:
    out = PKG / "out"
    after = {p.name for p in out.iterdir() if p.is_dir() and p.name[:8].isdigit()}
    new = sorted(after - before)
    if new:
        return out / new[-1]
    stamped = sorted(
        (p for p in out.iterdir() if p.is_dir() and p.name[:8].isdigit()),
        key=lambda p: p.name,
    )
    if not stamped:
        raise SystemExit("no out/ run directory")
    return stamped[-1]


def main() -> int:
    out = PKG / "out"
    out.mkdir(exist_ok=True)
    before = {p.name for p in out.iterdir() if p.is_dir() and p.name[:8].isdigit()}
    _run([PY, "-u", "fronts/run_fronts.py", "--methods", "grid,nsga2,safe_bo"])
    run_dir = _latest_run_dir(before)
    fronts = run_dir / "fronts.json"
    if not fronts.is_file():
        raise SystemExit(f"missing {fronts}")
    print("run_dir", run_dir, flush=True)
    cards_dir = run_dir / "cards"
    _run(
        [
            PY,
            "-u",
            "scripts/run_condition_cards.py",
            "--fronts",
            str(fronts),
            "--out",
            str(cards_dir),
        ]
    )
    cards = cards_dir / "condition_cards.json"
    payload = json.loads(cards.read_text(encoding="utf-8"))
    rows = payload.get("results") or []
    l1 = [row for row in rows if row.get("card_id") == "L1_efficient"]
    l1_path = cards_dir / "L1_efficient.json"
    l1_path.write_text(
        json.dumps({**payload, "results": l1}, indent=2) + "\n",
        encoding="utf-8",
    )
    print("wrote", l1_path, "n", len(l1), flush=True)
    _run(
        [
            PY,
            "-u",
            "scripts/make_paper_figures.py",
            "--fronts",
            str(fronts),
            "--cards",
            str(cards),
            "--out",
            str(run_dir / "paper_figs"),
        ]
    )
    print("READY", run_dir, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
