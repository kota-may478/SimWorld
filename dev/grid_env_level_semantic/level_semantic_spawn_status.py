#!/usr/bin/env python3
"""Read-only spawn progress from Cursor terminal log + registry (no UE connection)."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
REGISTRY = THIS_DIR / ".level_semantic_registry.json"
FILL_RE = re.compile(
    r"\[Fill\]\s+(\d+)/(\d+)\s+placed=(\d+)/(\d+)",
)
LABEL_DONE_RE = re.compile(
    r"\[LevelSemanticScan/collision\]\s+(\d+)/(\d+)\s+.*elapsed=([\d.]+)s",
)
TERMINAL_GLOB = Path.home() / ".cursor/projects"


def _find_run_log() -> Path | None:
    candidates: list[tuple[float, Path]] = []
    if not TERMINAL_GLOB.is_dir():
        return None
    for path in TERMINAL_GLOB.glob("**/terminals/*.txt"):
        try:
            head = path.read_text(encoding="utf-8", errors="replace")[:4000]
        except OSError:
            continue
        if "run_level_semantic_layer" not in head:
            continue
        if "[Fill]" not in head and "Phase2/Spawn" not in head:
            # might still be labeling — check full file tail cheaply
            try:
                tail = path.read_text(encoding="utf-8", errors="replace")[-8000:]
            except OSError:
                continue
            if "[Fill]" not in tail and "Phase1/Label" not in tail:
                continue
        candidates.append((path.stat().st_mtime, path))
    if not candidates:
        return None
    candidates.sort(key=lambda t: t[0], reverse=True)
    return candidates[0][1]


def _parse_running_ms(header: str) -> float | None:
    m = re.search(r"running_for_ms:\s*(\d+)", header)
    return float(m.group(1)) / 1000.0 if m else None


def main() -> int:
    reg = {}
    if REGISTRY.is_file():
        reg = json.loads(REGISTRY.read_text(encoding="utf-8"))

    status = reg.get("status", "unknown")
    total = int(reg.get("total_cells", 0) or 0)
    labeled = int(reg.get("labeled_count", 0) or 0)

    log_path = _find_run_log()
    if log_path is None:
        print("status:", status)
        print("labeled:", labeled, "/", total)
        print("spawn: log not found (Phase2 not started or Cursor log missing)")
        return 0

    text = log_path.read_text(encoding="utf-8", errors="replace")
    parts = text.split("---\n")
    body = parts[2] if len(parts) >= 3 else text

    fill_lines = FILL_RE.findall(body)
    label_elapsed_s: float | None = None
    for m in LABEL_DONE_RE.finditer(body):
        done, total_cells, elapsed = m.groups()
        if done == total_cells:
            label_elapsed_s = float(elapsed)

    running_s = _parse_running_ms(text)

    if not fill_lines:
        print("phase: labeling (or spawn not started)")
        print("registry:", status, f"labeled {labeled}/{total}")
        if label_elapsed_s:
            print(f"label_scan_elapsed: {label_elapsed_s/3600:.2f} h (in log)")
        print("log:", log_path)
        return 0

    _, _, placed_s, total_s = fill_lines[-1]
    placed = int(placed_s)
    total_spawn = int(total_s)
    pct = 100.0 * placed / total_spawn if total_spawn else 0.0

    print("phase: spawn (Phase2)")
    print(f"placed: {placed} / {total_spawn}  ({pct:.2f}%)")
    print("registry:", status, f"labeled {labeled}/{total}")

    if running_s is not None and label_elapsed_s is not None and placed > 0:
        spawn_s = max(1.0, running_s - label_elapsed_s)
        rate = placed / spawn_s
        remain = max(0, total_spawn - placed)
        eta_s = remain / rate if rate > 0 else float("inf")
        print(f"spawn_elapsed: {spawn_s/3600:.2f} h  rate: {rate*3600:.0f} blocks/h")
        print(f"eta_remaining: {eta_s/3600:.1f} h ({eta_s/86400:.1f} days)")
    else:
        print("eta: need more spawn progress in log")

    print("log:", log_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
