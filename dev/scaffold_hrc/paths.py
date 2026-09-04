"""Output paths for scaffold_hrc (gitignored under dev/**/out/)."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parent
OUT_DIR = PKG_DIR / "out"
CONFIG_DIR = PKG_DIR / "config"


def make_run_dir(root: Path | None = None, *, when: datetime | None = None) -> Path:
    """``out/YYYYMMDDHHMMSS`` (local time). Suffix ``_2`` if the second collides."""
    stamp = (when or datetime.now()).strftime("%Y%m%d%H%M%S")
    base = root or OUT_DIR
    path = base / stamp
    if not path.exists():
        path.mkdir(parents=True, exist_ok=True)
        return path
    n = 2
    while True:
        cand = base / f"{stamp}_{n}"
        if not cand.exists():
            cand.mkdir(parents=True, exist_ok=True)
            return cand
        n += 1
