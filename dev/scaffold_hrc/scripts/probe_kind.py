#!/usr/bin/env python3
"""Print stage-1 kind for a few gold utterances."""

from __future__ import annotations

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from fronts.catalog import parse_from_options  # noqa: E402
from fronts.hf_ground import HuggingFaceInstruct  # noqa: E402
from fronts.staged_ground import KIND_PROMPT, KINDS  # noqa: E402

TEXTS = (
    "急いで",
    "please go slow",
    "stay back",
    "keep more distance",
    "stay a bit away but otherwise as usual",
    "人からもっと離れて",
    "ゆっくり動いて",
    "減速して",
    "be careful",
)


def main() -> int:
    gen = HuggingFaceInstruct()
    for text in TEXTS:
        raw = gen.generate(KIND_PROMPT.format(text=text), max_new_tokens=8)
        parsed = parse_from_options(raw, KINDS)
        print("TEXT", text)
        print("RAW", repr(raw))
        print("PARSED", parsed)
        print("---")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
