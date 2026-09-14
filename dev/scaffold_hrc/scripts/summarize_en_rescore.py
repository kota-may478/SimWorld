#!/usr/bin/env python3
import json
from collections import defaultdict
from pathlib import Path

d = json.loads(Path("out/language_eval/results_171041_en_only.json").read_text())
for method in ("proposed", "keyword"):
    s = d["summary"][method]
    print("===", method, "===")
    for fam, block in s.items():
        if not isinstance(block, dict) or "strict" not in block:
            continue
        print(
            fam,
            "n=",
            block["n"],
            "strict=",
            round(block["strict"], 3),
            "theta=",
            round(block["theta_match"], 3),
        )
    fails = [
        r
        for r in d["items"][method]
        if not r.get("strict")
    ]
    print("fails", len(fails))
    by = defaultdict(list)
    for r in fails:
        by[r["family"]].append((r["uid"], r["text"], r.get("pred_id"), r.get("gold_id")))
    for fam, rows in by.items():
        print("--", fam)
        for row in rows[:12]:
            print(" ", row)
