#!/usr/bin/env python3
"""Rescore language Acc (proposed + keyword) on a fronts.json without Qwen.

Uses the same ground_utterance / keyword path as run_language_eval --proposed-only.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

from constraints.pareto import EvaluatedTheta  # noqa: E402
from fronts.ab_map import enumerate_table, pick_weighted  # noqa: E402
from fronts.catalog import apply_catalog_label, pref_to_catalog_id  # noqa: E402
from fronts.gold_language import gold_items  # noqa: E402
from fronts.language import Preference, ground_utterance  # noqa: E402
from fronts.run_language_eval import load_nsga2_rows, score_pref, summarize  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--front-json", type=Path, required=True)
    parser.add_argument(
        "--out",
        type=Path,
        default=PKG / "out" / "language_eval" / "results_rescore.json",
    )
    args = parser.parse_args()

    rows = load_nsga2_rows(args.front_json)
    table = enumerate_table(rows)
    embedder = None
    try:
        from fronts.hf_ground import HuggingFaceEmbedder

        embedder = HuggingFaceEmbedder()
        print("embedder ok")
    except Exception as exc:
        print("embedder unavailable", type(exc).__name__, exc)

    method_rows: dict[str, list] = defaultdict(list)
    for item in gold_items():
        start = Preference(item.start_alpha, item.start_beta)
        gold_pref = apply_catalog_label(item.gold_id, start, theta_table=table)
        gold_row = pick_weighted(rows, gold_pref.alpha, gold_pref.beta)

        proposed = ground_utterance(
            item.text, state=start, embedder=embedder, theta_table=table
        )
        pred_id = pref_to_catalog_id(proposed, start)
        pred_row = pick_weighted(rows, proposed.alpha, proposed.beta)
        scored = score_pref(
            pred_id, proposed, item.gold_id, item.accept_ids, gold_pref, gold_row, pred_row
        )
        scored.update({"uid": item.uid, "family": item.family, "text": item.text})
        method_rows["proposed"].append(scored)

        kw = ground_utterance(item.text, state=start, embedder=None, theta_table=table)
        kw_id = pref_to_catalog_id(kw, start)
        kw_row = pick_weighted(rows, kw.alpha, kw.beta)
        kw_scored = score_pref(
            kw_id, kw, item.gold_id, item.accept_ids, gold_pref, gold_row, kw_row
        )
        kw_scored.update({"uid": item.uid, "family": item.family, "text": item.text})
        method_rows["keyword"].append(kw_scored)

    summary = {m: summarize(rows_) for m, rows_ in method_rows.items()}
    payload = {
        "front": str(args.front_json),
        "n_gold": len(gold_items()),
        "n_nsga2_iso": len(rows),
        "proposed_only": True,
        "summary": summary,
        "items": {m: rows_ for m, rows_ in method_rows.items()},
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(args.out)
    for method, fam in summary.items():
        all_s = fam.get("all", {})
        print(
            method,
            "overall",
            round(all_s.get("strict", 0), 3),
            "theta",
            round(all_s.get("theta_match", 0), 3),
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
