#!/usr/bin/env python3
"""Re-score MiniLM nearest-cell without keyword/relative intercept."""

from __future__ import annotations

import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from fronts.ab_map import pick_weighted  # noqa: E402
from fronts.catalog import apply_catalog_label, pref_to_catalog_id  # noqa: E402
from fronts.gold_language import gold_items  # noqa: E402
from fronts.hf_ground import HuggingFaceEmbedder  # noqa: E402
from fronts.language import Preference, _nearest_cell  # noqa: E402
from fronts.run_language_eval import load_nsga2_rows, score_pref, summarize  # noqa: E402

RESULTS = PKG / "out" / "language_eval" / "results.json"


def main() -> int:
    payload = json.loads(RESULTS.read_text(encoding="utf-8"))
    rows = load_nsga2_rows(Path(payload["front"]))
    embedder = HuggingFaceEmbedder()
    scored: list[dict] = []
    for item in gold_items():
        start = Preference(item.start_alpha, item.start_beta)
        gold_pref = apply_catalog_label(item.gold_id, start)
        gold_row = pick_weighted(rows, gold_pref.alpha, gold_pref.beta)
        pref, _score = _nearest_cell(item.text, embedder)
        pred_id = pref_to_catalog_id(pref, start)
        pred_row = pick_weighted(rows, pref.alpha, pref.beta)
        row = score_pref(
            pred_id,
            pref,
            item.gold_id,
            item.accept_ids,
            gold_pref,
            gold_row,
            pred_row,
        )
        row.update(
            {
                "uid": item.uid,
                "text": item.text,
                "family": item.family,
                "notes": pref.notes,
            }
        )
        scored.append(row)
    payload["items"]["embed_pure"] = scored
    payload["summary"]["embed_pure"] = summarize(scored)
    RESULTS.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(payload["summary"]["embed_pure"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
