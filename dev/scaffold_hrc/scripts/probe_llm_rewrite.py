#!/usr/bin/env python3
"""Probe: Qwen rewrites long/composite utterances into canonical commands.

Does not change the proposed pipeline. Prints recover/break counts only.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from fronts.catalog import pref_to_catalog_id  # noqa: E402
from fronts.gold_language import gold_items  # noqa: E402
from fronts.hf_ground import HuggingFaceEmbedder, HuggingFaceInstruct  # noqa: E402
from fronts.language import Preference, ground_utterance  # noqa: E402

CANON = (
    "普通だがゆっくり",
    "普通で距離を取って",
    "離れて作業して",
    "ゆっくり動いて",
    "もう少しゆっくり",
    "かなりゆっくり",
    "もう少し急いで",
    "かなり急いで",
    "もう少し離れて",
    "かなり離れて",
    "もう少し慎重に",
    "かなり慎重に",
    "NOT_PREFERENCE",
    "慎重に",
    "普通に",
    "急いで",
    "リセット",
)

REWRITE_PROMPT = (
    "Rewrite the scaffold-robot operator utterance into exactly one command "
    "from this list. If it is not about robot speed or distance, reply "
    "NOT_PREFERENCE.\n"
    + "\n".join(CANON)
    + "\nUtterance: {text}\nCommand:"
)

_CONJ_RE = re.compile(
    r"だが|けど|いいが|ず普通|otherwise| but | and |, otherwise",
    re.IGNORECASE,
)


def needs_normalize(text: str) -> bool:
    if _CONJ_RE.search(text) is not None:
        return True
    return len(text) >= 18 and ("が" in text or " but " in text.lower())


def parse_canon(raw: str) -> str:
    blob = raw.strip().splitlines()[0].strip().strip("\"'`")
    hits = [item for item in CANON if item.lower() in blob.lower()]
    if not hits:
        return blob
    return max(hits, key=len)


def main() -> int:
    embedder = HuggingFaceEmbedder()
    generator = HuggingFaceInstruct()
    rows: list[dict] = []
    for item in gold_items():
        start = Preference(item.start_alpha, item.start_beta)
        base = ground_utterance(item.text, state=start, embedder=embedder)
        base_id = pref_to_catalog_id(base, start)
        base_ok = base_id == item.gold_id or base_id in item.accept_ids
        gated = needs_normalize(item.text)
        rec = {
            "uid": item.uid,
            "text": item.text,
            "gold_id": item.gold_id,
            "family": item.family,
            "base_id": base_id,
            "base_ok": base_ok,
            "gated": gated,
        }
        if gated:
            raw = generator.generate(
                REWRITE_PROMPT.format(text=item.text), max_new_tokens=16
            )
            rewritten = parse_canon(raw)
            if rewritten == "NOT_PREFERENCE":
                pref = Preference(
                    start.alpha,
                    start.beta,
                    kind="unchanged",
                    cell="normal",
                    notes="rewrite-ood",
                )
            else:
                pref = ground_utterance(
                    rewritten, state=start, embedder=embedder
                )
            new_id = pref_to_catalog_id(pref, start)
            rec.update(
                {
                    "raw": raw[:80],
                    "rewritten": rewritten,
                    "new_id": new_id,
                    "new_ok": new_id == item.gold_id or new_id in item.accept_ids,
                }
            )
        rows.append(rec)

    gated_rows = [row for row in rows if row["gated"]]
    recovered = [
        row
        for row in gated_rows
        if (not row["base_ok"]) and row.get("new_ok")
    ]
    broken = [
        row for row in gated_rows if row["base_ok"] and not row.get("new_ok")
    ]
    still_miss = [
        row
        for row in gated_rows
        if (not row["base_ok"]) and not row.get("new_ok")
    ]
    summary = {
        "n_gold": len(rows),
        "n_gated": len(gated_rows),
        "gated_base_ok": sum(1 for row in gated_rows if row["base_ok"]),
        "recovered": len(recovered),
        "broken": len(broken),
        "still_miss": len(still_miss),
        "net": len(recovered) - len(broken),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("recovered")
    for row in recovered:
        print(row["uid"], row["gold_id"], row["base_id"], "->", row["new_id"], row["rewritten"])
    print("broken")
    for row in broken:
        print(row["uid"], row["gold_id"], row["base_id"], "->", row["new_id"], row["rewritten"])
    print("still_miss")
    for row in still_miss:
        print(
            row["uid"],
            row["gold_id"],
            row["base_id"],
            "->",
            row.get("new_id"),
            row.get("rewritten"),
        )
    out = PKG / "out" / "language_eval" / "probe_llm_rewrite.json"
    out.write_text(json.dumps({"summary": summary, "items": rows}, ensure_ascii=False, indent=2), encoding="utf-8")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
