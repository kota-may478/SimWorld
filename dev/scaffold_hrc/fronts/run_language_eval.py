#!/usr/bin/env python3
"""Evaluate catalog LLM vs closed baselines on the gold utterance set."""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from constraints.pareto import EvaluatedTheta, Theta  # noqa: E402
from fronts.ab_map import enumerate_table, pick_by_lambda, pick_weighted  # noqa: E402
from fronts.baselines_language import (  # noqa: E402
    alpha_1d,
    discrete_3mode,
    nearest_iso_theta,
    parse_continuous_ab,
    parse_direct_theta,
    parse_lambda3,
    pick_continuous_ab,
    speed_scale,
)
from fronts.catalog import apply_catalog_label, pref_to_catalog_id  # noqa: E402
from fronts.gold_language import gold_items  # noqa: E402
from fronts.hf_ground import HuggingFaceInstruct  # noqa: E402
from fronts.language import Preference, ground_utterance  # noqa: E402

DEFAULT_FRONT = PKG / "out" / "20260904203058" / "fronts.json"
OUT_DIR = PKG / "out" / "language_eval"


def load_nsga2_rows(path: Path) -> tuple[EvaluatedTheta, ...]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    items = payload["methods"]["nsga2"]["iso_pareto"]
    rows: list[EvaluatedTheta] = []
    for item in items:
        rows.append(
            EvaluatedTheta(
                Theta(vmax_mps=item["vmax_mps"], dmin_m=item["dmin_m"]),
                tt=item["tt"],
                t_ssm=item.get("t_ssm", item["vmax_mps"]),
                si_min=item.get("si_min", 1.0),
                completed=bool(item.get("completed", True)),
                min_sep_m=item.get("min_sep_m", 0.0),
            )
        )
    return tuple(rows)


def theta_key(row: EvaluatedTheta) -> tuple[float, float]:
    return (round(row.theta.vmax_mps, 4), round(row.theta.dmin_m, 4))


def score_pref(
    pred_id: str,
    pref: Preference,
    gold_id: str,
    accept: tuple[str, ...],
    gold_pref: Preference,
    gold_row: EvaluatedTheta,
    pred_row: EvaluatedTheta,
) -> dict:
    return {
        "pred_id": pred_id,
        "gold_id": gold_id,
        "strict": pred_id == gold_id,
        "relaxed": pred_id in accept,
        "ab_match": abs(pref.alpha - gold_pref.alpha) < 1e-9
        and abs(pref.beta - gold_pref.beta) < 1e-9,
        "theta_match": theta_key(pred_row) == theta_key(gold_row),
        "alpha": pref.alpha,
        "beta": pref.beta,
        "vmax": pred_row.theta.vmax_mps,
        "dmin": pred_row.theta.dmin_m,
    }


DIRECT_PROMPT = (
    "Output JSON only with vmax_mps in [0.2, 1.0] and dmin_m in [0.35, 1.6] "
    "for a scaffold robot. Smaller vmax is slower. Larger dmin is farther.\n"
    "Utterance: {text}\nJSON:"
)
LAMBDA_PROMPT = (
    'Output JSON only: {{"lambda": [w_tt, w_vmax, w_sep]}} with weights in [0,1]. '
    "w_tt minimizes completion time, w_vmax prefers slow speed, "
    "w_sep prefers large human-robot gap.\n"
    "Utterance: {text}\nJSON:"
)
AB_PROMPT = (
    'Output JSON only: {{"alpha": x, "beta": y}} with x,y in [0,1]. '
    "alpha=0 efficiency, alpha=1 safety. beta=0 distance, beta=1 slow.\n"
    "Utterance: {text}\nJSON:"
)


def summarize(rows: list[dict]) -> dict:
    families: dict[str, list[dict]] = defaultdict(list)
    for row in rows:
        families[row["family"]].append(row)
        families["all"].append(row)
    out: dict[str, dict] = {}
    for name, packed in families.items():
        n = len(packed)
        out[name] = {
            "n": n,
            "strict": sum(1 for x in packed if x["strict"]) / n,
            "relaxed": sum(1 for x in packed if x["relaxed"]) / n,
            "ab_match": sum(1 for x in packed if x["ab_match"]) / n,
            "theta_match": sum(1 for x in packed if x["theta_match"]) / n,
        }
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--front-json", type=Path, default=DEFAULT_FRONT)
    parser.add_argument(
        "--proposed-only",
        action="store_true",
        help="Score keyword+embed proposed plus keyword; skip Qwen and other LLM baselines",
    )
    args = parser.parse_args()
    front_path = args.front_json
    rows = load_nsga2_rows(front_path)
    table = enumerate_table(rows)
    items = gold_items()
    generator = None
    embedder = None
    try:
        from fronts.hf_ground import HuggingFaceEmbedder

        embedder = HuggingFaceEmbedder()
    except Exception as exc:
        print("embedder unavailable", type(exc).__name__)
    if not args.proposed_only:
        generator = HuggingFaceInstruct()

    method_rows: dict[str, list[dict]] = defaultdict(list)

    def record(method: str, item, pred_id: str, pref: Preference, pred_row: EvaluatedTheta) -> None:
        start = Preference(item.start_alpha, item.start_beta)
        gold_pref = apply_catalog_label(item.gold_id, start, theta_table=table)
        gold_row = pick_weighted(rows, gold_pref.alpha, gold_pref.beta)
        scored = score_pref(
            pred_id,
            pref,
            item.gold_id,
            item.accept_ids,
            gold_pref,
            gold_row,
            pred_row,
        )
        scored.update(
            {
                "uid": item.uid,
                "text": item.text,
                "family": item.family,
                "notes": pref.notes,
            }
        )
        method_rows[method].append(scored)

    for item in items:
        start = Preference(item.start_alpha, item.start_beta)
        gold_pref = apply_catalog_label(item.gold_id, start, theta_table=table)

        proposed = ground_utterance(
            item.text, state=start, embedder=embedder, theta_table=table
        )
        pred_id = pref_to_catalog_id(proposed, start)
        record(
            "proposed",
            item,
            pred_id,
            proposed,
            pick_weighted(rows, proposed.alpha, proposed.beta),
        )

        keyed = ground_utterance(item.text, state=start, theta_table=table)
        record(
            "keyword",
            item,
            pref_to_catalog_id(keyed, start),
            keyed,
            pick_weighted(rows, keyed.alpha, keyed.beta),
        )

        if args.proposed_only:
            print(item.uid, pred_to_status(method_rows["proposed"][-1]), flush=True)
            continue

        if generator is not None:
            qwen = ground_utterance(
                item.text, state=start, generator=generator, theta_table=table
            )
            record(
                "proposed_qwen",
                item,
                pref_to_catalog_id(qwen, start),
                qwen,
                pick_weighted(rows, qwen.alpha, qwen.beta),
            )

        mode = discrete_3mode(item.text, start)
        record(
            "discrete_3mode",
            item,
            pref_to_catalog_id(mode, start),
            mode,
            pick_weighted(rows, mode.alpha, mode.beta),
        )

        one = alpha_1d(item.text, start)
        record(
            "alpha_1d",
            item,
            pref_to_catalog_id(one, start),
            one,
            pick_weighted(rows, one.alpha, one.beta),
        )

        scaled = speed_scale(item.text, start)
        record(
            "speed_scale",
            item,
            pref_to_catalog_id(scaled, start),
            scaled,
            pick_weighted(rows, scaled.alpha, scaled.beta),
        )

        if embedder is not None:
            from fronts.language import _nearest_cell

            near, _score = _nearest_cell(item.text, embedder)
            record(
                "embed_pure",
                item,
                pref_to_catalog_id(near, start),
                near,
                pick_weighted(rows, near.alpha, near.beta),
            )

        raw_theta = generator.generate(DIRECT_PROMPT.format(text=item.text), max_new_tokens=48)
        theta = parse_direct_theta(raw_theta)
        if theta is None:
            pref = Preference(
                start.alpha, start.beta, kind="unchanged", cell="normal", notes="miss"
            )
            pred_row = pick_weighted(rows, pref.alpha, pref.beta)
            pred_id = "unchanged"
        else:
            pred_row = nearest_iso_theta(theta.vmax_mps, theta.dmin_m, rows)
            pref = Preference(
                gold_pref.alpha,
                gold_pref.beta,
                kind="absolute",
                cell="normal",
                notes="direct-theta",
            )
            pred_id = "direct_theta"
        record("qwen_direct_theta", item, pred_id, pref, pred_row)

        raw_l = generator.generate(LAMBDA_PROMPT.format(text=item.text), max_new_tokens=48)
        lam = parse_lambda3(raw_l)
        if lam is None:
            pref = Preference(
                start.alpha, start.beta, kind="unchanged", cell="normal", notes="miss"
            )
            pred_row = pick_weighted(rows, pref.alpha, pref.beta)
            pred_id = "unchanged"
        else:
            total = sum(max(0.0, x) for x in lam) or 1.0
            weights = tuple(max(0.0, x) / total for x in lam)
            pred_row = pick_by_lambda(rows, weights)  # type: ignore[arg-type]
            pref = Preference(0.5, 0.5, kind="absolute", cell="normal", notes="lambda")
            pred_id = "lambda"
        record("qwen_lambda", item, pred_id, pref, pred_row)

        raw_ab = generator.generate(AB_PROMPT.format(text=item.text), max_new_tokens=32)
        ab = parse_continuous_ab(raw_ab)
        if ab is None:
            pref = Preference(
                start.alpha, start.beta, kind="unchanged", cell="normal", notes="miss"
            )
            pred_id = "unchanged"
        else:
            pref = Preference(ab[0], ab[1], kind="absolute", cell="normal", notes="cont-ab")
            pred_id = "continuous_ab"
        pred_row = pick_continuous_ab(rows, pref.alpha, pref.beta)
        record("qwen_continuous_ab", item, pred_id, pref, pred_row)

        print(item.uid, pred_to_status(method_rows["proposed"][-1]), flush=True)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    summary = {name: summarize(vals) for name, vals in method_rows.items()}
    payload = {
        "front": str(front_path),
        "n_gold": len(items),
        "n_nsga2_iso": len(rows),
        "proposed_only": args.proposed_only,
        "summary": summary,
        "items": {name: vals for name, vals in method_rows.items()},
    }
    out_path = OUT_DIR / ("results_staged.json" if args.proposed_only else "results.json")
    out_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(out_path)
    return 0


def pred_to_status(row: dict) -> str:
    mark = "ok" if row["relaxed"] else "miss"
    return f"{mark} {row['gold_id']}->{row['pred_id']}"


if __name__ == "__main__":
    raise SystemExit(main())
