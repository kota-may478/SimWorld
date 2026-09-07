#!/usr/bin/env python3
"""Ground a language preference onto the 5×5 (α, β) lookup."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from constraints.pareto import EvaluatedTheta, Theta  # noqa: E402
from fronts.grounding import proposed_from_text  # noqa: E402
from fronts.language import Preference, ground_utterance  # noqa: E402


def rows_from_dump(items: list[dict]) -> tuple[EvaluatedTheta, ...]:
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Map language to θ* via (α, β)")
    parser.add_argument("--text", required=True)
    parser.add_argument("--front-json", type=Path, default=None)
    parser.add_argument("--state-alpha", type=float, default=0.5)
    parser.add_argument("--state-beta", type=float, default=0.5)
    parser.add_argument("--embed", action="store_true", help="Hugging Face embeddings")
    parser.add_argument("--llm", action="store_true", help="Hugging Face instruct model")
    args = parser.parse_args()

    embedder = None
    generator = None
    if args.embed:
        from fronts.hf_ground import HuggingFaceEmbedder

        embedder = HuggingFaceEmbedder()
    if args.llm:
        from fronts.hf_ground import HuggingFaceInstruct

        generator = HuggingFaceInstruct()

    state = Preference(args.state_alpha, args.state_beta)
    if args.front_json is None:
        pref = ground_utterance(
            args.text, state=state, embedder=embedder, generator=generator
        )
        print(json.dumps(pref.__dict__, ensure_ascii=False, indent=2))
        return 0

    payload = json.loads(args.front_json.read_text(encoding="utf-8"))
    items = payload.get("combined_iso_pareto") or payload.get("iso_pareto") or []
    rows = rows_from_dump(items)
    if not rows:
        print("empty ISO front in JSON", file=sys.stderr)
        return 1
    result = proposed_from_text(
        args.text,
        rows,
        state=state,
        embedder=embedder,
        generator=generator,
    )
    print(
        json.dumps(
            {
                "alpha": result.alpha,
                "beta": result.beta,
                "cell": result.cell,
                "notes": result.notes,
                "theta": result.theta.as_dict(),
                "tt": result.tt,
                "min_sep_m": result.min_sep_m,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
