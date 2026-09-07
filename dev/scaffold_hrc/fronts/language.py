"""3×3 absolute phrases and 5×5 relative steps. Hugging Face is optional."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Mapping, Optional, Protocol, Sequence, Tuple

from constraints.pareto import EvaluatedTheta
from fronts.ab_map import AbKey, snap_level, step_preference

CellId = str


@dataclass(frozen=True)
class Preference:
    alpha: float
    beta: float
    kind: str = "absolute"
    cell: str = ""
    notes: str = ""


@dataclass(frozen=True)
class AbsoluteCell:
    cell_id: CellId
    alpha: float
    beta: float
    phrases: Tuple[str, ...]


ABSOLUTE_CELLS: Tuple[AbsoluteCell, ...] = (
    AbsoluteCell(
        "efficient",
        0.0,
        0.5,
        ("急いで", "早く", "効率よく", "hurry", "faster", "be efficient"),
    ),
    AbsoluteCell(
        "normal_distance",
        0.5,
        0.0,
        ("普通で近づきすぎない", "keep space, otherwise normal"),
    ),
    AbsoluteCell(
        "normal",
        0.5,
        0.5,
        ("いつも通り", "普通に", "as usual", "normal", "balanced"),
    ),
    AbsoluteCell(
        "normal_slow",
        0.5,
        1.0,
        ("普通だが急がない", "unhurried, otherwise normal"),
    ),
    AbsoluteCell(
        "safe_distance",
        1.0,
        0.0,
        ("離れて作業して", "距離を取って", "stay back", "keep more distance"),
    ),
    AbsoluteCell(
        "safe",
        1.0,
        0.5,
        ("慎重に", "安全に", "be careful", "cautiously"),
    ),
    AbsoluteCell(
        "safe_slow",
        1.0,
        1.0,
        ("ゆっくり動いて", "減速して", "go slow", "please go slow"),
    ),
)


RELATIVE_PHRASES = (
    "もう少し",
    "もうすこし",
    "a bit",
    "a little",
    "かなり",
    "a lot",
    "もっと",
)
_MUCH_RE = re.compile(r"\bmuch\b", re.IGNORECASE)


def has_relative_cue(text: str) -> bool:
    blob = text.strip().lower()
    if any(phrase.lower() in blob for phrase in RELATIVE_PHRASES):
        return True
    return _MUCH_RE.search(blob) is not None


class Embedder(Protocol):
    def encode(self, texts: Sequence[str]) -> Sequence[Sequence[float]]:
        ...


class Generator(Protocol):
    def generate(self, prompt: str) -> str:
        ...


def coarsen_level(value: float) -> float:
    snapped = snap_level(value)
    if snapped < 0.25:
        return 0.0
    if snapped < 0.75:
        return 0.5
    return 1.0


def cell_for_levels(alpha: float, beta: float) -> CellId:
    a = coarsen_level(alpha)
    b = coarsen_level(beta)
    if a <= 1e-12:
        return "efficient"
    for cell in ABSOLUTE_CELLS:
        if abs(cell.alpha - a) < 1e-12 and abs(cell.beta - b) < 1e-12:
            return cell.cell_id
    return "normal"


def parse_relative(text: str) -> Optional[Tuple[int, int, str]]:
    blob = text.strip().lower()
    mild = any(token in blob for token in ("もう少し", "もうすこし", "a bit", "a little"))
    strong = any(token in blob for token in ("もっと", "かなり", "a lot"))
    if _MUCH_RE.search(blob) is not None:
        strong = True
    if not mild and not strong:
        if "少し" in blob:
            mild = True
        else:
            return None
    strength = 1 if mild else 2
    if any(token in blob for token in ("ゆっくり", "slow", "減速", "遅く")):
        return 0, strength, "relative slower"
    if any(token in blob for token in ("離れ", "距離", "far", "stay back", "distance")):
        return 0, -strength, "relative farther"
    if any(token in blob for token in ("急い", "hurry", "早く", "fast")):
        return -strength, 0, "relative faster"
    if any(token in blob for token in ("慎重", "安全", "careful", "cautious")):
        return strength, 0, "relative more careful"
    return None


def _keyword_absolute(text: str) -> Optional[Preference]:
    blob = text.strip().lower()
    if any(token in blob for token in ("リセット", "reset")):
        return Preference(0.5, 0.5, kind="reset", cell="normal", notes="reset")
    if any(token in blob for token in ("いつも通り", "as usual")):
        return Preference(0.5, 0.5, kind="reset", cell="normal", notes="reset")
    if any(token in blob for token in ("ゆっくり", "go slow", "減速")):
        if "普通" in blob or "otherwise normal" in blob:
            return Preference(0.5, 1.0, cell="normal_slow", notes="keyword")
        return Preference(1.0, 1.0, cell="safe_slow", notes="keyword")
    if any(token in blob for token in ("距離", "離れて", "stay back", "keep space", "keep more distance")):
        if "普通" in blob:
            return Preference(0.5, 0.0, cell="normal_distance", notes="keyword")
        return Preference(1.0, 0.0, cell="safe_distance", notes="keyword")
    if any(token in blob for token in ("急い", "hurry", "早く", "効率", "faster", "efficient")):
        return Preference(0.0, 0.5, cell="efficient", notes="keyword")
    if any(token in blob for token in ("慎重", "安全", "careful", "cautious")):
        return Preference(1.0, 0.5, cell="safe", notes="keyword")
    if any(token in blob for token in ("普通", "normal", "balanced")):
        return Preference(0.5, 0.5, cell="normal", notes="keyword")
    return None


def _cosine(left: Sequence[float], right: Sequence[float]) -> float:
    num = sum(a * b for a, b in zip(left, right))
    n1 = sum(a * a for a in left) ** 0.5
    n2 = sum(b * b for b in right) ** 0.5
    if n1 < 1e-12 or n2 < 1e-12:
        return 0.0
    return num / (n1 * n2)


EMBED_MIN_COSINE = 0.40
GATE_MARGIN = 0.05

PREF_LEXICON = (
    "ゆっくり",
    "遅く",
    "減速",
    "速度",
    "速さ",
    "急い",
    "早く",
    "効率",
    "時間",
    "優先",
    "離れ",
    "距離",
    "近づ",
    "慎重",
    "安全",
    "危険",
    "危な",
    "普通",
    "slow",
    "fast",
    "hurry",
    "speed",
    "rush",
    "unhurried",
    "distance",
    "far",
    "stay back",
    "space",
    "berth",
    "away",
    "careful",
    "cautious",
    "safety",
    "efficient",
    "worker",
)

PREF_GATE_PHRASES = (
    "ロボットの速さを変えて",
    "人と距離を取って作業して",
    "慎重に動いて",
    "効率よく終わらせて",
    "change how fast the robot moves",
    "keep more space from the worker",
    "be more careful around people",
    "finish the job faster",
)

OOD_GATE_PHRASES = (
    "こんにちは",
    "感謝しています",
    "今日は晴れている？",
    "面白い話をして",
    "足場の組み方を説明して",
    "hello there",
    "thanks a lot",
    "what is for lunch",
    "say something funny",
    "explain how to assemble scaffolding",
)


def has_preference_lexicon(text: str) -> bool:
    blob = text.strip().lower()
    return any(token in blob for token in PREF_LEXICON)


def contrastive_preference_gate(
    text: str, embedder: Embedder
) -> tuple[bool, float, float]:
    packed = (text, *PREF_GATE_PHRASES, *OOD_GATE_PHRASES)
    vectors = list(embedder.encode(packed))
    query = vectors[0]
    n_pref = len(PREF_GATE_PHRASES)
    pref_sim = max(_cosine(query, vec) for vec in vectors[1 : 1 + n_pref])
    ood_sim = max(_cosine(query, vec) for vec in vectors[1 + n_pref :])
    passed = (pref_sim - ood_sim) >= GATE_MARGIN
    return passed, pref_sim, ood_sim


def is_preference_utterance(text: str, embedder: Embedder | None) -> bool:
    if has_preference_lexicon(text):
        return True
    if embedder is None:
        return False
    passed, _pref_sim, _ood_sim = contrastive_preference_gate(text, embedder)
    return passed


def _embed_absolute(text: str, embedder: Embedder) -> Optional[Preference]:
    if not is_preference_utterance(text, embedder):
        return None
    near, score = _nearest_cell(text, embedder)
    if score < EMBED_MIN_COSINE:
        return None
    return near


def _nearest_cell(text: str, embedder: Embedder) -> tuple[Preference, float]:
    phrases: list[str] = []
    owners: list[AbsoluteCell] = []
    for cell in ABSOLUTE_CELLS:
        for phrase in cell.phrases:
            phrases.append(phrase)
            owners.append(cell)
    vectors = list(embedder.encode([text, *phrases]))
    query = vectors[0]
    best_i = 0
    best = -1.0
    for i, vec in enumerate(vectors[1:]):
        score = _cosine(query, vec)
        if score > best:
            best = score
            best_i = i
    cell = owners[best_i]
    pref = Preference(
        cell.alpha,
        cell.beta,
        kind="absolute",
        cell=cell.cell_id,
        notes=f"embed:{cell.cell_id}:{best:.3f}",
    )
    return pref, best


def ground_utterance(
    text: str,
    *,
    state: Preference | None = None,
    embedder: Embedder | None = None,
    generator: Generator | None = None,
    theta_table: Mapping[AbKey, EvaluatedTheta] | None = None,
) -> Preference:
    blob = text.strip()
    if not blob:
        raise ValueError("empty utterance")
    current = state or Preference(0.5, 0.5, kind="reset", cell="normal")
    if generator is not None:
        from fronts.hf_ground import preference_from_llm

        parsed = preference_from_llm(
            blob, generator, state=current, theta_table=theta_table
        )
        if parsed is not None:
            return parsed
        if embedder is not None:
            near = _embed_absolute(blob, embedder)
            if near is not None:
                return near
        return Preference(
            current.alpha,
            current.beta,
            kind="unchanged",
            cell=cell_for_levels(current.alpha, current.beta),
            notes="no match",
        )
    if any(token in blob.lower() for token in ("リセット", "reset")):
        return Preference(0.5, 0.5, kind="reset", cell="normal", notes="reset")
    relative = parse_relative(blob)
    if relative is not None:
        d_alpha, d_beta, note = relative
        from fronts.catalog import relative_steps_to_label

        label = relative_steps_to_label(d_alpha, d_beta)
        alpha, beta = step_preference(
            current.alpha,
            current.beta,
            d_alpha=d_alpha,
            d_beta=d_beta,
            theta_table=theta_table,
        )
        return Preference(
            alpha,
            beta,
            kind="relative",
            cell=cell_for_levels(alpha, beta),
            notes=f"catalog-{label}" if label else note,
        )
    if "いつも通り" in blob or "as usual" in blob.lower():
        return Preference(0.5, 0.5, kind="reset", cell="normal", notes="reset")
    keyed = _keyword_absolute(blob)
    if keyed is not None:
        return keyed
    if embedder is not None:
        near = _embed_absolute(blob, embedder)
        if near is not None:
            return near
    return Preference(
        current.alpha,
        current.beta,
        kind="unchanged",
        cell=cell_for_levels(current.alpha, current.beta),
        notes="no match",
    )
