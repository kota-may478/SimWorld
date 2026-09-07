"""Staged catalog mapping: kind, then 3-level alpha/beta or relative steps."""

from __future__ import annotations

from typing import Mapping, Optional, Sequence

from fronts.ab_map import AbKey
from fronts.catalog import apply_catalog_label, parse_from_options
from fronts.language import Preference, cell_for_levels, has_relative_cue
from constraints.pareto import EvaluatedTheta

KINDS = ("absolute", "relative", "reset", "none")
ALPHAS = ("efficiency", "balanced", "safety")
BETAS = ("distance", "middle", "slow")
DIRECTIONS = ("slower", "faster", "farther", "safer")
STRENGTHS = ("a_bit", "much")
ALPHA_VALUE = {"efficiency": 0.0, "balanced": 0.5, "safety": 1.0}
BETA_VALUE = {"distance": 0.0, "middle": 0.5, "slow": 1.0}
RELATIVE_IDS = {
    ("slower", "a_bit"): "a_bit_slower",
    ("slower", "much"): "much_slower",
    ("faster", "a_bit"): "a_bit_faster",
    ("faster", "much"): "much_faster",
    ("farther", "a_bit"): "a_bit_farther",
    ("farther", "much"): "much_farther",
    ("safer", "a_bit"): "a_bit_safer",
    ("safer", "much"): "much_safer",
}
MAX_RETRIES = 2

KIND_PROMPT = (
    "Classify the scaffold-robot operator utterance.\n"
    "Options: absolute, relative, reset, none\n"
    "- absolute = a new overall style. Default if unsure. "
    "Examples: 急いで, ゆっくり動いて, stay back, be careful, 普通に.\n"
    "- relative = only a tweak of the CURRENT setting. Choose relative ONLY if "
    "the utterance contains もう少し / もっと / a bit / a little / かなり / much / a lot. "
    "Example: もう少しゆっくり, a bit farther.\n"
    "Bare ゆっくり / go slow / 減速 / stay back without those words is absolute.\n"
    "- reset = リセット, いつも通り, as usual, reset\n"
    "- none = not about speed or distance\n"
    "Do not choose relative unless the user is changing the current setting.\n"
    "Examples:\n"
    "Utterance: 急いで\nKind: absolute\n"
    "Utterance: ゆっくり動いて\nKind: absolute\n"
    "Utterance: stay back\nKind: absolute\n"
    "Utterance: please go slow\nKind: absolute\n"
    "Utterance: 減速して\nKind: absolute\n"
    "Utterance: もう少しゆっくり\nKind: relative\n"
    "Utterance: a bit slower\nKind: relative\n"
    "Utterance: リセット\nKind: reset\n"
    "Utterance: 今日の天気は？\nKind: none\n"
    "Utterance: {text}\n"
    "Kind:"
)
ALPHA_PROMPT = (
    "Pick the efficiency-versus-safety level. Reply with one option.\n"
    "Options: efficiency, balanced, safety\n"
    "- efficiency = hurry, finish fast, 急いで, 早く\n"
    "- balanced = normal, 普通, as usual, otherwise normal\n"
    "- safety = careful, stay back, go slow, 慎重, 安全, 離れて, ゆっくり\n"
    "Examples:\n"
    "Utterance: 急いで\nAlpha: efficiency\n"
    "Utterance: 普通に\nAlpha: balanced\n"
    "Utterance: stay back\nAlpha: safety\n"
    "Utterance: please go slow\nAlpha: safety\n"
    "Utterance: be careful\nAlpha: safety\n"
    "Utterance: {text}\n"
    "Alpha:"
)
BETA_PROMPT = (
    "Inside that safety level, pick distance versus slow. Reply with one option.\n"
    "Options: distance, middle, slow\n"
    "- distance = stay farther, more space, 離れて, 距離\n"
    "- middle = careful but not specifically far or slow, 慎重に, 安全に\n"
    "- slow = move slowly, ゆっくり, 減速, go slow\n"
    "If the user asked only to hurry / be efficient, choose middle.\n"
    "Examples:\n"
    "Utterance: 離れて作業して\nBeta: distance\n"
    "Utterance: please go slow\nBeta: slow\n"
    "Utterance: be careful\nBeta: middle\n"
    "Utterance: 急いで\nBeta: middle\n"
    "Utterance: {text}\n"
    "Beta:"
)
DIR_PROMPT = (
    "The user wants a relative tweak from the current setting.\n"
    "Options: slower, faster, farther, safer\n"
    "Reply with one option.\n"
    "Utterance: {text}\n"
    "Direction:"
)
STR_PROMPT = (
    "How strong is the relative tweak?\n"
    "Options: a_bit, much\n"
    "- a_bit = もう少し, a bit, a little\n"
    "- much = かなり, もっと, much, a lot\n"
    "Reply with one option.\n"
    "Utterance: {text}\n"
    "Strength:"
)


def _generate(generator: object, prompt: str, max_new_tokens: int) -> str:
    try:
        return generator.generate(prompt, max_new_tokens=max_new_tokens)
    except TypeError:
        return generator.generate(prompt)


def _ask(
    generator: object,
    prompt: str,
    options: Sequence[str],
    *,
    max_new_tokens: int = 8,
) -> Optional[str]:
    extra = " Reply with exactly one of: " + ", ".join(options) + "."
    text = prompt
    for _ in range(MAX_RETRIES):
        raw = _generate(generator, text, max_new_tokens)
        hit = parse_from_options(raw, options)
        if hit is not None:
            return hit
        text = prompt + extra
    return None


def cell_from_alpha_beta_labels(alpha_id: str, beta_id: str) -> str:
    return cell_for_levels(ALPHA_VALUE[alpha_id], BETA_VALUE[beta_id])


def staged_catalog_label(text: str, generator: object) -> Optional[str]:
    kind = _ask(generator, KIND_PROMPT.format(text=text), KINDS)
    if kind == "relative" and not has_relative_cue(text):
        kind = "absolute"
    if kind is None or kind == "none":
        return "unchanged" if kind == "none" else None
    if kind == "reset":
        return "reset"
    if kind == "absolute":
        alpha_id = _ask(generator, ALPHA_PROMPT.format(text=text), ALPHAS)
        if alpha_id is None:
            return None
        beta_id = _ask(generator, BETA_PROMPT.format(text=text), BETAS)
        if beta_id is None:
            return None
        return cell_from_alpha_beta_labels(alpha_id, beta_id)
    direction = _ask(generator, DIR_PROMPT.format(text=text), DIRECTIONS)
    if direction is None:
        return None
    strength = _ask(generator, STR_PROMPT.format(text=text), STRENGTHS)
    if strength is None:
        return None
    return RELATIVE_IDS[(direction, strength)]


def preference_from_staged(
    text: str,
    generator: object,
    *,
    state: Preference,
    theta_table: Mapping[AbKey, EvaluatedTheta] | None = None,
) -> Optional[Preference]:
    label = staged_catalog_label(text, generator)
    if label is None:
        return None
    return apply_catalog_label(label, state, theta_table=theta_table)
