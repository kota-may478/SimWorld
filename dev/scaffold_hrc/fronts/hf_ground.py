"""Hugging Face backends for utterance grounding.

Public models load from the local hub cache when present. Tests inject fakes
and never download. Optional HF_TOKEN enables InferenceClient as a fallback.
"""

from __future__ import annotations

import json
import os
import re
from typing import Mapping, Optional, Sequence

from constraints.pareto import EvaluatedTheta
from fronts.ab_map import AbKey, snap_level, step_preference
from fronts.catalog import apply_catalog_label, entry_for, parse_catalog_label
from fronts.language import Preference, cell_for_levels, has_relative_cue
from fronts.staged_ground import preference_from_staged

HF_EMBED_MODEL = "sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2"
HF_INSTRUCT_MODEL = os.environ.get(
    "SCAFFOLD_HF_MODEL", "Qwen/Qwen2.5-1.5B-Instruct"
)

_JSON_RE = re.compile(r"\{.*\}", re.DOTALL)


class HuggingFaceEmbedder:
    """Nearest-phrase grounding via a SentenceTransformer from Hugging Face."""

    def __init__(self, model_id: str = HF_EMBED_MODEL, model: object | None = None):
        self.model_id = model_id
        self._model = model

    def encode(self, texts: Sequence[str]) -> list[list[float]]:
        if self._model is None:
            from sentence_transformers import SentenceTransformer

            self._model = SentenceTransformer(self.model_id)
        vectors = self._model.encode(list(texts), normalize_embeddings=True)
        return [list(map(float, row)) for row in vectors]


def _resolve_token() -> str | None:
    env = os.environ.get("HF_TOKEN") or os.environ.get("HUGGING_FACE_HUB_TOKEN")
    if env:
        return env
    try:
        from huggingface_hub import get_token

        return get_token()
    except Exception:
        return None


def _chat_via_api(model_id: str, prompt: str, token: str) -> str:
    from huggingface_hub import InferenceClient

    errors: list[str] = []
    clients = (
        InferenceClient(token=token),
        InferenceClient(provider="novita", token=token),
    )
    for client in clients:
        try:
            reply = client.chat_completion(
                messages=[{"role": "user", "content": prompt}],
                model=model_id,
                max_tokens=96,
            )
            choice = reply.choices[0].message.content
            return str(choice or "")
        except Exception as exc:
            errors.append(f"{type(exc).__name__}: {exc}")
    raise RuntimeError("Hugging Face chat failed: " + " | ".join(errors[:2]))


def _pipe_text(pipe: object, prompt: str, *, max_new_tokens: int = 96) -> str:
    tokenizer = getattr(pipe, "tokenizer", None)
    rendered = prompt
    if tokenizer is not None and hasattr(tokenizer, "apply_chat_template"):
        rendered = tokenizer.apply_chat_template(
            [{"role": "user", "content": prompt}],
            tokenize=False,
            add_generation_prompt=True,
        )
    out = pipe(rendered, max_new_tokens=max_new_tokens)  # type: ignore[operator]
    if isinstance(out, list) and out:
        first = out[0]
        if isinstance(first, dict):
            return str(first.get("generated_text", first))
    return str(out)


def _load_local_pipe(model_id: str) -> object:
    import torch
    from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline

    tokenizer = AutoTokenizer.from_pretrained(model_id, local_files_only=True)
    model = AutoModelForCausalLM.from_pretrained(
        model_id,
        local_files_only=True,
        torch_dtype=torch.float32,
    )
    if getattr(model, "generation_config", None) is not None:
        model.generation_config.max_length = None
    return pipeline(
        "text-generation",
        model=model,
        tokenizer=tokenizer,
        device="cpu",
        return_full_text=False,
    )


class HuggingFaceInstruct:
    """Qwen chat: local weights if cached, else Hugging Face Inference API."""

    def __init__(self, model_id: str = HF_INSTRUCT_MODEL, model: object | None = None):
        self.model_id = model_id
        self._pipe = model

    def generate(self, prompt: str, *, max_new_tokens: int = 32) -> str:
        if self._pipe is not None:
            return _pipe_text(self._pipe, prompt, max_new_tokens=max_new_tokens)
        try:
            self._pipe = _load_local_pipe(self.model_id)
            return _pipe_text(self._pipe, prompt, max_new_tokens=max_new_tokens)
        except Exception:
            token = _resolve_token()
            if not token:
                raise RuntimeError(
                    "Hugging Face token missing and local model is not available"
                )
            return _chat_via_api(self.model_id, prompt, token)


def preference_from_llm(
    text: str,
    generator: HuggingFaceInstruct,
    *,
    state: Preference,
    theta_table: Mapping[AbKey, EvaluatedTheta] | None = None,
) -> Optional[Preference]:
    staged = preference_from_staged(
        text, generator, state=state, theta_table=theta_table
    )
    if staged is not None:
        return staged
    try:
        raw = generator.generate(
            "Map the utterance to JSON with keys kind, alpha, beta, d_alpha, d_beta.\n"
            f"Utterance: {text}\nJSON:",
            max_new_tokens=48,
        )
    except TypeError:
        raw = generator.generate(
            "Map the utterance to JSON with keys kind, alpha, beta, d_alpha, d_beta.\n"
            f"Utterance: {text}\nJSON:"
        )
    label = parse_catalog_label(raw)
    if label is not None:
        item = entry_for(label)
        if not (item.kind == "relative" and not has_relative_cue(text)):
            return apply_catalog_label(label, state, theta_table=theta_table)
    match = _JSON_RE.search(raw)
    if match is None:
        return None
    try:
        payload = json.loads(match.group(0))
    except json.JSONDecodeError:
        return None
    kind = str(payload.get("kind", "absolute")).lower()
    if kind == "reset":
        return Preference(0.5, 0.5, kind="reset", cell="normal", notes="llm-reset")
    if kind == "relative":
        if not has_relative_cue(text):
            kind = "absolute"
        else:
            try:
                d_alpha = int(payload.get("d_alpha", 0))
                d_beta = int(payload.get("d_beta", 0))
            except (TypeError, ValueError):
                return None
            alpha, beta = step_preference(
                state.alpha,
                state.beta,
                d_alpha=d_alpha,
                d_beta=d_beta,
                theta_table=theta_table,
            )
            return Preference(
                alpha,
                beta,
                kind="relative",
                cell=cell_for_levels(alpha, beta),
                notes="llm-relative",
            )
    if kind != "relative":
        try:
            alpha = snap_level(float(payload.get("alpha", state.alpha)))
            beta = snap_level(float(payload.get("beta", state.beta)))
        except (TypeError, ValueError):
            return None
        return Preference(
            alpha,
            beta,
            kind="absolute",
            cell=cell_for_levels(alpha, beta),
            notes="llm-absolute",
        )
    return None
