#!/usr/bin/env python3
"""Write HF_TOKEN to ~/.bashrc and download Qwen2.5-1.5B-Instruct. Never prints the token."""

from __future__ import annotations

import re
from pathlib import Path

from huggingface_hub import get_token, hf_hub_download, list_repo_files, snapshot_download

MODEL_ID = "Qwen/Qwen2.5-1.5B-Instruct"
MARKER = "# Hugging Face token (scaffold_hrc)"


def write_bashrc(token: str) -> str:
    path = Path.home() / ".bashrc"
    text = path.read_text(encoding="utf-8") if path.is_file() else ""
    line = f'export HF_TOKEN="{token}"'
    block = f"\n{MARKER}\n{line}\n"
    if MARKER in text or re.search(r"^export HF_TOKEN=", text, re.MULTILINE):
        text = re.sub(
            r"(?:^# Hugging Face token \(scaffold_hrc\)\n)?^export HF_TOKEN=.*\n?",
            "",
            text,
            flags=re.MULTILINE,
        )
        text = text.rstrip() + block
        path.write_text(text, encoding="utf-8")
        return "updated"
    path.write_text(text.rstrip() + block, encoding="utf-8")
    return "appended"


def main() -> int:
    token = get_token()
    if not token:
        print("no Hugging Face token in cache")
        return 1
    action = write_bashrc(token)
    print("bashrc", action)
    local = snapshot_download(MODEL_ID, token=token)
    files = list_repo_files(MODEL_ID, token=token)
    weights = [
        name
        for name in files
        if name.endswith(".safetensors") or name.endswith(".bin")
    ]
    for name in weights:
        path = hf_hub_download(MODEL_ID, filename=name, token=token)
        print("weight", name, "bytes", Path(path).stat().st_size)
    print("model", MODEL_ID)
    print("path", local)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
