#!/usr/bin/env python3
"""Phase A 完了ログ（ターミナル出力ファイル）の要約を表示。"""

from __future__ import annotations

import re
import sys
from pathlib import Path


def main() -> int:
    log_path = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    if log_path is None:
        candidates = sorted(
            Path(
                "/home/winder17wsl_ishizawalab/.cursor/projects/"
                "home-winder17wsl-ishizawalab-00-kotaprivate-Program-SimWorld/terminals"
            ).glob("*.txt"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        for p in candidates:
            text = p.read_text(encoding="utf-8", errors="replace")
            if "10000/10000" in text and "[Phase1] SUCCESS" in text:
                log_path = p
                break
    if log_path is None or not log_path.is_file():
        print("No Phase1 success log found", file=sys.stderr)
        return 1

    text = log_path.read_text(encoding="utf-8", errors="replace")
    print(f"[Phase1 log] file: {log_path}")

    for pat, label in [
        (r"\[Blocks\] done: (\d+/\d+).*", "blocks"),
        (r"\[Phase1\] SUCCESS[^\n]*", "result"),
        (r"exit_code: (\d+)", "exit"),
        (r"block_001_001.*", "sample_1_1"),
        (r"block_100_100.*", "sample_100_100"),
    ]:
        m = re.search(pat, text)
        if m:
            print(f"  {label}: {m.group(0).strip()[:120]}")

    if "10000/10000" in text and "[Phase1] SUCCESS" in text:
        print("[Phase1 log] VERIFIED OK")
        return 0
    print("[Phase1 log] INCOMPLETE", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
