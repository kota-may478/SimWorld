"""Clear notebook outputs so nbconvert validate passes (stream needs name field)."""

from __future__ import annotations

import json
from pathlib import Path

NB = Path(__file__).resolve().parent / "grid_env_hri_simulation.ipynb"


def main() -> None:
    nb = json.loads(NB.read_text(encoding="utf-8"))
    for cell in nb.get("cells", []):
        if cell.get("cell_type") != "code":
            cell.pop("outputs", None)
            cell.pop("execution_count", None)
            continue
        cell["outputs"] = []
        cell["execution_count"] = None
    NB.write_text(json.dumps(nb, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Cleared outputs: {NB}")


if __name__ == "__main__":
    main()
