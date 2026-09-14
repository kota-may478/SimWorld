#!/usr/bin/env python3
"""DEPRECATED entrypoint — see docs/ue_validation_design.md and ue/hrc_mission.py.

The 2026-09-08 teleport runner is not valid for paper UE numbers.
"""

from __future__ import annotations

import sys


def main() -> int:
    print(
        "REFUSED: ue/run_mission_theta.py teleport runner is deprecated.\n"
        "Pareto P is offline-only (oracle). UE must use ue/hrc_mission.py:\n"
        "  Spot yaw+speed locomotion, 2 humanoids, SSM/d_min stop-wait,\n"
        "  progressive member placement.\n"
        "See docs/ue_validation_design.md"
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
