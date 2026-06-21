#!/usr/bin/env python3
"""DEPRECATED for UE 5.3 — use rebuild_generated_level_props_editor.py instead.

repair tried to patch inherited PropMesh overrides, which fails on UE 5.3
(Output Log: fixed=0 failed=73). This script now forwards to rebuild.
"""
from __future__ import annotations

import os
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

import unreal

import rebuild_generated_level_props_editor as rebuild


def main() -> None:
    unreal.log_warning(
        "[RepairProps] repair is deprecated on UE 5.3 — running rebuild instead"
    )
    rebuild.main()


if __name__ == "__main__":
    main()
