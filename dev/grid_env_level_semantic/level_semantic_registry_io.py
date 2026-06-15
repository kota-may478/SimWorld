#!/usr/bin/env python3
"""Atomic registry I/O, checkpointing, and computed block records (no PIE spawn)."""

from __future__ import annotations

import json
import os
import tempfile
from dataclasses import asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from level_region import LevelRegionConfig

BlockIndex = Tuple[int, int]
BlockSemantic = str  # wall | floor | air
BlockMode = str  # T | F

REGISTRY_VERSION = 1


def semantics_key(gx: int, gy: int) -> str:
    return f"{gx:03d}_{gy:03d}"


def semantics_to_dict(semantics: Dict[BlockIndex, BlockSemantic]) -> Dict[str, str]:
    return {semantics_key(gx, gy): sem for (gx, gy), sem in sorted(semantics.items())}


def semantics_from_dict(raw: Dict[str, str]) -> Dict[BlockIndex, BlockSemantic]:
    out: Dict[BlockIndex, BlockSemantic] = {}
    for key, sem in raw.items():
        gx_s, gy_s = key.split("_", 1)
        out[(int(gx_s), int(gy_s))] = sem  # type: ignore[assignment]
    return out


def save_registry_atomic(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(payload, indent=2, ensure_ascii=False) + "\n"
    fd, tmp_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=str(path.parent),
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(text)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp_name, path)
    except Exception:
        try:
            os.unlink(tmp_name)
        except OSError:
            pass
        raise
    print(f"[Registry] saved {path} ({payload.get('status', '?')})")


def load_registry(path: Path) -> Optional[dict]:
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def registry_region_matches(data: dict, region: LevelRegionConfig) -> bool:
    reg = data.get("region") or {}
    if reg.get("grid_nx") != region.grid_nx or reg.get("grid_ny") != region.grid_ny:
        return False
    origin = reg.get("grid_origin_xy_cm")
    expected = list(region.grid_origin_xy_cm)
    if origin is not None and list(origin) != expected:
        return False
    return True


def can_resume_registry(
    data: Optional[dict],
    *,
    region: LevelRegionConfig,
    block_bottom_z_cm: float,
    subgrid: Optional[Tuple[int, int, int, int]],
) -> bool:
    if not data:
        return False
    if data.get("status") == "complete":
        return True
    if not registry_region_matches(data, region):
        return False
    if abs(float(data.get("block_bottom_z_cm", -1)) - block_bottom_z_cm) > 0.01:
        return False
    saved_sub = data.get("region", {}).get("subgrid")
    if subgrid is None and saved_sub not in (None, []):
        return False
    if subgrid is not None and list(subgrid) != list(saved_sub or []):
        return False
    return bool(data.get("semantics"))


def pending_cells(
    all_cells: List[BlockIndex],
    semantics: Dict[BlockIndex, BlockSemantic],
) -> List[BlockIndex]:
    return [c for c in all_cells if c not in semantics]


def build_block_record(
    *,
    region: LevelRegionConfig,
    block_bottom_z_cm: float,
    gx: int,
    gy: int,
    semantic: BlockSemantic,
    actor_name: str,
    block_bottom_to_actor_z_fn,
    mode_for_semantic_fn,
) -> dict:
    x, y = region.cell_center_xy_cm(gx, gy)
    actor_z = block_bottom_to_actor_z_fn(block_bottom_z_cm)
    mode = mode_for_semantic_fn(semantic)
    return {
        "gx": gx,
        "gy": gy,
        "semantic": semantic,
        "mode": mode,
        "block_bottom_z_cm": block_bottom_z_cm,
        "actor_name": actor_name,
        "world_cm": [x, y, actor_z],
    }


def blocks_from_semantics(
    *,
    region: LevelRegionConfig,
    block_bottom_z_cm: float,
    semantics: Dict[BlockIndex, BlockSemantic],
    block_actor_name_fn,
    block_bottom_to_actor_z_fn,
    mode_for_semantic_fn,
) -> Dict[str, dict]:
    blocks: Dict[str, dict] = {}
    for (gx, gy), sem in sorted(semantics.items()):
        if sem == "air":
            continue
        name = block_actor_name_fn(gx, gy)
        blocks[name] = build_block_record(
            region=region,
            block_bottom_z_cm=block_bottom_z_cm,
            gx=gx,
            gy=gy,
            semantic=sem,
            actor_name=name,
            block_bottom_to_actor_z_fn=block_bottom_to_actor_z_fn,
            mode_for_semantic_fn=mode_for_semantic_fn,
        )
    return blocks


def make_registry_payload(
    *,
    source_map: str,
    save_map: str,
    region: LevelRegionConfig,
    block_bottom_z_cm: float,
    height_adjust_steps: int,
    semantics: Dict[BlockIndex, BlockSemantic],
    blocks: Dict[str, dict],
    subgrid: Optional[Tuple[int, int, int, int]],
    status: str,
    labeled_count: int,
    total_cells: int,
    labels_only: bool,
) -> dict:
    return {
        "registry_version": REGISTRY_VERSION,
        "status": status,
        "labels_only": labels_only,
        "labeled_count": labeled_count,
        "total_cells": total_cells,
        "source_map": source_map,
        "save_map": save_map,
        "region": {
            "corner_a_xy_cm": list(region.corner_a_xy_cm),
            "corner_b_xy_cm": list(region.corner_b_xy_cm),
            "outward_margin_m": region.outward_margin_cm / 100.0,
            "grid_origin_xy_cm": list(region.grid_origin_xy_cm),
            "grid_nx": region.grid_nx,
            "grid_ny": region.grid_ny,
            "subgrid": list(subgrid) if subgrid else None,
        },
        "block_bottom_z_cm": block_bottom_z_cm,
        "height_adjust_steps": height_adjust_steps,
        "semantics": semantics_to_dict(semantics),
        "blocks": blocks,
    }
