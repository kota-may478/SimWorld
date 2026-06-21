#!/usr/bin/env python3
"""Catalog for Construction VOL.1 generated LevelProp blueprints."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

OUT_DIR = "/Game/SimWorld/LevelProps/Generated/Construction_VOL1"
MESH_DIR = "/Game/Construction_VOL1/Meshes"

CACHE_DIR = Path(__file__).resolve().parent / "cache"
from paths import PROP_CATALOG_CONSTRUCTION  # noqa: E402

DEFAULT_CATALOG_PATH = PROP_CATALOG_CONSTRUCTION
CONTENT_ROOT = Path("/mnt/c/UEProjects/SimWorld/Content")
EXPECTED_MESH_COUNT = 73


@dataclass(frozen=True)
class PropCatalogEntry:
    prop_type_id: str
    bp_name: str
    bp_path: str
    mesh_name: str
    mesh_path: str


def _prop_type_id_from_sm_name(sm_name: str) -> str:
    name = sm_name[3:] if sm_name.startswith("SM_") else sm_name
    return name.lower()


def _bp_name_from_sm_name(sm_name: str) -> str:
    name = sm_name[3:] if sm_name.startswith("SM_") else sm_name
    return f"BP_{name}"


def entry_from_sm_name(sm_name: str) -> PropCatalogEntry:
    bp_name = _bp_name_from_sm_name(sm_name)
    return PropCatalogEntry(
        prop_type_id=_prop_type_id_from_sm_name(sm_name),
        bp_name=bp_name,
        bp_path=f"{OUT_DIR}/{bp_name}.{bp_name}",
        mesh_name=sm_name,
        mesh_path=f"{MESH_DIR}/{sm_name}.{sm_name}",
    )


def entries_from_bp_names(bp_names: Iterable[str]) -> list[PropCatalogEntry]:
    items: list[PropCatalogEntry] = []
    for bp_name in sorted(bp_names):
        if not bp_name.startswith("BP_"):
            continue
        stem = bp_name[3:]
        sm_name = f"SM_{stem}"
        items.append(entry_from_sm_name(sm_name))
    return items


def discover_entries_from_meshes_dir(content_root: Path | None = None) -> list[PropCatalogEntry]:
    """All SM_* under Construction_VOL1/Meshes (expected 73)."""
    if content_root is None:
        content_root = CONTENT_ROOT
    mesh_dir = content_root / "Construction_VOL1/Meshes"
    if not mesh_dir.is_dir():
        return []
    sm_names = sorted(p.stem for p in mesh_dir.glob("SM_*.uasset"))
    return [entry_from_sm_name(name) for name in sm_names]


def discover_entries_from_content_dir(content_root: Path | None = None) -> list[PropCatalogEntry]:
    if content_root is None:
        content_root = CONTENT_ROOT
    gen_dir = content_root / "SimWorld/LevelProps/Generated/Construction_VOL1"
    if not gen_dir.is_dir():
        return []
    bp_names = [p.stem for p in gen_dir.glob("BP_*.uasset")]
    return entries_from_bp_names(bp_names)


def bp_asset_exists(entry: PropCatalogEntry, content_root: Path | None = None) -> bool:
    if content_root is None:
        content_root = CONTENT_ROOT
    rel = entry.bp_path.split("/Game/", 1)[-1].split(".", 1)[0]
    return (content_root / f"{rel}.uasset").is_file()


def missing_bp_entries(
    entries: list[PropCatalogEntry],
    content_root: Path | None = None,
) -> list[PropCatalogEntry]:
    return [e for e in entries if not bp_asset_exists(e, content_root)]


def save_catalog(entries: list[PropCatalogEntry], path: Path = DEFAULT_CATALOG_PATH) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "version": 2,
        "pack": "Construction_VOL1",
        "count": len(entries),
        "props": [asdict(entry) for entry in entries],
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def load_catalog(path: Path = DEFAULT_CATALOG_PATH) -> list[PropCatalogEntry]:
    if path.is_file():
        data = json.loads(path.read_text(encoding="utf-8"))
        return [PropCatalogEntry(**row) for row in data.get("props", [])]
    return discover_entries_from_meshes_dir()


def ensure_catalog(
    path: Path = DEFAULT_CATALOG_PATH,
    *,
    refresh: bool = False,
    from_meshes: bool = True,
) -> list[PropCatalogEntry]:
    if refresh or not path.is_file():
        entries = (
            discover_entries_from_meshes_dir()
            if from_meshes
            else discover_entries_from_content_dir()
        )
        if entries:
            save_catalog(entries, path)
        return entries
    entries = load_catalog(path)
    if entries:
        return entries
    entries = discover_entries_from_meshes_dir()
    if entries:
        save_catalog(entries, path)
    return entries
