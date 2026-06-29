#!/usr/bin/env python3
"""Canonical cache, registry, and run-artifact paths for grid_env_level_nav."""

from __future__ import annotations

from pathlib import Path

PKG_DIR = Path(__file__).resolve().parent
CACHE_DIR = PKG_DIR / "cache"
L0_CACHE_DIR = CACHE_DIR / "l0"
REGISTRY_DIR = CACHE_DIR / "registries"
SCENARIOS_DIR = PKG_DIR / "scenarios"
COMPACT_NAV_OUT_DIR = SCENARIOS_DIR / "compact_nav" / "out"
CONSTRUCTION_SITE_OUT_DIR = SCENARIOS_DIR / "construction_site" / "out"
SITE_TRANSPORT_20M_OUT_DIR = SCENARIOS_DIR / "site_transport_20m" / "out"

L0_MASK_STRICT = L0_CACHE_DIR / "l0_mask_30cm_strict.npz"
L0_MASK_DEFAULT = L0_CACHE_DIR / "l0_mask_30cm.npz"
L0_VIZ_STRICT = L0_CACHE_DIR / "l0_viz_30cm_strict.png"

COMPACT_NAV_REGISTRY = REGISTRY_DIR / "compact_nav_placement.json"
CONSTRUCTION_SITE_REGISTRY = REGISTRY_DIR / "construction_site_placement.json"
SITE_TRANSPORT_20M_REGISTRY = REGISTRY_DIR / "site_transport_20m_layout_01.json"
SITE_TRANSPORT_20M_LAYOUT_MANIFEST = REGISTRY_DIR / "site_transport_20m_layout_manifest.json"


def site_transport_registry_path(layout_id: str) -> Path:
    """JSON registry path for a site_transport_20m layout variant."""
    return REGISTRY_DIR / f"site_transport_20m_{layout_id}.json"
PROP_CATALOG_CONSTRUCTION = REGISTRY_DIR / "prop_catalog_construction_vol1.json"

ZONE_CATALOG_TEMPLATE = REGISTRY_DIR / "zone_catalog.template.json"
ZONE_REGISTRY = REGISTRY_DIR / "zone_registry.json"
ZONE_REGISTRY_100CM = REGISTRY_DIR / "zone_registry_100cm.json"

# Legacy paths (pre-reorg); kept for one release so old notebooks still resolve.
LEGACY_L0_MASK_STRICT = CACHE_DIR / "l0_mask_30cm_strict.npz"
LEGACY_COMPACT_NAV_REGISTRY = CACHE_DIR / "compact_nav_placement_registry.json"
LEGACY_COMPACT_NAV_RUN = CACHE_DIR / "compact_nav_run"
LEGACY_RUNS_DIR = CACHE_DIR / "runs"
LEGACY_COMPACT_NAV_RUN_DIR = LEGACY_RUNS_DIR / "compact_nav"
LEGACY_CONSTRUCTION_SITE_RUN_DIR = LEGACY_RUNS_DIR / "construction_site"
LEGACY_SITE_TRANSPORT_20M_RUN_DIR = LEGACY_RUNS_DIR / "site_transport_20m"
