#!/usr/bin/env python3
"""Backward-compat shim — use object_registry.py and l2_depth.py instead."""

from __future__ import annotations

from object_registry import (  # noqa: F401
    HUMAN_PROP_TYPE_ID,
    ObjectRegistry,
    RegistryEntry,
    RegistryUpdateResult,
    SightConfig,
    VisibleTarget,
    build_actor_maps,
    detection_from_world_pose,
    estimate_local_xy_from_detection,
    fetch_ue_sight_targets,
    is_dynamic_slot,
    update_object_registry_from_sight,
)

# Deprecated alias: sight memory is now ObjectRegistry (semantic only, no L2 painting).
SightMemory = ObjectRegistry

update_l2_from_sight = update_object_registry_from_sight

__all__ = [
    "HUMAN_PROP_TYPE_ID",
    "ObjectRegistry",
    "RegistryEntry",
    "RegistryUpdateResult",
    "SightConfig",
    "SightMemory",
    "VisibleTarget",
    "build_actor_maps",
    "detection_from_world_pose",
    "estimate_local_xy_from_detection",
    "fetch_ue_sight_targets",
    "is_dynamic_slot",
    "update_l2_from_sight",
    "update_object_registry_from_sight",
]
