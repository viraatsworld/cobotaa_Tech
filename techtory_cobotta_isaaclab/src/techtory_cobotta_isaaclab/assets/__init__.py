# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Paths to the scene USDs shipped with this package.

The files under ``usd/`` are a byte-identical copy of what
``techtory_cobotta_isaacsim/assets`` needs for this scene, made by
``scripts/sync_assets.py``. They are referenced, never edited: the corrections
Isaac Lab needs are applied at spawn time by :mod:`techtory_cobotta_isaaclab.spawners`.

Importing this module touches only the filesystem -- no ``pxr``, which must not be
loaded before Kit starts.
"""

from __future__ import annotations

from pathlib import Path

__all__ = [
    "ASSETS_DIR",
    "COBOTTA_RG6_USD",
    "HAMMER_USD",
    "SHELF_USD",
    "SODA_CAN_USD",
    "WORKCELL_USD",
]

ASSETS_DIR: Path = Path(__file__).parent / "usd"

COBOTTA_RG6_USD: Path = ASSETS_DIR / "robots" / "cvrb0609" / "cvrb0609_with_graph2.usd"
"""Cobotta Pro 900 (cvrb0609) with the OnRobot RG6 mounted on J6, as one articulation."""

WORKCELL_USD: Path = ASSETS_DIR / "workcells" / "techtory_cell.usd"
"""The Techtory cell: frame, table and the robot base plate."""

SHELF_USD: Path = ASSETS_DIR / "objects" / "shelf.usd"
"""A 1.0 m x 0.3 m x 0.8 m steel shelf with three boards."""

HAMMER_USD: Path = ASSETS_DIR / "objects" / "hammer1.usd"
SODA_CAN_USD: Path = ASSETS_DIR / "objects" / "soda_can.usd"

for _path in (COBOTTA_RG6_USD, WORKCELL_USD, SHELF_USD, HAMMER_USD, SODA_CAN_USD):
    if not _path.exists():  # pragma: no cover - a broken checkout, not a code path
        raise FileNotFoundError(f"{_path} is missing. Restore the asset copy with:\n  python scripts/sync_assets.py")
