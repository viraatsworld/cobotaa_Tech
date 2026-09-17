# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Shared paths, and a loader for the plain-Python modules.

``import techtory_cobotta_isaaclab.<anything>`` runs the package ``__init__``,
which registers the gym task and so imports Isaac Lab. The asset, gripper and
placement tests only need modules that are deliberately free of Isaac imports,
so they load those files directly and run in any Python 3.12 -- including one
without Isaac Lab installed.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

PROJECT_ROOT = Path(__file__).resolve().parent.parent
PACKAGE_DIR = PROJECT_ROOT / "source" / "techtory_cobotta_isaaclab" / "techtory_cobotta_isaaclab"
ASSETS_DIR = PACKAGE_DIR / "robot" / "assets"
SCRIPTS_DIR = PROJECT_ROOT / "scripts"


def load_module(path: Path, name: str) -> ModuleType:
    """Import one ``.py`` file as a module without importing its package."""
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None, path
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def joints() -> ModuleType:
    return load_module(PACKAGE_DIR / "robot" / "joints.py", "_techtory_joints")


def placement() -> ModuleType:
    return load_module(PACKAGE_DIR / "scene" / "placement.py", "_techtory_placement")


def builder() -> ModuleType:
    return load_module(SCRIPTS_DIR / "build_urdf_asset.py", "_techtory_build_urdf_asset")
