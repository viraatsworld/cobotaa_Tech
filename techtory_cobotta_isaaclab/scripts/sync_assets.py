# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Copy the scene USDs from ``techtory_cobotta_isaacsim`` into this package.

The robot, workcell, shelf and object USDs are authored and tuned for the Isaac
Sim demo. This package keeps a byte-identical copy of exactly the files those
five roots need -- every sublayer, reference and texture, found with
``UsdUtils.ComputeAllDependencies`` -- under ``src/techtory_cobotta_isaaclab/
assets/usd``, in the same relative layout so the internal references still
resolve. Nothing is edited on the way: the fixes Isaac Lab needs are applied at
spawn time, in ``techtory_cobotta_isaaclab.spawners``.

Run it after the Isaac Sim assets change, or with ``--check`` to see whether
the copy has drifted. Needs only ``pxr`` (the ``isaac-activate`` venv has it),
not Isaac Sim::

    python scripts/sync_assets.py            # copy / refresh
    python scripts/sync_assets.py --check    # report drift, exit 1 if any
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PACKAGE_ROOT.parent / "techtory_cobotta_isaacsim" / "assets"
DESTINATION = PACKAGE_ROOT / "src" / "techtory_cobotta_isaaclab" / "assets" / "usd"

# The scene's root USDs, relative to the assets directory. Everything else is
# pulled in as a dependency of one of these.
ROOT_USDS: tuple[str, ...] = (
    "robots/cvrb0609/cvrb0609_with_graph2.usd",
    "workcells/techtory_cell.usd",
    "objects/shelf.usd",
    "objects/hammer1.usd",
    "objects/soda_can.usd",
)

# Dependencies that are expected not to resolve to a file in the assets tree:
# Kit's built-in MDL materials, and the robot USD's grid environment on S3,
# which sits outside the default prim and is never composed when referenced.
ALLOWED_UNRESOLVED = ("OmniPBR.mdl", "gltf/pbr.mdl", "default_environment.usd")


def dependency_closure(source: Path) -> list[Path]:
    """Every file the root USDs need, relative to ``source``."""
    from pxr import Sdf, UsdUtils

    files: set[Path] = set()
    for root in ROOT_USDS:
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(source / root)))
        unexpected = [u for u in unresolved if not u.endswith(ALLOWED_UNRESOLVED)]
        if unexpected:
            raise RuntimeError(f"{root} has unresolved dependencies: {unexpected}")
        for path in [layer.realPath for layer in layers] + list(assets):
            resolved = Path(path).resolve()
            try:
                files.add(resolved.relative_to(source.resolve()))
            except ValueError as exc:
                raise RuntimeError(f"{root} depends on {resolved}, outside {source}") from exc
    return sorted(files)


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE, help="techtory_cobotta_isaacsim/assets")
    parser.add_argument("--check", action="store_true", help="Report drift without copying.")
    args = parser.parse_args()

    if not args.source.is_dir():
        print(f"[ERROR] source assets not found: {args.source}", file=sys.stderr)
        return 2

    files = dependency_closure(args.source)
    drifted = [
        f for f in files if not (DESTINATION / f).exists() or _digest(DESTINATION / f) != _digest(args.source / f)
    ]
    wanted = {DESTINATION / f for f in files}
    stale = sorted(p for p in DESTINATION.rglob("*") if p.is_file() and p not in wanted) if DESTINATION.exists() else []

    if args.check:
        for f in drifted:
            print(f"  drifted: {f}")
        for p in stale:
            print(f"  not needed: {p.relative_to(DESTINATION)}")
        print(f"{len(files)} files, {len(drifted)} drifted, {len(stale)} not needed")
        return 1 if drifted or stale else 0

    for f in drifted:
        (DESTINATION / f).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(args.source / f, DESTINATION / f)
        print(f"  copied: {f}")
    for p in stale:
        p.unlink()
        print(f"  removed: {p.relative_to(DESTINATION)}")
    total = sum((DESTINATION / f).stat().st_size for f in files)
    print(f"{len(files)} files ({total / 1e6:.1f} MB) in {DESTINATION}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
