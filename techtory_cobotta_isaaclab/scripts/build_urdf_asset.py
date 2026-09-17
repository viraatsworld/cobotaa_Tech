#!/usr/bin/env python3
"""Turn the Techtory Cobotta URDFs into a self-contained, Isaac-ready asset set.

Reads the flattened robot URDF and shelf URDF that ``techtory_cobotta_isaacsim``
already carries, resolves their ``package://`` mesh URIs against explicit search
paths, copies the meshes into the package, applies the corrections the shared
robot description cannot carry because they are simulator-specific, and writes
the result to ``techtory_cobotta_isaaclab/robot/assets/``. The Techtory cell and
robot base-plate meshes are copied alongside, for the scene.

No ROS, no Isaac Sim, no torch -- plain Python and the standard library, so it
runs anywhere and is cheap to re-run::

    python scripts/build_urdf_asset.py

The corrections are documented one by one in :data:`CORRECTIONS` and applied by
the ``_fix_*`` functions below.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import struct
import sys
import xml.etree.ElementTree as ElementTree
from pathlib import Path

CORRECTIONS = """\
  1. orphan links         -- the flattened URDF declares a bare <link name="world"/>
     whose joint is commented out. It is a second root, and the importer
     refuses (or silently picks one of) two roots. Dropped.
  2. ROS-only elements    -- <ros2_control>, <gazebo> and <transmission> mean
     nothing to Isaac and carry the mimic-plugin config of a different
     simulator. Stripped.
  3. RG6 <mimic>          -- dropped. All six gripper joints get their own
     actuator and one action writes `multiplier * q` to each, which is
     deterministic across importer versions. The multipliers are checked
     against GRIPPER_MIMIC in robot/robot_cfg.py by tests/test_urdf_asset.py.
  4. massless links       -- cobotta_pro_tool0 is declared bare. Fixed-joint
     merging is OFF for this robot (the wrist F/T sensor needs the RG6
     base_link to stay its own body), so every link must be a real rigid body.
     A 1 g placeholder inertial is added.
  5. shelf                -- the shelf1_ prefix becomes shelf_, and every link
     gets a 1 kg placeholder inertial. The shelf is spawned fixed-base, so its
     mass never matters, but a link without one does not import as a body.
  6. wrist F/T joint      -- joint_rg6 (J6 flange -> RG6 base) becomes a
     revolute joint locked to +/-1 mrad. The wrist F/T sensor is Isaac Lab's
     JointWrenchSensor, which reports the incoming-joint wrench of every
     NON-FIXED joint on Newton (fixed joints are excluded there). A locked
     revolute is rigid for every practical purpose and makes the flange wrench
     reportable on both PhysX and Newton with the stock sensor.
"""

PROJECT_ROOT = Path(__file__).resolve().parent.parent
# <workspace>/src/techtory_cobotta_isaaclab -> <workspace>
WORKSPACE = PROJECT_ROOT.parent.parent
ISAACSIM_ASSETS = PROJECT_ROOT.parent / "techtory_cobotta_isaacsim" / "assets"

DEFAULT_ROBOT_URDF = (
    ISAACSIM_ASSETS / "robots" / "cvrb0609_onrobot_gripper" / "cobotta_mit_gripper.urdf"
)
DEFAULT_SHELF_URDF = ISAACSIM_ASSETS / "objects" / "shelf.urdf"
DEFAULT_OUTPUT_DIR = (
    PROJECT_ROOT / "source" / "techtory_cobotta_isaaclab" / "techtory_cobotta_isaaclab"
    / "robot" / "assets"
)
DEFAULT_SEARCH_PATHS = (WORKSPACE / "install", WORKSPACE / "src")

MESH_SUBDIR = "meshes"
ROBOT_URDF_NAME = "cobotta_rg6.urdf"
SHELF_URDF_NAME = "shelf.urdf"
CELL_BOUNDS_NAME = "cell_bounds.json"

# The cell is not a URDF asset here: it is a single static mesh, spawned as a
# triangle-mesh collider (see scene/spawners.py). Only the meshes are needed.
CELL_MESH_URIS = {
    "cell": "package://techtory_cell_description/urdf/mesh/cell_link.STL",
    "base_plate": "package://techtory_cell_description/urdf/mesh/robot_base_link.STL",
}

ROS_ONLY_TAGS = ("ros2_control", "gazebo", "transmission")
PLACEHOLDER_MASS = 1.0e-3
SHELF_LINK_MASS = 1.0

# Must match FT_JOINT / WRIST_LOCK in robot/joints.py (tests/test_urdf_asset.py checks).
WRIST_JOINT = "joint_rg6"
WRIST_LOCK = 1.0e-3  # rad, each side
WRIST_AXIS = "0 0 1"  # the J6 flange axis


##
# package:// resolution.
##


def _search_paths(explicit: list[str] | None) -> list[Path]:
    """Where to look for ``package://<name>/...``, most specific first.

    A colcon workspace exposes each package twice -- ``install/<pkg>/share/<pkg>``
    and ``src/.../<pkg>`` -- and both layouts are probed, so this works against
    an installed workspace or a bare source checkout.
    """
    if explicit:
        return [Path(p) for p in explicit]
    env = os.environ.get("TECHTORY_SEARCH_PATH")
    if env:
        return [Path(p) for p in env.split(os.pathsep) if p]
    return list(DEFAULT_SEARCH_PATHS)


def _resolve_package_uri(uri: str, search_paths: list[Path]) -> Path:
    """Map ``package://<pkg>/<rest>`` onto a file on disk.

    Raises:
        FileNotFoundError: If no search path holds the referenced file. Failing
            loudly matters: a silently dropped mesh produces a robot that
            imports cleanly and is invisible.
    """
    package, _, relative = uri.removeprefix("package://").partition("/")
    candidates = [
        root / package / "share" / package / relative  # colcon install layout
        for root in search_paths
    ] + [root / package / relative for root in search_paths]  # source layout
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        f"cannot resolve {uri!r}. Looked in:\n  "
        + "\n  ".join(str(c) for c in candidates)
        + "\nPass --search-path or set TECHTORY_SEARCH_PATH."
    )


def _copy_mesh(uri: str, output_dir: Path, search_paths: list[Path]) -> str:
    """Copy one mesh into ``meshes/<package>/<rest>``; return its relative path."""
    source = _resolve_package_uri(uri, search_paths)
    package, _, relative = uri.removeprefix("package://").partition("/")
    destination = output_dir / MESH_SUBDIR / package / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    # follow_symlinks: a `colcon build --symlink-install` workspace links
    # share/ back at src/, and copying the link would leave the asset pointing
    # outside the package.
    shutil.copyfile(source, destination, follow_symlinks=True)
    return os.path.relpath(destination, output_dir)


def _copy_meshes(root: ElementTree.Element, output_dir: Path, search_paths: list[Path]) -> int:
    """Copy every referenced mesh into the package and rewrite its filename.

    Paths become relative to the URDF, which is what makes the written asset
    self-contained: nothing downstream needs ament, a ROS workspace, or an
    environment variable to find a mesh again.
    """
    copied: dict[str, str] = {}
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename", "")
        if not uri.startswith("package://"):
            continue
        if uri not in copied:
            copied[uri] = _copy_mesh(uri, output_dir, search_paths)
        mesh.set("filename", copied[uri])
    return len(copied)


##
# Robot corrections.
##


def _drop_orphan_links(root: ElementTree.Element) -> list[str]:
    """Remove links no joint refers to.

    The flattened URDF keeps ``<link name="world"/>`` from the workcell xacro
    but its ``joint_w`` is commented out, so it floats free as a second root.
    """
    referenced = set()
    for joint in root.findall("joint"):
        for tag in ("parent", "child"):
            element = joint.find(tag)
            if element is not None:
                referenced.add(element.get("link"))
    dropped = []
    for link in root.findall("link"):
        if link.get("name") not in referenced:
            root.remove(link)
            dropped.append(link.get("name"))
    return dropped


def _strip_ros_only(root: ElementTree.Element) -> int:
    """Drop ``<ros2_control>``, ``<gazebo>`` and ``<transmission>`` blocks."""
    count = 0
    for element in list(root):
        if element.tag in ROS_ONLY_TAGS:
            root.remove(element)
            count += 1
    return count


def read_mimics(root: ElementTree.Element) -> dict[str, float]:
    """Return ``{joint: multiplier}`` for every mimic joint, plus its leader at 1.

    Raises:
        ValueError: If a mimic has a non-zero offset or the joints follow more
            than one leader -- the gripper action writes ``multiplier * q`` and
            cannot express either.
    """
    leaders: set[str] = set()
    multipliers: dict[str, float] = {}
    for joint in root.findall("joint"):
        mimic = joint.find("mimic")
        if mimic is None:
            continue
        if float(mimic.get("offset", "0")) != 0.0:
            raise ValueError(f"{joint.get('name')} has a non-zero mimic offset")
        leaders.add(mimic.get("joint"))
        multipliers[joint.get("name")] = float(mimic.get("multiplier", "1"))
    if not multipliers:
        return {}
    if len(leaders) != 1:
        raise ValueError(f"mimic joints follow more than one leader: {sorted(leaders)}")
    return {leaders.pop(): 1.0, **multipliers}


def _drop_mimics(root: ElementTree.Element) -> dict[str, float]:
    """Remove every ``<mimic>`` and return the coupling that was removed."""
    coupling = read_mimics(root)
    for joint in root.findall("joint"):
        mimic = joint.find("mimic")
        if mimic is not None:
            joint.remove(mimic)
    return coupling


def _lock_wrist_joint(root: ElementTree.Element) -> bool:
    """Turn the fixed flange joint into a revolute joint locked to +/-WRIST_LOCK.

    Returns whether the joint was changed (False if it already was one).
    """
    joint = next((j for j in root.findall("joint") if j.get("name") == WRIST_JOINT), None)
    if joint is None:
        raise ValueError(f"{WRIST_JOINT} not found; the RG6 mount changed upstream")
    if joint.get("type") == "revolute":
        return False
    if joint.get("type") != "fixed":
        raise ValueError(f"{WRIST_JOINT} is {joint.get('type')!r}, expected 'fixed'")
    joint.set("type", "revolute")
    for tag in ("axis", "limit"):
        for element in joint.findall(tag):
            joint.remove(element)
    ElementTree.SubElement(joint, "axis", {"xyz": WRIST_AXIS})
    ElementTree.SubElement(
        joint,
        "limit",
        {"lower": f"{-WRIST_LOCK:g}", "upper": f"{WRIST_LOCK:g}", "effort": "1000", "velocity": "0.1"},
    )
    return True


def _add_placeholder_inertia(root: ElementTree.Element, mass: float) -> list[str]:
    """Give every link without an ``<inertial>`` a small solid one."""
    inertia = f"{mass * 1.0e-3:.9g}"
    touched = []
    for link in root.findall("link"):
        if link.find("inertial") is not None:
            continue
        inertial = ElementTree.Element("inertial")
        ElementTree.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ElementTree.SubElement(inertial, "mass", {"value": f"{mass:g}"})
        ElementTree.SubElement(
            inertial,
            "inertia",
            {"ixx": inertia, "ixy": "0", "ixz": "0", "iyy": inertia, "iyz": "0", "izz": inertia},
        )
        link.insert(0, inertial)
        touched.append(link.get("name"))
    return touched


##
# Shelf.
##


def _rename_shelf(root: ElementTree.Element, old: str = "shelf1_", new: str = "shelf_") -> int:
    """Rename every ``shelf1_`` link, joint and material to ``shelf_``."""
    count = 0
    for element in root.iter():
        for attribute in ("name", "link"):
            value = element.get(attribute)
            if value and value.startswith(old):
                element.set(attribute, new + value.removeprefix(old))
                count += 1
    return count


##
# Cell meshes.
##


def stl_bounds(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Axis-aligned bounding box of an STL, ASCII or binary, in file units."""
    data = path.read_bytes()
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3

    def _grow(x: float, y: float, z: float) -> None:
        for i, v in enumerate((x, y, z)):
            lo[i] = min(lo[i], v)
            hi[i] = max(hi[i], v)

    (count,) = struct.unpack_from("<I", data, 80) if len(data) >= 84 else (0,)
    if len(data) == 84 + 50 * count:
        for record in struct.iter_unpack("<12fH", data[84:]):
            for k in range(3):
                _grow(*record[3 + 3 * k : 6 + 3 * k])
    else:
        for line in data.decode("ascii", errors="ignore").splitlines():
            parts = line.split()
            if len(parts) == 4 and parts[0] == "vertex":
                _grow(*(float(p) for p in parts[1:]))
    return (lo[0], lo[1], lo[2]), (hi[0], hi[1], hi[2])


##
# Entry point.
##


def _write(tree: ElementTree.ElementTree, destination: Path) -> None:
    ElementTree.indent(tree, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def build(
    robot_urdf: Path, shelf_urdf: Path, output_dir: Path, search_paths: list[Path]
) -> dict[str, Path]:
    """Apply every correction and write the self-contained asset set.

    Returns:
        Paths of the written robot URDF, shelf URDF and cell bounds file.
    """
    for path in (robot_urdf, shelf_urdf):
        if not path.is_file():
            raise FileNotFoundError(f"{path} not found (expected techtory_cobotta_isaacsim beside this package)")
    output_dir.mkdir(parents=True, exist_ok=True)

    # -- robot
    tree = ElementTree.parse(robot_urdf)
    root = tree.getroot()
    orphans = _drop_orphan_links(root)
    stripped = _strip_ros_only(root)
    coupling = _drop_mimics(root)
    wrist = _lock_wrist_joint(root)
    inertia = _add_placeholder_inertia(root, PLACEHOLDER_MASS)
    meshes = _copy_meshes(root, output_dir, search_paths)
    robot_out = output_dir / ROBOT_URDF_NAME
    _write(tree, robot_out)

    # -- shelf
    shelf_tree = ElementTree.parse(shelf_urdf)
    shelf_root = shelf_tree.getroot()
    renamed = _rename_shelf(shelf_root)
    shelf_inertia = _add_placeholder_inertia(shelf_root, SHELF_LINK_MASS)
    shelf_out = output_dir / SHELF_URDF_NAME
    _write(shelf_tree, shelf_out)

    # -- cell meshes, with bounds recorded for env spacing
    bounds = {}
    for key, uri in CELL_MESH_URIS.items():
        relative = _copy_mesh(uri, output_dir, search_paths)
        lo, hi = stl_bounds(output_dir / relative)
        bounds[key] = {"mesh": relative, "min": lo, "max": hi}
    bounds_out = output_dir / CELL_BOUNDS_NAME
    bounds_out.write_text(json.dumps(bounds, indent=2) + "\n")

    print(f"[techtory] {robot_urdf} -> {robot_out}")
    print(f"[techtory]   {meshes} meshes copied into {output_dir / MESH_SUBDIR}")
    print(f"[techtory]   orphan link(s) dropped: {orphans}")
    print(f"[techtory]   {stripped} ROS-only block(s) stripped")
    print(f"[techtory]   mimic coupling dropped (now GRIPPER_MIMIC): {coupling}")
    print(f"[techtory]   {WRIST_JOINT} locked revolute (+/-{WRIST_LOCK} rad): {'converted' if wrist else 'already'}")
    print(f"[techtory]   placeholder inertia added to: {inertia}")
    print(f"[techtory] {shelf_urdf} -> {shelf_out}")
    print(f"[techtory]   {renamed} shelf names re-prefixed, {len(shelf_inertia)} link inertias added")
    for key, entry in bounds.items():
        lo, hi = entry["min"], entry["max"]
        size = tuple(round(h - l, 3) for l, h in zip(lo, hi, strict=True))
        print(f"[techtory] {key} mesh {entry['mesh']}: size {size} m")
    return {"robot": robot_out, "shelf": shelf_out, "bounds": bounds_out}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Corrections applied:\n" + CORRECTIONS,
    )
    parser.add_argument(
        "--urdf", type=Path, default=DEFAULT_ROBOT_URDF,
        help=f"flattened robot URDF (default: {DEFAULT_ROBOT_URDF})",
    )
    parser.add_argument(
        "--shelf-urdf", type=Path, default=DEFAULT_SHELF_URDF,
        help=f"flattened shelf URDF (default: {DEFAULT_SHELF_URDF})",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR,
        help=f"where to write the assets (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--search-path", action="append", metavar="DIR",
        help="root to resolve package:// URIs against; repeatable. Overrides "
             "TECHTORY_SEARCH_PATH.",
    )
    args = parser.parse_args(argv)

    build(args.urdf, args.shelf_urdf, args.output_dir, _search_paths(args.search_path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
