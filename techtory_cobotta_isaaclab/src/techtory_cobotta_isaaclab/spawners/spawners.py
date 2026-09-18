# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Spawn functions that port the Isaac Sim demo's USD fixes to Isaac Lab.

Each follows the pattern of Isaac Lab's own
``spawn_from_usd_with_compliant_contact_material``: spawn the file with
``_spawn_from_usd_file`` (which also applies every property the config carries),
then correct the spawned prim. ``@clone`` runs this once, on the source
environment, and every cloned environment inherits the result.

Imported only when a scene is spawned, i.e. after Kit is up, so ``pxr`` is safe.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

# Private, but it is the entry point Isaac Lab's own USD spawner wrappers use.
# tests/test_robot_cfg.py fails if an Isaac Lab upgrade moves it.
from isaaclab.sim.spawners.from_files.from_files import _spawn_from_usd_file
from isaaclab.sim.spawners.materials.physics_materials import spawn_physics_material
from isaaclab.sim.utils import bind_physics_material, clone

if TYPE_CHECKING:
    from pxr import Usd

    from . import spawners_cfg

__all__ = ["spawn_cobotta_rg6", "spawn_graspable_usd", "spawn_static_usd"]

logger = logging.getLogger(__name__)

_MERGE_RULE = "collection:collisionmeshes:expansionRule"
_MERGE_INCLUDES = "collection:collisionmeshes:includes"


@clone
def spawn_cobotta_rg6(
    prim_path: str,
    cfg: spawners_cfg.CobottaRg6UsdFileCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Spawn the Cobotta + RG6 and give the gripper real collision shapes."""
    prim = _spawn_from_usd_file(prim_path, cfg.usd_path, cfg, translation, orientation)
    _drop_dangling_references(prim.GetStage(), prim_path)
    gripper_path = f"{prim_path}/{cfg.gripper_prim}"
    _repair_mesh_merge_colliders(prim.GetStage(), gripper_path)

    material_path = f"{prim_path}/{cfg.pad_material_path}"
    spawn_physics_material(material_path, cfg.pad_material)
    for link in cfg.pad_links:
        bind_physics_material(f"{gripper_path}/{link}/collisions", material_path)
    return prim


def _drop_dangling_references(stage: Usd.Stage, root_path: str) -> None:
    """Remove internal references under ``root_path`` whose target prim does not exist.

    The demo's USDs carry two: the RG6 ``base`` collider points at
    ``/colliders/base`` and the cell's ``world`` visuals at ``/visuals/world``,
    neither of which is authored. They contribute nothing -- Kit only warns --
    but Newton's USD importer rejects any stage with composition errors.

    An arc authored inside a referenced layer cannot be cancelled from a stronger
    one, so the reference is removed from the loaded layer itself. The edit stays
    in memory: the file on disk is untouched and ``sync_assets.py --check`` still
    sees a byte-identical copy.
    """
    from pxr import Pcp

    root = stage.GetPrimAtPath(root_path)
    paths = [
        error.rootSite.path
        for error in stage.GetCompositionErrors()
        if error.errorType == Pcp.ErrorType_UnresolvedPrimPath and error.rootSite.path.HasPrefix(root.GetPath())
    ]
    dropped = []
    for path in paths:
        for spec in stage.GetPrimAtPath(path).GetPrimStack():
            for ref in spec.referenceList.GetAddedOrExplicitItems():
                if not ref.assetPath and not spec.layer.GetPrimAtPath(ref.primPath):
                    spec.referenceList.RemoveItemEdits(ref)
                    dropped.append(f"{spec.path} -> {ref.primPath}")
    if dropped:
        logger.info("Dangling references dropped under '%s': %s", root_path, dropped)


def _repair_mesh_merge_colliders(stage: Usd.Stage, root_path: str) -> None:
    """Make every mesh-merge collider under ``root_path`` merge the meshes below it.

    A collector's ``explicitOnly`` collection resolves to exactly the prims it
    names -- here, the collector Xform itself -- so it merges nothing. Widening it
    to ``expandPrims`` includes the meshes underneath.

    Only collectors that belong to a link are widened. The RG6 also carries one on
    its root Xform that gathers every link's colliders; with no rigid body of its
    own, widening it would weld one static copy of the open gripper into the world.
    It is switched off instead.
    """
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        raise ValueError(f"Gripper prim '{root_path}' does not exist.")

    widened, disabled = [], []
    for prim in Usd.PrimRange(root):
        rule = prim.GetAttribute(_MERGE_RULE)
        if not rule:
            continue
        if prim.GetParent().HasAPI(UsdPhysics.RigidBodyAPI):
            rule.Set("expandPrims")
            includes = prim.GetRelationship(_MERGE_INCLUDES)
            if includes and not includes.GetTargets():
                includes.SetTargets([prim.GetPath()])
            widened.append(prim.GetParent().GetName())
        elif prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
            disabled.append(str(prim.GetPath()))

    # Fail loudly: a gripper without shapes still opens and closes, and the only
    # symptom is that nothing it closes on is ever held.
    if not widened:
        raise RuntimeError(f"No mesh-merge colliders found under '{root_path}'; the gripper would have no shapes.")
    logger.info("RG6 colliders widened on %s; stray collectors disabled: %s", sorted(widened), disabled)


@clone
def spawn_static_usd(
    prim_path: str,
    cfg: spawners_cfg.StaticUsdFileCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Spawn a URDF-imported fixture as static colliders (see :class:`StaticUsdFileCfg`)."""
    from pxr import Usd, UsdPhysics

    prim = _spawn_from_usd_file(prim_path, cfg.usd_path, cfg, translation, orientation)
    _drop_dangling_references(prim.GetStage(), prim_path)
    # Collect first: deactivating mid-traversal prunes the subtree being walked.
    joints = [p.GetPath() for p in Usd.PrimRange(prim) if p.IsA(UsdPhysics.Joint)]
    for path in joints:
        prim.GetStage().GetPrimAtPath(path).SetActive(False)
    # Colliders without a body are static in every backend. A disabled body is
    # not: Newton loads it anyway, as a free body with no joint to the world.
    bodies = [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for body in bodies:
        body.RemoveAPI(UsdPhysics.RigidBodyAPI)
        body.RemoveAPI(UsdPhysics.MassAPI)
    logger.info("Static fixture '%s': %d joints deactivated, %d bodies removed", prim_path, len(joints), len(bodies))
    return prim


@clone
def spawn_graspable_usd(
    prim_path: str,
    cfg: spawners_cfg.GraspableUsdFileCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Spawn a free object and give its rigid body :attr:`~GraspableUsdFileCfg.mass`."""
    from pxr import Usd, UsdPhysics

    prim = _spawn_from_usd_file(prim_path, cfg.usd_path, cfg, translation, orientation)
    bodies = [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(bodies) != 1:
        raise ValueError(f"'{cfg.usd_path}' must hold exactly one rigid body, found {len(bodies)}.")
    UsdPhysics.MassAPI.Apply(bodies[0]).CreateMassAttr(float(cfg.mass))
    return prim
