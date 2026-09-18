# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Spawner configs for the scene USDs.

Each is a :class:`~isaaclab.sim.UsdFileCfg` with the one correction its asset
needs, applied at spawn time. The spawn functions are named by string so that
resolving a task config never imports ``pxr`` -- that has to wait for Kit.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import MISSING

from isaaclab.sim import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg

__all__ = ["CobottaRg6UsdFileCfg", "GraspableUsdFileCfg", "StaticUsdFileCfg"]


@configclass
class CobottaRg6UsdFileCfg(UsdFileCfg):
    """The Cobotta + RG6 USD, with the gripper made solid.

    As authored, none of the six RG6 links has a collision shape: each link's
    ``collisions`` Xform is a ``PhysxMeshMergeCollisionAPI`` collector whose
    collection is ``explicitOnly`` and names only itself, so it merges zero meshes.
    The fingers then pass through anything. The spawn function widens those
    collections to ``expandPrims`` and binds a high-friction material to the pads.
    See ``techtory_cobotta_isaacsim/plan-gripper-effort.md`` section 1a.
    """

    func: Callable | str = "techtory_cobotta_isaaclab.spawners.spawners:spawn_cobotta_rg6"

    make_uninstanceable: bool = True
    """The collision Xforms are instanceable, and instance proxies cannot be edited."""

    gripper_prim: str = "onrobot_rg6"
    """Gripper subtree, relative to the spawned prim."""

    pad_links: tuple[str, ...] = ("left_inner_finger", "right_inner_finger")
    """Links whose ``collisions`` prim gets :attr:`pad_material`."""

    pad_material: PhysxRigidBodyMaterialCfg = PhysxRigidBodyMaterialCfg(
        static_friction=1.2, dynamic_friction=1.1, restitution=0.0
    )
    """Finger-pad contact material, as in the Isaac Sim demo's ``add_grip_friction``."""

    pad_material_path: str = "gripper_pad_material"
    """Where the pad material is created, relative to the spawned prim so it clones with it."""


@configclass
class StaticUsdFileCfg(UsdFileCfg):
    """A fixture authored as an articulation of fixed joints, spawned as static colliders.

    The workcell and the shelf come from URDF imports: dynamic links welded
    together by fixed joints under an articulation root. For scenery that is both
    wasteful and wrong -- the cell's two links are authored at 1.4 t each. The spawn function
    deactivates every joint (which takes the articulation root with it, since the
    root API sits on ``root_joint``) and removes the bodies' rigid-body and mass
    APIs, leaving plain static colliders on PhysX and Newton alike.

    Set :attr:`collision_props` to a triangle-mesh approximation for an asset whose
    authored convex hulls are too coarse -- the cell's hull spans the whole room,
    robot included.
    """

    func: Callable | str = "techtory_cobotta_isaaclab.spawners.spawners:spawn_static_usd"

    make_uninstanceable: bool = True
    """The collision prims are instanceable, and instance proxies cannot be edited."""


@configclass
class GraspableUsdFileCfg(UsdFileCfg):
    """A free object the gripper can pick up.

    Neither object file authors a mass, so PhysX would integrate one from the
    collision shape at the default density. The spawn function writes
    :attr:`mass` onto the file's single rigid body. Friction comes from the
    inherited :attr:`physics_material`, and a concave object's collision
    approximation from :attr:`collision_props`.
    """

    func: Callable | str = "techtory_cobotta_isaaclab.spawners.spawners:spawn_graspable_usd"

    mass: float = MISSING
    """Mass of the object's rigid body [kg]."""
