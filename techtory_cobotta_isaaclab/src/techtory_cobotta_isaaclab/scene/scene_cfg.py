# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The Techtory cell as an Isaac Lab scene: workcell, shelf, objects, robot, wrist F/T.

Each environment is one cell, spawned at the environment origin; every pose
comes from :mod:`.layout`. The asset configs are importable on their own, for a
scene of your own::

    @configclass
    class MySceneCfg(InteractiveSceneCfg):
        workcell = WORKCELL_CFG
        robot = COBOTTA_RG6_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        wrist_ft = WRIST_FT_CFG
"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import JointWrenchSensorCfg
from isaaclab.utils import configclass
from isaaclab_physx.sim.schemas import (
    PhysxCollisionPropertiesCfg,
    PhysxConvexDecompositionPropertiesCfg,
    PhysxTriangleMeshPropertiesCfg,
)
from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg

from techtory_cobotta_isaaclab.assets import HAMMER_USD, SHELF_USD, SODA_CAN_USD, WORKCELL_USD
from techtory_cobotta_isaaclab.robot.robot_cfg import COBOTTA_RG6_CFG
from techtory_cobotta_isaaclab.scene.layout import HAMMER_BODY, SHELF, SODA_CAN_BODY
from techtory_cobotta_isaaclab.spawners import GraspableUsdFileCfg, StaticUsdFileCfg

__all__ = [
    "HAMMER_CFG",
    "SHELF_CFG",
    "SODA_CAN_CFG",
    "WORKCELL_CFG",
    "WRIST_FT_CFG",
    "TechtoryCellSceneCfg",
]

##
# Fixtures.
##

WORKCELL_CFG = AssetBaseCfg(
    prim_path="{ENV_REGEX_NS}/Workcell",
    spawn=StaticUsdFileCfg(
        usd_path=str(WORKCELL_USD),
        # The cell's two colliders are authored as convex hulls of whole links; the
        # cell_link hull is a box around the entire room, robot included. Static
        # colliders may use the exact triangle mesh, so they do.
        collision_props=PhysxCollisionPropertiesCfg(mesh_collision_property=PhysxTriangleMeshPropertiesCfg()),
    ),
)
"""The Techtory cell, at the environment origin (its frame is the cell frame)."""

SHELF_CFG = AssetBaseCfg(
    prim_path="{ENV_REGEX_NS}/Shelf",
    # Box colliders already; only the joints and bodies need switching off.
    spawn=StaticUsdFileCfg(usd_path=str(SHELF_USD)),
    init_state=AssetBaseCfg.InitialStateCfg(pos=SHELF.pos, rot=SHELF.rot),
)
"""The shelf, standing on the table."""

##
# Objects.
##

# The pads carry 1.2 / 1.1 (CobottaRg6UsdFileCfg.pad_material). PhysX averages
# the two materials in a contact, so the object needs the same or the grip only
# gets halfway.
_GRASPABLE_MATERIAL = PhysxRigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.1, restitution=0.0)

HAMMER_CFG = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/Hammer",
    spawn=GraspableUsdFileCfg(
        usd_path=str(HAMMER_USD),
        mass=0.3,
        physics_material=_GRASPABLE_MATERIAL,
        # The authored convex hull of a hammer is a solid wedge from head to
        # handle: it fills the notch the pads aim for, so they close on a sloping
        # face and squeeze the part out. Decomposition keeps the handle a handle.
        collision_props=PhysxCollisionPropertiesCfg(
            mesh_collision_property=PhysxConvexDecompositionPropertiesCfg(max_convex_hulls=32, error_percentage=2.0)
        ),
    ),
    init_state=RigidObjectCfg.InitialStateCfg(pos=HAMMER_BODY.pos, rot=HAMMER_BODY.rot),
)
"""A 0.3 kg hammer lying on the shelf's middle board."""

SODA_CAN_CFG = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/SodaCan",
    # A can is convex, so the authored hull is exact enough.
    spawn=GraspableUsdFileCfg(usd_path=str(SODA_CAN_USD), mass=0.35, physics_material=_GRASPABLE_MATERIAL),
    init_state=RigidObjectCfg.InitialStateCfg(pos=SODA_CAN_BODY.pos, rot=SODA_CAN_BODY.rot),
)
"""A full 0.35 kg soda can standing on the shelf's middle board, within reach."""

##
# Sensors.
##

WRIST_FT_CFG = JointWrenchSensorCfg(prim_path="{ENV_REGEX_NS}/Robot")
"""Incoming joint wrench of every robot link; the RG6 ``base_link`` entry is the wrist F/T.

The robot must be spawned at ``{ENV_REGEX_NS}/Robot``. Read the wrist through
:func:`techtory_cobotta_isaaclab.robot.wrist_wrench`, which also adds the load of
a held object.
"""

##
# Scene.
##


@configclass
class TechtoryCellSceneCfg(InteractiveSceneCfg):
    """One Techtory cell per environment: fixtures, robot, objects and the wrist sensor.

    ``env_spacing`` must clear the cell's footprint: it spans x in [-1.28, 0.90]
    and y in [-0.86, 1.32] m around its origin, so 3 m leaves 0.8 m between cells.
    """

    # A floor under every cell for anything knocked off the table. A thin static
    # box, not GroundPlaneCfg: that one downloads its grid from Nucleus. 200 m
    # covers 4096 cells at 3 m spacing.
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.CuboidCfg(
            size=(200.0, 200.0, 0.5),
            collision_props=PhysxCollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.05, 0.1, 0.35)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.25)),
    )

    workcell = WORKCELL_CFG
    shelf = SHELF_CFG
    robot = COBOTTA_RG6_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    hammer = HAMMER_CFG
    soda_can = SODA_CAN_CFG
    wrist_ft = WRIST_FT_CFG

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=800.0),
    )
