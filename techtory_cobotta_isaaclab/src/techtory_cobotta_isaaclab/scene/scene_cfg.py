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
    glass_x_pos = CELL_GLASS_CFGS["+x"]
    glass_x_neg = CELL_GLASS_CFGS["-x"]
    glass_y_pos = CELL_GLASS_CFGS["+y"]
    glass_y_neg = CELL_GLASS_CFGS["-y"]
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

from techtory_cobotta_isaaclab.assets import HAMMER_USD, PALLET_USD, SHELF_USD, SODA_CAN_USD, WORKCELL_USD
from techtory_cobotta_isaaclab.robot.robot_cfg import COBOTTA_RG6_CFG
from techtory_cobotta_isaaclab.scene.layout import (
    CELL_FRAME_INNER,
    CELL_FRAME_TOP_Z,
    HAMMER_BODY,
    PALLET,
    SHELF,
    SODA_CAN_BODY,
    SPAWN_ZONE,
    TABLE_TOP_Z,
)
from techtory_cobotta_isaaclab.spawners import GraspableUsdFileCfg, StaticUsdFileCfg

__all__ = [
    "CELL_GLASS_CFGS",
    "GRASPABLE_OBJECT_CFGS",
    "HAMMER_CFG",
    "PALLET_CFG",
    "SHELF_CFG",
    "SODA_CAN_CFG",
    "SPAWN_ZONE_MARKER_CFG",
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
        # colliders may use the exact triangle mesh, so they do -- ~470k triangles.
        #
        # 5 mm contact offset: PhysX's default is several centimetres, and against a
        # mesh that dense (the table top is a slotted plate, 16 mm grooves every 52 mm)
        # it generated a flood of speculative contacts. At 1024 environments that
        # overflowed the GPU collision stack, and even with room about 1 hammer in 250
        # lying on the table was kicked over in its first steps. With 5 mm: no
        # overflow, 0 of 8192 resets go wrong, and stepping is ~50% faster
        # (scripts/check_pick_to_bin.py --resets).
        collision_props=PhysxCollisionPropertiesCfg(
            mesh_collision_property=PhysxTriangleMeshPropertiesCfg(), contact_offset=0.005, rest_offset=0.0
        ),
    ),
)
"""The Techtory cell, at the environment origin (its frame is the cell frame)."""

_GLASS_THICKNESS = 0.01


def _glass(side: str) -> AssetBaseCfg:
    """One side's glass: a thin box from the table top to the top rail, its inner face on the frame's."""
    plane = CELL_FRAME_INNER[side]
    outward = 1.0 if plane > 0 else -1.0
    center_z = (TABLE_TOP_Z + CELL_FRAME_TOP_Z) / 2.0
    height = CELL_FRAME_TOP_Z - TABLE_TOP_Z
    if side.endswith("x"):  # spans the cell in y, between the -y and +y frames
        y0, y1 = CELL_FRAME_INNER["-y"], CELL_FRAME_INNER["+y"]
        size = (_GLASS_THICKNESS, y1 - y0, height)
        pos = (plane + outward * _GLASS_THICKNESS / 2.0, (y0 + y1) / 2.0, center_z)
    else:  # spans the cell in x, between the -x and +x frames
        x0, x1 = CELL_FRAME_INNER["-x"], CELL_FRAME_INNER["+x"]
        size = (x1 - x0, _GLASS_THICKNESS, height)
        pos = ((x0 + x1) / 2.0, plane + outward * _GLASS_THICKNESS / 2.0, center_z)
    name = {"+x": "XPos", "-x": "XNeg", "+y": "YPos", "-y": "YNeg"}[side]
    return AssetBaseCfg(
        prim_path=f"{{ENV_REGEX_NS}}/CellGlass{name}",
        # No rigid body: a static collider, like the cell's own.
        spawn=sim_utils.CuboidCfg(
            size=size,
            collision_props=PhysxCollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.75, 0.88, 0.95), opacity=0.12),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=pos),
    )


CELL_GLASS_CFGS: dict[str, AssetBaseCfg] = {side: _glass(side) for side in ("+x", "-x", "+y", "-y")}
"""The glass in the frame's four sides, which the cell's mesh does not have.

The real cell is glazed between its aluminium posts and rails; the cell USD
(and the workcell URDF it came from) has only the frame, so the arm could reach
out through ~70% of each side. Each pane is a 10 mm box over the whole side,
table top to top rail, with its inner face on the frame's inner face
(``layout.CELL_FRAME_INNER``): behind the posts, so it changes nothing where the
frame already collides.
"""

SHELF_CFG = AssetBaseCfg(
    prim_path="{ENV_REGEX_NS}/Shelf",
    # Box colliders already; only the joints and bodies need switching off.
    spawn=StaticUsdFileCfg(usd_path=str(SHELF_USD)),
    init_state=AssetBaseCfg.InitialStateCfg(pos=SHELF.pos, rot=SHELF.rot),
)
"""The shelf, standing on the table."""

PALLET_CFG = AssetBaseCfg(
    prim_path="{ENV_REGEX_NS}/Pallet",
    # Box colliders already; the converter's rigid body and articulation root are switched off.
    spawn=StaticUsdFileCfg(usd_path=str(PALLET_USD)),
    init_state=AssetBaseCfg.InitialStateCfg(pos=PALLET.pos, rot=PALLET.rot),
)
"""The blue pallet (the "bin") on the table. Not part of :class:`TechtoryCellSceneCfg`; tasks that need it add it."""

SPAWN_ZONE_MARKER_CFG = AssetBaseCfg(
    prim_path="{ENV_REGEX_NS}/SpawnZone",
    # No collision_props: drawn only, nothing touches it.
    spawn=sim_utils.CuboidCfg(
        size=(2.0 * SPAWN_ZONE.half[0], 2.0 * SPAWN_ZONE.half[1], 0.0005),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.7, 0.2), opacity=0.35),
    ),
    init_state=AssetBaseCfg.InitialStateCfg(pos=(*SPAWN_ZONE.center, TABLE_TOP_Z + 0.0005)),
)
"""A green rectangle on the table showing :data:`layout.SPAWN_ZONE`, where pick tasks lay their object."""

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

GRASPABLE_OBJECT_CFGS: dict[str, RigidObjectCfg] = {"hammer": HAMMER_CFG, "soda_can": SODA_CAN_CFG}
"""The objects a pick task can be given, by the names in :data:`grasp_objects.GRASP_OBJECTS`."""

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
    glass_x_pos = CELL_GLASS_CFGS["+x"]
    glass_x_neg = CELL_GLASS_CFGS["-x"]
    glass_y_pos = CELL_GLASS_CFGS["+y"]
    glass_y_neg = CELL_GLASS_CFGS["-y"]
    shelf = SHELF_CFG
    robot = COBOTTA_RG6_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    hammer = HAMMER_CFG
    soda_can = SODA_CAN_CFG
    wrist_ft = WRIST_FT_CFG

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=800.0),
    )
