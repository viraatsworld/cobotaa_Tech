# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The Techtory cell: cell shell, base plate, shelf, hammer, the robot and its
wrist force/torque sensor.

One copy per environment. The environment origin is the cell's own origin
(``cell_link``), so every pose here reads the same as in the workcell URDF and
in ``techtory_cobotta_isaacsim``. The layout itself lives in
:mod:`.placement`. Everything here spawns without Kit, so the scene runs on
PhysX and on Newton alike.
"""

from __future__ import annotations

import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import JointWrenchSensorCfg
from isaaclab.utils.configclass import configclass

from techtory_cobotta_isaaclab.robot.robot_cfg import ASSETS_DIR, COBOTTA_CFG
from techtory_cobotta_isaaclab.scene.placement import (
    BASE_PLATE_POS,
    CELL_POS,
    HAMMER_POS,
    HAMMER_ROT,
    SHELF_POS,
    SHELF_ROT,
)
from techtory_cobotta_isaaclab.scene.spawners import GraspableUsdFileCfg, StaticMeshFileCfg

__all__ = ["HAMMER_USD", "TechtoryCellSceneCfg"]

_CELL_MESHES = ASSETS_DIR / "meshes" / "techtory_cell_description" / "urdf" / "mesh"

# The hammer only exists as a USD, and this repo keeps USD out of git. It is
# therefore used in place from techtory_cobotta_isaacsim, which sits beside
# this package in the workspace. hammer1.usd is self-contained (no external
# asset paths). Override with TECHTORY_HAMMER_USD.
# parents: [0] scene/  [1] techtory_cobotta_isaaclab/ (module)  [2] source/techtory_cobotta_isaaclab/
#          [3] source/  [4] techtory_cobotta_isaaclab/ (project)  [5] <workspace>/src/
HAMMER_USD: str = os.environ.get(
    "TECHTORY_HAMMER_USD",
    str(
        Path(__file__).resolve().parents[5]
        / "techtory_cobotta_isaacsim" / "assets" / "objects" / "hammer1.usd"
    ),
)


@configclass
class TechtoryCellSceneCfg(InteractiveSceneCfg):
    """The Cobotta Pro + RG6 in the Techtory cell, with the shelf and hammer."""

    # The robot is fixed-base and the cell stands on the floor, so the stock
    # infinite plane is fine here (roxfr3's box-ground workaround was for a
    # wheeled base resting on the plane under PhysX).
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())

    # Exact triangle-mesh collider on PhysX, visual-only on Newton -- see
    # spawners.StaticMeshFileCfg.collide.
    cell = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Cell",
        spawn=StaticMeshFileCfg(mesh_path=str(_CELL_MESHES / "cell_link.STL"), color=(0.55, 0.57, 0.6)),
        init_state=AssetBaseCfg.InitialStateCfg(pos=CELL_POS),
    )

    # A 30 x 30 x 1.7 cm plate: convex, so it collides on every backend.
    base_plate = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/BasePlate",
        spawn=StaticMeshFileCfg(
            mesh_path=str(_CELL_MESHES / "robot_base_link.STL"), collide=True, color=(0.75, 0.75, 0.78)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=BASE_PLATE_POS),
    )

    # Seven primitive boxes, welded: fixed base, and fixed-joint merging ON so
    # the whole shelf is one static body. Boxes are exact as convex hulls.
    shelf = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Shelf",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=str(ASSETS_DIR / "shelf.urdf"),
            fix_base=True,
            merge_fixed_joints=True,
            collision_from_visuals=False,
            joint_drive=None,
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=SHELF_POS, rot=SHELF_ROT),
    )

    # Starts where techtory_cobotta_isaacsim puts it, just above the shelf's
    # middle board, and settles onto it in the first few steps.
    hammer = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Hammer",
        spawn=GraspableUsdFileCfg(usd_path=HAMMER_USD),
        init_state=RigidObjectCfg.InitialStateCfg(pos=HAMMER_POS, rot=HAMMER_ROT),
    )

    robot = COBOTTA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # Reports the incoming-joint wrench of every non-fixed joint of the robot;
    # observations.wrist_wrench picks the RG6 base_link entry out of it.
    wrist_ft = JointWrenchSensorCfg(prim_path="{ENV_REGEX_NS}/Robot")

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )
