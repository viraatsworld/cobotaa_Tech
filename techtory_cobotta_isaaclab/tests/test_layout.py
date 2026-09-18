# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The cell layout against USD's own composition of the Isaac Sim demo's scene.

``scene/layout.py`` hand-computes where things go; these tests rebuild the
demo's authoring (``techtory_cobotta_isaacsim/spawners/spawn_objects.py``) as an
in-memory USD stage and let ``pxr`` do the maths. Kit-less: plain ``pxr`` only.
"""

from __future__ import annotations

import math

import pytest
from pxr import Gf, Usd, UsdGeom

from techtory_cobotta_isaaclab.assets import HAMMER_USD, SHELF_USD, SODA_CAN_USD, WORKCELL_USD
from techtory_cobotta_isaaclab.scene import layout
from techtory_cobotta_isaaclab.scene.layout import Pose

pytestmark = pytest.mark.unit

MIDDLE_BOARD_TOP = 0.36  # shelf frame, from the shelf USD's box colliders


def _place(stage: Usd.Stage, path: str, usd: str, pos, rotate_xyz_deg) -> Usd.Prim:
    """Reference ``usd`` at ``path`` with the demo's translate + rotateXYZ ops."""
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(str(usd))
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xform.AddRotateXYZOp().Set(Gf.Vec3d(*rotate_xyz_deg))
    return prim


def _world_pose(prim: Usd.Prim) -> Pose:
    transform = Gf.Transform(UsdGeom.XformCache().GetLocalToWorldTransform(prim))
    t, q = transform.GetTranslation(), transform.GetRotation().GetQuat()
    rot = (*q.GetImaginary(), q.GetReal())
    return Pose(tuple(t), rot if rot[3] >= 0 else tuple(-c for c in rot))


def _assert_pose_close(actual: Pose, expected: Pose, tol: float = 1e-5) -> None:
    assert actual.pos == pytest.approx(expected.pos, abs=tol)
    # q and -q are the same rotation
    dot = abs(sum(a * b for a, b in zip(actual.rot, expected.rot, strict=True)))
    assert dot == pytest.approx(1.0, abs=tol)


@pytest.fixture
def demo_stage() -> Usd.Stage:
    """The demo's shelf and hammer, authored exactly as ``add_shelf`` / ``add_hammer`` do."""
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    _place(stage, "/World/Shelf", SHELF_USD, (0.61, 0.27, 0.94), (0.0, 0.0, 90.0))
    _place(stage, "/World/Shelf/Hammer", HAMMER_USD, (0.04318, 0.0, 0.43525), (90.0, 0.0, 180.0))
    return stage


@pytest.mark.parametrize("angles", [(90.0, 0.0, 180.0), (0.0, 0.0, 90.0), (30.0, -45.0, 120.0), (-10.0, 80.0, 5.0)])
def test_rotate_xyz_matches_usd(angles: tuple[float, float, float]) -> None:
    stage = Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/X", "Xform")
    UsdGeom.Xformable(prim).AddRotateXYZOp().Set(Gf.Vec3d(*angles))
    _assert_pose_close(Pose((0.0, 0.0, 0.0), layout.quat_from_rotate_xyz_deg(*angles)), _world_pose(prim))


def test_hammer_body_offset_matches_the_file() -> None:
    stage = Usd.Stage.Open(str(HAMMER_USD))
    _assert_pose_close(layout.HAMMER_BODY_OFFSET, _world_pose(stage.GetPrimAtPath("/World/hammer")))


def test_hammer_body_pose_matches_the_demo(demo_stage: Usd.Stage) -> None:
    """Same body orientation and x/y as the demo; only the drop height differs."""
    demo_body = _world_pose(demo_stage.GetPrimAtPath("/World/Shelf/Hammer/hammer"))
    demo_on_shelf = Pose((0.04318, 0.0, 0.43525), layout.quat_from_rotate_xyz_deg(90.0, 0.0, 180.0))
    _assert_pose_close(
        layout.compose(layout.compose(layout.SHELF, demo_on_shelf), layout.HAMMER_BODY_OFFSET), demo_body
    )

    assert layout.HAMMER_BODY.pos[:2] == pytest.approx(demo_body.pos[:2], abs=1e-6)
    _assert_pose_close(Pose(demo_body.pos, layout.HAMMER_BODY.rot), demo_body)


def test_hammer_starts_just_above_the_middle_board(demo_stage: Usd.Stage) -> None:
    """The demo drops the hammer 6.5 cm; ours starts 1-5 mm above the board."""
    hammer = demo_stage.GetPrimAtPath("/World/Shelf/Hammer")
    UsdGeom.Xformable(hammer).GetOrderedXformOps()[0].Set(Gf.Vec3d(*layout.HAMMER_ON_SHELF.pos))
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    lowest = bbox.ComputeWorldBound(hammer).ComputeAlignedRange().GetMin()[2]
    gap = lowest - (layout.SHELF.pos[2] + MIDDLE_BOARD_TOP)
    assert 0.001 <= gap <= 0.005


def test_soda_can_starts_just_above_the_middle_board() -> None:
    stage = Usd.Stage.Open(str(SODA_CAN_USD))
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    body = stage.GetPrimAtPath("/World/node_/mesh_")
    lowest_in_body = bbox.ComputeWorldBound(body).ComputeAlignedRange().GetMin()[2]
    gap = layout.SODA_CAN_BODY.pos[2] + lowest_in_body - (layout.SHELF.pos[2] + MIDDLE_BOARD_TOP)
    assert 0.001 <= gap <= 0.006


def test_robot_mount_sits_on_the_base_plate() -> None:
    stage = Usd.Stage.Open(str(WORKCELL_USD))
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    plate = stage.GetPrimAtPath("/World/techtory_demo_description/robot_base_plate_link")
    plate_range = bbox.ComputeWorldBound(plate).ComputeAlignedRange()
    assert 0.0 < layout.ROBOT_MOUNT.pos[2] - plate_range.GetMax()[2] < 0.01
    centre = (plate_range.GetMin() + plate_range.GetMax()) / 2.0
    assert layout.ROBOT_MOUNT.pos[:2] == pytest.approx((centre[0], centre[1]), abs=0.005)
    assert layout.ROBOT_MOUNT.rot == pytest.approx((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))
