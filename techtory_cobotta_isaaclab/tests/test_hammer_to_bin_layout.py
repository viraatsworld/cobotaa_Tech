# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The hammer-to-bin layout: where the hammer spawns, where the bin is, and what the arm may reach.

Kit-less: plain ``pxr`` for the hammer's geometry, plain Python for the rest.
"""

from __future__ import annotations

import math

import pytest
from pxr import Gf, Usd, UsdGeom

from techtory_cobotta_isaaclab.assets import HAMMER_USD, PALLET_USD
from techtory_cobotta_isaaclab.scene import layout
from techtory_cobotta_isaaclab.scene.layout import Box2D, Pose
from techtory_cobotta_isaaclab.tasks.hammer_to_bin.config.cobotta.env_cfg import HammerToBinEnvCfg

pytestmark = pytest.mark.unit

BIN_PLACE_IN_BASE = (0.54, -0.115)
"""``bin_place`` in ``techtory_cobotta_system/config/poses_hammer.yaml``: the real system's drop pose."""

HOME_TCP_IN_BASE = (0.563, 0.120, 0.246)
"""The TCP at ``HOME_POSE`` in the robot base frame, measured in simulation (check_hammer_to_bin.py)."""

CLEARANCE = 0.015
"""Least gap between a spawned hammer and anything else on the table [m]."""

SHELF_LEGS = Box2D((0.61, 0.27), (0.15, 0.50))
"""The shelf's 40 mm legs sit inside its boards' footprint (shelf.urdf), so the boards bound it."""


def _to_base(point: tuple[float, float, float]) -> tuple[float, float, float]:
    """A cell-frame point in the robot base frame."""
    rel = tuple(p - m for p, m in zip(point, layout.ROBOT_MOUNT.pos, strict=True))
    return layout.quat_rotate(layout.quat_conj(layout.ROBOT_MOUNT.rot), rel)


def _gap(a: Box2D, b: Box2D) -> float:
    """Distance between two rectangles (negative: they overlap)."""
    dx = abs(a.center[0] - b.center[0]) - a.half[0] - b.half[0]
    dy = abs(a.center[1] - b.center[1]) - a.half[1] - b.half[1]
    return max(dx, dy)


def _reach_of_spawns() -> Box2D:
    """Everything a spawned hammer can cover, at any yaw."""
    r = layout.HAMMER_FOOTPRINT_RADIUS
    area = layout.HAMMER_SPAWN_AREA
    return Box2D(area.center, (area.half[0] + r, area.half[1] + r))


##
# The hammer.
##


def test_hammer_box_matches_the_mesh() -> None:
    """The bounding box and grasp geometry in layout.py, against hammer1.usd's own mesh."""
    stage = Usd.Stage.Open(str(HAMMER_USD))
    mesh = stage.GetPrimAtPath("/World/hammer/hamLo")
    to_root = UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    points = [to_root.Transform(Gf.Vec3d(*map(float, p))) for p in UsdGeom.Mesh(mesh).GetPointsAttr().Get()]
    lo = [min(p[i] for p in points) for i in range(3)]
    hi = [max(p[i] for p in points) for i in range(3)]
    # layout keeps these in the body frame; the body is the file root turned by HAMMER_BODY_OFFSET
    center = layout.quat_rotate(layout.HAMMER_BODY_OFFSET.rot, layout.HAMMER_BOX_CENTER)
    half = [abs(c) for c in layout.quat_rotate(layout.HAMMER_BODY_OFFSET.rot, layout.HAMMER_BOX_HALF)]
    assert center == pytest.approx([(a + b) / 2 for a, b in zip(lo, hi, strict=True)], abs=5e-4)
    assert half == pytest.approx([(b - a) / 2 for a, b in zip(lo, hi, strict=True)], abs=5e-4)


def test_grasp_point_is_on_the_handle() -> None:
    grasp = layout.quat_rotate(layout.HAMMER_BODY_OFFSET.rot, layout.HAMMER_GRASP_POINT)  # file frame
    # The handle runs from x = -0.03 to the end; the head is at x < -0.03.
    assert -0.03 < grasp[0] < 0.15
    axis = layout.HAMMER_HANDLE_AXIS
    assert math.hypot(*axis) == pytest.approx(1.0)


def test_hammer_lies_flat_just_above_the_table() -> None:
    def corner(signs: tuple[int, int, int]) -> Pose:
        box = zip(layout.HAMMER_BOX_CENTER, signs, layout.HAMMER_BOX_HALF, strict=True)
        return Pose(tuple(c + s * h for c, s, h in box))

    signs = [(sx, sy, sz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    lowest = min(layout.compose(layout.HAMMER_TABLE_BODY, corner(s)).pos[2] for s in signs)
    assert lowest - layout.WORK_SURFACE_Z == pytest.approx(0.0033, abs=2e-4)


##
# The bin.
##


def test_pallet_is_where_the_real_system_drops_the_hammer() -> None:
    assert _to_base(layout.PALLET.pos)[:2] == pytest.approx(BIN_PLACE_IN_BASE, abs=1e-6)


def test_pallet_usd_matches_its_urdf() -> None:
    stage = Usd.Stage.Open(str(PALLET_USD))
    bound = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "guide"]).ComputeWorldBound(stage.GetDefaultPrim())
    size = bound.ComputeAlignedRange().GetSize()
    assert list(size) == pytest.approx(
        [2 * layout.PALLET_FOOTPRINT.half[0], 2 * layout.PALLET_FOOTPRINT.half[1], 0.075]
    )


def test_bin_regions_nest() -> None:
    assert _gap(layout.BIN_FOOTPRINT, layout.BIN_INTERIOR) < 0 and all(
        f + 0.03 == pytest.approx(i) for f, i in zip(layout.BIN_FOOTPRINT.half, layout.BIN_INTERIOR.half, strict=True)
    )
    assert all(i < p for i, p in zip(layout.BIN_INTERIOR.half, layout.PALLET_FOOTPRINT.half, strict=True))
    assert layout.BIN_FOOTPRINT.contains(*layout.BIN_TARGET[:2])
    assert layout.BIN_TARGET[2] > layout.BIN_RIM_Z + 0.1  # carried over the wall, not into it


def test_hammer_fits_the_release_footprint_at_any_yaw() -> None:
    assert min(layout.BIN_FOOTPRINT.half) > layout.HAMMER_FOOTPRINT_RADIUS


##
# The spawn area.
##


def test_spawned_hammers_stay_on_the_table() -> None:
    reach = _reach_of_spawns()
    table = layout.TABLE_TOP
    for sx in (-1, 1):
        for sy in (-1, 1):
            assert table.contains(reach.center[0] + sx * reach.half[0], reach.center[1] + sy * reach.half[1])


@pytest.mark.parametrize(
    "obstacle", [layout.PALLET_FOOTPRINT, SHELF_LEGS, layout.ROBOT_BASE_PLATE], ids=["pallet", "shelf", "base plate"]
)
def test_spawned_hammers_clear_the_fixtures(obstacle: Box2D) -> None:
    assert _gap(_reach_of_spawns(), obstacle) >= CLEARANCE


##
# The arm's workspace.
##


@pytest.fixture(scope="module")
def workspace() -> tuple[tuple[float, ...], tuple[float, ...]]:
    arm = HammerToBinEnvCfg().actions.arm
    return arm.workspace_min, arm.workspace_max


def _inside(point, workspace) -> bool:
    lo, hi = workspace
    return all(a <= p <= b for p, a, b in zip(point, lo, hi, strict=True))


def test_home_tcp_is_in_the_workspace(workspace) -> None:
    assert _inside(HOME_TCP_IN_BASE, workspace)


def test_every_grasp_target_is_in_the_workspace(workspace) -> None:
    """Over the spawn area, at every yaw: the handle point plus the grasp height."""
    area = layout.HAMMER_SPAWN_AREA
    rest_z = layout.WORK_SURFACE_Z + layout.HAMMER_REST_HEIGHT
    for fx in (-1, 0, 1):
        for fy in (-1, 0, 1):
            for yaw in range(0, 360, 15):
                center = (area.center[0] + fx * area.half[0], area.center[1] + fy * area.half[1])
                body_rot = layout.quat_mul(layout.yaw_deg(yaw), layout.HAMMER_TABLE_BODY.rot)
                c = layout.quat_rotate(body_rot, layout.HAMMER_BOX_CENTER)
                g = layout.quat_rotate(body_rot, layout.HAMMER_GRASP_POINT)
                handle = (center[0] - c[0] + g[0], center[1] - c[1] + g[1], rest_z + g[2])
                target = (handle[0], handle[1], handle[2] + layout.HAMMER_GRASP_HEIGHT)
                assert _inside(_to_base(target), workspace), (center, yaw)


def test_bin_target_is_in_the_workspace(workspace) -> None:
    # The TCP is above the hammer's centre when carrying it; allow for that.
    for dz in (0.0, 0.05):
        x, y, z = layout.BIN_TARGET
        assert _inside(_to_base((x, y, z + dz)), workspace)
