# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The pick-to-bin layout: the objects, the bin, the spawn zone, and what the arm may reach.

Kit-less: plain ``pxr`` for the objects' geometry, plain Python for the rest.
"""

from __future__ import annotations

import math

import pytest
from pxr import Gf, Usd, UsdGeom

from techtory_cobotta_isaaclab.assets import HAMMER_USD, PALLET_USD, SODA_CAN_USD
from techtory_cobotta_isaaclab.scene import layout
from techtory_cobotta_isaaclab.scene.grasp_objects import GRASP_OBJECTS, HAMMER, SODA_CAN, GraspableObject
from techtory_cobotta_isaaclab.scene.layout import Box2D, Pose
from techtory_cobotta_isaaclab.scene.scene_cfg import GRASPABLE_OBJECT_CFGS
from techtory_cobotta_isaaclab.tasks.pick_to_bin.config.cobotta.env_cfg import PickToBinEnvCfg

pytestmark = pytest.mark.unit

BIN_PLACE_IN_BASE = (0.54, -0.115)
"""``bin_place`` in ``techtory_cobotta_system/config/poses_hammer.yaml``: the real system's drop pose."""

HOME_TCP_IN_BASE = (0.563, 0.120, 0.246)
"""The TCP at ``HOME_POSE`` in the robot base frame, measured in simulation (check_pick_to_bin.py)."""

CLEARANCE = 0.015
"""Least gap between the spawn zone and anything else on the table [m]."""

OBJECTS = list(GRASP_OBJECTS.values())
OBJECT_IDS = [o.name for o in OBJECTS]


def _to_base(point: tuple[float, float, float]) -> tuple[float, float, float]:
    """A cell-frame point in the robot base frame."""
    rel = tuple(p - m for p, m in zip(point, layout.ROBOT_MOUNT.pos, strict=True))
    return layout.quat_rotate(layout.quat_conj(layout.ROBOT_MOUNT.rot), rel)


def _gap(a: Box2D, b: Box2D) -> float:
    """Distance between two rectangles (negative: they overlap)."""
    dx = abs(a.center[0] - b.center[0]) - a.half[0] - b.half[0]
    dy = abs(a.center[1] - b.center[1]) - a.half[1] - b.half[1]
    return max(dx, dy)


def _mesh_box_in_body(usd, mesh_path: str, body: GraspableObject) -> tuple[list[float], list[float]]:
    """The mesh's bounding box centre and half-extents, in the object's body frame."""
    stage = Usd.Stage.Open(str(usd))
    mesh = stage.GetPrimAtPath(mesh_path)
    to_root = UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    points = [to_root.Transform(Gf.Vec3d(*map(float, p))) for p in UsdGeom.Mesh(mesh).GetPointsAttr().Get()]
    lo = [min(p[i] for p in points) for i in range(3)]
    hi = [max(p[i] for p in points) for i in range(3)]
    center_root = [(a + b) / 2 for a, b in zip(lo, hi, strict=True)]
    half_root = [(b - a) / 2 for a, b in zip(lo, hi, strict=True)]
    to_body = layout.quat_conj(body.body_offset.rot)
    center = layout.quat_rotate(to_body, tuple(c - o for c, o in zip(center_root, body.body_offset.pos, strict=True)))
    half = [abs(h) for h in layout.quat_rotate(to_body, tuple(half_root))]
    return list(center), half


##
# The objects.
##


def test_every_object_has_an_asset() -> None:
    assert set(GRASP_OBJECTS) == set(GRASPABLE_OBJECT_CFGS)


@pytest.mark.parametrize(
    ("obj", "usd", "mesh"),
    [(HAMMER, HAMMER_USD, "/World/hammer/hamLo"), (SODA_CAN, SODA_CAN_USD, "/World/node_/mesh_")],
    ids=["hammer", "soda_can"],
)
def test_object_box_matches_its_mesh(obj: GraspableObject, usd, mesh: str) -> None:
    center, half = _mesh_box_in_body(usd, mesh, obj)
    assert center == pytest.approx(list(obj.box_center), abs=5e-4)
    assert half == pytest.approx(list(obj.box_half), abs=5e-4)


@pytest.mark.parametrize("obj", OBJECTS, ids=OBJECT_IDS)
def test_object_rests_just_above_the_table(obj: GraspableObject) -> None:
    assert 0.0 < obj.rest_clearance < 0.005


@pytest.mark.parametrize("obj", OBJECTS, ids=OBJECT_IDS)
def test_grasp_point_is_inside_the_object(obj: GraspableObject) -> None:
    for g, c, h in zip(obj.grasp_point, obj.box_center, obj.box_half, strict=True):
        assert abs(g - c) < h
    if obj.grasp_axis is not None:
        assert math.hypot(*obj.grasp_axis) == pytest.approx(1.0)
    low, high = obj.jaw_on_object
    assert -0.62 < low < high < 0.62


def test_hammer_grasp_is_on_the_handle() -> None:
    grasp = layout.quat_rotate(HAMMER.body_offset.rot, HAMMER.grasp_point)  # file frame
    assert -0.03 < grasp[0] < 0.15  # the head is at x < -0.03


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
    assert all(
        f + 0.03 == pytest.approx(i) for f, i in zip(layout.BIN_FOOTPRINT.half, layout.BIN_INTERIOR.half, strict=True)
    )
    assert all(i < p for i, p in zip(layout.BIN_INTERIOR.half, layout.PALLET_FOOTPRINT.half, strict=True))
    assert layout.BIN_FOOTPRINT.contains(*layout.BIN_TARGET[:2])
    assert layout.BIN_TARGET[2] > layout.BIN_RIM_Z + 0.1  # carried over the wall, not into it


@pytest.mark.parametrize("obj", OBJECTS, ids=OBJECT_IDS)
def test_object_fits_the_release_footprint_at_any_yaw(obj: GraspableObject) -> None:
    assert min(layout.BIN_FOOTPRINT.half) > obj.footprint_radius


##
# The spawn zone.
##


def test_spawn_zone_is_on_the_table() -> None:
    zone, table = layout.SPAWN_ZONE, layout.TABLE_TOP
    for sx in (-1, 1):
        for sy in (-1, 1):
            assert table.contains(zone.center[0] + sx * zone.half[0], zone.center[1] + sy * zone.half[1])


@pytest.mark.parametrize(
    "obstacle",
    [layout.PALLET_FOOTPRINT, layout.SHELF_FOOTPRINT, layout.ROBOT_BASE_PLATE],
    ids=["pallet", "shelf", "base plate"],
)
def test_spawn_zone_clears_the_fixtures(obstacle: Box2D) -> None:
    assert _gap(layout.SPAWN_ZONE, obstacle) >= CLEARANCE


@pytest.mark.parametrize("obj", OBJECTS, ids=OBJECT_IDS)
def test_every_object_has_room_in_the_zone(obj: GraspableObject) -> None:
    assert min(layout.SPAWN_ZONE.shrunk(obj.footprint_radius).half) > 0.05


##
# The cell's glass.
##


@pytest.mark.parametrize(
    "inside",
    [layout.PALLET_FOOTPRINT, layout.SHELF_FOOTPRINT, layout.SPAWN_ZONE, layout.ROBOT_BASE_PLATE, layout.TABLE_TOP],
    ids=["pallet", "shelf", "spawn zone", "base plate", "table top"],
)
def test_glass_is_outside_everything_in_the_cell(inside: Box2D) -> None:
    frame = layout.CELL_FRAME_INNER
    assert inside.center[0] + inside.half[0] < frame["+x"]
    assert inside.center[0] - inside.half[0] > frame["-x"]
    assert inside.center[1] + inside.half[1] < frame["+y"]
    assert inside.center[1] - inside.half[1] > frame["-y"]


def test_glass_panes_close_the_frame_openings() -> None:
    from techtory_cobotta_isaaclab.scene.scene_cfg import CELL_GLASS_CFGS

    for side, cfg in CELL_GLASS_CFGS.items():
        axis = 0 if side.endswith("x") else 1
        inner = cfg.init_state.pos[axis] - math.copysign(cfg.spawn.size[axis] / 2.0, cfg.init_state.pos[axis])
        assert inner == pytest.approx(layout.CELL_FRAME_INNER[side])
        bottom = cfg.init_state.pos[2] - cfg.spawn.size[2] / 2.0
        top = cfg.init_state.pos[2] + cfg.spawn.size[2] / 2.0
        assert bottom == pytest.approx(layout.TABLE_TOP_Z) and top == pytest.approx(layout.CELL_FRAME_TOP_Z)
        assert cfg.spawn.collision_props is not None


def test_workspace_is_inside_the_glass(workspace) -> None:
    lo, hi = workspace
    frame = layout.CELL_FRAME_INNER
    for x in (lo[0], hi[0]):
        for y in (lo[1], hi[1]):
            cx, cy, _ = layout.compose(layout.ROBOT_MOUNT, Pose((x, y, 0.0))).pos
            assert frame["-x"] < cx < frame["+x"] and frame["-y"] < cy < frame["+y"]


##
# The arm's workspace.
##


@pytest.fixture(scope="module")
def workspace() -> tuple[tuple[float, ...], tuple[float, ...]]:
    arm = PickToBinEnvCfg().actions.arm
    return arm.workspace_min, arm.workspace_max


def _inside(point, workspace) -> bool:
    lo, hi = workspace
    return all(a <= p <= b for p, a, b in zip(point, lo, hi, strict=True))


def test_home_tcp_is_in_the_workspace(workspace) -> None:
    assert _inside(HOME_TCP_IN_BASE, workspace)


@pytest.mark.parametrize("obj", OBJECTS, ids=OBJECT_IDS)
def test_every_grasp_target_is_in_the_workspace(obj: GraspableObject, workspace) -> None:
    """Over the whole spawn zone, at every yaw: the grasp point plus the grasp height."""
    zone = layout.SPAWN_ZONE.shrunk(obj.footprint_radius)
    resting = obj.resting_body
    for fx in (-1, 0, 1):
        for fy in (-1, 0, 1):
            for yaw in range(0, 360, 15):
                center = (zone.center[0] + fx * zone.half[0], zone.center[1] + fy * zone.half[1])
                rot = layout.quat_mul(layout.yaw_deg(yaw), resting.rot)
                c = layout.quat_rotate(rot, obj.box_center)
                g = layout.quat_rotate(rot, obj.grasp_point)
                body_z = resting.pos[2]
                target = (center[0] - c[0] + g[0], center[1] - c[1] + g[1], body_z + g[2] + obj.grasp_height)
                assert _inside(_to_base(target), workspace), (obj.name, center, yaw)


def test_bin_target_is_in_the_workspace(workspace) -> None:
    # The TCP is above the object's centre when carrying it; allow for that.
    for dz in (0.0, 0.08):
        x, y, z = layout.BIN_TARGET
        assert _inside(_to_base((x, y, z + dz)), workspace)


@pytest.mark.parametrize("obj", OBJECTS, ids=OBJECT_IDS)
def test_initial_pose_is_at_rest_in_the_zone(obj: GraspableObject) -> None:
    """The object config's own initial state: resting, box centre on the zone's centre."""
    cfg = PickToBinEnvCfg(grasp_object=obj.name)
    state = cfg.scene.object.init_state
    center = layout.compose(Pose(tuple(state.pos), tuple(state.rot)), Pose(obj.box_center)).pos
    assert center[:2] == pytest.approx(layout.SPAWN_ZONE.center, abs=1e-6)
    assert state.pos[2] == pytest.approx(obj.resting_body.pos[2])
