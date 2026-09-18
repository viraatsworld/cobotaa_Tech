# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The copied USDs: complete, unchanged, and shaped the way the configs assume.

Kit-less: plain ``pxr``. PhysX schemas are not registered without Kit, so their
application is read from the raw ``apiSchemas`` metadata.
"""

from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path

import pytest
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

from techtory_cobotta_isaaclab.assets import ASSETS_DIR, COBOTTA_RG6_USD
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    ARM_JOINTS,
    FT_BODY,
    GRIPPER_JOINT,
    GRIPPER_MIMIC_JOINTS,
    TCP_OFFSET,
)

pytestmark = pytest.mark.unit

_SYNC_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "sync_assets.py"


def _sync_assets_module():
    spec = importlib.util.spec_from_file_location("sync_assets", _SYNC_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _applied(prim: Usd.Prim) -> list[str]:
    schemas = prim.GetMetadata("apiSchemas")
    return list(schemas.GetAddedOrExplicitItems()) if schemas else []


@pytest.fixture(scope="module")
def robot() -> Usd.Stage:
    return Usd.Stage.Open(str(COBOTTA_RG6_USD))


def _joints(stage: Usd.Stage) -> dict[str, Usd.Prim]:
    return {p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}


def test_copied_roots_resolve_inside_the_package() -> None:
    sync = _sync_assets_module()
    root = ASSETS_DIR.resolve()
    for relative in sync.ROOT_USDS:
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(ASSETS_DIR / relative)))
        for path in [layer.realPath for layer in layers] + list(assets):
            assert Path(path).resolve().is_relative_to(root), f"{relative} reaches outside the package: {path}"
        assert all(u.endswith(sync.ALLOWED_UNRESOLVED) for u in unresolved), unresolved


def test_copies_are_byte_identical_to_the_isaacsim_assets() -> None:
    sync = _sync_assets_module()
    if not sync.DEFAULT_SOURCE.is_dir():
        pytest.skip(f"{sync.DEFAULT_SOURCE} not present")
    for relative in sync.dependency_closure(sync.DEFAULT_SOURCE):
        copy, original = ASSETS_DIR / relative, sync.DEFAULT_SOURCE / relative
        assert hashlib.sha256(copy.read_bytes()).digest() == hashlib.sha256(original.read_bytes()).digest(), (
            f"{relative} drifted -- run scripts/sync_assets.py"
        )


def test_one_articulation_rooted_on_the_world_joint(robot: Usd.Stage) -> None:
    roots = [p for p in robot.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    assert [p.GetName() for p in roots] == ["root_joint"]
    assert not UsdPhysics.Joint(roots[0]).GetBody0Rel().GetTargets(), "root joint should be fixed to the world"


def test_joint_set_is_arm_plus_gripper(robot: Usd.Stage) -> None:
    movable = {n for n, p in _joints(robot).items() if not p.IsA(UsdPhysics.FixedJoint)}
    assert movable == {*ARM_JOINTS, GRIPPER_JOINT, *GRIPPER_MIMIC_JOINTS}


def test_mimic_joints_follow_finger_joint(robot: Usd.Stage) -> None:
    joints = _joints(robot)
    for name in GRIPPER_MIMIC_JOINTS:
        assert "PhysxMimicJointAPI:rotX" in _applied(joints[name])
        targets = joints[name].GetRelationship("physxMimicJoint:rotX:referenceJoint").GetTargets()
        assert [t.name for t in targets] == [GRIPPER_JOINT]
    assert "PhysxMimicJointAPI:rotX" not in _applied(joints[GRIPPER_JOINT])


def test_ft_body_is_the_fixed_mount_on_the_flange(robot: Usd.Stage) -> None:
    """The F/T frame is the child side of gripper_joint; identity there means it is base_link's frame."""
    mount = UsdPhysics.FixedJoint(_joints(robot)["gripper_joint"])
    assert [t.name for t in mount.GetBody0Rel().GetTargets()] == ["cobotta_pro_J6"]
    assert [t.name for t in mount.GetBody1Rel().GetTargets()] == [FT_BODY]
    assert tuple(mount.GetLocalPos1Attr().Get()) == pytest.approx((0.0, 0.0, 0.0))
    assert mount.GetLocalRot1Attr().Get() == Gf.Quatf(1.0, 0.0, 0.0, 0.0)


def test_root_link_sits_at_the_spawn_prim(robot: Usd.Stage) -> None:
    """Isaac Lab writes init_state onto the root link, so it must coincide with the file's root."""
    base = robot.GetPrimAtPath("/World/techtory_demo_description/cobotta_pro_base_link")
    assert UsdGeom.XformCache().GetLocalToWorldTransform(base) == Gf.Matrix4d(1.0)


def test_tcp_lies_between_the_pads(robot: Usd.Stage) -> None:
    cache = UsdGeom.XformCache()
    to_ft = cache.GetLocalToWorldTransform(robot.GetPrimAtPath(f"/World/onrobot_rg6/{FT_BODY}")).GetInverse()
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
    for side in ("left", "right"):
        bound = bbox.ComputeWorldBound(robot.GetPrimAtPath(f"/World/onrobot_rg6/{side}_inner_finger/collisions"))
        bound.Transform(to_ft)
        pad = bound.ComputeAlignedRange()
        assert pad.GetMin()[2] < TCP_OFFSET[2] < pad.GetMax()[2]


def test_gripper_link_colliders_are_mesh_merge_collectors(robot: Usd.Stage) -> None:
    """What spawn_cobotta_rg6 repairs. If the asset is ever fixed upstream, the repair becomes a no-op."""
    collectors = [
        p
        for p in robot.GetPrimAtPath("/World/onrobot_rg6").GetChildren()
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetChild("collisions")
    ]
    assert len(collectors) == 6
    for link in collectors:
        assert "PhysxMeshMergeCollisionAPI" in _applied(link.GetChild("collisions"))
