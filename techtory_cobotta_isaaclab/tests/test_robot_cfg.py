# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Regression tests for ``COBOTTA_RG6_CFG`` and the action space, against the robot USD.

Kit-less: importing the configs builds dataclasses only, and the USD is read with plain ``pxr``.
"""

from __future__ import annotations

import math
import re

import pytest
from pxr import Usd, UsdPhysics

from techtory_cobotta_isaaclab.assets import COBOTTA_RG6_USD
from techtory_cobotta_isaaclab.robot import ActionsCfg
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    ARM_JOINTS,
    COBOTTA_RG6_CFG,
    GRIPPER_JOINT,
    GRIPPER_MIMIC_JOINTS,
    HOME_POSE,
)

pytestmark = pytest.mark.unit

PER_DEG = 180.0 / math.pi


@pytest.fixture(scope="module")
def usd_stage() -> Usd.Stage:
    # Prims do not keep their stage alive, so the stage gets a fixture of its own.
    return Usd.Stage.Open(str(COBOTTA_RG6_USD))


@pytest.fixture(scope="module")
def usd_joints(usd_stage: Usd.Stage) -> dict[str, Usd.Prim]:
    return {
        p.GetName(): p for p in usd_stage.Traverse() if p.IsA(UsdPhysics.Joint) and not p.IsA(UsdPhysics.FixedJoint)
    }


def _limits_rad(joint: Usd.Prim) -> tuple[float, float]:
    revolute = UsdPhysics.RevoluteJoint(joint)
    return math.radians(revolute.GetLowerLimitAttr().Get()), math.radians(revolute.GetUpperLimitAttr().Get())


def test_actuators_cover_every_joint_exactly_once(usd_joints: dict[str, Usd.Prim]) -> None:
    for joint in usd_joints:
        owners = [
            name
            for name, actuator in COBOTTA_RG6_CFG.actuators.items()
            if any(re.fullmatch(expr, joint) for expr in actuator.joint_names_expr)
        ]
        assert len(owners) == 1, f"{joint} is driven by {owners}"


def test_init_state_sets_every_joint_inside_its_limits(usd_joints: dict[str, Usd.Prim]) -> None:
    joint_pos = COBOTTA_RG6_CFG.init_state.joint_pos
    assert set(joint_pos) == set(usd_joints)
    for name, value in joint_pos.items():
        lower, upper = _limits_rad(usd_joints[name])
        assert lower <= value <= upper, name
    assert {k: joint_pos[k] for k in ARM_JOINTS} == HOME_POSE


def test_arm_gains_are_the_usd_gains_in_si(usd_joints: dict[str, Usd.Prim]) -> None:
    """USD authors angular gains per degree; Isaac Lab takes them per radian. A 57x error hides here."""
    arm = COBOTTA_RG6_CFG.actuators["arm"]
    for name in ARM_JOINTS:
        drive = UsdPhysics.DriveAPI(usd_joints[name], "angular")
        assert arm.stiffness[name] / PER_DEG == pytest.approx(drive.GetStiffnessAttr().Get(), rel=1e-4)
        assert arm.damping[name] / PER_DEG == pytest.approx(drive.GetDampingAttr().Get(), rel=1e-4)


def test_gripper_drive_is_the_demos_force_limited_grip() -> None:
    drive = COBOTTA_RG6_CFG.actuators["gripper_drive"]
    assert drive.joint_names_expr == [GRIPPER_JOINT]
    assert drive.joint_effort_limit == 5.0
    assert drive.stiffness == pytest.approx(3.0 * PER_DEG)
    assert drive.damping == pytest.approx(0.1 * PER_DEG)


def test_mimic_joints_are_undriven() -> None:
    mimic = COBOTTA_RG6_CFG.actuators["gripper_mimic"]
    assert set(mimic.joint_names_expr) == set(GRIPPER_MIMIC_JOINTS)
    assert mimic.stiffness == 0.0 and mimic.damping == 0.0


def test_gripper_command_stays_inside_the_joint(usd_joints: dict[str, Usd.Prim]) -> None:
    gripper = ActionsCfg().gripper
    lower, upper = _limits_rad(usd_joints[GRIPPER_JOINT])
    clip_low, clip_high = gripper.clip[GRIPPER_JOINT]
    assert lower < clip_low < clip_high < upper
    # +1 opens (finger_joint negative), -1 closes.
    assert gripper.scale * 1.0 == pytest.approx(clip_low)
    assert gripper.scale * -1.0 == pytest.approx(clip_high)


def test_spawn_functions_resolve() -> None:
    """The spawners use Isaac Lab's private _spawn_from_usd_file; fail here if an upgrade moves it."""
    from isaaclab.utils.string import string_to_callable

    from techtory_cobotta_isaaclab.scene.scene_cfg import HAMMER_CFG, SHELF_CFG, WORKCELL_CFG

    for spawn in (COBOTTA_RG6_CFG.spawn, WORKCELL_CFG.spawn, SHELF_CFG.spawn, HAMMER_CFG.spawn):
        assert callable(string_to_callable(spawn.func))
