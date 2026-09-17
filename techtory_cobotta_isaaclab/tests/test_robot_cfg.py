# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Regression tests for ``COBOTTA_CFG`` itself, not the URDF it wraps.

Importing ``robot_cfg`` needs Isaac Lab installed but no simulator or GPU: it
only builds config dataclasses. Skipped when Isaac Lab is absent.

The skip is a marker, not a module-level ``pytest.importorskip``: in a shell
that has sourced ROS 2, the ``launch_testing`` pytest plugin turns a skip
raised during collection into "no tests collected" for the WHOLE run, which
silently hides every other test file.
"""

from __future__ import annotations

import importlib.util
import re

import pytest

pytestmark = pytest.mark.skipif(
    importlib.util.find_spec("isaaclab") is None, reason="Isaac Lab is not installed"
)


@pytest.fixture(scope="module")
def cfg():
    from techtory_cobotta_isaaclab.robot.robot_cfg import COBOTTA_CFG

    return COBOTTA_CFG


@pytest.fixture(scope="module")
def names() -> tuple[tuple[str, ...], tuple[str, ...]]:
    from techtory_cobotta_isaaclab.robot.joints import ARM_JOINTS, FT_JOINT, GRIPPER_JOINTS

    return ARM_JOINTS, (*GRIPPER_JOINTS, FT_JOINT)


def test_fixed_joints_are_not_merged(cfg) -> None:
    """Merging would delete cobotta_pro_tool0, the EE frame."""
    assert cfg.spawn.merge_fixed_joints is False


def test_base_is_fixed(cfg) -> None:
    assert cfg.spawn.fix_base is True


def test_actuators_cover_every_joint_exactly_once(cfg, names) -> None:
    arm, gripper = names
    claimed: list[str] = []
    for actuator in cfg.actuators.values():
        for joint in (*arm, *gripper):
            if any(re.fullmatch(expr, joint) for expr in actuator.joint_names_expr):
                claimed.append(joint)
    assert sorted(claimed) == sorted((*arm, *gripper))


def test_arm_effort_limits_partition_the_arm(cfg, names) -> None:
    arm, _ = names
    limits = cfg.actuators["arm"].effort_limit_sim
    for joint in arm:
        assert sum(bool(re.fullmatch(expr, joint)) for expr in limits) == 1, joint


def test_initial_joint_pos_partitions_the_joints(cfg, names) -> None:
    arm, gripper = names
    patterns = cfg.init_state.joint_pos
    for joint in (*arm, *gripper):
        assert sum(bool(re.fullmatch(expr, joint)) for expr in patterns) == 1, joint
