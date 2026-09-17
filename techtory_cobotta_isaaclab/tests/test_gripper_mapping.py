# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The one-scalar gripper command and its fan-out to the six RG6 joints."""

from __future__ import annotations

import pytest

from conftest import joints

J = joints()


def test_plus_one_opens_minus_one_closes() -> None:
    assert J.gripper_command_to_leader(1.0) == pytest.approx(J.GRIPPER_OPEN)
    assert J.gripper_command_to_leader(-1.0) == pytest.approx(J.GRIPPER_CLOSED)
    assert J.gripper_command_to_leader(0.0) == pytest.approx(0.0)


def test_action_cfg_defaults_match_the_mapping() -> None:
    """GripperActionCfg uses scale/offset; they must be this same mapping."""
    scale = 0.5 * (J.GRIPPER_OPEN - J.GRIPPER_CLOSED)
    offset = 0.5 * (J.GRIPPER_OPEN + J.GRIPPER_CLOSED)
    for command in (-1.0, -0.3, 0.0, 0.7, 1.0):
        assert offset + scale * command == pytest.approx(J.gripper_command_to_leader(command))


def test_every_joint_gets_multiplier_times_leader() -> None:
    targets = J.gripper_targets(0.4)
    assert list(targets) == list(J.GRIPPER_JOINTS)
    for name, multiplier in J.GRIPPER_MIMIC.items():
        assert targets[name] == pytest.approx(multiplier * 0.4)
    assert targets[J.GRIPPER_LEADER] == pytest.approx(0.4)


def test_targets_are_clamped_per_joint() -> None:
    limits = {name: (-0.5, 0.5) for name in J.GRIPPER_JOINTS}
    targets = J.gripper_targets(0.9, limits)
    for name, multiplier in J.GRIPPER_MIMIC.items():
        assert targets[name] == pytest.approx(0.5 if multiplier > 0 else -0.5)
