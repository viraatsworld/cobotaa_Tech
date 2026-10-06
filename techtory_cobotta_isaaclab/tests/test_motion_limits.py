# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The arm's motion limits, pinned: MoveIt's limits at 100%, and the per-step moves they allow.

The pick-to-bin arm action moves the joint targets at most velocity x dt per
physics step and the TCP at most ``pos_step`` / ``yaw_step`` per policy step.
These tests pin those numbers, so changing a speed, ``sim.dt`` or the
decimation shows up here instead of silently changing how fast the arm may move.
The source is the real robot's MoveIt config; the COBOTTA PRO catalogue gives
joint ranges (checked against the USD in test_robot_cfg.py) but no joint speeds
or accelerations.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from techtory_cobotta_isaaclab.robot import ARM_JOINTS, COBOTTA_RG6_CFG
from techtory_cobotta_isaaclab.tasks.pick_to_bin.config.cobotta.env_cfg import PickToBinEnvCfg

pytestmark = pytest.mark.unit

MOVEIT_LIMITS = Path(__file__).resolve().parents[2] / "techtory_cobotta_moveit" / "config" / "joint_limits.yaml"

# The agreed per-step limits [rad]: MoveIt's max_velocity x 0.01 s (physics step)
# and x 0.04 s (policy step, decimation 4).
PER_PHYSICS_STEP = (0.0039794, 0.0033161, 0.0039794, 0.0049742, 0.0049742, 0.0059690)
PER_POLICY_STEP = (0.0159174, 0.0132645, 0.0159174, 0.0198967, 0.0198967, 0.0238761)


@pytest.fixture(scope="module")
def cfg() -> PickToBinEnvCfg:
    return PickToBinEnvCfg()


@pytest.fixture(scope="module")
def moveit() -> dict:
    yaml = pytest.importorskip("yaml")
    if not MOVEIT_LIMITS.exists():
        pytest.skip(f"{MOVEIT_LIMITS} not in this checkout")
    return yaml.safe_load(MOVEIT_LIMITS.read_text())["joint_limits"]


def _velocity_limits() -> list[float]:
    limits = COBOTTA_RG6_CFG.actuators["arm"].joint_velocity_limit
    return [limits[j] for j in ARM_JOINTS]


def test_velocity_limits_are_moveits_at_full_scale(moveit: dict) -> None:
    assert _velocity_limits() == pytest.approx([moveit[j]["max_velocity"] for j in ARM_JOINTS])


def test_acceleration_limit_is_moveits(cfg: PickToBinEnvCfg, moveit: dict) -> None:
    assert all(cfg.actions.arm.joint_acceleration_limit == moveit[j]["max_acceleration"] for j in ARM_JOINTS)


def test_timing_is_10_ms_physics_and_25_hz_policy(cfg: PickToBinEnvCfg) -> None:
    assert cfg.sim.dt == pytest.approx(0.01)
    assert cfg.decimation == 4


def test_per_step_joint_limits_are_unchanged(cfg: PickToBinEnvCfg) -> None:
    physics_dt = cfg.sim.dt
    policy_dt = cfg.sim.dt * cfg.decimation
    assert [v * physics_dt for v in _velocity_limits()] == pytest.approx(PER_PHYSICS_STEP, abs=1e-6)
    assert [v * policy_dt for v in _velocity_limits()] == pytest.approx(PER_POLICY_STEP, abs=1e-6)


def test_per_step_tcp_limits_are_unchanged(cfg: PickToBinEnvCfg) -> None:
    arm = cfg.actions.arm
    policy_dt = cfg.sim.dt * cfg.decimation
    assert arm.pos_step == pytest.approx(0.005)  # 0.125 m/s
    assert arm.yaw_step == pytest.approx(0.012)  # 0.3 rad/s
    assert arm.tcp_acceleration == pytest.approx(0.5)
    assert arm.yaw_acceleration == pytest.approx(1.0)
    # The yaw rate stays under the slowest joint's limit, J2's 0.33 rad/s.
    assert arm.yaw_step / policy_dt < min(_velocity_limits())
    # And the TCP speed is far below the catalogue's 1,800 mm/s collaborative maximum (COBOTTA PRO 900).
    assert arm.pos_step / policy_dt < 1.8
