# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Observation terms for pick-to-bin.

Positions are given in the robot base frame, like the base task's. Angles are
given as sin/cos pairs, never as raw quaternions: a quaternion and its negative
are the same rotation, and the grasp error is only defined modulo a half turn,
so raw values would jump where nothing physical does. The grasp yaw error is
encoded as sin/cos of *twice* the error, which is smooth across that half turn.
For an object that is round seen from above, both yaws read 0.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.utils.math import quat_apply_inverse, quat_unique, subtract_frame_transforms

from techtory_cobotta_isaaclab.scene import layout

from . import state

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

__all__ = [
    "bin_offset_b",
    "grasp_offset_b",
    "grasp_target_b",
    "grasp_yaw_error_sincos",
    "gripper_closed",
    "jaw_position",
    "object_ang_vel_b",
    "object_grasped",
    "object_lin_vel_b",
    "object_picked",
    "object_quat_b",
    "object_yaw_sincos",
    "tcp_pos_b",
    "tcp_yaw_sincos",
]


def _base(env: ManagerBasedEnv) -> tuple[torch.Tensor, torch.Tensor]:
    arm = state.robot(env)
    return arm.data.root_link_pos_w.torch - env.scene.env_origins, arm.data.root_link_quat_w.torch


def _point_b(env: ManagerBasedEnv, point_e: torch.Tensor) -> torch.Tensor:
    base_pos, base_quat = _base(env)
    return subtract_frame_transforms(base_pos, base_quat, point_e)[0]


def _vector_b(env: ManagerBasedEnv, vector_e: torch.Tensor) -> torch.Tensor:
    return quat_apply_inverse(_base(env)[1], vector_e)


def _sincos(angle: torch.Tensor) -> torch.Tensor:
    return torch.stack([torch.sin(angle), torch.cos(angle)], dim=-1)


# The base is only turned about the vertical, so a heading in the base frame is
# the env-frame heading minus the base's own.
def _base_yaw(env: ManagerBasedEnv) -> torch.Tensor:
    x_axis = _vector_b(env, torch.tensor([[1.0, 0.0, 0.0]], device=env.device).expand(env.num_envs, 3))
    return -torch.atan2(x_axis[:, 1], x_axis[:, 0])


##
# Actor: what the real robot can provide (joint states, forward kinematics, an object pose estimate).
##


def tcp_pos_b(env: ManagerBasedEnv) -> torch.Tensor:
    """TCP position [m] ``(N, 3)``."""
    return _point_b(env, state.tcp_pose(env)[0])


def tcp_yaw_sincos(env: ManagerBasedEnv) -> torch.Tensor:
    """TCP heading as ``(sin, cos)`` ``(N, 2)``."""
    return _sincos(state.tcp_yaw(env) - _base_yaw(env))


def gripper_closed(env: ManagerBasedEnv) -> torch.Tensor:
    """1 while the gripper rule holds the jaw closed ``(N, 1)``."""
    return state.gripper_rule(env).closed.float().unsqueeze(-1)


def grasp_target_b(env: ManagerBasedEnv) -> torch.Tensor:
    """Where the TCP grasps the object [m] ``(N, 3)``."""
    return _point_b(env, state.grasp_target(env))


def object_yaw_sincos(env: ManagerBasedEnv) -> torch.Tensor:
    """Heading of the object's grasp axis as ``(sin, cos)`` ``(N, 2)``; tells the hammer's head end from the other."""
    if state.spec(env).grasp_axis is None:
        return _sincos(torch.zeros(env.num_envs, device=env.device))
    return _sincos(state.object_yaw(env) - _base_yaw(env))


def grasp_offset_b(env: ManagerBasedEnv) -> torch.Tensor:
    """Grasp target minus TCP [m] ``(N, 3)``."""
    return _vector_b(env, state.grasp_target(env) - state.tcp_pose(env)[0])


def grasp_yaw_error_sincos(env: ManagerBasedEnv) -> torch.Tensor:
    """``(sin 2e, cos 2e)`` of the grasp yaw error ``e`` ``(N, 2)``: smooth across the jaw's half-turn symmetry."""
    return _sincos(2.0 * state.grasp_yaw_error(env))


def bin_offset_b(env: ManagerBasedEnv) -> torch.Tensor:
    """:data:`layout.BIN_TARGET` minus the object's centre [m] ``(N, 3)``."""
    target = torch.tensor(layout.BIN_TARGET, device=env.device).expand(env.num_envs, 3)
    return _vector_b(env, target - state.object_center(env))


##
# Critic only: simulator ground truth.
##


def object_quat_b(env: ManagerBasedEnv) -> torch.Tensor:
    """Full object orientation ``(x, y, z, w)`` with ``w >= 0`` ``(N, 4)``."""
    base_pos, base_quat = _base(env)
    pos, quat = state.object_pose(env)
    return quat_unique(subtract_frame_transforms(base_pos, base_quat, pos, quat)[1])


def object_lin_vel_b(env: ManagerBasedEnv) -> torch.Tensor:
    return _vector_b(env, state.target_object(env).data.root_lin_vel_w.torch)


def object_ang_vel_b(env: ManagerBasedEnv) -> torch.Tensor:
    return _vector_b(env, state.target_object(env).data.root_ang_vel_w.torch)


def jaw_position(env: ManagerBasedEnv) -> torch.Tensor:
    """``finger_joint`` [rad] ``(N, 1)``."""
    return state.jaw_position(env).unsqueeze(-1)


def object_grasped(env: ManagerBasedEnv) -> torch.Tensor:
    return state.object_grasped(env).float().unsqueeze(-1)


def object_picked(env: ManagerBasedEnv) -> torch.Tensor:
    return state.object_picked(env).float().unsqueeze(-1)
