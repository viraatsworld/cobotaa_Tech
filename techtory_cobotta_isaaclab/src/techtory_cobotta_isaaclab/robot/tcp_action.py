# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The arm action for pick-and-place: small top-down TCP moves, tracked by differential IK.

Configure it with :class:`~techtory_cobotta_isaaclab.robot.actions.TopDownTcpTargetActionCfg`;
this module imports ``pxr`` (through Isaac Lab's task-space actions) and is only
loaded once Kit is up, by the action manager.

Why not Isaac Lab's relative-mode IK: it adds each delta to the *measured* TCP
pose and starts the IK from the *measured* joints, so with a zero action the
joint target becomes wherever gravity has pulled the arm, and the arm sinks.
Isaac Lab's own IK tasks hide this by switching gravity off on the arm. This
term instead keeps an integrated TCP target, like MoveIt Servo does: the pose
error that gravity leaves keeps the joint target ahead of the sag, so the drive
holds the arm up.

It also corrects the TCP Jacobian. Isaac Lab shifts the body Jacobian to the
offset frame with the offset as given in the *body* frame, while the Jacobian
is in the *root* frame: right only when the two frames are aligned. Here the
RG6's approach axis points down, so the 0.25 m TCP offset would be taken as
0.25 m *up* -- a Jacobian for a point half a metre from the TCP, which turns
every yaw or tilt correction into an unpredicted translation.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs.mdp.actions.task_space_actions import DifferentialInverseKinematicsAction
from isaaclab.utils.math import quat_apply, quat_inv, quat_mul, skew_symmetric_matrix

from techtory_cobotta_isaaclab.robot.top_down import rate_limit, step_target, top_down_quat, x_axis_yaw

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from techtory_cobotta_isaaclab.robot.actions import TopDownTcpTargetActionCfg

__all__ = ["TopDownTcpTargetAction"]


class TopDownTcpTargetAction(DifferentialInverseKinematicsAction):
    """4-D action ``[dx, dy, dz, dyaw]``: a step of the commanded TCP target, in the robot base frame.

    Per policy step: the action is clipped to [-1, 1], scaled to at most
    :attr:`~TopDownTcpTargetActionCfg.pos_step` / :attr:`~TopDownTcpTargetActionCfg.yaw_step`,
    and added to the commanded target, which is then held within
    :attr:`~TopDownTcpTargetActionCfg.max_lead_pos` / :attr:`~TopDownTcpTargetActionCfg.max_lead_yaw`
    of the actual TCP so it cannot run away while the arm is blocked, and inside
    the optional workspace box. Roll and pitch are not commanded: the target
    always points straight down.

    Per physics step: differential IK towards the target, then the joint target
    is moved towards the IK solution by at most each joint's velocity limit times
    the physics step, so the arm never exceeds its speed caps -- also where the
    damped least-squares solution spikes near a singularity.
    """

    cfg: TopDownTcpTargetActionCfg

    def __init__(self, cfg: TopDownTcpTargetActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        n = self.num_envs
        self._target_pos = torch.zeros(n, 3, device=self.device)
        self._target_yaw = torch.zeros(n, device=self.device)
        self._joint_target = self._asset.data.joint_pos_target.torch[:, self._joint_ids].clone()
        # After a reset, body poses are stale until one physics step has run (the
        # reset only writes joint states). So the first physics step of an
        # episode holds the reset joint targets (countdown 2), the second
        # re-anchors the TCP target on the now-valid TCP pose (countdown 1).
        self._anchor_countdown = torch.full((n,), 2, dtype=torch.long, device=self.device)
        self._max_step = self._asset.data.joint_vel_limits.torch[:, self._joint_ids] * env.physics_dt
        self._step_scale = torch.tensor([cfg.pos_step] * 3 + [cfg.yaw_step], device=self.device)
        self._workspace = None
        if cfg.workspace_min is not None and cfg.workspace_max is not None:
            self._workspace = (
                torch.tensor(cfg.workspace_min, device=self.device),
                torch.tensor(cfg.workspace_max, device=self.device),
            )

    @property
    def action_dim(self) -> int:
        return 4

    @property
    def target_pos_b(self) -> torch.Tensor:
        """Commanded TCP position [m] in the robot base frame, ``(num_envs, 3)``."""
        return self._target_pos

    @property
    def target_yaw(self) -> torch.Tensor:
        """Commanded TCP yaw [rad] in the robot base frame, ``(num_envs,)``."""
        return self._target_yaw

    def process_actions(self, actions: torch.Tensor) -> None:
        self._raw_actions[:] = actions
        self._processed_actions[:] = actions.clamp(-1.0, 1.0) * self._step_scale
        live = self._anchor_countdown == 0
        if not live.any():
            return
        ee_pos, ee_quat = self._compute_frame_pose()
        pos, yaw = step_target(
            self._target_pos,
            self._target_yaw,
            ee_pos,
            x_axis_yaw(ee_quat),
            self._processed_actions,
            self.cfg.max_lead_pos,
            self.cfg.max_lead_yaw,
            self._workspace,
        )
        self._target_pos[live] = pos[live]
        self._target_yaw[live] = yaw[live]

    def apply_actions(self) -> None:
        ee_pos, ee_quat = self._compute_frame_pose()
        joint_pos = self._asset.data.joint_pos.torch[:, self._joint_ids]

        anchor = self._anchor_countdown == 1
        if anchor.any():
            self._target_pos[anchor] = ee_pos[anchor]
            self._target_yaw[anchor] = x_axis_yaw(ee_quat)[anchor]
        hold = (self._anchor_countdown == 2).unsqueeze(-1)
        self._anchor_countdown = (self._anchor_countdown - 1).clamp(min=0)

        command = torch.cat([self._target_pos, top_down_quat(self._target_yaw)], dim=-1)
        self._ik_controller.set_command(command, ee_pos, ee_quat)
        joint_des = self._ik_controller.compute(ee_pos, ee_quat, self._compute_frame_jacobian(), joint_pos)
        limited = rate_limit(self._joint_target, joint_des, self._max_step)
        self._joint_target = torch.where(hold, self._joint_target, limited)
        self._asset.set_joint_position_target_index(target=self._joint_target, joint_ids=self._joint_ids)

    def _compute_frame_jacobian(self) -> torch.Tensor:
        """Geometric Jacobian of the TCP in the root frame.

        ``v_tcp = v_link + w x r``, with ``r`` the link-to-TCP vector **in the
        root frame**: the body-frame offset turned by the link's orientation.
        The offset's rotation is identity, so the angular rows are the link's.
        """
        self._jacobian_b[:] = self.jacobian_b
        data = self._asset.data
        link_quat_b = quat_mul(quat_inv(data.root_quat_w.torch), data.body_quat_w.torch[:, self._body_idx])
        lever_b = quat_apply(link_quat_b, self._offset_pos)
        self._jacobian_b[:, 0:3, :] += torch.bmm(-skew_symmetric_matrix(lever_b), self._jacobian_b[:, 3:, :])
        return self._jacobian_b

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        if env_ids is None:
            env_ids = slice(None)
        self._raw_actions[env_ids] = 0.0
        self._processed_actions[env_ids] = 0.0
        # The reset events have just written the drive targets (home); hold them.
        self._joint_target[env_ids] = self._asset.data.joint_pos_target.torch[env_ids][:, self._joint_ids]
        self._anchor_countdown[env_ids] = 2
