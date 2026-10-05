# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The gripper rule: binary open/close from where the TCP and the object are. Not learned.

It is an action term with no action -- ``action_dim`` 0, the action manager
hands it an empty slice -- so it runs every physics step alongside the arm and
drives ``finger_joint`` exactly like the robot's own gripper command would. On
the real robot the same rule is a small node sending open/close to the gripper.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass

from techtory_cobotta_isaaclab.robot.robot_cfg import GRIPPER_CLOSED, GRIPPER_JOINT, GRIPPER_OPEN

from . import state

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

__all__ = ["ProximityGripperAction", "ProximityGripperActionCfg"]

OPEN, CLOSED, RELEASED = 0, 1, 2
"""Rule states. RELEASED is open for good: the object has been let go over the bin."""


class ProximityGripperAction(ActionTerm):
    """Closes the jaw at the object's grasp point, reopens after a miss, opens for good over the bin.

    * OPEN -> CLOSED when the TCP is at the grasp pose (:func:`state.at_grasp_pose`):
      within 8 mm of the grasp target seen from above and 6 mm in height, yaw
      within 15 deg modulo a half turn.
    * CLOSED -> OPEN on a miss: the jaw closed on nothing, or has been closed
      :attr:`ProximityGripperActionCfg.close_timeout` without holding the object
      (stalled on the table, say). It will not close again until the TCP has
      backed off :attr:`ProximityGripperActionCfg.rearm_distance`, so a miss
      cannot chatter.
    * CLOSED -> RELEASED when every corner of the object is over the bin, within
      the release height (:func:`state.object_over_bin`).
    """

    cfg: ProximityGripperActionCfg

    def __init__(self, cfg: ProximityGripperActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self._finger_ids, _ = self._asset.find_joints([GRIPPER_JOINT])
        n = self.num_envs
        self._state = torch.full((n,), OPEN, dtype=torch.long, device=self.device)
        self._rearm_pending = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._closed_ever = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._picked_ever = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._closed_without_grasp = torch.zeros(n, device=self.device)
        self._lost_this_step = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._empty = torch.zeros(n, 0, device=self.device)
        self._open = torch.full((n, 1), GRIPPER_OPEN, device=self.device)
        self._closed = torch.full((n, 1), GRIPPER_CLOSED, device=self.device)

    @property
    def action_dim(self) -> int:
        return 0

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._empty

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._empty

    @property
    def closed(self) -> torch.Tensor:
        """The jaw is commanded closed."""
        return self._state == CLOSED

    @property
    def released(self) -> torch.Tensor:
        """The object has been let go over the bin this episode."""
        return self._state == RELEASED

    @property
    def closed_ever(self) -> torch.Tensor:
        """The rule has closed the jaw at least once this episode: the TCP reached the grasp pose."""
        return self._closed_ever

    @property
    def picked_ever(self) -> torch.Tensor:
        """The object has been picked (grasped and lifted) at least once this episode."""
        return self._picked_ever

    @property
    def lost_this_step(self) -> torch.Tensor:
        """A picked object slipped out during the current policy step: the rule saw the jaw close on nothing.

        The rule's own miss detection, not a dip in :func:`state.object_grasped`:
        that reads the jaw's speed, which jitters under load while carrying.
        """
        return self._lost_this_step

    def process_actions(self, actions: torch.Tensor) -> None:
        # A new policy step: events are reported per step.
        self._lost_this_step[:] = False

    def apply_actions(self) -> None:
        s = self._state
        if self.cfg.rearm_distance > 0.0:
            self._rearm_pending &= state.grasp_distance(self._env) < self.cfg.rearm_distance
        close = (s == OPEN) & ~self._rearm_pending & state.at_grasp_pose(self._env)
        s[close] = CLOSED
        self._closed_ever |= close

        grasped = state.object_grasped(self._env)
        self._closed_without_grasp = torch.where(
            (s == CLOSED) & ~grasped,
            self._closed_without_grasp + self._env.physics_dt,
            torch.zeros_like(self._closed_without_grasp),
        )
        missed = (s == CLOSED) & (
            state.jaw_closed_on_nothing(self._env) | (self._closed_without_grasp > self.cfg.close_timeout)
        )
        s[missed] = OPEN
        self._rearm_pending |= missed
        self._lost_this_step |= missed & self._picked_ever

        self._picked_ever |= grasped & (state.object_lift(self._env) > state.PICK_LIFT)
        release = (s == CLOSED) & self._picked_ever & state.object_over_bin(self._env)
        s[release] = RELEASED

        target = torch.where((s == CLOSED).unsqueeze(-1), self._closed, self._open)
        self._asset.set_joint_position_target_index(target=target, joint_ids=self._finger_ids)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        if env_ids is None:
            env_ids = slice(None)
        self._state[env_ids] = OPEN
        self._rearm_pending[env_ids] = False
        self._closed_ever[env_ids] = False
        self._picked_ever[env_ids] = False
        self._closed_without_grasp[env_ids] = 0.0
        self._lost_this_step[env_ids] = False


@configclass
class ProximityGripperActionCfg(ActionTermCfg):
    """Config for :class:`ProximityGripperAction`."""

    class_type: type[ActionTerm] = ProximityGripperAction
    asset_name: str = "robot"

    rearm_distance: float = 0.03
    """After a miss, the TCP must back off this far from the grasp target before the rule closes again [m]."""

    close_timeout: float = 3.0
    """Closed this long [s] without holding the handle counts as a miss.

    Closing from fully open onto the handle is 1.1 rad at 0.6 rad/s: about 1.8 s.
    """
