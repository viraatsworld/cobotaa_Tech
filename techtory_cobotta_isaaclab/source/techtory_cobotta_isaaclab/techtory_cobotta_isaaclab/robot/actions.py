# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Action terms for the Cobotta Pro + RG6.

The arm uses Isaac Lab's stock joint-position action. The gripper needs its own
term: the RG6 is six joints moving as one, and a stock ``JointPositionAction``
over six joints would be a 6-D action that lets a policy tear the linkage apart.
:class:`GripperAction` takes one scalar and fans it out through the mimic
multipliers in :data:`.joints.GRIPPER_MIMIC`.

:class:`ActionsCfg` assembles the full 7-D space. Declaration order fixes the
layout: ``arm`` is ``0:6``, ``gripper`` is ``6``.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs import mdp
from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.utils.configclass import configclass

from techtory_cobotta_isaaclab.robot.joints import (
    ARM_JOINTS,
    GRIPPER_CLOSED,
    GRIPPER_MIMIC,
    GRIPPER_OPEN,
)

if TYPE_CHECKING:
    # Import the runtime Articulation class for ANNOTATIONS ONLY. Naming the
    # `isaaclab.assets.articulation` submodule at runtime bypasses the lazy
    # loader on `isaaclab.assets` and pulls in SimulationContext -> isaacsim
    # -> pxr. This module is imported while a task config is being resolved,
    # which happens BEFORE SimulationApp starts, and loading pxr that early
    # corrupts the allocator: Kit dies with `free(): invalid pointer` a
    # second into startup, with nothing pointing back here.
    from isaaclab.assets.articulation import Articulation
    from isaaclab.envs import ManagerBasedEnv

__all__ = [
    "ActionsCfg",
    "GripperAction",
    "GripperActionCfg",
]


class GripperAction(ActionTerm):
    """Open and close the RG6 from a single scalar.

    Absolute, not an offset: the action commands a ``finger_joint`` angle over
    its own range, which is easier to reason about (and to clamp) than a delta
    from wherever the gripper happened to start. At the default scale and
    offset, ``+1`` is fully open and ``-1`` fully closed.

    Every joint in ``mimic`` gets ``multiplier * finger_joint``, clamped to its
    own limits -- the vectorised form of :func:`.joints.gripper_targets`.
    """

    cfg: GripperActionCfg
    _asset: Articulation

    def __init__(self, cfg: GripperActionCfg, env: ManagerBasedEnv) -> None:
        super().__init__(cfg, env)

        names = list(cfg.mimic)
        self._joint_ids, self._joint_names = self._asset.find_joints(names, preserve_order=True)
        if list(self._joint_names) != names:
            raise ValueError(f"expected gripper joints {names}, resolved {self._joint_names}")

        self._multipliers = torch.tensor(
            [cfg.mimic[name] for name in names], device=self.device
        ).unsqueeze(0)
        self._raw_actions = torch.zeros(self.num_envs, 1, device=self.device)
        self._processed_actions = torch.zeros(self.num_envs, len(names), device=self.device)

        # Clamp to each joint's own travel. A policy is free to emit anything;
        # the joint is not, and driving a position target past a hard limit
        # just loads the actuator against the stop.
        limits = self._asset.data.joint_pos_limits.torch[:, self._joint_ids, :]
        self._lower = limits[..., 0]
        self._upper = limits[..., 1]

    """
    Properties.
    """

    @property
    def action_dim(self) -> int:
        return 1

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        """The per-joint targets actually commanded, radians, in ``mimic`` order."""
        return self._processed_actions

    """
    Operations.
    """

    def process_actions(self, actions: torch.Tensor) -> None:
        self._raw_actions[:] = actions
        leader = self._raw_actions * self.cfg.scale + self.cfg.offset
        self._processed_actions = torch.clamp(leader * self._multipliers, self._lower, self._upper)

    def apply_actions(self) -> None:
        self._asset.set_joint_position_target_index(
            target=self._processed_actions, joint_ids=self._joint_ids
        )

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        indices = slice(None) if env_ids is None else env_ids
        self._raw_actions[indices] = 0.0


@configclass
class GripperActionCfg(ActionTermCfg):
    """Configuration for the one-dimensional RG6 action.

    Args:
        asset_name: Scene entity to drive.
        mimic: ``{joint: multiplier}``; every joint receives
            ``multiplier * finger_joint``.
        scale: ``finger_joint`` radians per unit of policy output. Negative,
            because ``finger_joint`` closes towards positive angles.
        offset: ``finger_joint`` angle, radians, at zero policy output -- the
            half-open midpoint.
    """

    class_type: type[ActionTerm] = GripperAction

    asset_name: str = "robot"
    mimic: dict[str, float] = GRIPPER_MIMIC
    scale: float = 0.5 * (GRIPPER_OPEN - GRIPPER_CLOSED)
    offset: float = 0.5 * (GRIPPER_OPEN + GRIPPER_CLOSED)


@configclass
class ActionsCfg:
    """The standard 7-D action space: ``[q1..q6 offset, grip]``.

    Term order is declaration order, and that is what fixes the index layout --
    reordering these fields renames every index in the README's command table.
    """

    arm: mdp.JointPositionActionCfg = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=list(ARM_JOINTS),
        preserve_order=True,
        scale=0.5,
        use_default_offset=True,
    )

    # ONE action for all six RG6 joints, which is why this is not a stock
    # JointPositionAction: that term's dimension is its joint count.
    gripper: GripperActionCfg = GripperActionCfg()
