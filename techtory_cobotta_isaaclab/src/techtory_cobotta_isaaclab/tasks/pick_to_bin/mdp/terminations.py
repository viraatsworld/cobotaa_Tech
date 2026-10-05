# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""How a pick-to-bin episode ends, besides the clock: success, or the object lost for good.

Both need the object *at rest* for a few steps, not just slow for one: an object
is momentarily still at the top of a bounce, or the instant it slips from the jaw.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.managers import ManagerTermBase, TerminationTermCfg

from techtory_cobotta_isaaclab.scene import layout

from . import state

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

__all__ = ["DroppedOutsideBin", "ObjectResting"]


class _Held(ManagerTermBase):
    """Fires once :meth:`_condition` has held for ``hold_steps`` consecutive steps."""

    def __init__(self, cfg: TerminationTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._count = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._count[slice(None) if env_ids is None else env_ids] = 0

    def _condition(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        raise NotImplementedError

    def _held(self, env: ManagerBasedRLEnv, hold_steps: int) -> torch.Tensor:
        self._count = torch.where(self._condition(env), self._count + 1, torch.zeros_like(self._count))
        return self._count >= hold_steps


def _let_go_and_resting(env: ManagerBasedRLEnv) -> torch.Tensor:
    return ~state.gripper_rule(env).closed & (state.object_speed(env) < state.REST_SPEED)


class ObjectResting(_Held):
    """Success: the object rests inside the bin, out of the jaw, for ``hold_steps`` steps (0.4 s at 25 Hz).

    Logs ``Metrics/success_rate`` over the environments being reset.
    """

    def __init__(self, cfg: TerminationTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._succeeded = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        ids = slice(None) if env_ids is None else env_ids
        self._env.extras.setdefault("log", {})["Metrics/success_rate"] = self._succeeded[ids].float().mean().item()
        self._succeeded[ids] = False
        super().reset(env_ids)

    def _condition(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        return state.object_in_bin(env) & _let_go_and_resting(env)

    def __call__(self, env: ManagerBasedRLEnv, hold_steps: int = 10) -> torch.Tensor:
        done = self._held(env, hold_steps)
        self._succeeded |= done
        return done


class DroppedOutsideBin(_Held):
    """Failure: the object is lost for good.

    Either it was picked, is out of the jaw and has come to rest outside the
    bin for ``hold_steps`` steps -- a pause mid-carry or a squeeze on the table
    never counts, because the object is still grasped then -- or it has fallen
    off the table, picked or not.
    """

    def _condition(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        picked = state.gripper_rule(env).picked_ever
        return picked & _let_go_and_resting(env) & ~state.object_in_bin(env)

    def __call__(self, env: ManagerBasedRLEnv, hold_steps: int = 5, fall_depth: float = 0.04) -> torch.Tensor:
        """``fall_depth`` [m]: lowest corner this far below the table top means it fell off."""
        fell = state.object_corners(env)[..., 2].amin(dim=1) < layout.TABLE_TOP_Z - fall_depth
        return self._held(env, hold_steps) | fell
