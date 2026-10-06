# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Staged rewards for pick-to-bin: progress shaping plus one-off milestone bonuses.

Units: Isaac Lab's reward manager multiplies every term by the policy step
``dt``. Every term here divides by ``dt`` again, so a term's **weight is the
reward itself** -- +5 per milestone, k per metre of progress, -0.005 per step --
exactly as in the task's reward table, independent of the control rate.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs import mdp
from isaaclab.managers import ManagerTermBase, RewardTermCfg

from . import state

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

__all__ = [
    "ApproachProgress",
    "NearBin",
    "PickedObject",
    "PlacedInBin",
    "ReachedObject",
    "TransportProgress",
    "action_rate",
    "joint_acceleration",
    "lost_in_transport",
    "per_step",
    "termination_event",
]


def per_step(env: ManagerBasedRLEnv) -> torch.Tensor:
    """1 per step (weight = reward per step, e.g. a time penalty)."""
    return torch.full((env.num_envs,), 1.0 / env.step_dt, device=env.device)


def action_rate(env: ManagerBasedRLEnv) -> torch.Tensor:
    """``|a_t - a_(t-1)|^2`` per step: the most effective single smoothness term."""
    return mdp.action_rate_l2(env) / env.step_dt


def joint_acceleration(env: ManagerBasedRLEnv) -> torch.Tensor:
    """``|q_target''|^2`` of the arm's commanded joint targets [rad^2/s^4], per step.

    The commanded acceleration, not the simulator's measured one: the stiff
    servos micro-vibrate at ~10 rad/s^2 whatever the command (measured in free
    air), which the policy cannot change and a penalty should not chase.
    """
    return env.action_manager.get_term("arm").joint_target_acceleration_sq / env.step_dt


def termination_event(env: ManagerBasedRLEnv, term_name: str) -> torch.Tensor:
    """1 on the step the termination ``term_name`` fires (weight = reward for it).

    Terminations are computed before rewards in each step, so this sees the
    current step's verdict.
    """
    return env.termination_manager.get_term(term_name).float() / env.step_dt


##
# Progress shaping.
##


class _Progress(ManagerTermBase):
    """Pays ``potential(t-1) - potential(t)`` on steps where the subclass's gate holds.

    The previous potential is updated every step, gated or not, so a gated-off
    stretch is never paid later. NaN after a reset makes the first step pay 0.
    Summed over an episode the payments telescope: moving back and forth
    cannot farm them.
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._prev = torch.full((env.num_envs,), float("nan"), device=env.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._prev[slice(None) if env_ids is None else env_ids] = float("nan")

    def _pay(self, env: ManagerBasedRLEnv, potential: torch.Tensor, gate: torch.Tensor) -> torch.Tensor:
        gain = torch.nan_to_num(self._prev - potential, nan=0.0)
        self._prev = potential
        return torch.where(gate, gain, torch.zeros_like(gain)) / env.step_dt


class ApproachProgress(_Progress):
    """TCP closing in on the grasp pose: potential ``|TCP - grasp target| + yaw_weight * |yaw error|``.

    The yaw error is modulo a half turn, as the gripper rule measures it, so
    the policy is paid for turning towards whichever of the two correct jaw
    orientations is nearer. Only while the object is not grasped.
    """

    def __call__(self, env: ManagerBasedRLEnv, yaw_weight: float = 0.1) -> torch.Tensor:
        """``yaw_weight`` [m/rad]: 0.1 makes a quarter turn count like 16 cm."""
        potential = state.grasp_distance(env) + yaw_weight * state.grasp_yaw_error(env).abs()
        return self._pay(env, potential, ~state.object_grasped(env))


class TransportProgress(_Progress):
    """Object closing in on :data:`~techtory_cobotta_isaaclab.scene.layout.BIN_TARGET`, above the bin.

    Paid only if the object is grasped at both the previous and the current
    step: carrying it there counts, knocking or throwing it there does not.
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._was_grasped = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        self._was_grasped[slice(None) if env_ids is None else env_ids] = False

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        grasped = state.object_grasped(env)
        gate = grasped & self._was_grasped
        self._was_grasped = grasped
        return self._pay(env, state.bin_target_distance(env), gate)


##
# One-off milestones.
##


class _Once(ManagerTermBase):
    """Pays 1 the first step :meth:`_condition` holds in an episode, never again until reset."""

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._paid = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._paid[slice(None) if env_ids is None else env_ids] = False

    def _condition(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        raise NotImplementedError

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        new = self._condition(env) & ~self._paid
        self._paid |= new
        return new.float() / env.step_dt


class ReachedObject(_Once):
    """The TCP reached the grasp pose, i.e. the rule closed the jaw (8 mm, 6 mm in height, 15 deg)."""

    def _condition(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        return state.gripper_rule(env).closed_ever


class PickedObject(_Once):
    """The object is grasped and lifted off the table (:func:`state.object_picked`)."""

    def _condition(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        return state.object_picked(env)


class NearBin(_Once):
    """The grasped object has entered the bin's vicinity (:func:`state.object_near_bin`)."""

    def _condition(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        return state.object_grasped(env) & state.object_near_bin(env)


class PlacedInBin(_Once):
    """The rule let the object go over the bin, every corner inside (:func:`state.object_over_bin`)."""

    def _condition(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        return state.gripper_rule(env).released


##
# Failure events.
##


def lost_in_transport(env: ManagerBasedRLEnv) -> torch.Tensor:
    """1 on each step a picked object slips out of the jaw outside the bin's vicinity.

    Read from the gripper rule, which sees the jaw close on nothing. Letting go
    over the bin is the rule's release, not a loss.
    """
    lost = state.gripper_rule(env).lost_this_step & ~state.object_near_bin(env)
    return lost.float() / env.step_dt
