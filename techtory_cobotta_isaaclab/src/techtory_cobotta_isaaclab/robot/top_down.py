# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Yaw and orientation helpers for a gripper that points straight down.

Shared by :mod:`.tcp_action` (which drives the arm) and the hammer-to-bin task
(which reads the same angles), so every term agrees on what "yaw" means.

Plain torch, no ``pxr``: safe to import before Kit starts.

The TCP frame is ``base_link``'s, moved 0.25 m along its +z, the approach axis.
The RG6's pads sit at -/+ y of that frame, so the jaw closes along the TCP's y
axis. "Pointing down" is a half turn about the robot base's x axis, and the
TCP's *yaw* is the heading of its x axis in the base's x/y plane.
"""

from __future__ import annotations

import math

import torch

from isaaclab.utils.math import quat_apply, quat_from_euler_xyz, quat_mul

__all__ = [
    "POINTING_DOWN",
    "rate_limit",
    "smooth_step",
    "step_target",
    "top_down_quat",
    "wrap_half_turn",
    "wrap_to_pi",
    "x_axis_yaw",
]

POINTING_DOWN: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
"""TCP orientation ``(x, y, z, w)`` at yaw 0: a half turn about x, approach axis along -z."""


def wrap_to_pi(angle: torch.Tensor) -> torch.Tensor:
    """Wrap angles [rad] into ``[-pi, pi)``."""
    return torch.remainder(angle + math.pi, 2.0 * math.pi) - math.pi


def wrap_half_turn(angle: torch.Tensor) -> torch.Tensor:
    """Wrap angles [rad] into ``[-pi/2, pi/2)``: equal modulo a half turn.

    For things that look the same turned by 180 deg -- a parallel jaw, a handle.
    """
    return torch.remainder(angle + math.pi / 2.0, math.pi) - math.pi / 2.0


def x_axis_yaw(quat: torch.Tensor) -> torch.Tensor:
    """Heading [rad] of a frame's x axis in its parent's x/y plane. ``quat`` is ``(..., 4)``, ``(x, y, z, w)``."""
    x_axis = torch.zeros(*quat.shape[:-1], 3, device=quat.device, dtype=quat.dtype)
    x_axis[..., 0] = 1.0
    heading = quat_apply(quat, x_axis)
    return torch.atan2(heading[..., 1], heading[..., 0])


def top_down_quat(yaw: torch.Tensor) -> torch.Tensor:
    """TCP orientation ``(N, 4)`` pointing down with its x axis at ``yaw`` [rad], in the robot base frame."""
    zeros = torch.zeros_like(yaw)
    down = torch.tensor(POINTING_DOWN, device=yaw.device, dtype=yaw.dtype).expand(yaw.shape[0], 4)
    return quat_mul(quat_from_euler_xyz(zeros, zeros, yaw), down)


def step_target(
    target_pos: torch.Tensor,
    target_yaw: torch.Tensor,
    tcp_pos: torch.Tensor,
    tcp_yaw: torch.Tensor,
    delta: torch.Tensor,
    max_lead_pos: float,
    max_lead_yaw: float,
    workspace: tuple[torch.Tensor, torch.Tensor] | None = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Move a commanded TCP target by ``delta = [dx, dy, dz, dyaw]`` ``(N, 4)``.

    The result leads the actual TCP by at most ``max_lead_pos`` per axis [m] and
    ``max_lead_yaw`` [rad], so it cannot run away while the arm is blocked, and
    its position stays inside ``workspace = (lower, upper)`` when one is given.

    Returns:
        The new target position ``(N, 3)`` and yaw ``(N,)``.
    """
    lead = (target_pos + delta[:, :3] - tcp_pos).clamp(-max_lead_pos, max_lead_pos)
    lead_yaw = wrap_to_pi(target_yaw + delta[:, 3] - tcp_yaw).clamp(-max_lead_yaw, max_lead_yaw)
    pos = tcp_pos + lead
    if workspace is not None:
        pos = torch.maximum(torch.minimum(pos, workspace[1]), workspace[0])
    return pos, wrap_to_pi(tcp_yaw + lead_yaw)


def rate_limit(current: torch.Tensor, desired: torch.Tensor, max_step: torch.Tensor) -> torch.Tensor:
    """Move ``current`` towards ``desired`` by at most ``max_step`` per element."""
    return current + (desired - current).clamp(-max_step, max_step)


def smooth_step(
    previous_step: torch.Tensor, error: torch.Tensor, max_step: torch.Tensor, max_step_change: torch.Tensor
) -> torch.Tensor:
    """This tick's move towards ``error`` under a speed and an acceleration limit.

    Per element, in units per tick: the step is at most ``max_step`` (velocity x
    dt) and differs from ``previous_step`` by at most ``max_step_change``
    (acceleration x dt^2). It is also never more than what still lets it stop
    within ``error`` after this tick, so the target is approached on a
    trapezoidal profile and not overshot. A plain clip on the step change would
    carry the speed past the goal and oscillate around it.

    The stopping bound is the discrete one: braking from a step ``s`` by ``a``
    per tick covers ``s^2 / 2a - s / 2`` more, so ``s + s^2 / 2a - s / 2 <= |error|``
    gives ``s <= (-a + sqrt(a^2 + 8 a |error|)) / 2``. The continuous
    ``sqrt(2 a |error|)`` overshoots by about one tick's step.
    """
    a = max_step_change
    stopping = 0.5 * (torch.sqrt(a * a + 8.0 * a * error.abs()) - a)
    reach = torch.minimum(torch.minimum(error.abs(), max_step), stopping)
    desired = torch.sign(error) * reach
    return torch.maximum(torch.minimum(desired, previous_step + max_step_change), previous_step - max_step_change)
