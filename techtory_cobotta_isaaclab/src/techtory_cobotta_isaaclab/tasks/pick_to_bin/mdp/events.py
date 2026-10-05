# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Reset events for pick-to-bin."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_apply, quat_from_euler_xyz, quat_mul

from techtory_cobotta_isaaclab.scene import layout

from . import state

if TYPE_CHECKING:
    from isaaclab.assets import RigidObject
    from isaaclab.envs import ManagerBasedEnv

__all__ = ["reset_object_in_spawn_zone"]

_OBJECT = SceneEntityCfg("object")


def reset_object_in_spawn_zone(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg = _OBJECT,
    yaw_range: tuple[float, float] = (-math.pi, math.pi),
) -> None:
    """Lay the object at rest on the table, inside :data:`layout.SPAWN_ZONE`, at a random yaw.

    Its box centre is uniform over the zone shrunk by the object's footprint
    radius, so the whole object lies inside the zone at any yaw. The yaw is
    about the world's vertical and about the box centre -- not Isaac Lab's
    ``reset_root_state_uniform``, which applies its yaw in the body's own frame:
    for the hammer, whose body frame is turned so the part lies flat, that would
    stand it on its side.
    """
    obj = state.spec(env)
    part: RigidObject = env.scene[asset_cfg.name]
    n = len(env_ids)
    device = env.device

    zone = layout.SPAWN_ZONE.shrunk(obj.footprint_radius)
    zone_center = torch.tensor(zone.center, device=device)
    zone_half = torch.tensor(zone.half, device=device)
    xy = zone_center + (2.0 * torch.rand(n, 2, device=device) - 1.0) * zone_half
    yaw = yaw_range[0] + torch.rand(n, device=device) * (yaw_range[1] - yaw_range[0])

    resting = obj.resting_body
    rest_quat = torch.tensor(resting.rot, device=device).expand(n, 4)
    zeros = torch.zeros(n, device=device)
    quat = quat_mul(quat_from_euler_xyz(zeros, zeros, yaw), rest_quat)

    box_center_b = torch.tensor(obj.box_center, device=device).expand(n, 3)
    rest_center_z = resting.pos[2] + quat_apply(rest_quat, box_center_b)[:, 2]
    center = torch.cat([xy, rest_center_z.unsqueeze(-1)], dim=-1)
    pos = center - quat_apply(quat, box_center_b) + env.scene.env_origins[env_ids]

    part.write_root_pose_to_sim_index(root_pose=torch.cat([pos, quat], dim=-1), env_ids=env_ids)
    part.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=device), env_ids=env_ids)
