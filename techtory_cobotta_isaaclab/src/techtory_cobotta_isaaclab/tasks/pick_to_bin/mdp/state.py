# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""What state a pick-to-bin episode is in: one definition, shared by every term.

The gripper rule, the observations, the rewards and the terminations all read
the grasp distance, the yaw error, "grasped", "picked" and "in the bin" from
here, so they can never disagree about them. What the object is comes from the
environment config's ``grasp_object`` name (:data:`grasp_objects.GRASP_OBJECTS`);
the scene entity is always called ``object``.

Positions are in the **env frame** -- the cell frame, the world minus the env
origin -- the frame every number in :mod:`techtory_cobotta_isaaclab.scene.layout`
is given in. Plain torch; the asset classes are type-checking imports only.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch

from isaaclab.utils.math import combine_frame_transforms, quat_apply

from techtory_cobotta_isaaclab.robot.robot_cfg import GRIPPER_CLOSED, GRIPPER_JOINT, TCP_BODY, TCP_OFFSET
from techtory_cobotta_isaaclab.robot.top_down import wrap_half_turn, x_axis_yaw
from techtory_cobotta_isaaclab.scene import layout
from techtory_cobotta_isaaclab.scene.grasp_objects import GRASP_OBJECTS, GraspableObject
from techtory_cobotta_isaaclab.scene.layout import Box2D

if TYPE_CHECKING:
    from isaaclab.assets import Articulation, RigidObject
    from isaaclab.envs import ManagerBasedEnv

    from .actions import ProximityGripperAction

GRASP_RADIUS = 0.008
"""The rule closes the jaw when the TCP is this close to the grasp target, seen from above [m]...

Closing further off centre, one pad reaches a light object first and shoves it
round before the other arrives, and the grip ends up skewed.
"""

GRASP_HEIGHT_TOLERANCE = 0.006
"""...within this of its height [m] -- lower, and the closing pads land on the table, not the object...."""

GRASP_YAW_TOLERANCE = math.radians(15.0)
"""...and its yaw is within this of the grasp axis, modulo a half turn [rad]."""

JAW_RADIUS = 0.04
"""A grasped object's grasp point is within this of the TCP [m]."""

PICK_LIFT = 0.025
"""A grasped object counts as picked once it is this far above where it rested [m]."""

REST_SPEED = 0.05
"""An object slower than this [m/s] is at rest."""

NEAR_BIN_MARGIN = 0.10
"""The bin's vicinity: the object's centre within this of the interior, seen from above [m]..."""

NEAR_BIN_HEIGHT = 0.30
"""...and its lowest corner less than this above the rim [m]."""

_CORNER_SIGNS = torch.tensor(
    [[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)], dtype=torch.float32
)

_ids: dict[tuple[int, str], int] = {}
_consts: dict[tuple[str, torch.device], torch.Tensor] = {}


def _const(name: str, value, device) -> torch.Tensor:
    key = (name, torch.device(device))
    if key not in _consts:
        _consts[key] = torch.as_tensor(value, dtype=torch.float32, device=device)
    return _consts[key]


def _body_id(robot: Articulation, name: str) -> int:
    key = (id(robot), "body:" + name)
    if key not in _ids:
        _ids[key] = robot.find_bodies(name)[0][0]
    return _ids[key]


def _joint_id(robot: Articulation, name: str) -> int:
    key = (id(robot), "joint:" + name)
    if key not in _ids:
        _ids[key] = robot.find_joints(name)[0][0]
    return _ids[key]


def spec(env: ManagerBasedEnv) -> GraspableObject:
    """The object this environment picks."""
    return GRASP_OBJECTS[env.cfg.grasp_object]


def robot(env: ManagerBasedEnv) -> Articulation:
    return env.scene["robot"]


def target_object(env: ManagerBasedEnv) -> RigidObject:
    return env.scene["object"]


def gripper_rule(env: ManagerBasedEnv) -> ProximityGripperAction:
    """The gripper rule's action term (its state machine)."""
    return env.action_manager.get_term("gripper")


##
# Poses.
##


def tcp_pose(env: ManagerBasedEnv) -> tuple[torch.Tensor, torch.Tensor]:
    """TCP position ``(N, 3)`` and orientation ``(N, 4)`` in the env frame."""
    arm = robot(env)
    i = _body_id(arm, TCP_BODY)
    offset = _const("tcp_offset", TCP_OFFSET, env.device).expand(env.num_envs, 3)
    pos, quat = combine_frame_transforms(
        arm.data.body_link_pos_w.torch[:, i], arm.data.body_link_quat_w.torch[:, i], offset
    )
    return pos - env.scene.env_origins, quat


def object_pose(env: ManagerBasedEnv) -> tuple[torch.Tensor, torch.Tensor]:
    """The object body's position ``(N, 3)`` and orientation ``(N, 4)`` in the env frame."""
    obj = target_object(env)
    return obj.data.root_link_pos_w.torch - env.scene.env_origins, obj.data.root_link_quat_w.torch


def _object_point(env: ManagerBasedEnv, field: str) -> torch.Tensor:
    obj = spec(env)
    pos, quat = object_pose(env)
    point = _const(f"{obj.name}:{field}", getattr(obj, field), env.device).expand(env.num_envs, 3)
    return pos + quat_apply(quat, point)


def grasp_point(env: ManagerBasedEnv) -> torch.Tensor:
    """Where the pads close on the object, in the env frame ``(N, 3)``."""
    return _object_point(env, "grasp_point")


def grasp_target(env: ManagerBasedEnv) -> torch.Tensor:
    """Where the TCP goes to grasp: the object's grasp height straight above its grasp point ``(N, 3)``."""
    target = grasp_point(env).clone()
    target[:, 2] += spec(env).grasp_height
    return target


def object_center(env: ManagerBasedEnv) -> torch.Tensor:
    """Centre of the object's bounding box in the env frame ``(N, 3)``."""
    return _object_point(env, "box_center")


def object_corners(env: ManagerBasedEnv) -> torch.Tensor:
    """The 8 corners of the object's oriented bounding box in the env frame ``(N, 8, 3)``."""
    obj = spec(env)
    pos, quat = object_pose(env)
    center = _const(f"{obj.name}:box_center", obj.box_center, env.device)
    half = _const(f"{obj.name}:box_half", obj.box_half, env.device)
    corners_b = center + _CORNER_SIGNS.to(env.device) * half
    corners = quat_apply(quat.unsqueeze(1).expand(-1, 8, 4), corners_b.expand(env.num_envs, 8, 3))
    return pos.unsqueeze(1) + corners


def object_speed(env: ManagerBasedEnv) -> torch.Tensor:
    return torch.linalg.norm(target_object(env).data.root_lin_vel_w.torch, dim=-1)


##
# Grasp geometry.
##


def tcp_yaw(env: ManagerBasedEnv) -> torch.Tensor:
    """Heading of the TCP's x axis in the env frame [rad]. The jaw closes across it."""
    return x_axis_yaw(tcp_pose(env)[1])


def object_yaw(env: ManagerBasedEnv) -> torch.Tensor:
    """Heading of the object's grasp axis in the env frame [rad]; 0 for an object that is round from above."""
    obj = spec(env)
    if obj.grasp_axis is None:
        return torch.zeros(env.num_envs, device=env.device)
    axis = _const(f"{obj.name}:grasp_axis", obj.grasp_axis, env.device).expand(env.num_envs, 3)
    heading = quat_apply(object_pose(env)[1], axis)
    return torch.atan2(heading[:, 1], heading[:, 0])


def grasp_yaw_error(env: ManagerBasedEnv) -> torch.Tensor:
    """TCP yaw minus the grasp axis's yaw [rad], wrapped into ``[-pi/2, pi/2)``; 0 for round objects.

    Modulo a half turn, because a parallel jaw looks the same turned by 180 deg:
    either way round is a correct grasp.
    """
    if spec(env).grasp_axis is None:
        return torch.zeros(env.num_envs, device=env.device)
    return wrap_half_turn(tcp_yaw(env) - object_yaw(env))


def grasp_distance(env: ManagerBasedEnv) -> torch.Tensor:
    """TCP to grasp target [m]."""
    return torch.linalg.norm(tcp_pose(env)[0] - grasp_target(env), dim=-1)


def at_grasp_pose(env: ManagerBasedEnv) -> torch.Tensor:
    """The rule's close condition: TCP at the grasp target with the jaw across the grasp axis."""
    offset = tcp_pose(env)[0] - grasp_target(env)
    horizontal = torch.linalg.norm(offset[:, :2], dim=-1) < GRASP_RADIUS
    vertical = offset[:, 2].abs() < GRASP_HEIGHT_TOLERANCE
    return horizontal & vertical & (grasp_yaw_error(env).abs() < GRASP_YAW_TOLERANCE)


##
# The jaw.
##


def _finger(env: ManagerBasedEnv) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    arm = robot(env)
    j = _joint_id(arm, GRIPPER_JOINT)
    return arm.data.joint_pos.torch[:, j], arm.data.joint_vel.torch[:, j], arm.data.joint_pos_target.torch[:, j]


def jaw_stalled(env: ManagerBasedEnv, blocked_margin: float = 0.05, stall_speed: float = 0.1) -> torch.Tensor:
    """``finger_joint`` stopped short of its target: something is between the pads.

    The same stall test as :func:`techtory_cobotta_isaaclab.robot.payload_wrench`.
    """
    pos, vel, target = _finger(env)
    return (target - pos > blocked_margin) & (vel.abs() < stall_speed)


def jaw_closed_on_nothing(env: ManagerBasedEnv, margin: float = 0.04) -> torch.Tensor:
    """The jaw has closed almost completely: whatever it closed on, it was not the object."""
    return _finger(env)[0] > GRIPPER_CLOSED - margin


def jaw_position(env: ManagerBasedEnv) -> torch.Tensor:
    return _finger(env)[0]


##
# Episode milestones.
##


def object_grasped(env: ManagerBasedEnv) -> torch.Tensor:
    """Rule closed, jaw stalled at the object's width, and the object between the pads.

    Closing on air, on the table, or squeezing the object out of the jaw is not a grasp.
    """
    near = torch.linalg.norm(tcp_pose(env)[0] - grasp_point(env), dim=-1) < JAW_RADIUS
    low, high = spec(env).jaw_on_object
    jaw = jaw_position(env)
    return gripper_rule(env).closed & jaw_stalled(env) & near & (jaw > low) & (jaw < high)


def object_lift(env: ManagerBasedEnv) -> torch.Tensor:
    """How far the object's lowest corner is above where it rested on the table [m]."""
    return object_corners(env)[..., 2].amin(dim=1) - (layout.TABLE_TOP_Z + spec(env).rest_clearance)


def object_picked(env: ManagerBasedEnv) -> torch.Tensor:
    """Grasped **and** lifted :data:`PICK_LIFT` off the table: a squeeze on the table is not a pick."""
    return object_grasped(env) & (object_lift(env) > PICK_LIFT)


def _inside(points_xy: torch.Tensor, box: Box2D, env: ManagerBasedEnv) -> torch.Tensor:
    center = _const(f"box_c:{box}", box.center, env.device)
    half = _const(f"box_h:{box}", box.half, env.device)
    return ((points_xy - center).abs() <= half).all(dim=-1)


def object_over_bin(env: ManagerBasedEnv) -> torch.Tensor:
    """The rule's release condition: every corner over :data:`layout.BIN_FOOTPRINT`, within the release height."""
    corners = object_corners(env)
    lowest = corners[..., 2].amin(dim=1)
    over = _inside(corners[..., :2], layout.BIN_FOOTPRINT, env).all(dim=1)
    return over & (lowest >= layout.BIN_RIM_Z) & (lowest <= layout.BIN_RIM_Z + layout.BIN_RELEASE_HEIGHT)


def object_near_bin(env: ManagerBasedEnv) -> torch.Tensor:
    """The object is in the bin's vicinity: approaching it, before the rule can release."""
    vicinity = Box2D(layout.BIN_INTERIOR.center, tuple(h + NEAR_BIN_MARGIN for h in layout.BIN_INTERIOR.half))
    lowest = object_corners(env)[..., 2].amin(dim=1)
    return _inside(object_center(env)[:, :2], vicinity, env) & (lowest < layout.BIN_RIM_Z + NEAR_BIN_HEIGHT)


def object_in_bin(env: ManagerBasedEnv, tolerance: float = 0.005) -> torch.Tensor:
    """Every corner inside the pallet's walls, the lowest one below the rim."""
    corners = object_corners(env)
    interior = Box2D(layout.BIN_INTERIOR.center, tuple(h + tolerance for h in layout.BIN_INTERIOR.half))
    inside = _inside(corners[..., :2], interior, env).all(dim=1)
    return inside & (corners[..., 2].amin(dim=1) < layout.BIN_RIM_Z)


def bin_target_distance(env: ManagerBasedEnv) -> torch.Tensor:
    """Object centre to :data:`layout.BIN_TARGET` [m]."""
    return torch.linalg.norm(object_center(env) - _const("bin_target", layout.BIN_TARGET, env.device), dim=-1)
