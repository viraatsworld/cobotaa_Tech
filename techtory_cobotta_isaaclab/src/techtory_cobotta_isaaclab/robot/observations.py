# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Observation terms for the Cobotta + RG6 in the Techtory cell.

Four families:

* **Joints** -- the six arm joints and ``finger_joint``, through Isaac Lab's
  stock terms. The five mimic joints follow ``finger_joint`` and add nothing.
* **TCP** -- the tool centre point (MoveIt's ``cobotta_pro_tool0``) in the robot
  base frame.
* **Wrist wrench** -- the 6-axis force/torque between the arm flange and the
  gripper, see :func:`wrist_wrench`.
* **Objects** -- the hammer and soda-can poses in the robot base frame.

Poses are given in the robot base frame (``cobotta_pro_base_link``), not the
world: parallel environments sit on a grid in one stage, so world coordinates
would tell a policy which environment it is in.

:class:`ObservationsCfg` wires all of it into one 48-value ``policy`` group.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs import mdp
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.configclass import configclass
from isaaclab.utils.math import (
    combine_frame_transforms,
    quat_apply,
    quat_apply_inverse,
    quat_unique,
    subtract_frame_transforms,
)

from techtory_cobotta_isaaclab.robot.robot_cfg import ARM_JOINTS, FT_BODY, GRIPPER_JOINT, TCP_BODY, TCP_OFFSET

if TYPE_CHECKING:
    # Annotations only: importing the runtime asset classes here would pull in
    # pxr before Kit starts (see roxfr3_isaaclab's observations.py for the crash).
    from isaaclab.assets import Articulation, RigidObject
    from isaaclab.envs import ManagerBasedEnv

__all__ = [
    "FT_ROBOT_ENTITY",
    "FT_SENSOR_ENTITY",
    "TCP_ENTITY",
    "ObservationsCfg",
    "no_wrench",
    "object_pose_b",
    "payload_wrench",
    "tcp_pose_b",
    "wrist_wrench",
]

_ROBOT = SceneEntityCfg("robot")

# Scene selections the terms below need. They must reach a term through its
# ``params`` -- the observation manager resolves names to indices only there, not
# in a function's default arguments.
TCP_ENTITY = SceneEntityCfg("robot", body_names=[TCP_BODY])
"""The robot, with the TCP's parent body selected."""
FT_SENSOR_ENTITY = SceneEntityCfg("wrist_ft", body_names=[FT_BODY])
"""The joint-wrench sensor, with the F/T body selected."""
FT_ROBOT_ENTITY = SceneEntityCfg("robot", joint_names=[GRIPPER_JOINT], body_names=[FT_BODY])
"""The robot, with ``finger_joint`` and the F/T body selected."""

##
# Poses.
##


def tcp_pose_b(
    env: ManagerBasedEnv, asset_cfg: SceneEntityCfg, offset: tuple[float, float, float] = TCP_OFFSET
) -> torch.Tensor:
    """TCP position [m] and orientation ``(x, y, z, w)`` in the robot base frame.

    Returns shape ``(num_envs, 7)``. The TCP is ``offset`` in the frame of the
    single body ``asset_cfg`` selects (:data:`TCP_ENTITY`); the quaternion is
    returned with ``w >= 0``.
    """
    robot: Articulation = env.scene[asset_cfg.name]
    body_pos = robot.data.body_link_pos_w.torch[:, asset_cfg.body_ids[0]]
    body_quat = robot.data.body_link_quat_w.torch[:, asset_cfg.body_ids[0]]
    offset_b = torch.tensor(offset, device=env.device).expand(env.num_envs, 3)
    tcp_pos_w, tcp_quat_w = combine_frame_transforms(body_pos, body_quat, offset_b)
    pos_b, quat_b = subtract_frame_transforms(
        robot.data.root_link_pos_w.torch, robot.data.root_link_quat_w.torch, tcp_pos_w, tcp_quat_w
    )
    return torch.cat([pos_b, quat_unique(quat_b)], dim=-1)


def object_pose_b(env: ManagerBasedEnv, object_cfg: SceneEntityCfg, robot_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """An object's position [m] and orientation ``(x, y, z, w)`` in the robot base frame.

    Returns shape ``(num_envs, 7)``; the quaternion is returned with ``w >= 0``.
    """
    robot: Articulation = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[object_cfg.name]
    pos_b, quat_b = subtract_frame_transforms(
        robot.data.root_link_pos_w.torch,
        robot.data.root_link_quat_w.torch,
        obj.data.root_link_pos_w.torch,
        obj.data.root_link_quat_w.torch,
    )
    return torch.cat([pos_b, quat_unique(quat_b)], dim=-1)


##
# Wrist force/torque.
##

# The wrist wrench is reported as the LOAD the gripper puts on the arm -- what a
# physical F/T sensor between flange and tool reads: at rest, the tool's weight,
# pointing along gravity. PhysX's incoming joint wrench is the opposite, the
# arm's support of the gripper (measured with scripts/check_ft_payload.py: the
# hanging gripper reads -9.81 N along its own +z, which points down).
_RAW_TO_LOAD = -1.0


def wrist_wrench(env: ManagerBasedEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """6-axis force [N] / torque [N·m] between the arm flange and the gripper.

    Returns shape ``(num_envs, 6)``: ``(Fx, Fy, Fz, Tx, Ty, Tz)`` in the RG6
    ``base_link`` frame (ROS ``onrobot_rg6_base_link``; +z points out of the
    flange towards the fingers), torque about its origin, which sits on the
    flange. It is the wrench of the fixed ``gripper_joint``, read from the
    ``wrist_ft`` :class:`~isaaclab.sensors.JointWrenchSensor`, and reported as
    the load the gripper puts on the arm -- what a wrist F/T sensor between arm
    and gripper reads. Hanging still, it is the gripper's weight (1.0 kg as
    authored) pointing along gravity; a held object adds its own.

    Simulator ground truth: no noise, bias or bandwidth limit, and not tared.

    Args:
        env: The environment.
        sensor_cfg: The joint-wrench sensor with the F/T body selected (:data:`FT_SENSOR_ENTITY`).
    """
    return _RAW_TO_LOAD * mdp.body_incoming_wrench(env, sensor_cfg)


def no_wrench(env: ManagerBasedEnv) -> torch.Tensor:
    """Zeros in place of :func:`wrist_wrench`, on backends without the wrist F/T.

    Returns shape ``(num_envs, 6)``. Newton's joint-wrench sensor skips links
    attached by a fixed joint, and the F/T sits on one (``gripper_joint``).
    Keeping the six values holds the 48-D layout, so observation indices --
    scripts/play.py, a trained policy -- are the same on every backend.
    """
    return torch.zeros(env.num_envs, 6, device=env.device)


def payload_wrench(
    env: ManagerBasedEnv,
    robot_cfg: SceneEntityCfg,
    objects: Sequence[str] = ("hammer", "soda_can"),
    jaw_radius: float = 0.10,
    stall_speed: float = 0.1,
    blocked_margin: float = 0.05,
) -> torch.Tensor:
    """Estimated share of the wrist wrench [N, N·m] due to held objects, in the F/T frame.

    Returns shape ``(num_envs, 6)``, force then torque, in the same frame and sign
    as :func:`wrist_wrench`. It is **already part of** :func:`wrist_wrench` --
    PhysX's joint wrench includes a held object (scripts/check_ft_payload.py
    measures a held can's full weight to within 2 %) -- so do not add the two. Use it as a
    separate grip-load channel, like the Isaac Sim demo's ``/grip_contact``, e.g.
    to tell a payload from the gripper's own weight without taring.

    A vectorised port of the demo's ``GripContactSensor``: Newton's second law on
    each object. An object held by the gripper feels ``F_grip + m*g = m*a``, so it
    pushes back on the gripper with ``m*(g - a)`` -- its weight at rest, plus its
    inertia when accelerated. The squeeze is internal to the grasp and cancels
    out. Torque is taken about the F/T origin through the object's centre of mass.

    An object counts as held when its centre of mass is within ``jaw_radius`` of
    the TCP and ``finger_joint`` has stalled -- slower than ``stall_speed`` while
    more than ``blocked_margin`` short of its target.

    Limitations, as in the demo: the object's rotational inertia (``I*alpha``) is
    ignored, and a surface sharing the load -- an object pressed onto the shelf
    while gripped -- is not detected; the full ``m*(g - a)`` is still attributed
    to the gripper.

    Args:
        env: The environment.
        robot_cfg: The robot, with ``finger_joint`` and the F/T body selected (:data:`FT_ROBOT_ENTITY`).
        objects: Scene names of the rigid objects that can be picked up.
        jaw_radius: Max distance from the TCP to an object's centre of mass [m].
        stall_speed: ``finger_joint`` speed below which the jaw counts as stopped [rad/s].
        blocked_margin: How far short of its target a stopped jaw must be [rad].
    """
    robot: Articulation = env.scene[robot_cfg.name]
    finger = robot_cfg.joint_ids[0]
    ft_pos = robot.data.body_link_pos_w.torch[:, robot_cfg.body_ids[0]]
    ft_quat = robot.data.body_link_quat_w.torch[:, robot_cfg.body_ids[0]]
    tcp_pos = ft_pos + quat_apply(ft_quat, torch.tensor(TCP_OFFSET, device=env.device).expand(env.num_envs, 3))

    shortfall = robot.data.joint_pos_target.torch[:, finger] - robot.data.joint_pos.torch[:, finger]
    stalled = (shortfall > blocked_margin) & (robot.data.joint_vel.torch[:, finger].abs() < stall_speed)

    gravity = torch.tensor(env.sim.cfg.gravity, device=env.device)
    force_w = torch.zeros(env.num_envs, 3, device=env.device)
    torque_w = torch.zeros_like(force_w)
    for name in objects:
        obj: RigidObject = env.scene[name]
        com = obj.data.body_com_pos_w.torch[:, 0]
        held = stalled & (torch.linalg.norm(com - tcp_pos, dim=-1) < jaw_radius)
        mass = obj.data.body_mass.torch[:, 0:1]
        load = mass * (gravity - obj.data.body_com_acc_w.torch[:, 0, :3]) * held.unsqueeze(-1)
        force_w += load
        torque_w += torch.linalg.cross(com - ft_pos, load, dim=-1)

    return torch.cat([quat_apply_inverse(ft_quat, force_w), quat_apply_inverse(ft_quat, torque_w)], dim=-1)


##
# Ready-made policy group.
##

_ARM = SceneEntityCfg("robot", joint_names=list(ARM_JOINTS), preserve_order=True)
_FINGER = SceneEntityCfg("robot", joint_names=[GRIPPER_JOINT])


@configclass
class ObservationsCfg:
    """Observation specifications for the Cobotta + RG6 in the Techtory cell."""

    @configclass
    class PolicyCfg(ObsGroup):
        """The 48 values a policy sees, concatenated in declaration order.

        ``6 + 6 + 1 + 1 + 7 + 6 + 7 + 7 + 7 = 48``.
        """

        arm_pos = ObsTerm(func=mdp.joint_pos, params={"asset_cfg": _ARM})
        arm_vel = ObsTerm(func=mdp.joint_vel, params={"asset_cfg": _ARM})
        gripper_pos = ObsTerm(func=mdp.joint_pos, params={"asset_cfg": _FINGER})
        gripper_vel = ObsTerm(func=mdp.joint_vel, params={"asset_cfg": _FINGER})
        tcp_pose = ObsTerm(func=tcp_pose_b, params={"asset_cfg": TCP_ENTITY})
        wrist_wrench = ObsTerm(func=wrist_wrench, params={"sensor_cfg": FT_SENSOR_ENTITY})
        hammer_pose = ObsTerm(func=object_pose_b, params={"object_cfg": SceneEntityCfg("hammer")})
        soda_can_pose = ObsTerm(func=object_pose_b, params={"object_cfg": SceneEntityCfg("soda_can")})
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
