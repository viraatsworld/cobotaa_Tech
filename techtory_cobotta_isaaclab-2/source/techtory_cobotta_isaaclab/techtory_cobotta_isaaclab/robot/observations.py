# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Observation terms for the Cobotta Pro + RG6 in the Techtory cell.

Four families:

* **Arm** -- the six joint positions and velocities, what ``/joint_states``
  carried in the ROS demo.
* **Gripper** -- the driven ``finger_joint``. The five followers carry
  ``multiplier * finger_joint``, so they add no information.
* **Tool** -- the ``cobotta_pro_tool0`` pose in the robot base frame.
* **Wrist force/torque** -- the 6-axis wrench at the J6 flange <-> RG6
  interface, the same PhysX quantity
  ``techtory_cobotta_isaacsim/spawners/ft_sensor.py`` samples. See
  :func:`wrist_wrench` for what it does and does not see.

Plus the hammer pose, so a policy knows where the object is.

:class:`ObservationsCfg` wires all of it into one 41-value policy group.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.envs import mdp
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.configclass import configclass
from isaaclab.utils.math import subtract_frame_transforms

from techtory_cobotta_isaaclab.robot.joints import (
    ARM_JOINTS,
    EE_BODY,
    FT_BODY,
    GRIPPER_LEADER,
)

if TYPE_CHECKING:
    # Annotations only -- see the note in actions.py: importing the runtime
    # Articulation class before SimulationApp starts crashes Kit.
    from isaaclab.assets.articulation import Articulation
    from isaaclab.assets.rigid_object import RigidObject
    from isaaclab.envs import ManagerBasedEnv
    from isaaclab.sensors import JointWrenchSensor

__all__ = [
    "ObservationsCfg",
    "arm_joint_pos",
    "arm_joint_vel",
    "body_pose_b",
    "gripper_joint_pos",
    "gripper_joint_vel",
    "object_pose_b",
    "wrist_wrench",
]

_ROBOT = SceneEntityCfg("robot")


def _asset(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg) -> Articulation:
    return env.scene[asset_cfg.name]


def _body_id(asset: Articulation, asset_cfg: SceneEntityCfg, default: str) -> int:
    """One body index: the manager-resolved one if given, else looked up by name.

    Resolved by name, never hardcoded -- the index depends on link ordering,
    which the importer does not promise to keep.
    """
    ids = asset_cfg.body_ids
    if isinstance(ids, list | tuple) and len(ids) == 1:
        return int(ids[0])
    matched, names = asset.find_bodies(default, preserve_order=True)
    if len(matched) != 1:
        raise ValueError(f"expected exactly one body named {default!r}, found {names}")
    return int(matched[0])


def _root_pose(asset: Articulation) -> tuple[torch.Tensor, torch.Tensor]:
    return asset.data.root_link_pos_w.torch, asset.data.root_link_quat_w.torch


##
# Articulation.
##


def _joint_pos(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg, names: tuple[str, ...]) -> torch.Tensor:
    asset = _asset(env, asset_cfg)
    ids, _ = asset.find_joints(list(names), preserve_order=True)
    return asset.data.joint_pos.torch[:, ids]


def _joint_vel(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg, names: tuple[str, ...]) -> torch.Tensor:
    asset = _asset(env, asset_cfg)
    ids, _ = asset.find_joints(list(names), preserve_order=True)
    return asset.data.joint_vel.torch[:, ids]


def arm_joint_pos(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """The six arm joint positions, radians, ``cobotta_pro_joint_1..6`` order."""
    return _joint_pos(env, asset_cfg, ARM_JOINTS)


def arm_joint_vel(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """The six arm joint velocities, rad/s, ``cobotta_pro_joint_1..6`` order."""
    return _joint_vel(env, asset_cfg, ARM_JOINTS)


def gripper_joint_pos(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """``finger_joint`` angle, radians: -0.628 fully open, +0.628 fully closed."""
    return _joint_pos(env, asset_cfg, (GRIPPER_LEADER,))


def gripper_joint_vel(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """``finger_joint`` rate, rad/s; positive is closing."""
    return _joint_vel(env, asset_cfg, (GRIPPER_LEADER,))


##
# Poses in the robot base frame.
##


def body_pose_b(
    env: ManagerBasedEnv,
    asset_cfg: SceneEntityCfg = _ROBOT,
    body_name: str = EE_BODY,
) -> torch.Tensor:
    """Position ``(x, y, z)`` and orientation ``(x, y, z, w)`` of one robot body,
    expressed in the robot's base frame.

    The base frame, not the world: parallel worlds sit on a grid in one stage,
    so world coordinates would encode which environment a sample came from.
    """
    asset = _asset(env, asset_cfg)
    body = _body_id(asset, asset_cfg, body_name)
    root_pos, root_quat = _root_pose(asset)
    pos, quat = subtract_frame_transforms(
        root_pos,
        root_quat,
        asset.data.body_link_pos_w.torch[:, body],
        asset.data.body_link_quat_w.torch[:, body],
    )
    return torch.cat([pos, quat], dim=-1)


def object_pose_b(
    env: ManagerBasedEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("hammer"),
    robot_cfg: SceneEntityCfg = _ROBOT,
) -> torch.Tensor:
    """Position and ``(x, y, z, w)`` orientation of a rigid object, in the robot
    base frame."""
    obj: RigidObject = env.scene[object_cfg.name]
    root_pos, root_quat = _root_pose(_asset(env, robot_cfg))
    pos, quat = subtract_frame_transforms(
        root_pos, root_quat, obj.data.root_link_pos_w.torch, obj.data.root_link_quat_w.torch
    )
    return torch.cat([pos, quat], dim=-1)


##
# Wrist force/torque.
##


def wrist_wrench(
    env: ManagerBasedEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("wrist_ft"),
    body_name: str = FT_BODY,
    negate: bool = True,
    bias: tuple[float, float, float, float, float, float] | None = None,
) -> torch.Tensor:
    """The 6-axis wrench at the J6 flange <-> RG6 interface.

    Returns ``(fx, fy, fz, tx, ty, tz)`` in N and N.m, one row per environment.

    **What it reads.** Isaac Lab's :class:`~isaaclab.sensors.JointWrenchSensor`
    (scene entity ``wrist_ft``), at the incoming joint of the RG6 ``base_link``
    -- the constraint wrench the solver applies in ``joint_rg6`` to hold
    everything distal to it (the gripper). On PhysX that is the same table
    ``get_measured_joint_forces()`` returns, which is what
    ``techtory_cobotta_isaacsim``'s ``WristFTSensor`` reads; on Newton it is
    ``body_parent_f``. ``joint_rg6`` is a locked revolute rather than a fixed
    joint precisely so Newton reports it (see ``robot/joints.py``).

    **Frame.** The sensor's ``incoming_joint_frame`` convention: the child-side
    joint frame, with the joint anchor as the torque reference -- what a real
    F/T sensor bolted there would measure. Check X/Y against a known push
    before trusting them: the Z magnitude alone will not catch a rotated frame.

    **Sign.** The raw reading is the force the wrist exerts *on the tool*: at
    rest it holds the tool up, so it points away from gravity. ``negate=True``
    (default, matching ``FT_NEGATE = True`` in ``scripts/main.py``) flips it to
    "force the environment applies to the tool" -- at rest that is the tool
    weight, about 9.81 N for the 1.0 kg authored RG6, with near-zero torque in
    the home pose because the gripper's centre of mass sits on the J6 axis.

    **Bias.** ``bias`` is subtracted after the sign flip. Pass the at-rest
    reading to tare out the tool weight, as a real F/T pipeline would. Note
    this is a fixed offset: once the wrist rotates, gravity on the tool moves
    between axes and a fixed tare no longer cancels it.

    **What it does NOT see.** A grasped object. Its weight reaches the gripper
    through pad <-> object *contact*, which PhysX leaves out of the joint
    reaction table -- lift the hammer and this does not move (verified in
    ``techtory_cobotta_isaacsim``, plan-ft-implement.md section 7; IsaacLab
    issue #1092). Contacts against the gripper's own links -- pushing the
    fingers into the shelf -- do show up, because those loads travel through
    the articulation. The reading is also noise-free and bias-free, unlike a
    real sensor.
    """
    sensor: JointWrenchSensor = env.scene.sensors[sensor_cfg.name]
    ids, names = sensor.find_bodies(body_name, preserve_order=True)
    if len(ids) != 1:
        raise ValueError(f"expected one reported body named {body_name!r}, found {names}")
    body = ids[0]
    wrench = torch.cat(
        [sensor.data.force.torch[:, body, :], sensor.data.torque.torch[:, body, :]], dim=-1
    )
    if negate:
        wrench = -wrench
    if bias is not None:
        wrench = wrench - torch.tensor(bias, device=wrench.device, dtype=wrench.dtype)
    return wrench


##
# Ready-made policy group.
##


@configclass
class ObservationsCfg:
    """Observation specifications for the Cobotta MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """The 41 values a policy sees, concatenated in declaration order.

        ``6 + 6 + 1 + 1 + 7 + 6 + 7 + 7 = 41``.
        """

        arm_pos = ObsTerm(func=arm_joint_pos)
        arm_vel = ObsTerm(func=arm_joint_vel)
        gripper_pos = ObsTerm(func=gripper_joint_pos)
        gripper_vel = ObsTerm(func=gripper_joint_vel)
        ee_pose = ObsTerm(
            func=body_pose_b,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=[EE_BODY]), "body_name": EE_BODY},
        )
        wrist_wrench = ObsTerm(
            func=wrist_wrench,
            params={"sensor_cfg": SceneEntityCfg("wrist_ft"), "body_name": FT_BODY},
        )
        hammer_pose = ObsTerm(func=object_pose_b)
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
