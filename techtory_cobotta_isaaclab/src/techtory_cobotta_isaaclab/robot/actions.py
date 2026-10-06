# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Action space for the Cobotta + RG6: six arm joints and one gripper command.

Both terms are Isaac Lab's stock joint-position action. The gripper needs no
custom term: only ``finger_joint`` is driven -- PhysX moves the five mimic joints
-- so a one-joint position action is already one-dimensional.

:class:`ActionsCfg` assembles the 7-D space. Declaration order fixes the layout.
"""

from __future__ import annotations

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs import mdp
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils.configclass import configclass

from techtory_cobotta_isaaclab.robot.robot_cfg import (
    ARM_JOINTS,
    GRIPPER_CLOSED,
    GRIPPER_JOINT,
    GRIPPER_OPEN,
    TCP_BODY,
    TCP_OFFSET,
)

__all__ = ["ActionsCfg", "TopDownTcpTargetActionCfg"]


@configclass
class ActionsCfg:
    """The 7-D action space: ``[q1..q6 offsets, grip]``.

    Reordering these fields renames every index in the README's command table.
    """

    arm: mdp.JointPositionActionCfg = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=list(ARM_JOINTS),
        preserve_order=True,
        # Offsets from the home pose, +-0.5 rad at unit action.
        scale=0.5,
        use_default_offset=True,
    )

    # Absolute jaw command: +1 fully open, -1 fully closed, 0 half open (94 mm),
    # the same sense as roxfr3_isaaclab's gripper. finger_joint closes towards +,
    # hence the negative scale. The clip holds the target inside the joint's
    # travel, so a policy cannot load the drive against the stop.
    gripper: mdp.JointPositionActionCfg = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[GRIPPER_JOINT],
        scale=GRIPPER_OPEN,
        offset=0.0,
        use_default_offset=False,
        clip={GRIPPER_JOINT: (GRIPPER_OPEN, GRIPPER_CLOSED)},
    )


@configclass
class TopDownTcpTargetActionCfg(DifferentialInverseKinematicsActionCfg):
    """4-D arm action ``[dx, dy, dz, dyaw]``: steps of a top-down TCP target, in the robot base frame.

    See :class:`~techtory_cobotta_isaaclab.robot.tcp_action.TopDownTcpTargetAction`.
    The defaults keep a full-scale action inside the arm's MoveIt speed limits
    (0.33-0.60 rad/s) at a 25 Hz policy; the term also rate-limits the joint
    targets, so the limits hold whatever the IK asks for.
    """

    # Named lazily: the term's module imports pxr, which must wait for Kit.
    class_type: type | str = "techtory_cobotta_isaaclab.robot.tcp_action:TopDownTcpTargetAction"

    asset_name: str = "robot"
    joint_names: list[str] = list(ARM_JOINTS)
    body_name: str = TCP_BODY
    body_offset: DifferentialInverseKinematicsActionCfg.OffsetCfg = DifferentialInverseKinematicsActionCfg.OffsetCfg(
        pos=TCP_OFFSET
    )
    # Absolute pose mode: the term hands the controller its integrated target.
    controller: DifferentialIKControllerCfg = DifferentialIKControllerCfg(
        command_type="pose", use_relative_mode=False, ik_method="dls"
    )

    pos_step: float = 0.005
    """Largest TCP move per policy step [m]: 0.125 m/s at 25 Hz."""

    yaw_step: float = 0.012
    """Largest yaw move per policy step [rad]: 0.3 rad/s at 25 Hz, under the slowest joint's 0.33 rad/s."""

    tcp_acceleration: float = 0.5
    """Largest acceleration of the commanded TCP target [m/s^2], per axis.

    The action sets the target's velocity; it ramps to it at this rate, and the
    target glides there physics step by physics step instead of jumping once per
    policy step. Full speed (0.125 m/s) in 0.25 s. About what MoveIt's 1 rad/s^2
    joint limit allows at the Cobotta's ~0.6 m working reach.
    """

    yaw_acceleration: float = 1.0
    """Largest acceleration of the commanded TCP yaw [rad/s^2]: MoveIt's joint limit, as yaw is mostly J6."""

    max_lead_pos: float = 0.03
    """How far the commanded target may lead the actual TCP [m], per axis."""

    max_lead_yaw: float = 0.1
    """How far the commanded yaw may lead the actual TCP yaw [rad]."""

    joint_acceleration_limit: float | tuple[float, ...] | None = 1.0
    """Largest acceleration of each arm joint's target [rad/s^2], one value or one per joint; None: off.

    MoveIt's ``max_acceleration`` for every Cobotta joint
    (``techtory_cobotta_moveit/config/joint_limits.yaml``). The joint targets
    brake in time to stop on the IK solution rather than overshoot it. It makes
    the arm lag its commanded TCP target a little; the policy sees that lag
    (``tcp_target_lead``) and a controller must brake for it -- one that does not
    overshoots (measured: a non-braking scripted probe fell to 1/32 successes; a
    braking one makes 30/32, with commanded joint accelerations within the limit).
    The joint targets always keep the velocity limits and the soft joint limits.
    """

    workspace_min: tuple[float, float, float] | None = None
    """Lower corner of the box the commanded TCP is kept in [m], robot base frame. None: no bound."""

    workspace_max: tuple[float, float, float] | None = None
    """Upper corner of that box [m], robot base frame."""
