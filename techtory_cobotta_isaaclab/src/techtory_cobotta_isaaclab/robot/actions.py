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

from isaaclab.envs import mdp
from isaaclab.utils.configclass import configclass

from techtory_cobotta_isaaclab.robot.robot_cfg import ARM_JOINTS, GRIPPER_CLOSED, GRIPPER_JOINT, GRIPPER_OPEN

__all__ = ["ActionsCfg"]


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
