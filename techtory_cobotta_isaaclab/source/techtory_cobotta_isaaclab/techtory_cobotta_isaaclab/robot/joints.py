# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Joint names, frames, the home pose and the gripper coupling.

Plain Python on purpose -- no Isaac, no torch -- so the asset tests and the
gripper-mapping tests can run in an interpreter that has neither. Everything
Isaac-facing (``robot_cfg``, ``actions``, ``observations``) imports from here, so
a joint name is written down exactly once.
"""

from __future__ import annotations

import math

__all__ = [
    "ARM_JOINTS",
    "EE_BODY",
    "FT_BODY",
    "FT_JOINT",
    "WRIST_LOCK",
    "GRIPPER_CLOSED",
    "GRIPPER_JOINTS",
    "GRIPPER_LEADER",
    "GRIPPER_MIMIC",
    "GRIPPER_OPEN",
    "HOME_POSE",
    "MOUNT_POS",
    "MOUNT_ROT",
    "ROOT_BODY",
    "gripper_command_to_leader",
    "gripper_targets",
]

##
# Joints.
##

ARM_JOINTS: tuple[str, ...] = tuple(f"cobotta_pro_joint_{i}" for i in range(1, 7))

# The RG6 is a four-bar linkage modelled as a tree: one driven joint and five
# followers. The URDF couples them with <mimic>; the asset builder strips that
# (see scripts/build_urdf_asset.py) and the coupling lives here instead, so the
# gripper action writes `multiplier * q` to every joint. Values are copied from
# the <mimic> elements, and tests/test_urdf_asset.py checks they still match.
GRIPPER_LEADER = "finger_joint"
GRIPPER_MIMIC: dict[str, float] = {
    "finger_joint": 1.0,
    "left_inner_knuckle_joint": -1.0,
    "left_inner_finger_joint": 1.0,
    "right_outer_knuckle_joint": -1.0,
    "right_inner_knuckle_joint": -1.0,
    "right_inner_finger_joint": 1.0,
}
GRIPPER_JOINTS: tuple[str, ...] = tuple(GRIPPER_MIMIC)

# finger_joint travel, radians. Same convention as the MuJoCo port and the RG6
# ParallelGripperCommand: negative opens, positive closes.
GRIPPER_OPEN = -0.628319
GRIPPER_CLOSED = 0.628319

##
# Frames.
##

ROOT_BODY = "cobotta_pro_base_link"

# The RG6 base, bolted to the J6 flange through `joint_rg6`. Its incoming-joint
# wrench is the flange <-> gripper wrench -- where a real wrist F/T sensor
# sits. Same quantity techtory_cobotta_isaacsim/spawners/ft_sensor.py reads
# from PhysX.
FT_BODY = "base_link"

# `joint_rg6` is fixed in the description. The asset builder turns it into a
# revolute joint locked to +/-WRIST_LOCK rad, because Isaac Lab's
# JointWrenchSensor skips fixed joints on Newton. It is held at 0 by its
# limits and a centring drive, so it behaves as rigid; it is not an action.
FT_JOINT = "joint_rg6"
WRIST_LOCK = 1.0e-3

# Virtual tool frame 0.25 m out from the RG6 base, between the fingertips.
EE_BODY = "cobotta_pro_tool0"

##
# Placement and start pose.
##

# The workcell mount chain, from techtory_cobotta_workcell.urdf.xacro:
#   cell_link -> robot_base_plate_link   xyz (-0.275, -0.24, 0.94)
#   robot_base_plate_link -> cobotta_pro_base_link   xyz (0, 0, 0.02) rpy (0, 0, pi/2)
MOUNT_POS: tuple[float, float, float] = (-0.275, -0.24, 0.96)
# (x, y, z, w) -- Isaac Lab 3.x order, not 2.x's (w, x, y, z). Yaw +90 degrees.
MOUNT_ROT: tuple[float, float, float, float] = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))

# techtory_cobotta_isaacsim/scripts/main.py HOME_JOINT_POSITIONS, which the
# MuJoCo port also uses. The all-zero pose points the arm straight up and puts
# the wrist through the cell roof, so a non-zero start is mandatory.
HOME_POSE: dict[str, float] = {
    "cobotta_pro_joint_1": 0.0,
    "cobotta_pro_joint_2": 0.349066,  # 20 deg
    "cobotta_pro_joint_3": 1.309,  # 75 deg
    "cobotta_pro_joint_4": 0.0,
    "cobotta_pro_joint_5": 1.48353,  # 85 deg
    "cobotta_pro_joint_6": 0.0,
}

##
# Gripper mapping.
##


def gripper_command_to_leader(
    command: float, open_pos: float = GRIPPER_OPEN, closed_pos: float = GRIPPER_CLOSED
) -> float:
    """Map a policy output in ``[-1, 1]`` to a ``finger_joint`` angle.

    ``+1`` is fully open and ``-1`` fully closed, the same sense as the ROX/FR3
    gripper action. Values outside ``[-1, 1]`` extrapolate; the caller clamps to
    the joint limits.
    """
    scale = 0.5 * (open_pos - closed_pos)
    offset = 0.5 * (open_pos + closed_pos)
    return offset + scale * command


def gripper_targets(
    leader: float,
    limits: dict[str, tuple[float, float]] | None = None,
    mimic: dict[str, float] = GRIPPER_MIMIC,
) -> dict[str, float]:
    """Per-joint position targets for a ``finger_joint`` angle.

    Each joint gets ``multiplier * leader``, clamped to its own limits if given.
    The vectorised version in ``actions.GripperAction`` does exactly this.
    """
    targets = {}
    for name, multiplier in mimic.items():
        target = multiplier * leader
        if limits is not None and name in limits:
            lower, upper = limits[name]
            target = min(max(target, lower), upper)
        targets[name] = target
    return targets
