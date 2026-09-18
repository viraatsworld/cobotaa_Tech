# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The Cobotta Pro 900 + OnRobot RG6, for Isaac Lab.

Everything a task needs::

    from techtory_cobotta_isaaclab.robot import (
        COBOTTA_RG6_CFG,  # ArticulationCfg for InteractiveSceneCfg
        ActionsCfg,  # the 7-D action space
        ObservationsCfg,  # the matching 48-D observation group
    )
"""

from techtory_cobotta_isaaclab.robot.actions import ActionsCfg as ActionsCfg
from techtory_cobotta_isaaclab.robot.observations import (
    FT_ROBOT_ENTITY as FT_ROBOT_ENTITY,
)
from techtory_cobotta_isaaclab.robot.observations import (
    FT_SENSOR_ENTITY as FT_SENSOR_ENTITY,
)
from techtory_cobotta_isaaclab.robot.observations import (
    TCP_ENTITY as TCP_ENTITY,
)
from techtory_cobotta_isaaclab.robot.observations import (
    ObservationsCfg as ObservationsCfg,
)
from techtory_cobotta_isaaclab.robot.observations import (
    no_wrench as no_wrench,
)
from techtory_cobotta_isaaclab.robot.observations import (
    object_pose_b as object_pose_b,
)
from techtory_cobotta_isaaclab.robot.observations import (
    payload_wrench as payload_wrench,
)
from techtory_cobotta_isaaclab.robot.observations import (
    tcp_pose_b as tcp_pose_b,
)
from techtory_cobotta_isaaclab.robot.observations import (
    wrist_wrench as wrist_wrench,
)
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    ARM_ACTUATOR_NEWTON as ARM_ACTUATOR_NEWTON,
)
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    ARM_JOINTS as ARM_JOINTS,
)
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    COBOTTA_RG6_CFG as COBOTTA_RG6_CFG,
)
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    FT_BODY as FT_BODY,
)
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    GRIPPER_CLOSED as GRIPPER_CLOSED,
)
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    GRIPPER_JOINT as GRIPPER_JOINT,
)
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    GRIPPER_MIMIC_JOINTS as GRIPPER_MIMIC_JOINTS,
)
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    GRIPPER_OPEN as GRIPPER_OPEN,
)
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    HOME_POSE as HOME_POSE,
)
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    ROBOT_BASE_BODY as ROBOT_BASE_BODY,
)
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    TCP_BODY as TCP_BODY,
)
from techtory_cobotta_isaaclab.robot.robot_cfg import (
    TCP_OFFSET as TCP_OFFSET,
)
