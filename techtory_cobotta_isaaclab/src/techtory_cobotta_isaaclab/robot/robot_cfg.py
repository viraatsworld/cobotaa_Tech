# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The Cobotta Pro 900 + OnRobot RG6 as an Isaac Lab articulation.

This is the only module to touch when retuning the robot. It names the joint
groups and the frames tasks care about, declares the actuators, and assembles
the :class:`ArticulationCfg` a scene drops in as ``robot``.

The robot is spawned from the USD the Isaac Sim demo uses
(``cvrb0609_with_graph2.usd``): arm and gripper are one articulation, 12 DOF,
rooted in a fixed joint to the world. Its default prim also carries the demo's
ROS OmniGraph and a grid environment, but those sit outside the default prim and
are never composed when the file is referenced.

Every number that tunes the robot was carried over from the Isaac Sim demo,
where it was measured against this exact asset (see
``techtory_cobotta_isaacsim/plan-gripper-effort.md``). The one deliberate change
is J6's effort limit, noted where it is set.
"""

from __future__ import annotations

import math

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationRootPropertiesCfg

from techtory_cobotta_isaaclab.assets import COBOTTA_RG6_USD
from techtory_cobotta_isaaclab.scene.layout import ROBOT_MOUNT
from techtory_cobotta_isaaclab.spawners import CobottaRg6UsdFileCfg

__all__ = [
    "ARM_ACTUATOR_NEWTON",
    "ARM_JOINTS",
    "COBOTTA_RG6_CFG",
    "FT_BODY",
    "GRIPPER_CLOSED",
    "GRIPPER_JOINT",
    "GRIPPER_MIMIC_JOINTS",
    "GRIPPER_OPEN",
    "HOME_POSE",
    "ROBOT_BASE_BODY",
    "TCP_BODY",
    "TCP_OFFSET",
]

##
# Joint groups and frames.
##

ARM_JOINTS: tuple[str, ...] = tuple(f"cobotta_pro_joint_{i}" for i in range(1, 7))

GRIPPER_JOINT = "finger_joint"
"""The one driven RG6 joint. Positive closes the jaw."""

# The rest of the RG6 four-bar. Each is a PhysX mimic joint (PhysxMimicJointAPI,
# gearing +-1) that follows finger_joint -- still a real DOF, but the solver moves
# it, so it gets no drive of its own.
GRIPPER_MIMIC_JOINTS: tuple[str, ...] = (
    "left_inner_knuckle_joint",
    "right_inner_knuckle_joint",
    "right_outer_knuckle_joint",
    "left_inner_finger_joint",
    "right_inner_finger_joint",
)

# finger_joint's limits are +-0.6283 rad (+-36 deg). Commands stay just inside them
# so a position goal can be reached rather than pressed against the stop.
GRIPPER_OPEN = -0.62
"""finger_joint fully open: 151 mm between the pads."""
GRIPPER_CLOSED = 0.62
"""finger_joint fully closed: pads touching."""

ROBOT_BASE_BODY = "cobotta_pro_base_link"
"""The articulation's root link; poses in observations are given in its frame."""

FT_BODY = "base_link"
"""The RG6 base, child of the fixed ``gripper_joint`` on ``cobotta_pro_J6``.

Its incoming joint wrench is the flange <-> gripper wrench, i.e. what a wrist
F/T sensor bolted between arm and gripper measures. ROS calls this frame
``onrobot_rg6_base_link``.
"""

TCP_BODY = "base_link"
TCP_OFFSET: tuple[float, float, float] = (0.0, 0.0, 0.25)
"""The tool centre point in :data:`TCP_BODY`'s frame: 0.25 m along the approach axis.

Identical to MoveIt's tip link ``cobotta_pro_tool0`` (techtory_cobotta_workcell.urdf.xacro),
and inside the pads, which span 0.195-0.268 m.
"""

##
# Initial state.
##

# The Isaac Sim demo's start pose (scripts/main.py). The all-zero pose is not
# usable here: it points the arm straight up and puts the wrist through the roof.
HOME_POSE: dict[str, float] = {
    "cobotta_pro_joint_1": 0.0,
    "cobotta_pro_joint_2": 0.349066,
    "cobotta_pro_joint_3": 1.309,
    "cobotta_pro_joint_4": 0.0,
    "cobotta_pro_joint_5": 1.48353,
    "cobotta_pro_joint_6": 0.0,
}

##
# Actuators.
##

# USD authors angular drive gains per DEGREE; Isaac Lab takes them per RADIAN.
# Every gain below is written as the USD number times this, so the provenance
# stays visible and the factor-of-57 error the demo's notes warn about cannot
# creep in.
_PER_DEG = 180.0 / math.pi

# Drive gains as the robot USD authors them (per degree). They are very stiff:
# with the 60 N*m limit the drive saturates for any real error, so the arm moves
# at its velocity limit and stops on target -- how MoveIt drove it in the demo.
_ARM_STIFFNESS_PER_DEG = (76146.0, 252286.0, 88625.0, 78852.0, 70255.0, 70255.0)
_ARM_DAMPING_PER_DEG = (38.0729, 126.1428, 44.31, 35.0, 35.0, 35.0)

# MoveIt's limits (techtory_cobotta_moveit/config/joint_limits.yaml), which the
# robot USD also authors as maxJointVelocity.
_ARM_VELOCITY_LIMIT = (0.397935, 0.331612, 0.397935, 0.497418, 0.497418, 0.596903)

_ARM_ACTUATOR = ImplicitActuatorCfg(
    joint_names_expr=["cobotta_pro_joint_[1-6]"],
    stiffness={j: k * _PER_DEG for j, k in zip(ARM_JOINTS, _ARM_STIFFNESS_PER_DEG, strict=True)},
    damping={j: d * _PER_DEG for j, d in zip(ARM_JOINTS, _ARM_DAMPING_PER_DEG, strict=True)},
    joint_velocity_limit=dict(zip(ARM_JOINTS, _ARM_VELOCITY_LIMIT, strict=True)),
    # The USD authors 60 N*m on J1-J5 but 1 N*m on J6 -- the URDF's placeholder
    # effort="1" that was raised everywhere else. 1 N*m lets any off-axis payload
    # twist the wrist, so J6 gets the same 60.
    joint_effort_limit=60.0,
)

# The same arm for Newton (MuJoCo-Warp). The USD gains rely on PhysX enforcing
# the joint velocity limit while the drive saturates; MuJoCo-Warp does not
# enforce it, and with those gains the arm overshoots and never settles. These
# stay mostly out of saturation instead: at the 60 N*m limit the damping alone
# caps the speed near 60 / 200 = 0.3 rad/s, the bottom of MoveIt's range, and
# gravity sags J2/J3 by ~0.01 rad (measured with scripts/play.py).
ARM_ACTUATOR_NEWTON = _ARM_ACTUATOR.replace(stiffness=2000.0, damping=200.0, armature=0.1)
"""Arm drive for the Newton presets; the task swaps it in (see ``BaseEnvCfg``)."""

# The Isaac Sim demo's force-limited grip (spawn_robot.configure_gripper_drive and
# stabilize_gripper_joints):
#   * 5 N*m on finger_joint is ~62 N at the pads (0.080 m lever), mid-range of
#     the RG6's 25-120 N. Anything in 1-10 N*m stalls at the same angle and holds;
#     the as-shipped 6000 N*m crushed and ejected objects.
#   * armature and friction on all six gripper joints stop the near-massless
#     fingers ringing on contact, which otherwise also keeps a stall from settling.
#   * 0.6 rad/s closing speed keeps touchdown gentle.
_GRIP_ARMATURE = 0.15
_GRIP_VELOCITY_LIMIT = 0.6
# Isaac Lab applies this as a static-friction effort [N*m]. The demo set PhysX's
# legacy jointFriction to 0.4, a different parameterisation; this value was
# re-checked with scripts/check_ft_payload.py (the grip holds and the stall is quiet).
_GRIP_FRICTION = 0.4

_GRIPPER_DRIVE_ACTUATOR = ImplicitActuatorCfg(
    joint_names_expr=[GRIPPER_JOINT],
    stiffness=3.0 * _PER_DEG,
    damping=0.1 * _PER_DEG,
    joint_effort_limit=5.0,
    joint_velocity_limit=_GRIP_VELOCITY_LIMIT,
    armature=_GRIP_ARMATURE,
    friction=_GRIP_FRICTION,
)

# The mimic joints get zero gains -- a drive would fight the mimic constraint --
# but the same armature, friction and speed cap as the driver.
_GRIPPER_MIMIC_ACTUATOR = ImplicitActuatorCfg(
    joint_names_expr=list(GRIPPER_MIMIC_JOINTS),
    stiffness=0.0,
    damping=0.0,
    joint_velocity_limit=_GRIP_VELOCITY_LIMIT,
    armature=_GRIP_ARMATURE,
    friction=_GRIP_FRICTION,
)

##
# The articulation.
##

COBOTTA_RG6_CFG = ArticulationCfg(
    spawn=CobottaRg6UsdFileCfg(
        usd_path=str(COBOTTA_RG6_USD),
        articulation_props=PhysxArticulationRootPropertiesCfg(
            # The asset authors self-collisions ON at the root. Once the RG6 links
            # have shapes, the four-bar's overlapping knuckles push each other
            # apart and the jaw jams with nothing in it.
            enabled_self_collisions=False,
            # As authored; restated so the articulation's solver budget is visible here.
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=1,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=ROBOT_MOUNT.pos,
        rot=ROBOT_MOUNT.rot,
        joint_pos={
            **HOME_POSE,
            # Jaw half-open (94 mm). Mimic joints must start consistent with it:
            # gearing +-1 times 0 is 0.
            GRIPPER_JOINT: 0.0,
            **dict.fromkeys(GRIPPER_MIMIC_JOINTS, 0.0),
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "arm": _ARM_ACTUATOR,
        "gripper_drive": _GRIPPER_DRIVE_ACTUATOR,
        "gripper_mimic": _GRIPPER_MIMIC_ACTUATOR,
    },
)
"""Cobotta Pro 900 + RG6 in the Techtory cell, ready for ``InteractiveSceneCfg``."""
