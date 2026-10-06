# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Pick-to-bin with the Cobotta + RG6: an object from the table's spawn zone into the blue pallet.

Built on the base task (:class:`BaseEnvCfg`), which keeps its own empty reward
and stays the sandbox for ``scripts/play.py``. This task swaps in:

* the pallet, the spawn zone and the object to pick, lying on the table;
* the 4-D top-down TCP action for the arm, and the gripper rule;
* actor/critic observations, staged rewards, success/failure terminations.

Which object is picked is the ``grasp_object`` name: a :data:`GRASP_OBJECTS`
entry, with its asset in :data:`GRASPABLE_OBJECT_CFGS`. To add an object, add
both and subclass :class:`PickToBinEnvCfg` with the new name.

Rewards are given per event or per step, exactly as weighted below (see
``mdp/rewards.py`` for why the reward manager's ``dt`` does not apply).
"""

from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab_physx.physics import PhysxCfg

from techtory_cobotta_isaaclab.robot import ARM_JOINTS, FT_SENSOR_ENTITY, TopDownTcpTargetActionCfg, wrist_wrench
from techtory_cobotta_isaaclab.scene.grasp_objects import GRASP_OBJECTS
from techtory_cobotta_isaaclab.scene.layout import SPAWN_ZONE, quat_rotate
from techtory_cobotta_isaaclab.scene.scene_cfg import (
    GRASPABLE_OBJECT_CFGS,
    HAMMER_CFG,
    PALLET_CFG,
    SPAWN_ZONE_MARKER_CFG,
    TechtoryCellSceneCfg,
)
from techtory_cobotta_isaaclab.tasks.base.config.cobotta.env_cfg import BaseEnvCfg, BasePhysicsCfg

from ... import mdp

##
# Physics
##


@configclass
class PickToBinPhysicsCfg(BasePhysicsCfg):
    """The base task's Isaac Sim PhysX settings, with GPU contact buffers sized for thousands of cells.

    Each cell collides as its full ~470k-triangle mesh. The 5 mm contact offset
    on it (``WORKCELL_CFG``) keeps the contact work small, but PhysX's 64 MB
    default collision stack is still sized for small scenes; these leave room for
    the arm and the object touching the table in every environment.
    """

    isaacsim_physx: PhysxCfg = PhysxCfg(
        bounce_threshold_velocity=0.01,
        friction_correlation_distance=0.00625,
        gpu_collision_stack_size=2**30,
        gpu_max_rigid_contact_count=2**24,
        gpu_max_rigid_patch_count=2**20,
        gpu_found_lost_pairs_capacity=2**23,
        gpu_heap_capacity=2**27,
        gpu_temp_buffer_capacity=2**25,
    )
    default: PhysxCfg = isaacsim_physx


##
# Scene
##


@configclass
class PickToBinSceneCfg(TechtoryCellSceneCfg):
    """The cell with the pallet, the spawn zone and one object to pick, lying in the zone.

    The cell, shelf, pallet and robot all collide (PhysX); the zone is only drawn.
    ``object`` is replaced by the asset of the environment's ``grasp_object``.
    """

    pallet = PALLET_CFG
    spawn_zone = SPAWN_ZONE_MARKER_CFG
    object = HAMMER_CFG.replace(prim_path="{ENV_REGEX_NS}/Object")
    hammer = None
    soda_can = None


##
# MDP settings
##

_ARM = SceneEntityCfg("robot", joint_names=list(ARM_JOINTS), preserve_order=True)


@configclass
class ActionsCfg:
    """``[dx, dy, dz, dyaw]`` for the arm. The gripper is the rule, which takes no action."""

    # The commanded TCP stays within the spawn zone, the bin and the way between
    # them, robot base frame (x: out to the bin, y: towards the shelf is -y,
    # z: up from the base). z >= 5 mm keeps the TCP 23 mm over the table; the
    # cell's own colliders stop the arm too, this keeps exploration where the
    # task is. tests/test_pick_to_bin_layout.py checks the home pose, every
    # grasp target and the bin target are inside.
    arm: TopDownTcpTargetActionCfg = TopDownTcpTargetActionCfg(
        workspace_min=(-0.25, -0.75, 0.005), workspace_max=(0.80, 0.27, 0.45)
    )
    gripper: mdp.ProximityGripperActionCfg = mdp.ProximityGripperActionCfg()


@configclass
class ObservationsCfg:
    """The actor sees what the real robot can provide; the critic also gets ground truth."""

    @configclass
    class PolicyCfg(ObsGroup):
        """39 values: ``6 + 6 + 3 + 2 + 4 + 1 + 3 + 2 + 3 + 2 + 3 + 4``, concatenated in this order."""

        arm_pos = ObsTerm(func=mdp.joint_pos, params={"asset_cfg": _ARM})
        arm_vel = ObsTerm(func=mdp.joint_vel, params={"asset_cfg": _ARM})
        tcp_pos = ObsTerm(func=mdp.tcp_pos_b)
        tcp_yaw = ObsTerm(func=mdp.tcp_yaw_sincos)
        tcp_target_lead = ObsTerm(func=mdp.tcp_target_lead)
        gripper_closed = ObsTerm(func=mdp.gripper_closed)
        grasp_target = ObsTerm(func=mdp.grasp_target_b)
        object_yaw = ObsTerm(func=mdp.object_yaw_sincos)
        grasp_offset = ObsTerm(func=mdp.grasp_offset_b)
        grasp_yaw_error = ObsTerm(func=mdp.grasp_yaw_error_sincos)
        bin_offset = ObsTerm(func=mdp.bin_offset_b)
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        """Simulator ground truth for the value function only: 19 values."""

        object_quat = ObsTerm(func=mdp.object_quat_b)
        object_lin_vel = ObsTerm(func=mdp.object_lin_vel_b)
        object_ang_vel = ObsTerm(func=mdp.object_ang_vel_b)
        wrist_wrench = ObsTerm(func=wrist_wrench, params={"sensor_cfg": FT_SENSOR_ENTITY})
        jaw_position = ObsTerm(func=mdp.jaw_position)
        grasped = ObsTerm(func=mdp.object_grasped)
        picked = ObsTerm(func=mdp.object_picked)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class EventCfg:
    """Resets run in this order: robot and scene to default, then the object to a random pose in the spawn zone."""

    # Drive targets included, or the arm would spring back towards the last command.
    reset_scene = EventTerm(func=mdp.reset_scene_to_default, mode="reset", params={"reset_joint_targets": True})
    reset_object = EventTerm(func=mdp.reset_object_in_spawn_zone, mode="reset")


@configclass
class RewardsCfg:
    """The staged reward table. Weights are the reward per event or per step."""

    # Approach: k * (phi_{t-1} - phi_t), phi = distance + 0.1 m/rad * |yaw error|
    approach_progress = RewTerm(func=mdp.ApproachProgress, weight=1.0, params={"yaw_weight": 0.1})
    reached_object = RewTerm(func=mdp.ReachedObject, weight=5.0)
    # Pick: grasped and lifted 2.5 cm
    picked_object = RewTerm(func=mdp.PickedObject, weight=10.0)
    # Transport: k * (d_{t-1} - d_t) towards the point above the bin, only while grasped
    transport_progress = RewTerm(func=mdp.TransportProgress, weight=1.0)
    near_bin = RewTerm(func=mdp.NearBin, weight=5.0)
    # Place: the rule let go with the whole object over the bin
    placed_in_bin = RewTerm(func=mdp.PlacedInBin, weight=20.0)
    # Success: resting in the bin for 0.4 s (ends the episode)
    success = RewTerm(func=mdp.termination_event, weight=50.0, params={"term_name": "success"})
    # Failure
    dropped_outside_bin = RewTerm(func=mdp.termination_event, weight=-10.0, params={"term_name": "dropped_outside_bin"})
    lost_in_transport = RewTerm(func=mdp.lost_in_transport, weight=-5.0)
    # Efficiency
    time_penalty = RewTerm(func=mdp.per_step, weight=-0.005)
    # Smoothness: small, so the policy still learns to reach the goal first; raise once
    # it does if the motion is still jerky. A jittery start (|a_t - a_(t-1)|^2 ~ 8) costs
    # ~2 over an episode, against +5..+50 for the milestones.
    action_rate = RewTerm(func=mdp.action_rate, weight=-0.0005)
    joint_acceleration = RewTerm(func=mdp.joint_acceleration, weight=-1.0e-4)


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    success = DoneTerm(func=mdp.ObjectResting, params={"hold_steps": 10})
    dropped_outside_bin = DoneTerm(func=mdp.DroppedOutsideBin, params={"hold_steps": 5, "fall_depth": 0.04})


##
# Environment configuration
##


@configclass
class PickToBinEnvCfg(BaseEnvCfg):
    """An object from the spawn zone into the bin; which object is ``grasp_object``.

    Timing: ``sim.dt = 0.01`` (the gripper's tuning) with ``decimation = 4``, so
    the policy runs at 25 Hz. A pick-and-place at the arm's real speed limits
    takes 10-14 s (scripted, scripts/check_pick_to_bin.py); 20 s episodes
    (500 policy steps) leave room for a second grasp attempt.

    Size: 4096 environments. Measured on this machine (RTX 4000 Ada 20 GB,
    125 GB RAM), stepping with zero action: 1024 envs 13k steps/s and 9.8 GB of
    GPU memory, 2048 envs 20k / 10.5 GB, 4096 envs 23k / 12.3 GB and 45 GB of
    RAM, 8192 envs 26k / 14.5 GB and 86 GB of RAM. Past 4096 the gain is 12%
    for twice the RAM and a slower start; 4096 leaves GPU memory for PPO and
    for the contacts a moving arm adds.
    """

    grasp_object: str = "hammer"
    """The object to pick: a key of :data:`GRASP_OBJECTS` and :data:`GRASPABLE_OBJECT_CFGS`."""

    scene: PickToBinSceneCfg = PickToBinSceneCfg(num_envs=4096, env_spacing=3.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        self.decimation = 4
        self.sim.render_interval = self.decimation
        self.sim.physics = PickToBinPhysicsCfg()
        self.episode_length_s = 20.0
        # The object, resting with its box centre at the zone's centre (resets then move it about).
        obj = GRASP_OBJECTS[self.grasp_object]
        resting = obj.resting_body
        offset = quat_rotate(resting.rot, obj.box_center)
        pos = (SPAWN_ZONE.center[0] - offset[0], SPAWN_ZONE.center[1] - offset[1], resting.pos[2])
        self.scene.object = GRASPABLE_OBJECT_CFGS[self.grasp_object].replace(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=pos, rot=resting.rot),
        )
        # The robot's 5 N*m grip was tuned not to crush the soda can on the shelf. On
        # the hammer's 25 mm handle the jaw is nearly closed, where each pad moves
        # ~79 mm per rad of finger_joint: 5 N*m is ~32 N per pad, and the handle
        # slipped out as soon as it left the table (measured). 15 N*m is ~95 N,
        # inside the RG6's 25-120 N.
        self.scene.robot.actuators["gripper_drive"].joint_effort_limit = 15.0


@configclass
class HammerToBinEnvCfg(PickToBinEnvCfg):
    """The hammer, lying flat, gripped across its handle."""

    grasp_object: str = "hammer"


@configclass
class HammerToBinEnvCfg_PLAY(HammerToBinEnvCfg):
    """A few environments, for watching a trained policy."""

    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 16


@configclass
class SodaCanToBinEnvCfg(PickToBinEnvCfg):
    """The soda can, standing, gripped round its top."""

    grasp_object: str = "soda_can"


@configclass
class SodaCanToBinEnvCfg_PLAY(SodaCanToBinEnvCfg):
    """A few environments, for watching a trained policy."""

    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 16
