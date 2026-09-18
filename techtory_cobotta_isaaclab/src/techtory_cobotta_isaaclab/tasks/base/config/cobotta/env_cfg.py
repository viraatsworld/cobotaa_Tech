# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""A minimal, reward-free environment: the Cobotta + RG6 in the Techtory cell.

This is not a task. It exists so the robot and its cell can be stepped, driven
and inspected, and so a real task has something concrete to subclass::

    @configclass
    class PickCanEnvCfg(BaseEnvCfg):
        rewards: MyRewardsCfg = MyRewardsCfg()
        terminations: MyTerminationsCfg = MyTerminationsCfg()

Rewards, commands and curriculum are deliberately left empty. What is not
optional is set here: the physics backend and the timestep the gripper was tuned
for.
"""

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab.visualizers import VisualizerCfg
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg
from isaaclab_physx.physics import PhysxCfg

from isaaclab_tasks.utils import PresetCfg, preset

from techtory_cobotta_isaaclab.robot import ARM_ACTUATOR_NEWTON, ActionsCfg, ObservationsCfg, no_wrench
from techtory_cobotta_isaaclab.scene.scene_cfg import TechtoryCellSceneCfg

from ... import mdp

##
# Physics presets
##


@configclass
class BasePhysicsCfg(PresetCfg):
    """Physics presets: Newton (MuJoCo-Warp) by default, Isaac Sim PhysX for the gripper.

    * **Newton** (``newton_mjwarp``, the default; ``newton_wraps`` is an alias)
      steps the arm and the cell without Kit, with softer arm gains
      (``ARM_ACTUATOR_NEWTON``). It cannot follow the RG6's parallel motion --
      five ``PhysxMimicJointAPI`` joints, a PhysX schema -- and its joint-wrench
      sensor skips links attached by a fixed joint, which is exactly where the
      wrist F/T sits, so the wrench observation reads zero. The cell's own
      colliders sit on Xforms, which Newton ignores; the shelf's boxes collide.
      Grasping, wrist-wrench readings and contact with the cell need
      ``physics=isaacsim_physx``.
    * **OvPhysX** simulates the robot and the wrist sensor correctly, but not the
      RG6's colliders: they are ``PhysxMeshMergeCollisionAPI`` collectors, which
      OvPhysX does not build, so the fingers close through anything and no object
      can be held (measured with scripts/check_ft_payload.py). That also rules out
      Isaac Lab's ``physx`` auto preset, which picks OvPhysX whenever Kit is not
      otherwise needed.

    ``isaacsim_physx`` is what the Isaac Sim demo ran on, and what the gripper
    tuning was measured against.
    """

    # Contact settings from Isaac Lab's Franka lift task: bounce_threshold keeps
    # small impacts inelastic, the short friction correlation distance resolves
    # friction on small patches like the RG6 pads.
    isaacsim_physx: PhysxCfg = PhysxCfg(bounce_threshold_velocity=0.01, friction_correlation_distance=0.00625)
    # Solver settings from Isaac Lab's Franka lift task; two substeps keep the
    # 0.01 s step at 5 ms for the gripper contacts.
    newton_mjwarp: NewtonCfg = NewtonCfg(
        solver_cfg=MJWarpSolverCfg(
            solver="newton",
            integrator="implicitfast",
            njmax=300,
            nconmax=200,
            impratio=1.0,
            cone="pyramidal",
            iterations=100,
            ls_iterations=15,
            use_mujoco_contacts=False,
        ),
        num_substeps=2,
        debug_mode=False,
    )
    newton_wraps: NewtonCfg = newton_mjwarp
    default: NewtonCfg = newton_mjwarp


##
# MDP settings
##


@configclass
class EventCfg:
    """Configuration for events."""

    # Robot back to the home pose -- drive targets included, or the arm would
    # spring back towards the last command -- and the objects back on the shelf.
    reset_scene = EventTerm(func=mdp.reset_scene_to_default, mode="reset", params={"reset_joint_targets": True})


@configclass
class RewardsCfg:
    """Deliberately empty. This is a robot in a cell, not a task -- bring your own."""


@configclass
class TerminationsCfg:
    """Only the episode clock. A task adds whatever failure means to it."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


##
# Environment configuration
##


@configclass
class BaseEnvCfg(ManagerBasedRLEnvCfg):
    """The Cobotta + RG6 in the Techtory cell, with the standard action and
    observation spaces and no reward.

    ``sim.dt = 0.01`` with ``decimation = 2`` runs the policy at 50 Hz. The RG6's
    anti-ringing tuning (armature, joint friction, speed cap) was settled at the
    Isaac Sim demo's coarser 1/60 s step, so this is on the safe side of it.
    """

    # Scene settings: 3 m spacing clears the cell's 2.2 m footprint
    scene: TechtoryCellSceneCfg = TechtoryCellSceneCfg(num_envs=16, env_spacing=3.0)
    # Basic settings: the 48-D policy observation and the 7-D action space
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 20.0
        # visualizer camera: outside the cell's open corner, looking at the robot and shelf
        self.sim.default_visualizer_cfg = VisualizerCfg(eye=(2.2, -2.0, 2.2), lookat=(0.15, 0.0, 1.1))
        # simulation settings
        self.sim.dt = 0.01
        self.sim.render_interval = self.decimation
        self.sim.physics = BasePhysicsCfg()
        # Backend-specific robot terms, selected by the same physics= name: the
        # USD's arm gains and the wrist F/T only work on Isaac Sim PhysX. The
        # zero wrench keeps the observation at 48 values on Newton.
        actuators = self.scene.robot.actuators
        actuators["arm"] = preset(default=ARM_ACTUATOR_NEWTON, isaacsim_physx=actuators["arm"])
        policy = self.observations.policy
        policy.wrist_wrench = preset(default=ObsTerm(func=no_wrench), isaacsim_physx=policy.wrist_wrench)
