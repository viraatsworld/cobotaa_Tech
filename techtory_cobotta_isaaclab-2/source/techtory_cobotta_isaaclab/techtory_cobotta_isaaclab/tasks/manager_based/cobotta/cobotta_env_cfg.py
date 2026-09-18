# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""A minimal, reward-free environment: the Cobotta Pro + RG6 in the Techtory cell.

This is not a task. It exists so the robot can be stepped, driven and inspected
in its cell, and so a real task has something concrete to subclass::

    @configclass
    class PickHammerEnvCfg(CobottaEnvCfg):
        rewards: MyRewardsCfg = MyRewardsCfg()
        terminations: MyTerminationsCfg = MyTerminationsCfg()

Everything a task normally supplies -- rewards, terminations beyond the clock,
commands, curriculum -- is deliberately left empty.
"""

from __future__ import annotations

from isaaclab.envs import ManagerBasedRLEnvCfg, mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.physics import PhysxAutoCfg
from isaaclab.utils.configclass import configclass
from isaaclab.visualizers import VisualizerCfg
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg
from isaaclab_ov.physics import OvPhysxCfg
from isaaclab_physx.physics import PhysxCfg

from isaaclab_tasks.utils import PresetCfg

from techtory_cobotta_isaaclab.robot import ActionsCfg, ObservationsCfg
from techtory_cobotta_isaaclab.scene import TechtoryCellSceneCfg
from techtory_cobotta_isaaclab.scene.placement import ENV_SPACING

__all__ = [
    "CobottaEnvCfg",
    "CobottaPhysicsCfg",
    "EventCfg",
    "RewardsCfg",
    "TerminationsCfg",
]


@configclass
class CobottaPhysicsCfg(PresetCfg):
    """Selectable physics backends (``physics=NAME`` or ``play.py --physics NAME``).

    * ``physx`` (default) -- Isaac Sim PhysX when Kit is present, OvPhysX when
      it is not. What techtory_cobotta_isaacsim measured the wrist wrench on.
    * ``newton_mjwarp`` -- Newton with the MuJoCo Warp solver, kitless. The URDFs
      are converted by the standalone ``isaacsim-asset-isolated`` importer
      (Isaac Lab's ``importers`` extra), so no Kit process is needed.

    The Newton settings follow what the MuJoCo port of this cell needed:
    ``elliptic`` cones (a pyramidal cone makes the available friction depend on
    direction, which a pinch grasp feels) with ``impratio`` raised alongside,
    and ``implicitfast`` so the damped servos stay stable at the 5 ms step.
    ``njmax``/``nconmax`` are sized for the fingers, shelf boards and hammer.
    """

    isaacsim_physx: PhysxCfg = PhysxCfg()
    ovphysx: OvPhysxCfg = OvPhysxCfg()
    physx: PhysxAutoCfg = PhysxAutoCfg(isaacsim_physx=isaacsim_physx, ovphysx=ovphysx)
    newton_mjwarp: NewtonCfg = NewtonCfg(
        solver_cfg=MJWarpSolverCfg(
            njmax=400,
            nconmax=150,
            cone="elliptic",
            impratio=10.0,
            integrator="implicitfast",
        ),
        num_substeps=1,
        use_cuda_graph=True,
    )
    default: PhysxAutoCfg = physx


@configclass
class RewardsCfg:
    """Deliberately empty. This is a robot in its cell, not a task -- bring your own."""


@configclass
class TerminationsCfg:
    """Only the episode clock. A task adds whatever failure means to it."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class EventCfg:
    """Back to the start on every reset: arm at home, gripper at 0, hammer on the shelf."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


@configclass
class CobottaEnvCfg(ManagerBasedRLEnvCfg):
    """The Cobotta Pro + RG6 in the Techtory cell, with the standard 7-D action
    and 41-D observation spaces and no reward.

    ``sim.dt = 1/200`` with ``decimation = 4`` gives a 50 Hz policy. The fine
    physics step is for the gripper: the RG6 linkage links weigh 50 g, and
    stiff contact between them and the hammer rings at coarser steps -- which
    the wrist wrench then reports as noise.
    """

    scene: TechtoryCellSceneCfg = TechtoryCellSceneCfg(num_envs=16, env_spacing=ENV_SPACING)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self) -> None:
        self.decimation = 4
        self.episode_length_s = 10.0

        # From the open front of the cell, looking at the robot and the shelf.
        # Applies to whichever visualizer is selected (--viz kit, newton_gl, ...).
        self.sim.default_visualizer_cfg = VisualizerCfg(eye=(2.6, -2.2, 2.2), lookat=(0.1, 0.1, 1.1))

        self.sim.dt = 1.0 / 200.0
        self.sim.render_interval = self.decimation
        self.sim.physics = CobottaPhysicsCfg()
