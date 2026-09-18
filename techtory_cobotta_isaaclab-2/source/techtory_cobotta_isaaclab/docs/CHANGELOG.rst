Changelog
---------

0.2.0 (2026-09-10)
~~~~~~~~~~~~~~~~~~

Added
^^^^^

* Newton (MuJoCo Warp) support, kitless: ``newton_mjwarp`` physics preset, and
  ``scripts/play.py --physics NAME`` (a Hydra ``physics=NAME`` preset selector).
* ``wrist_ft``: Isaac Lab ``JointWrenchSensorCfg`` in the scene.
* Package import exposes the codeless ``physxSchema`` / ``newton`` USD schema plugins
  the standalone URDF importer needs without Kit.

Changed
^^^^^^^

* ``wrist_wrench`` reads ``JointWrenchSensor`` (Isaac Lab removed
  ``ArticulationData.body_incoming_joint_wrench_b``).
* ``joint_rg6`` is a revolute joint locked to +/-1 mrad (Newton's joint-wrench
  sensor skips fixed joints), with a centring ``wrist_lock`` actuator.
* Scene spawners are kitless: the cell/plate STLs and the hammer wrapper are
  written with ``pxr`` and cached. On Newton the cell is visual-only and the
  hammer a single convex hull.
* Camera set through ``sim.default_visualizer_cfg`` (``env_cfg.viewer`` is deprecated).

Fixed
^^^^^

* The hammer path resolved one directory too deep.
* Agent and training scripts import ``add_launcher_args`` / ``launch_simulation``
  from ``isaaclab.app`` (moved out of ``isaaclab_tasks.utils``).
* ``sb3_ppo_cfg.yaml``: ``policy_kwargs`` is a mapping (``process_sb3_cfg`` no longer
  evaluates ``"dict(...)"`` strings), and ``n_minibatches`` replaces a fixed
  ``batch_size``.
* The hammer's millimetre unit fix-up was scaled twice by Isaac Lab's xform-op
  standardisation; the reference now sits below a clean rigid-body root.

0.1.0 (2026-09-10)
~~~~~~~~~~~~~~~~~~

Added
^^^^^

* ``techtory_cobotta_isaaclab.robot``: the Denso Cobotta Pro (CVRB0609) + OnRobot
  RG6 as an Isaac Lab robot, ported from ``techtory_cobotta_isaacsim`` without ROS 2.

  * ``joints`` -- joint names, frames, home pose and the RG6 mimic coupling; no
    Isaac imports.
  * ``robot_cfg`` -- ``COBOTTA_CFG``: fixed base, fixed-joint merging off so the
    wrist F/T frame survives, actuator gains that hold the placeholder-mass arm.
  * ``actions`` -- ``GripperAction`` (one scalar fanned out to six RG6 joints) and
    the 7-D ``ActionsCfg``.
  * ``observations`` -- arm, gripper, tool pose, **wrist force/torque**
    (``wrist_wrench``) and hammer pose, wired into the 41-D ``ObservationsCfg``.

* ``techtory_cobotta_isaaclab.scene``: the Techtory cell (exact triangle-mesh
  collider), base plate, shelf and graspable hammer.
* ``Techtory-Cobotta-Base-v0``: reward-free ``CobottaEnvCfg`` with rsl_rl and sb3
  PPO configs.
* ``scripts/build_urdf_asset.py``: ROS-free asset builder; ``scripts/play.py``
  per-joint demo with F/T readout.
