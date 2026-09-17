# techtory_cobotta_isaaclab

The **Denso Cobotta Pro (CVRB0609)** arm with an **OnRobot RG6** gripper, standing in the
**Techtory cell** with its **shelf and hammer**, packaged as an
[Isaac Lab](https://isaac-sim.github.io/IsaacLab/) robot for GPU reinforcement learning on
Isaac Sim — with a **6-axis wrist force/torque sensor** in the observation.

It is a port of `src/techtory_cobotta_isaacsim` with ROS 2 removed, built the same way as
`src/roxfr3_isaaclab`. The robot description, start pose, gripper tuning and F/T measurement
point carry over; `rclpy`, `ros2_control`, the OmniGraph bridge and the launch layer do not.
What replaces them is an `ArticulationCfg`, a scene, and action and observation terms that
drop straight into a `ManagerBasedRLEnv`.

| | |
| --- | --- |
| **Action** | 7-D: `[q1..q6 offset, grip]` |
| **Observation** | 41-D: arm, gripper, tool pose, **wrist wrench**, hammer pose, last action |
| **Degrees of freedom** | 6 arm + 6 RG6 linkage joints (driven by one gripper action) + 1 locked wrist joint (the F/T mount) |
| **Base** | Fixed, on the cell's base plate |
| **Scene** | Cell shell, base plate, shelf, hammer: one cell per environment |
| **Physics** | Newton / MuJoCo Warp (kitless, verified) and PhysX |
| **Registered task** | `Techtory-Cobotta-Base-v0` |

**This is a robot in its cell, not a task.** `CobottaEnvCfg` has no rewards and no
terminations beyond the episode clock. Bring your own. It exists to be subclassed, and to
make the robot runnable on its own.

> **Status.** Verified end to end on **Newton (MuJoCo Warp), kitless**, in
> `~/Main/Environment/env_isaaclab`: `python scripts/play.py --physics newton_mjwarp --viz newton_gl`
> runs the full sweep, every arm joint tracks, and the wrist F/T reads 9.84 N / 0.004 N·m at
> rest, matching `techtory_cobotta_isaacsim`'s PhysX baseline (9.809 N / 0.003–0.005 N·m). The 19
> plain-Python tests pass, and the 5 config tests in `test_robot_cfg.py` pass in `env_isaaclab`
> (run by hand there, since that venv has no pytest). The **PhysX** path is configured but not verified here: that venv has no Kit, so
> `physics=physx` resolves to OvPhysX, whose optional `ovphysx` wheel is not installed (see
> [Physics backends](#physics-backends)).

---

## Requirements

* Linux with an NVIDIA GPU.
* Isaac Sim 6.0 and Isaac Lab 3.0, with Python 3.12 (the same stack as `roxfr3_isaaclab`).
* The sibling package `techtory_cobotta_isaacsim` in the same workspace. The hammer is used
  in place from there, because this repo keeps USD files out of git (see
  [Model scope](#model-scope)).
* A ROS 2 workspace is needed **only** to regenerate the assets, and only for its installed
  mesh packages. Day-to-day use needs none.

## Installation

### On this machine (kitless, Newton)

Isaac Lab (a source checkout at `~/Main/Environment/IsaacLab`, installed editable) lives in
`~/Main/Environment/env_isaaclab`, Python 3.12, **without Isaac Sim / Kit**. Two additions
make this package run there:

```bash
source ~/Main/Environment/env_isaaclab/bin/activate
cd ~/Techtory/src/techtory_cobotta_isaaclab

# This extension, editable.
pip install -e source/techtory_cobotta_isaaclab

# Isaac Lab's `importers` extra: the standalone URDF importer. Without Kit, Isaac Lab's
# UrdfConverter imports `isaacsim.asset.importer.urdf` from this wheel; without it the robot
# and shelf cannot be converted. (It pins newton-usd-schemas==0.4.1, within newton's >=0.4.1.)
pip install --extra-index-url https://pypi.nvidia.com \
    "isaacsim-asset-isolated==6.1.0.0" "tinyobjloader==2.0.0rc13"
```

Equivalently, from the Isaac Lab checkout: `pip install -e "source/isaaclab[importers]"`.
pytest is not part of that venv; `pip install pytest` to run the Isaac-Lab-dependent tests
there (the plain-Python ones run in any Python 3.12).

### With Isaac Sim (Kit, PhysX)

Isaac Sim 6.0.1 is also installed in `~/Main/Environment/isaac6`, without Isaac Lab. To use
Kit and Isaac Sim PhysX, install Isaac Lab and this extension into that venv:

```bash
source ~/Main/Environment/isaac6/bin/activate
pip install --extra-index-url https://pypi.nvidia.com \
    isaaclab "isaaclab-rl[rsl-rl]" isaaclab-tasks isaaclab-physx
cd ~/Techtory/src/techtory_cobotta_isaaclab
pip install -e source/techtory_cobotta_isaaclab pytest
```

Against a **source checkout** of Isaac Lab, install its packages editable instead:

```bash
for pkg in isaaclab isaaclab_rl isaaclab_tasks isaaclab_physx isaaclab_assets; do
    pip install -e "<IsaacLab>/source/${pkg}"
done
```

Verify:

```bash
python -c "from techtory_cobotta_isaaclab.robot import COBOTTA_CFG; print(COBOTTA_CFG.spawn.asset_path)"
python scripts/list_envs.py         # Techtory-Cobotta-Base-v0
pytest tests -q                     # 24 passed with Isaac Lab; 19 passed, 5 skipped without
```

### From scratch, with uv

Isaac Lab's wheels come from `pypi.nvidia.com` and torch from the PyTorch CUDA index; both
are declared in this project's `pyproject.toml`:

```bash
cd techtory_cobotta_isaaclab
uv venv --python 3.12
source .venv/bin/activate

# Isaac Sim first: it is the big one, and Isaac Lab resolves against it.
uv pip install --index-strategy unsafe-best-match --prerelease allow \
    "isaacsim[all,extscache]==6.0.1"

# Then Isaac Lab and the RL bindings.
uv pip install --index-strategy unsafe-best-match --prerelease allow \
    isaaclab "isaaclab-rl[rsl-rl]" isaaclab-tasks isaaclab-physx

# Finally this extension, editable.
uv pip install -e source/techtory_cobotta_isaaclab pytest
```

> **If you have sourced a ROS 2 workspace in the same shell**, `PYTHONPATH` still points at
> `/opt/ros/<distro>/lib/python3.*/site-packages`, and the interpreter will import ROS
> packages (and ROS's pytest plugins) ahead of its own. Run `PYTHONPATH= python ...`, or use
> a shell that has not sourced ROS.

### Using it from another project

```toml
# pyproject.toml
[project]
dependencies = ["techtory-cobotta-isaaclab-ext"]

[tool.uv.sources]
techtory-cobotta-isaaclab-ext = { path = "../techtory_cobotta_isaaclab/source/techtory_cobotta_isaaclab", editable = true }
```

## Using it in an RL agent

Subclass the environment and add what a task needs:

```python
import gymnasium as gym
from isaaclab.envs import mdp
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils.configclass import configclass

from techtory_cobotta_isaaclab.tasks.manager_based.cobotta.cobotta_env_cfg import CobottaEnvCfg


@configclass
class MyRewardsCfg:
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    # ... your terms


@configclass
class PickHammerEnvCfg(CobottaEnvCfg):
    rewards: MyRewardsCfg = MyRewardsCfg()


cfg = PickHammerEnvCfg()
cfg.scene.num_envs = 256
env = gym.make("Techtory-Cobotta-Base-v0", cfg=cfg)
```

Or take the pieces and build a scene of your own:

```python
from isaaclab.scene import InteractiveSceneCfg
from techtory_cobotta_isaaclab.robot import COBOTTA_CFG, ActionsCfg, ObservationsCfg
from techtory_cobotta_isaaclab.scene import TechtoryCellSceneCfg

@configclass
class MySceneCfg(TechtoryCellSceneCfg):      # the cell, plus your own objects
    cube = my_cube_cfg

@configclass
class RobotOnlySceneCfg(InteractiveSceneCfg):  # or just the robot
    robot = COBOTTA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
```

`ObservationsCfg` reads a scene entity named `hammer`; drop the `hammer_pose` term if your
scene has none. If the robot is registered under a name other than `"robot"`, pass it
through on every action term (`asset_name=`) and give each observation term
`params={"asset_cfg": SceneEntityCfg("cobotta")}`.

## Actions

One 7-element action vector per environment. Index ranges are fixed by the declaration
order of the fields in `ActionsCfg` (`robot/actions.py`).

| Index | Term | Meaning | Units at default scale |
| --- | --- | --- | --- |
| `0:6` | `arm` | `cobotta_pro_joint_1..6` **offset from the home pose** | ±0.5 rad per unit |
| `6` | `gripper` | RG6 opening, **absolute** | `+1` fully open → `−1` fully closed |

* **Arm** is Isaac Lab's stock `JointPositionAction` with `use_default_offset=True`: an
  all-zero action holds the home pose.
* **Gripper** is one scalar for all six RG6 joints. It maps to a `finger_joint` angle,
  `+1 → −0.628 rad` (open) and `−1 → +0.628 rad` (closed), and each joint gets
  `multiplier × finger_joint` from `GRIPPER_MIMIC` (`robot/joints.py`), clamped to its own
  limits. The RG6 is a four-bar linkage modelled as one driven joint plus five followers.
  The URDF's `<mimic>` is stripped so this is the only coupling, which keeps the linkage
  consistent by construction rather than by importer behaviour. A stock
  `JointPositionAction` over the six joints would be a 6-D action that lets a policy tear
  the linkage apart.

```python
import torch

action = torch.zeros(env.unwrapped.num_envs, 7, device=env.unwrapped.device)
action[:, 1] = 0.4     # shoulder (joint 2) +0.2 rad from home
action[:, 6] = -1.0    # close the gripper
obs, reward, terminated, truncated, info = env.step(action)
```

Scales are configurable:

```python
cfg.actions.arm.scale = 0.25
cfg.actions.gripper.scale = -0.4   # narrower range around the offset
```

## Observations

The `policy` observation group concatenates to 41 values, in this order (`robot/observations.py`):

| Slice | Term | Meaning |
| --- | --- | --- |
| `0:6` | `arm_pos` | `cobotta_pro_joint_1..6` positions, rad |
| `6:12` | `arm_vel` | `cobotta_pro_joint_1..6` velocities, rad/s |
| `12:13` | `gripper_pos` | `finger_joint` angle, rad (−0.628 open … +0.628 closed) |
| `13:14` | `gripper_vel` | `finger_joint` rate, rad/s (positive = closing) |
| `14:17` | `ee_pose` (position) | `cobotta_pro_tool0` in the robot base frame, m |
| `17:21` | `ee_pose` (orientation) | `cobotta_pro_tool0` orientation `(x, y, z, w)` |
| **`21:24`** | **`wrist_wrench` (force)** | **`(fx, fy, fz)` at the J6 flange ↔ RG6, N** |
| **`24:27`** | **`wrist_wrench` (torque)** | **`(τx, τy, τz)` at the same point, N·m** |
| `27:30` | `hammer_pose` (position) | hammer in the robot base frame, m |
| `30:34` | `hammer_pose` (orientation) | hammer orientation `(x, y, z, w)` |
| `34:41` | `last_action` | the previous 7-D action |

* `cobotta_pro_tool0` is a virtual frame 0.25 m out from the RG6 base, between the
  fingertips.
* Poses are given in the **robot base frame**, not the world: parallel environments sit on
  a grid in one stage, and world coordinates would encode which environment a sample came
  from.
* Quaternions are **`(x, y, z, w)`**, the Isaac Lab 3.x order, not 2.x's `(w, x, y, z)`.
* Only `finger_joint` is observed. The other five RG6 joints carry
  `multiplier × finger_joint`, so they add no information.
* Pose and wrench are simulator ground truth: noise-free, no bias, no bandwidth limit.

Individual terms are importable if you want a different grouping:

```python
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from techtory_cobotta_isaaclab.robot import observations as cobotta_obs

@configclass
class CriticCfg(ObsGroup):
    arm = ObsTerm(func=cobotta_obs.arm_joint_pos)
    ft = ObsTerm(
        func=cobotta_obs.wrist_wrench,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=["base_link"]), "negate": True},
    )
```

## The wrist force/torque sensor

`wrist_wrench` reads Isaac Lab's **`JointWrenchSensor`** (scene entity `wrist_ft`) at the
incoming joint of the RG6 `base_link`: the constraint wrench the solver applies in `joint_rg6`
(J6 flange → gripper) to hold everything distal to it. That is where a real wrist F/T sensor
bolts in. On PhysX it is the same table `techtory_cobotta_isaacsim/spawners/ft_sensor.py`
reads with `get_measured_joint_forces()`; on Newton it is `body_parent_f`. The MuJoCo port
(`techtory_cobotta_mujoco`) places its `force`/`torque` sensor pair at the same interface.

| | |
| --- | --- |
| **Source** | `env.scene["wrist_ft"].data.force` / `.torque`, entry for body `base_link` |
| **Layout** | `(fx, fy, fz, τx, τy, τz)`, N and N·m |
| **Frame** | `incoming_joint_frame`: the child-side joint frame, torque about the joint anchor |
| **Sign** | `negate=True` (default): force the **environment applies to the tool** |
| **At rest, home pose** | Measured on Newton: F = (−0.09, 0.01, +9.84) N, \|T\| = 0.004 N·m |
| **Rate** | Every policy step (50 Hz); the underlying value updates every physics step (200 Hz) |

**Why `joint_rg6` is a locked revolute joint, not a fixed one.** The description mounts the
gripper with a fixed joint. Isaac Lab removed the old `ArticulationData.body_incoming_joint_wrench_b`
in favour of `JointWrenchSensor`, and on Newton that sensor skips `FIXED` joints: there is no
reportable joint at the flange. The asset builder therefore turns `joint_rg6` into a revolute
joint limited to ±1 mrad about the flange axis, centred by a soft drive (`wrist_lock` in
`robot_cfg.py`). It behaves as rigid, it is not part of the action, and the stock sensor then
reports the flange wrench on every backend. The rest pose above shows it costs nothing: the
reading matches the fixed-joint PhysX baseline.

**Sign convention.** Raw PhysX gives the force the wrist exerts *on the tool*; at rest it
holds the tool up, so it points against gravity. The default `negate=True` matches
`FT_NEGATE = True` in `techtory_cobotta_isaacsim/scripts/main.py`. Pass `negate=False` for
the raw sense.

**Baseline.** `techtory_cobotta_isaacsim` measured the wrist row at **9.809 N** on its Z axis at
rest (1.0 kg = 0.7 kg base + 6 × 0.05 kg linkage), with torques of 0.003–0.005 N·m because in
the home pose the gripper's centre of mass sits essentially on the J6 axis
(`plan-ft-implement.md` §1). This package reads **9.84 N and 0.004 N·m** on Newton. +Fz is the
tool's weight in the joint frame, because the gripper hangs down in the home pose. As the arm
moves, the weight shifts between axes: bending joint 2 by +0.3 rad reads F = (−3.03, −0.34, 9.89) N.

**Frame caveat.** The Z magnitude alone will not catch a rotated frame. Push the gripper
along a known direction and confirm X/Y before trusting them downstream.

**Taring.** `bias=(fx, fy, fz, τx, τy, τz)` is subtracted after the sign flip. Pass the
at-rest reading to remove the tool weight, as a real F/T pipeline would. It is a fixed
offset: once the wrist rotates, the tool's weight moves between axes and a fixed tare no
longer cancels it.

**What it does not see: a grasped payload.** An object held in the jaws loads the gripper
through pad ↔ object *contact*, and PhysX leaves that out of the joint-reaction table. Lift
the hammer and this reading does not move (verified in `techtory_cobotta_isaacsim`,
`plan-ft-implement.md` §7; IsaacLab issue #1092). Loads that travel through the gripper's own
links, such as pushing the fingers into the shelf, do show up. For payload weight,
`techtory_cobotta_isaacsim/spawners/grip_contact_sensor.py` applies Newton's second law to
the object (`m·(g − a)`); the hammer pose in the observation is the input such a term would
need.

**Reading it outside the observation:**

```python
sensor = env.unwrapped.scene["wrist_ft"]
idx = sensor.find_bodies("base_link")[0][0]
force = -sensor.data.force.torch[:, idx, :]     # [num_envs, 3], environment -> tool
torque = -sensor.data.torque.torch[:, idx, :]   # [num_envs, 3]
```

The sensor reports every movable joint of the robot (`sensor.body_names` lists their child
bodies), so the other entries are free joint-reaction readings for the arm too.

## Running

```bash
python scripts/play.py --physics newton_mjwarp --viz newton_gl   # Newton + GL window (kitless)
python scripts/play.py --physics newton_mjwarp --viz none        # Newton, printed readings only
python scripts/play.py --physics newton_mjwarp                   # same: newton_gl is the default window on Newton
python scripts/play.py                                           # PhysX + Kit viewport (needs Kit)
python scripts/play.py --num_envs 4 --phase_seconds 1.5
```

`play.py` holds each command for `--phase_seconds`: settle, every arm joint `+0.3 rad` then
`−0.3 rad` from home, home, then the gripper open, closed and half-open. It prints the arm,
`finger_joint`, the tool position and the wrist wrench as each phase ends. Measured on Newton,
one environment, `--phase_seconds 0.5`:

```
  settle         q=[-0.00 +0.35 +1.31 -0.00 +1.48 +0.00] grip=+0.000 ee=[+0.562 +0.120 +0.241] | F=(  -0.09   +0.01   +9.84) |F|=  9.84 N  |T|=0.004 N.m
  joint 1 +      q=[+0.30 +0.35 +1.31 -0.00 +1.48 -0.00] grip=+0.000 ee=[+0.501 +0.280 +0.241] | F=(  -0.03   +0.29   +9.83) |F|=  9.83 N  |T|=0.015 N.m
  joint 2 +      q=[-0.00 +0.66 +1.31 -0.00 +1.48 +0.00] grip=+0.000 ee=[+0.546 +0.120 +0.073] | F=(  -3.03   -0.34   +9.89) |F|= 10.35 N  |T|=0.133 N.m
  ...
  joint 6 -      q=[+0.00 +0.35 +1.31 -0.00 +1.48 -0.30] grip=+0.000 ee=[+0.562 +0.120 +0.241] | F=(  -0.08   +0.02  +10.00) |F|= 10.00 N  |T|=0.004 N.m
  home           q=[-0.00 +0.35 +1.31 -0.00 +1.48 +0.00] grip=+0.000 ee=[+0.562 +0.120 +0.240] | F=(  -0.10   -0.03   +9.82) |F|=  9.82 N  |T|=0.005 N.m
  gripper open   q=[+0.00 +0.35 +1.31 -0.00 +1.48 -0.00] grip=-0.544 ee=[+0.562 +0.120 +0.240] | F=(  -0.09   +0.01  +10.00) |F|= 10.00 N  |T|=0.004 N.m
  gripper close  q=[+0.00 +0.35 +1.31 -0.00 +1.48 -0.00] grip=+0.397 ee=[+0.562 +0.120 +0.240] | F=(  -0.09   +0.01   +9.91) |F|=  9.91 N  |T|=0.005 N.m
```

Every arm joint reaches its ±0.30 rad target. At 0.5 s per phase the gripper stops short of
±0.628 (−0.544 / +0.397), because its 0.6 rad/s speed cap needs about 1 s for the full travel.
The default 2 s phases reach it.

`--headless` from Isaac Lab 2.x no longer exists; `--viz none` replaces it. Other visualizers:
`--viz viser` (browser), `--viz kit` (needs Kit), `--viz kit,newton_gl`. The camera pose comes
from `sim.default_visualizer_cfg` in `cobotta_env_cfg.py`, so every visualizer starts at the
same view.

Dummy agents and training (append `physics=newton_mjwarp` for Newton, as below):

```bash
python scripts/zero_agent.py   --task Techtory-Cobotta-Base-v0 --num_envs 4   --viz none physics=newton_mjwarp
python scripts/random_agent.py --task Techtory-Cobotta-Base-v0 --num_envs 4   --viz none physics=newton_mjwarp
python scripts/rsl_rl/train.py --task Techtory-Cobotta-Base-v0 --num_envs 256 --viz none physics=newton_mjwarp
python scripts/sb3/train.py    --task Techtory-Cobotta-Base-v0 --num_envs 256 --viz none physics=newton_mjwarp
python scripts/rsl_rl/play.py  --task Techtory-Cobotta-Base-v0 --num_envs 4   physics=newton_mjwarp
```

On Newton in `env_isaaclab`, `zero_agent`, `random_agent`, a one-iteration `rsl_rl/train`
(`--max_iterations 1`) and a one-iteration `sb3/train` all run to completion. The `play`
scripts for trained checkpoints (`rsl_rl/play.py`, `sb3/play.py`) are **not verified**.

These scripts come from `roxfr3_isaaclab`, which targets an older Isaac Lab. Two things
broke against the current checkout and are fixed here. `add_launcher_args` and
`launch_simulation` moved from `isaaclab_tasks.utils` to `isaaclab.app`. SB3's
`policy_kwargs` must be a YAML mapping (`activation_fn: 'nn.ELU'`), not a `"dict(...)"`
string. `roxfr3_isaaclab` itself still has both. Current Isaac Lab also ships unified
entrypoints (`isaaclab_rl.entrypoints`: `run_train_cli`, `run_play_cli`, …), which would be
the lower-maintenance base if these scripts drift again.

`Techtory-Cobotta-Base-v0` has no reward, so training against it only proves the plumbing.

### Physics backends

`CobottaPhysicsCfg` (`cobotta_env_cfg.py`) declares:

| Name | Backend | Status here |
| --- | --- | --- |
| `newton_mjwarp` | Newton, MuJoCo Warp solver, kitless | **Verified** (`env_isaaclab`) |
| `physx` (default) | Isaac Sim PhysX with Kit, OvPhysX without | Configured; not verified. `env_isaaclab` lacks the optional `ovphysx` wheel (`pip install --extra-index-url https://pypi.nvidia.com ovphysx`) |
| `isaacsim_physx` / `ovphysx` | Either PhysX explicitly | as above |

Select one with `play.py --physics NAME`, or the Hydra token `physics=NAME` on any script:

```bash
python scripts/play.py --physics newton_mjwarp --viz newton_gl
python scripts/zero_agent.py --task Techtory-Cobotta-Base-v0 physics=newton_mjwarp --viz none
python scripts/rsl_rl/train.py --task Techtory-Cobotta-Base-v0 physics=newton_mjwarp --viz none
```

**`play.py --physics` is the preset, not Isaac Lab's launcher override.** Scripts that pass
`--physics` straight to the launcher get `make_physics_cfg(NAME)`: every physics config is
swapped for a bare `NewtonCfg()`, which discards this env's tuned solver (elliptic cones,
`impratio=10`, `implicitfast`, contact buffers sized for the gripper). `play.py` turns
`--physics X` into `physics=X` so the tuned preset is used. On Newton it also switches the
default window from Kit (which cannot run beside a kitless backend) to `newton_gl`.

**What differs on Newton, and why:**

* **The cell is visual-only.** MuJoCo Warp collides every mesh as its convex hull, and the
  cell's hull is a solid 2 m block with the robot inside (the MuJoCo port made the same call).
  PhysX keeps the exact triangle mesh. The robot's working volume does not reach the cell
  walls; the base plate and shelf collide on every backend.
* **The hammer collides as one convex hull.** Newton turns `convexDecomposition` into CoACD
  using builder-wide settings no per-shape attribute reaches. On the hammer that produced a
  near-degenerate piece MuJoCo refused to compile ("mesh surface area is too small"). A single
  hull fills the handle notch, so a pinch grasp on the handle is less realistic on Newton than
  on PhysX. `play.py` does not grasp.
* Both choices are made at spawn time from the active physics config
  (`scene/spawners.py: newton_physics_active()`), so they follow the preset with no scene
  preset of their own.

## Running the tests

```bash
pytest tests -q            # 24 tests with Isaac Lab; 19 without (5 skip). No GPU either way.
```

| File | Covers |
| --- | --- |
| `test_urdf_asset.py` | The committed URDFs are self-contained and carry every correction: one root, no mimics or ROS blocks, local meshes, every link a body, the F/T joint, home pose within limits, shelf layout |
| `test_gripper_mapping.py` | `±1 → open/closed`, the mimic fan-out, per-joint clamping |
| `test_placement.py` | Quaternion helpers, the hammer pose composed from the shelf, env spacing vs the cell's size |
| `test_robot_cfg.py` | `merge_fixed_joints=False`, fixed base, actuator and init-pose coverage (needs Isaac Lab) |

The first three files load only plain-Python modules (`robot/joints.py`,
`scene/placement.py`, `scripts/build_urdf_asset.py`), so they run in any Python 3.12.

## Regenerating the assets

`techtory_cobotta_isaaclab/robot/assets/` holds the robot URDF, the shelf URDF and ~26 MB of
meshes (Cobotta `.dae`, RG6 `.stl`, cell `.STL`), committed so the package works on a clone
with no ROS workspace present. The meshes are tracked with Git LFS (`.gitattributes`).
Regenerate only when the description changes:

```bash
python scripts/build_urdf_asset.py
```

No ROS and no Isaac Sim are needed. The inputs are the already-flattened URDFs in
`techtory_cobotta_isaacsim/assets/`. `package://` mesh URIs resolve against
`<workspace>/install` and `<workspace>/src`; override with `--search-path` (repeatable) or
`TECHTORY_SEARCH_PATH`. Isaac Lab converts the URDFs and meshes to USD on first launch and
caches them under the system temp dir; nothing in this repo needs rebuilding for that.

The builder applies the corrections the shared description cannot carry, because they are
specific to a physics simulator:

* **Drops the orphan `world` link.** The flattened URDF keeps `<link name="world"/>` but its
  joint is commented out, so it is a second root.
* **Strips `<ros2_control>`, `<gazebo>` and `<transmission>`.**
* **Drops the five RG6 `<mimic>`s.** The coupling moves to `GRIPPER_MIMIC` and the gripper
  action. `test_urdf_asset.py` checks the table against the source URDF.
* **Gives `cobotta_pro_tool0` a 1 g inertial.** Fixed-joint merging is off, so every link has
  to be a real body.
* **Turns `joint_rg6` into a revolute joint locked to ±1 mrad**, so the wrist F/T sensor can
  report it on Newton (see [The wrist force/torque sensor](#the-wrist-forcetorque-sensor)).
* **Re-prefixes the shelf** `shelf1_` → `shelf_`, with 1 kg placeholder inertials. The shelf
  is fixed-base, so its mass never matters.
* **Copies the cell and base-plate meshes** and records their bounds in `cell_bounds.json`.

## Physics settings that are not optional

**`merge_fixed_joints=False`.** Merging would delete `cobotta_pro_tool0`, the tool frame the
EE-pose observation reads, and it keeps the body list identical across backends.
`tests/test_robot_cfg.py` asserts it.

**USD schema plugins, without Kit.** The standalone URDF importer applies `PhysxJointAPI` (to
every movable joint) and `NewtonMassAPI` (to every link), and nothing registers either in a
kitless process. The error is "Cannot find a valid schema for the provided schema identifier
…". The package `__init__.py` puts the codeless plugins on `PXR_PLUGINPATH_NAME` at import:
`ovstage`'s `physxSchema` and `newton_usd_schemas`. It has to be `ovstage`'s: the importer's own
`physxSchemaFallback` declares the same types, and the OvPhysX manager would later register
`physxSchema` beside it, declaring every type twice. OpenUSD reads the path only once, when
its schema registry is first built. That is early enough here, and it does nothing when Kit
is installed.

**Self-collisions off, on the PhysX side explicitly.** The URDF converter's
`self_collision=False` writes only `newton:selfCollisionEnabled`; PhysX reads
`physxArticulation:enabledSelfCollisions`, which otherwise defaults to on. The RG6 knuckles
and fingers overlap by design, and with self-collision on the linkage jams. This is the
same reason `techtory_cobotta_isaacsim` calls `disable_articulation_self_collisions()`.

**The cell is an exact triangle mesh on PhysX.** The cell shell fills only ~18 % of its own
convex hull; as a convex hull it is a solid 2 m block with the robot buried inside.
`scene/spawners.py` writes the STL to a cached USD mesh with `pxr` (Isaac Lab's
`MeshConverter` needs Kit) and sets `physics:approximation = "none"` on a static (no rigid
body) collider, which is what `make_cell_collisions_static` does in the Isaac Sim package. On
Newton it is visual-only; see [Physics backends](#physics-backends).

**The hammer is massed, high-friction and, on PhysX, convex-decomposed.** Its convex hull is a
solid wedge that fills the handle notch, so the pads would close on a slope and squeeze it
out. `GraspableUsdFileCfg` ports `configure_graspable_object`: `convexDecomposition` (32 hulls,
PhysX; one hull on Newton), 0.3 kg, and friction 1.2/1.1 (PhysX averages the two materials in
a contact). The hammer file is a saved Kit stage in millimetres, with its unit fix-up authored
as xform ops on the hammer prim. The spawner references that prim one level below a clean,
identity-transform rigid-body root, because Isaac Lab rewrites the spawn root's xform ops and
bakes `scale:unitsResolve` into them. With the ops on the root the hammer was scaled twice,
and MuJoCo rejected it as "mesh volume is too small".

**Arm gains hold placeholder dynamics.** `denso_robot_descriptions` gives every arm link
1 kg with an identity inertia tensor and every joint `effort="1"`. The arm actuator
(stiffness 4000, damping 200, effort 300 N·m, 600 on joint 2, 400 on joint 3) comes from
`techtory_cobotta_mujoco/config/actuators.yaml` and is chosen to hold the arm, not to model
the drive train. Joint efforts are therefore meaningless. The wrist wrench is unaffected:
it only sees what is distal to the flange.

**Gripper gains come from the Isaac Sim tuning, converted.** `spawn_robot.py` writes USD
drive gains, which are **per degree** (stiffness 3.0, damping 0.1). Isaac Lab gains are per
radian, so they appear here multiplied by 180/π (≈172 and ≈5.7). Effort 5 N·m (≈60 N at the
jaw), armature 0.15, friction 0.4, speed cap 0.6 rad/s.

**Timestep.** `sim.dt = 1/200`, `decimation = 4`: a 50 Hz policy. The fine step is for the
50 g RG6 linkage links, whose stiff contacts ring at coarser steps. That ringing then
shows up in the wrist wrench.

## Verifying a fresh install

On a new machine, confirm in order:

1. `pytest tests -q`: 24 passed with Isaac Lab installed (19 passed, 5 skipped without).
2. `python scripts/list_envs.py` lists `Techtory-Cobotta-Base-v0`.
3. `python scripts/play.py --physics newton_mjwarp --viz newton_gl`: the robot stands on the
   base plate in the home pose without sagging, and the hammer settles on the shelf's middle
   board.
4. The `settle` line reads `|F| ≈ 9.8 N` and `|T|` near zero (9.84 N / 0.004 N·m here).
5. Every `joint n ±` phase moves its joint by about ±0.3 rad.
6. `python scripts/zero_agent.py --task Techtory-Cobotta-Base-v0 physics=newton_mjwarp --num_envs 4 --viz none`
   runs without NaNs, and `rsl_rl/train.py … --max_iterations 1` completes.

Still unchecked: the X/Y axes of the wrist wrench against a known push (the Z magnitude alone
cannot reveal a rotated frame), and everything on PhysX.

Converted assets are cached: the URDFs by Isaac Lab (a `urdf_import_*` directory in the
temp dir, regenerated per run) and the cell mesh and hammer wrapper by this package
(`$TMPDIR/IsaacLab/techtory_cobotta/`, reused while the sources are unchanged). Delete the
latter to force a rebuild.

## Layout

Following the [Isaac Lab project structure](https://isaac-sim.github.io/IsaacLab/v3.0.0-beta2/source/overview/own-project/project_structure.html):

```
techtory_cobotta_isaaclab/
├── scripts/
│   ├── build_urdf_asset.py     URDFs -> self-contained assets, ROS-free
│   ├── play.py                 per-joint demo with wrist F/T readout
│   ├── list_envs.py, zero_agent.py, random_agent.py
│   ├── rsl_rl/                 train.py, play.py
│   └── sb3/                    train.py, play.py
├── source/techtory_cobotta_isaaclab/
│   └── techtory_cobotta_isaaclab/
│       ├── robot/
│       │   ├── joints.py           joint names, frames, home pose, gripper coupling (no Isaac)
│       │   ├── robot_cfg.py        COBOTTA_CFG: actuators, import settings, start state
│       │   ├── actions.py          GripperAction + the 7-D ActionsCfg
│       │   ├── observations.py     terms incl. wrist_wrench, the 41-D ObservationsCfg
│       │   └── assets/             committed URDFs and meshes
│       ├── scene/
│       │   ├── placement.py        where everything sits in the cell (no Isaac)
│       │   ├── spawners.py         static triangle-mesh cell, graspable hammer
│       │   └── scene_cfg.py        TechtoryCellSceneCfg
│       └── tasks/manager_based/cobotta/
│           ├── cobotta_env_cfg.py  CobottaEnvCfg: reward-free, meant to be subclassed
│           └── agents/             PPO configs
└── tests/
```

Retuning the robot means editing `robot_cfg.py`; moving things in the cell means editing
`scene/placement.py`. Nothing else should need to change.

## Model scope

Link transforms, joint limits and meshes come from `denso_robot_descriptions`,
`onrobot_rg_description` and `techtory_cell_description`. Masses, inertias and servo gains
are simulation placeholders and tuning, **not** an identification of the physical robot, so
wrench magnitudes are indicative rather than calibrated. The robot's real safety limits and
hardware driver are separate concerns.

The hammer is the one asset not committed here. It exists only as a USD, and this repo
keeps USD out of git, so it is loaded from `techtory_cobotta_isaacsim/assets/objects/hammer1.usd`.
That file is self-contained. Point `TECHTORY_HAMMER_USD` at another copy if the packages are
not side by side.

Not modelled: the RealSense camera, any payload channel on the F/T sensor (see above),
sensor noise and bias drift.
