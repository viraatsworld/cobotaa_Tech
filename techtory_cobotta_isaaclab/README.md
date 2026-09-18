# techtory_cobotta_isaaclab

An Isaac Lab (3.0) environment of the Techtory cell: a Denso Cobotta Pro 900 (cvrb0609) with an
OnRobot RG6 gripper and a wrist force/torque sensor, working at a shelf that holds a hammer and a
soda can. The robot, gripper, cell, shelf and objects are loaded from the same USDs that
[`techtory_cobotta_isaacsim`](../techtory_cobotta_isaacsim) uses, with the same tuning. There is no
ROS in this package.

The registered task is:

- `TechtoryCobottaIsaaclab-Base-COBOTTA` — the cell with the robot and no reward: a base to build tasks on.

## Virtual environment & installation

This package uses [uv](https://docs.astral.sh/uv/) to manage its Python virtual environment and dependencies.

First, set up and synchronize the virtual environment:

```bash
cd src/dynamic_planning_demo/techtory_cobotta_isaaclab

# Create and synchronize the virtual environment (.venv)
uv sync
```

To also install the optional `isaacsim` extra (required for the Isaac Sim PhysX physics backend) or developer dependencies:

```bash
# Sync with Isaac Sim runtime dependencies
uv sync --extra isaacsim

# Sync with development dependencies (pytest, ruff, etc.)
uv sync --group dev

# Or sync everything (all extras and dev dependencies)
uv sync --all-extras --group dev
```

`uv sync` resolves dependencies against `uv.lock` and installs the package in editable mode. The install registers the package under the `isaaclab.tasks` entry point, which is how the `isaaclab` CLI discovers its tasks. Because it is editable, code changes take effect without reinstalling.

This is not a colcon package: it has only a `pyproject.toml`, so `colcon build` leaves it alone.

## Run it

All scripts and CLI commands are run using `uv run` from the package directory. To run tasks with the Isaac Sim PhysX physics backend, pass `--extra isaacsim`:

```bash
cd src/dynamic_planning_demo/techtory_cobotta_isaaclab

# Isaac Lab CLI: Run random agent with Isaac Sim PhysX
uv run --extra isaacsim isaaclab random_agent --task TechtoryCobottaIsaaclab-Base-COBOTTA --num_envs 16 physics=isaacsim_physx --viz kit

# Isaac Lab CLI: Run zero agent (Newton by default, or with Isaac Sim PhysX)
uv run isaaclab zero_agent --task TechtoryCobottaIsaaclab-Base-COBOTTA --num_envs 16
uv run --extra isaacsim isaaclab zero_agent --task TechtoryCobottaIsaaclab-Base-COBOTTA --num_envs 16 physics=isaacsim_physx --viz kit

# Training and play
uv run --extra isaacsim isaaclab train --rl_library rsl_rl --task TechtoryCobottaIsaaclab-Base-COBOTTA
uv run --extra isaacsim isaaclab play  --rl_library rsl_rl --task TechtoryCobottaIsaaclab-Base-COBOTTA --checkpoint latest

# This package's environments and their presets
uv run python scripts/list_envs.py --show_presets

# Drive each arm joint in turn, then the gripper; prints joints, TCP, wrist wrench, objects
uv run python scripts/play.py  --viz newton_gl                      # Isaac Sim viewport
uv run python scripts/play.py --viz none             # headless, printed readout only

# Self-test of the F/T sensor and the grasp; prints PASS or FAIL (runs on Isaac Sim PhysX)
uv run --extra isaacsim python scripts/check_ft_payload.py
```

The base task has no reward, so `train` only exercises the plumbing. Subclass it with your
rewards; see [Using the robot in an RL agent](#using-the-robot-in-an-rl-agent). Agent configs for
rsl_rl, rl_games, skrl and sb3 are registered.

**Physics: Newton by default, Isaac Sim PhysX for the gripper.** `physics=` picks one of:

- **`newton_mjwarp`** (default; `newton_wraps` is an alias) runs without Kit. The arm uses softer
  gains (`ARM_ACTUATOR_NEWTON`), because MuJoCo-Warp does not enforce the joint velocity limit that
  the USD's very stiff gains rely on. Newton cannot follow the RG6's parallel motion, which is made of
  PhysX mimic joints. Its joint-wrench sensor skips links attached by a fixed joint, and the F/T
  sensor sits on exactly such a joint, so `wrist_wrench` reads zero; the observation stays 48-D.
  The cell's own colliders are authored on Xforms, which Newton ignores; the shelf's boxes collide.
- **`isaacsim_physx`** is what the Isaac Sim demo ran on and what the gripper was tuned against:
  use it for grasping, wrist-wrench readings and contact with the cell. `check_ft_payload.py`
  always runs on it.
- **OvPhysX** is not offered. It simulates the robot and the wrist sensor correctly, but does not
  build the RG6's colliders (`PhysxMeshMergeCollisionAPI`), so the fingers close through anything
  and nothing can be held. That also rules out Isaac Lab's `physx` auto preset, which picks OvPhysX
  whenever Kit is not otherwise needed.

## The scene

One cell per environment, placed at the environment origin. All poses are in the cell frame, the
same frame as the workcell URDF and the Isaac Sim demo (`scene/layout.py`).

| Scene entity | What | Where | Notes |
| --- | --- | --- | --- |
| `workcell` | `techtory_cell.usd` | origin | static; exact (triangle-mesh) colliders |
| `robot` | `cvrb0609_with_graph2.usd` | `(-0.275, -0.24, 0.96)`, yaw 90° | on the base plate, in the demo's home pose |
| `shelf` | `shelf.usd` | `(0.61, 0.27, 0.94)`, yaw 90° | static; boards' tops at 0.16 / 0.36 / 0.80 m |
| `hammer` | `hammer1.usd` | middle board | 0.3 kg, convex decomposition |
| `soda_can` | `soda_can.usd` | middle board, robot end | 0.35 kg, ~0.8 m from the robot base |
| `wrist_ft` | `JointWrenchSensor` | on the robot | the wrist F/T sensor, [see below](#force--torque-sensor) |

The simulation uses `sim.dt = 0.01` s with `decimation = 2`, so the policy runs at 50 Hz. Episodes
last 20 s. Environments are 3 m apart, because the cell is about 2.2 m square. A reset puts the
robot back in its home pose and the objects back on the shelf.

**Assets.** The 28 USD files the scene needs (44 MB) are copied, byte-identical, into
`src/techtory_cobotta_isaaclab/assets/usd`, in the same layout as `techtory_cobotta_isaacsim/assets`.
When those assets change, refresh the copy with:

```bash
uv run python scripts/sync_assets.py            # copy what changed
uv run python scripts/sync_assets.py --check    # report drift only
```

The files are never edited. The Isaac Sim demo corrects them at runtime, and so does this package,
at spawn time (`spawners/`):

| Isaac Sim demo (`spawners/*.py`) | Here |
| --- | --- |
| `fix_gripper_collisions` | `spawn_cobotta_rg6`: widens the six RG6 link colliders' mesh-merge collections, so the fingers have shapes |
| `disable_articulation_self_collisions` | `enabled_self_collisions=False` on the articulation |
| `configure_gripper_drive`, `stabilize_gripper_joints` | the `gripper_drive` / `gripper_mimic` actuators in `robot/robot_cfg.py` |
| `add_grip_friction` | `spawn_cobotta_rg6`: 1.2 / 1.1 friction material on the pads |
| `make_cell_collisions_static` | `spawn_static_usd` (cell and shelf): bodies removed, joints deactivated, exact meshes for the cell |
| `configure_graspable_object` | `spawn_graspable_usd` plus the object configs: mass, friction, convex decomposition for the hammer |

Deliberate differences from the demo:

- **No ROS bridge or OmniGraph.** The robot USD's ROS action graph lies outside its default prim,
  so it is not loaded.
- **Hammer height.** The hammer is placed 3 mm above the board. The demo drops it 6.5 cm, so it
  lands differently on every reset.
- **J6 effort limit.** J6 gets 60 N·m like J1–J5. The USD authors 1 N·m there, a leftover of the
  URDF's `effort="1"` placeholder.

## Using the robot in an RL agent

Subclass the base environment and add what makes it a task:

```python
import gymnasium as gym

from isaaclab.envs import mdp
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils.configclass import configclass

from techtory_cobotta_isaaclab.tasks.base.config.cobotta.env_cfg import BaseEnvCfg


@configclass
class MyRewardsCfg:
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    # ... your terms


@configclass
class PickCanEnvCfg(BaseEnvCfg):
    rewards: MyRewardsCfg = MyRewardsCfg()


cfg = PickCanEnvCfg()
cfg.scene.num_envs = 256
env = gym.make("TechtoryCobottaIsaaclab-Base-COBOTTA", cfg=cfg)
```

Or build a scene of your own from the pieces:

```python
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass

from techtory_cobotta_isaaclab.robot import COBOTTA_RG6_CFG, ActionsCfg, ObservationsCfg
from techtory_cobotta_isaaclab.scene.scene_cfg import SODA_CAN_CFG, WORKCELL_CFG, WRIST_FT_CFG


@configclass
class MySceneCfg(InteractiveSceneCfg):
    workcell = WORKCELL_CFG
    robot = COBOTTA_RG6_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    soda_can = SODA_CAN_CFG
    wrist_ft = WRIST_FT_CFG  # needs the robot at {ENV_REGEX_NS}/Robot
```

The action and observation configs assume the scene names `robot`, `wrist_ft`, `hammer` and
`soda_can`. If you rename one, pass the new name on each action term (`asset_name=`) and in the
observation terms' `SceneEntityCfg`s. Drop the object-pose terms if your scene has other objects.

## Sending commands

One 7-element action vector per environment. The index ranges are fixed by the declaration order
of the fields in `ActionsCfg` (`robot/actions.py`).

| Index | Term | Meaning | Range at default scale |
| --- | --- | --- | --- |
| `0:6` | arm | `cobotta_pro_joint_1..6` offset from the home pose | ±0.5 rad |
| `6` | gripper | jaw command | +1 fully open … −1 fully closed |

```python
import torch

action = torch.zeros(env.unwrapped.num_envs, 7, device=env.unwrapped.device)
action[:, 1] = 0.4     # shoulder (joint 2) 0.2 rad past home
action[:, 6] = -1.0    # close the gripper
obs, reward, terminated, truncated, info = env.step(action)
```

Scales are configurable:

```python
cfg.actions.arm.scale = 0.25
```

**The arm** takes joint-position targets through the robot USD's drive gains, converted to SI.
Those gains are very stiff, so the drive is effectively torque-limited at 60 N·m and velocity-limited
at MoveIt's limits (`techtory_cobotta_moveit/config/joint_limits.yaml`: 0.33–0.60 rad/s). A command
therefore moves the arm at constant speed and stops on target. A 0.5 rad step takes about 1.5 s.
The home pose is the Isaac Sim demo's (j2 = 0.349, j3 = 1.309, j5 = 1.484 rad, the rest 0), with the
gripper pointing down. The all-zero pose is not usable: it points the arm straight up into the roof.

**The gripper** takes an absolute command, the same sense as `roxfr3_isaaclab`'s gripper. It sets
the position target of `finger_joint`, the one driven RG6 joint; the other five are PhysX mimic
joints that follow it. The target is clipped to ±0.62 rad, just inside the joint's ±0.628.

| Command | `finger_joint` target [rad] | Opening between the pads |
| --- | --- | --- |
| +1 | −0.62 | ~151 mm |
| +0.5 | −0.31 | ~129 mm |
| 0 | 0 | ~94 mm |
| −0.5 | +0.31 | ~49 mm |
| −1 | +0.62 | ~0 mm |

The openings are the demo's measurements (`techtory_cobotta_isaacsim/plan-gripper-effort.md`). The
drive is force-limited: 5 N·m on `finger_joint`, about 62 N at the pads. Closing on an object
therefore stops the jaw short of its target rather than crushing the object. On the soda can the
jaw stalls at +0.26 rad, with the finger quiet at 0.04 rad/s — below the 0.1 rad/s the ROS gripper
controller needs to declare a stall.

## Reading state

The `policy` observation group concatenates to 48 values, in this order:

| Slice | Term | Meaning |
| --- | --- | --- |
| `0:6` | `arm_pos` | `cobotta_pro_joint_1..6`, rad |
| `6:12` | `arm_vel` | `cobotta_pro_joint_1..6`, rad/s |
| `12:13` | `gripper_pos` | `finger_joint`, rad (−0.628 open … +0.628 closed) |
| `13:14` | `gripper_vel` | `finger_joint`, rad/s |
| `14:17` | `tcp_pose` | TCP position, m, robot base frame |
| `17:21` | `tcp_pose` | TCP orientation `(x, y, z, w)`, robot base frame |
| `21:24` | `wrist_wrench` | wrist force `(Fx, Fy, Fz)`, N, F/T frame |
| `24:27` | `wrist_wrench` | wrist torque `(Tx, Ty, Tz)`, N·m, F/T frame |
| `27:30` | `hammer_pose` | hammer position, m, robot base frame |
| `30:34` | `hammer_pose` | hammer orientation `(x, y, z, w)` |
| `34:37` | `soda_can_pose` | soda can position, m, robot base frame |
| `37:41` | `soda_can_pose` | soda can orientation `(x, y, z, w)` |
| `41:48` | `last_action` | the previous 7-D action |

Frames and conventions:

- **Quaternions** are `(x, y, z, w)`, the Isaac Lab 3.x order, not 2.x's `(w, x, y, z)`. They are
  returned with `w ≥ 0`.
- **Robot base frame** is `cobotta_pro_base_link`. Positions are relative to the robot, not the
  world: parallel environments share one stage, so world coordinates would tell a policy which
  environment it is in.
- **The TCP** is MoveIt's `cobotta_pro_tool0`: 0.25 m along the RG6's approach axis, between the
  pads. At home it is at `(0.563, 0.120, 0.246)` m.

The individual terms are importable, for groups of your own. `payload_wrench` is not in the default
group; see [Force / torque sensor](#force--torque-sensor).

```python
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.utils.configclass import configclass

from techtory_cobotta_isaaclab.robot import FT_ROBOT_ENTITY, FT_SENSOR_ENTITY, payload_wrench, wrist_wrench


@configclass
class CriticCfg(ObsGroup):
    wrist = ObsTerm(func=wrist_wrench, params={"sensor_cfg": FT_SENSOR_ENTITY})
    grip_load = ObsTerm(func=payload_wrench, params={"robot_cfg": FT_ROBOT_ENTITY})
```

Pass scene selections through `params`, as above. The observation manager resolves names such as
`body_names` only there, not in a function's default arguments.

For anything not covered, go to the assets directly. Data properties are `ProxyArray`s; use
`.torch`:

```python
robot = env.unwrapped.scene["robot"]
robot.data.joint_pos.torch                       # [num_envs, 12]: arm, finger_joint and mimic joints
ids, names = robot.find_joints(["cobotta_pro_joint_.*"], preserve_order=True)
```

## Force / torque sensor

**Where.** Between the arm flange and the gripper: the fixed joint `gripper_joint`, from
`cobotta_pro_J6` to the RG6 `base_link`. That is where a physical wrist F/T sensor bolts in.

**Frame.** The RG6 `base_link` frame (ROS: `onrobot_rg6_base_link`). Its origin is on the flange
and +z points out of the flange towards the fingers. Torque is taken about that origin.

**Sign.** Reported as the **load the gripper puts on the arm**, which is what a physical sensor
reads. Hanging still, the reading is the gripper's weight pointing along gravity; a held object
adds its own.

**Source.** Isaac Lab's `JointWrenchSensor` (scene entity `wrist_ft`), read through
`robot/observations.py:wrist_wrench`. It is simulator ground truth: no noise, bias or bandwidth
limit, and it is not tared.

Measured with `scripts/check_ft_payload.py` on Isaac Sim PhysX:

| Situation | Wrist reading |
| --- | --- |
| Home, at rest (gripper 1.0 kg as authored, +z pointing down) | F = (0, 0, +9.81) N, T ≈ (−0.001, −0.006, 0) N·m |
| Wrist (j5) tilted +0.3 rad | F = (−2.90, 0, +9.37) N, T = (0, −0.306, 0) N·m — matches the gripper links' masses and centres of mass exactly |
| Holding the 0.35 kg soda can | ΔF = (0, 0, +3.38) N: 98 % of its weight (3.43 N) |
| Shoulder lifting the can by 0.2 rad | load along gravity swings between 1.5 and 17.1 N during the move, then settles at 13.3 N = (1.0 + 0.35) kg × g |

**A held object is in the reading.** PhysX's incoming joint wrench includes the load of an object
held in the gripper. (The Isaac Sim demo's notes, from Isaac Sim 6.0.1, found the opposite, and it
added a `/grip_contact` channel to compensate. On Isaac Sim 6.1 the check above measures the
payload in the joint wrench.) `payload_wrench` is a port of that channel: an estimate of the part of
the wrist wrench due to held objects, from Newton's second law on each object. It is **already
included** in `wrist_wrench`, so do not add the two. Use it on its own, e.g. to separate the payload
from the gripper's weight without taring.

**Taring.** The reading includes the gripper's own weight. To get only what the tool touches,
subtract the reading taken at the same pose with nothing held, as a physical sensor's bias removal
would.

**Raw sensor data.** The raw sensor data has the *opposite* sign: it is the arm's support of the
gripper.

```python
sensor = env.unwrapped.scene["wrist_ft"]
row = sensor.find_bodies("base_link")[0][0]
load_force = -sensor.data.force.torch[:, row]     # [num_envs, 3], N, as wrist_wrench reports it
load_torque = -sensor.data.torque.torch[:, row]   # [num_envs, 3], N*m
```

## Project structure

```
src/techtory_cobotta_isaaclab/
├── assets/            # the copied USDs and their paths
├── spawners/          # spawn-time corrections of those USDs
├── robot/             # ArticulationCfg, actions, observations -- the robot on its own
├── scene/             # cell layout (plain numbers) and the scene configs
└── tasks/base/        # the registered task: env config, agent configs
scripts/               # play, check_ft_payload, list_envs, sync_assets
tests/                 # kit-less unit tests
```

Retune the robot in `robot/robot_cfg.py`. Move things in `scene/layout.py`. Add a task family as a
sibling of `tasks/base`, or another robot configuration as a sibling of `tasks/base/config/cobotta`.

## Development

```bash
uv sync --group dev                            # adds pytest and dev tools to the venv
uv run pytest                                  # kit-less, a few seconds
uvx ruff check . && uvx ruff format --check .
uv run --extra isaacsim python scripts/check_ft_payload.py  # the simulation self-test (GPU, about a minute)
```

The unit tests check the configs against the USDs themselves:

- `test_layout.py` checks the cell layout against USD's own composition of the Isaac Sim demo's
  authoring.
- `test_usd_assets.py` checks the asset copy (complete, byte-identical to the originals) and the
  robot structure the configs rely on.
- `test_robot_cfg.py` checks the actuators, the home pose, and the gains' degree-to-radian
  conversion.
- `test_registration.py` checks the task registration and the CLI entry point.

## Troubleshooting

- **pytest fails importing `lark` or `launch`.** A ROS-sourced shell puts ROS's pytest plugins on
  `PYTHONPATH`, and the virtual environment lacks their dependencies. `pyproject.toml` disables them
  (`-p no:launch_testing -p no:launch_ros`), so run pytest via `uv run pytest` from the package directory.
- **"Unresolved reference prim path" warnings.** These come from the Isaac Sim demo's USD files
  (e.g. the RG6's unused `base` prim) and are harmless. Kit mutes them; kit-less tools print them.
- **Objects pass through the fingers.** Check that you are on Isaac Sim PhysX
  (`physics=isaacsim_physx`; the default is Newton). See [Run it](#run-it) for why the others
  cannot grip.
- **The arm is slow.** It runs at MoveIt's joint velocity limits, as on the real robot's MoveIt
  setup. Raise `joint_velocity_limit` in `robot/robot_cfg.py` if a task needs faster motion.
