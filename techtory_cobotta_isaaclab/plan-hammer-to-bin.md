# Plan: RL task — hammer from the cell's work surface into the blue bin

## Context

`techtory_cobotta_isaaclab` has a base env, `TechtoryCobottaIsaaclab-Base-COBOTTA` (PhysX only). It is a
sandbox for driving and testing the robot, so its `RewardsCfg` is empty by design.
Goal: a new, separately registered, trainable RL task where the Cobotta picks the **hammer from a random pose on
the cell's work surface (table top, z = 0.94 in the cell frame)** and drops it into the **blue pallet bin** —
the same job as the real behavior tree `techtory_cobotta_system/trees/techtory_cobotta_system_hammer_to_bin.xml`.
The shelf start pose is out of scope for now. Sim-to-real follows later, so the design keeps what the policy sees and
outputs to things the real robot has.

Decisions:
- **Arm: TCP deltas.** Each step the policy moves a commanded TCP target a little; IK follows it.
- **Gripper: not learned.** A rule closes it (existing force-limited `finger_joint` drive) when the TCP is at the
  handle with the right yaw, and opens it when the whole hammer is over the bin. Binary open/close only.
- **Real speed limits** (MoveIt's 0.33–0.60 rad/s) during training, and **enforced** by the action term, not assumed.
- Hammer pose randomized on the table; bin fixed where the real cell has it.

Measured facts the design rests on:
- **Relative-mode IK drifts.** Isaac Lab's `DifferentialInverseKinematicsAction` builds the command from the
  *measured* TCP pose and the joint target from the *measured* `joint_pos` (`isaaclab/envs/mdp/actions/task_space_actions.py:190-219`).
  With zero action, the target equals the sagged state, so the arm sinks. Isaac Lab's own IK lift task hides this by disabling
  gravity on the arm (`FRANKA_PANDA_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True`), which isn't an option here.
- `reset_root_state_uniform` applies yaw in the **body** frame (`events.py:2175`, `quat_mul(default, delta)`), which
  would tip the flat-lying hammer.
- Hammer bounding box (`hammer1.usd`, body frame): 0.211 × 0.020 × 0.102 m. Footprint diagonal is 0.234 m, smaller than the bin's
  0.37 m interior width, so the hammer fits the bin at **any** yaw. No yaw alignment is needed for the drop, only a
  check that the whole hammer is inside.

## How it fits together (one episode)

reset → hammer at random (x, y, yaw) on the table, arm at home, gripper opening
→ policy steps the TCP target to the handle and turns to the grasp yaw → rule closes the jaw
→ policy lifts the hammer (now "picked"), carries it to a point **above** the bin → rule opens when all hammer corners are inside
→ success when the hammer rests inside the bin for 0.4 s (episode ends).

## As built (2026-10-05)

Implemented as planned below, with these changes. Each was found by measuring in simulation:

- **TCP Jacobian fix.** Isaac Lab's `DifferentialInverseKinematicsAction` shifts the Jacobian to the TCP with
  the offset as given in the body frame, but the Jacobian is in the root frame. With the RG6 pointing down,
  the 0.25 m TCP offset counted as 0.25 m *up*, and the TCP sank onto the table while the target sat 3 cm
  above it. `TopDownTcpTargetAction._compute_frame_jacobian` rotates the offset into the root frame first.
- **Grasp height 25 mm** above the handle centreline (was 11 mm). The RG6's pad tips drop as the jaw closes:
  22 mm above the TCP fully open, 32 mm below it at the handle's width. Lower grasps hit the table first.
  This matches the real `pick` pose at 33 mm above the table.
- **Close condition 8 mm horizontal, 6 mm vertical, 15° yaw** (was a 1.5 cm sphere). Off-centre closes
  shoved the light hammer round before the second pad arrived. "Grasped" also requires the jaw to stall
  within the handle's width (`JAW_ON_HANDLE`, +0.38 to +0.56 rad). The rule reopens after 3 s closed
  without a grasp.
- **Grip 15 N·m** on `finger_joint` in this task (base task: 5 N·m). Near closed, each pad moves about
  79 mm/rad, so 5 N·m is only ~32 N per pad, and the handle slipped out as soon as it left the table.
  15 N·m is ~95 N per pad, inside the RG6's 25–120 N.
- **No collisions with the cell's mesh.** Its 470k-triangle collider overflowed PhysX's GPU collision stack
  at 1024 envs (needed ~600 MB). Even with bigger buffers, about 1 in 250 hammers was thrown in its first
  steps, from poses that were fine in other envs. The task uses `WORKCELL_VISUAL_CFG` plus invisible boxes
  for the table top and the base plate, with default PhysX buffers: 1024/1024 resets are clean.
- **Workspace box** on the commanded TCP (robot base frame x ∈ [−0.20, 0.80], y ∈ [−0.72, 0.27],
  z ∈ [0.005, 0.45]). It stands in for the cell's frame and keeps exploration where the task is.
- **`lost_in_transport`** comes from the gripper rule seeing the jaw close on nothing after a pick, not
  from a dip in "grasped". "Grasped" reads the jaw's speed, which jitters under load and gave false −5s.
- **"Picked"** is measured against the table rest height, which every reset lays the hammer at, instead of
  a height recorded at reset.
- **Spawn area**: robot base frame x ∈ [−0.10, 0.20], y ∈ [−0.60, −0.30] (box centre, any yaw), clear of the
  pallet, shelf and plate by ≥ 1.5 cm at any yaw (`tests/test_hammer_to_bin_layout.py`).
- **Reward weights are per event or per step.** Every term divides by `step_dt`, the time penalty included,
  so −0.005 is per step, as in the reward table.

Measured results:

- `check_hammer_to_bin.py --resets --num_envs 1024`: PASS (tilt ≤ 0.6°, all hammers in their area, jaws open).
- `--hold`: 0.00 mm TCP drift over 10 s.
- Scripted pick-and-place on 64 envs: 62 successes. Both failures were slips in transport.
  Hold drift while holding the hammer ≤ 0.7 mm. Arm joint speed ≤ 1.00× its limit.
- `isaaclab train`: 1024 envs at ~2.15 s per PPO iteration (32 steps), so ~2.4 h for 4000 iterations.

## Implementation

### 0. Keep the repo copy in sync
Write this plan over `techtory_cobotta_isaaclab/plan-hammer-to-bin.md`.

### 1. Layout numbers — `src/techtory_cobotta_isaaclab/scene/layout.py`
Pure-Python constants, same style as the existing ones (cell frame, `(x, y, z, w)`):
- `TABLE_TOP_Z = 0.94` (workcell URDF: plate, shelf and pallet all sit at 0.94).
- `PALLET = Pose((-0.16, 0.3, 0.94))` — from `techtory_cobotta_workcell.urdf.xacro` / `spawn_objects.add_pallet`.
  Interior 0.57 × 0.37 m, floor at +0.015, rim at +0.075 (`assets/urdf/pallet.urdf`). In the robot base frame this is
  (0.54, −0.115), matching `bin_place` in `poses_hammer.yaml`.
- `BIN_FOOTPRINT` — the interior shrunk by a 3 cm margin (0.51 × 0.31 m), used by the release, near-bin and success checks.
- `BIN_TARGET` — transport target: footprint centre, **0.12 m above the rim**. Straight-line shaping then pays for
  carrying the hammer over the wall, not dragging it along the table into it.
- `HAMMER_HALF_EXTENTS` + box centre in the body frame — from the measured bbox, for the hammer's 8 oriented-box corners.
- `HAMMER_ON_TABLE` — lying flat (same orientation as `HAMMER_ON_SHELF`, root at `TABLE_TOP_Z + 0.0133`, 3 mm clearance),
  composed with `HAMMER_BODY_OFFSET`.
- `HAMMER_GRASP_OFFSET` + `HAMMER_GRASP_YAW` — handle grasp point and jaw yaw in the hammer body frame (handle mid-point,
  from the mesh; the handle is 2 cm thick, well inside the RG6's 151 mm opening).
- `HAMMER_SPAWN_RANGE` — x/y box + yaw ∈ [−π, π] on the table, inside reach, clear of the pallet, shelf and base plate.
  It's shrunk by the hammer's half-diagonal (0.117 m), so no yaw causes overlap. Starting candidate in the base frame:
  x ∈ [0.35, 0.60], y ∈ [0.20, 0.45]. **Verify against the workcell mesh** (raycast down at the corners → hits 0.94).

### 2. Pallet asset
- `scripts/sync_assets.py`: add `"objects/pallet.usd"` to `ROOT_USDS`, then run it.
- `assets/__init__.py`: `PALLET_USD` (+ `__all__`, existence check).
- `scene/scene_cfg.py`: `PALLET_CFG = AssetBaseCfg(spawn=StaticUsdFileCfg(...), init_state=PALLET)`.
  `StaticUsdFileCfg` already strips the rigid-body/articulation APIs the pallet USD authors.

### 3. Arm action — `robot/actions.py` (reusable)
`TopDownTcpTargetAction`: subclass of `DifferentialInverseKinematicsAction`. It reuses its frame pose/Jacobian helpers
(with `body_offset=TCP_OFFSET`) and the `DifferentialIKController` in **absolute** pose mode (`use_relative_mode=False`,
`ik_method="dls"`). 4-D action **[dx, dy, dz, dyaw]** in the robot base frame.

Each step (`process_actions`):
1. Clip the raw action to [−1, 1]. The Gaussian policy output is unbounded, and Isaac Lab's `clip` acts on the scaled value only.
2. Scale: **5 mm** per step for translation (0.125 m/s at 25 Hz), and **0.012 rad** per step for yaw (0.3 rad/s, under the slowest
   0.33 rad/s cap; 0.03 rad/step would have been 0.75 rad/s).
3. **Integrate a commanded TCP target**: `target_pos += d`, `target_yaw += dyaw`. Roll and pitch stay fixed at "tool pointing down".
   Clamp the target to within **3 cm / 0.1 rad of the actual TCP**, so it can't run away while the arm is blocked or saturated.
   This is how MoveIt Servo behaves too, which helps deployment.

Each physics step (`apply_actions`):
4. IK on the pose error between the target and the actual TCP gives `joint_des = q + dq`. A steady pose error (gravity sag,
   hammer weight) keeps the joint target offset from the measured state, so the drive holds against gravity instead of
   following the sag.
5. **Rate-limit the joint target**: `target_q = prev_target_q + clamp(joint_des − prev_target_q, ±v_lim · physics_dt)`, with
   `v_lim` = the `_ARM_VELOCITY_LIMIT` values from `robot_cfg.py`. The speed cap is then guaranteed, including near singularities where DLS spikes.

`reset(env_ids)`: target = actual TCP pose at reset; `prev_target_q` = home joints.

### 4. Gripper rule — `tasks/hammer_to_bin/mdp/actions.py`
`ProximityGripperAction` (`ActionTerm`, `action_dim = 0`: the policy gives it no input, and the `ActionManager` slices an empty range).
Per env, in `apply_actions`:
- OPEN → CLOSE when `|TCP − grasp pt| < 1.5 cm` **and** `|Δyaw| < 15°`, where `Δyaw = wrap_to_±π/2(tcp_yaw − handle_yaw)`.
  The error is **wrapped modulo π**, because the handle and the parallel jaw are both symmetric under 180°.
- CLOSED, but the jaw reached `GRIPPER_CLOSED` (closed on nothing) → OPEN once the TCP moves away (missed grasp, retry allowed).
- CLOSED with the hammer → OPEN only when **all 8 oriented-box corners** of the hammer are inside `BIN_FOOTPRINT` (x, y) and the
  hammer's lowest corner is between the rim and rim + 0.15 m. A reference point alone could drop a 21 cm hammer across the rim. After that it stays open.
- Writes `GRIPPER_OPEN` / `GRIPPER_CLOSED` to `finger_joint` (same drive as today). Exposes its state for observations and rewards.
- `reset(env_ids)`: state OPEN, latches cleared.
On the real robot, the same rule runs as a small ROS node sending binary gripper commands.

### 5. Task family `src/techtory_cobotta_isaaclab/tasks/hammer_to_bin/`
Same layout as `tasks/base/`: `mdp/{__init__.py, __init__.pyi, actions.py, events.py, observations.py, rewards.py, terminations.py}`,
`config/cobotta/{__init__.py, env_cfg.py, agents/{__init__.py, rsl_rl_ppo_cfg.py}}`.

**Shared state helpers** (`mdp/observations.py`, used by the rule, rewards and terminations so they always agree):
- `grasp_yaw_error(env)` — `Δyaw` wrapped modulo π as above.
- `hammer_grasped(env)` — rule CLOSED **and** `finger_joint` stalled short of its target (same stall test as
  `payload_wrench` in `robot/observations.py`) **and** the hammer is within jaw radius of the TCP.
- `hammer_picked(env)` — grasped **and** the hammer is lifted **≥ 2.5 cm above its rest height**, recorded at reset. A squeeze on the table is not a pick.
- `hammer_corners_w(env)` — the 8 oriented-box corners. `hammer_in_bin(env)` — all corners inside `BIN_FOOTPRINT`, lowest corner below the rim.

**Scene** `HammerToBinSceneCfg(TechtoryCellSceneCfg)`: add `pallet`; hammer `init_state = HAMMER_ON_TABLE`; `soda_can = None`. Keep the shelf (it's in the real cell).

**Observations** — no raw quaternions in the actor:
- `policy` (actor — only things the real robot can provide): arm joint pos/vel (12); TCP position (3) + TCP yaw as sin/cos (2);
  gripper state from the rule (1); hammer grasp-point position (3) + hammer yaw as sin/cos (2) (from perception on the real robot);
  grasp point − TCP (3); **grasp yaw error as sin(2Δyaw)/cos(2Δyaw)** (2, smooth across the 180° symmetry);
  `BIN_TARGET` − hammer (3); last action (4).
- `critic` (privileged, sim only): hammer lin/ang velocity, full hammer quaternion (canonicalized, `quat_unique`), `wrist_wrench`,
  jaw position, grasped/picked flags. rsl_rl `obs_groups = {"actor": ["policy"], "critic": ["policy", "critic"]}`.

**Rewards** — staged, with distance-based progress shaping (`mdp/rewards.py`). The hammer-to-bin task subclasses the base env
and adds all of these. The base sandbox is unchanged, so `play.py` and `check_ft_payload.py` keep working.

| Stage | Term | Condition | Reward |
|---|---|---|---:|
| Approach hammer | `approach_progress` | `k·(φ_{t-1} − φ_t)`, potential **`φ = |TCP − grasp pt| + c·|Δyaw|`** (Δyaw wrapped mod π, c ≈ 0.1 m/rad); only while the hammer is not grasped | k = 1.0 |
| | `reached_hammer` | first time the rule's close condition holds (1.5 cm **and** 15°) | +5 (once) |
| Pick | `picked_hammer` | first time `hammer_picked` (grasped **and** lifted ≥ 2.5 cm) | +10 (once) |
| Transport | `transport_progress` | `k·(d_{t-1} − d_t)`, `d = |hammer − BIN_TARGET|`, **only if the hammer is grasped at both t−1 and t**, else 0 | k = 1.0 |
| | `near_bin` | first time the grasped hammer has all corners over `BIN_FOOTPRINT` within 0.15 m of the rim | +5 (once) |
| Place | `placed_in_bin` | the rule releases (its corner check passed) | +20 (once) |
| Success | `success` | `hammer_in_bin`, not grasped, speed < 0.05 m/s, for N = 10 consecutive steps (0.4 s); ends the episode | +50 (once) |
| Failure | `dropped_outside_bin` | was picked, **not grasped now** (rule not CLOSED), at rest (< 0.05 m/s) and not `hammer_in_bin` — or below the table top (z < 0.9); ends the episode | −10 |
| | `lost_in_transport` | picked, then the grasp is lost (grasped at t−1, not at t) outside the bin vicinity | −5 (per event) |
| Efficiency | `time_penalty` | every step | −0.005 |

Implementation notes:
- **Progress terms** are `ManagerTermBase` classes holding `φ_{t-1}` / `d_{t-1}` per env. Reset stores NaN, so the first step
  pays 0. The previous value updates every step, even when the term is gated off, so transport pays only for movement made while
  holding the hammer. Potential-based shaping telescopes, so moving back and forth can't farm it.
- **One-off bonuses** keep a sticky per-env flag that resets on episode reset, so grasp → drop → regrasp can't farm +10.
- **dt scaling:** Isaac Lab's `RewardManager` multiplies every term by `step_dt` (0.04 s). Event and progress terms divide by
  `step_dt`, so the configured weight is the real reward per event (+5 is +5, not +0.2). `time_penalty` keeps the normal scaling.
- Every term is logged under `Episode_Reward/<term>`. `success` also logs `Metrics/success_rate` on reset.
- Both ks start at 1.0. The approach potential spans ~0.5–0.65 m, so shaping totals ~0.6 per episode. If learning stalls before the
  first +5, raise k to ~10 rather than changing the bonuses.

**Terminations**: `time_out`; `success`; `dropped_outside_bin` (same conditions as the reward row, including **not grasped**, so a
pause mid-carry or a squeeze on the table never ends the episode).

**Reset state** (Isaac Lab resets each env on its own when a termination fires; reset events run in `EventCfg` declaration order):

| What | Reset to | How |
|---|---|---|
| Arm | `HOME_POSE`, zero velocity, drive targets = home | existing `reset_scene_to_default(reset_joint_targets=True)` |
| Gripper joint | `finger_joint` + mimics at 0 (half open), as `COBOTTA_RG6_CFG` authors it | same event. The rule commands OPEN from step 0, and the jaw is fully open in ~1 s, long before the arm can reach the hammer |
| Hammer | flat on the table, random x, y and **world-vertical** yaw in `HAMMER_SPAWN_RANGE`, zero velocity | **new** `reset_hammer_on_table` (`mdp/events.py`), after the event above: orientation = `yaw_world ⊗ HAMMER_ON_TABLE.rot`, rotated about the hammer's centre of mass |
| Pallet, shelf, workcell | static | — |
| Arm action | TCP target = actual TCP, previous joint target = home | `TopDownTcpTargetAction.reset` |
| Gripper rule | OPEN, latches cleared | `ProximityGripperAction.reset` |
| Reward state | φ/d = NaN, flags False, success counter 0, rest height recorded | each term's `reset` |

**Timing**: `sim.dt = 0.01` (gripper tuning), `decimation = 4` → policy at 25 Hz; `episode_length_s = 20` → 500 policy steps.

**Registration** (`config/cobotta/__init__.py`): `TechtoryCobottaIsaaclab-HammerToBin-COBOTTA` and `-Play` (few envs), with the rsl_rl entry point.

**PPO** (`agents/rsl_rl_ppo_cfg.py`, from the base one): `num_steps_per_env = 32`, `max_iterations ≈ 4000`, `gamma = 0.995`,
`lam = 0.95`, MLP [256, 128, 64], asymmetric `obs_groups`, `experiment_name = "techtory_cobotta_hammer_to_bin"`, `num_envs = 1024` to start.

### 6. Scripted sanity check — `scripts/check_hammer_to_bin.py`
A hand-written controller using the **same 4-D action**: above the grasp point, turn to the grasp yaw, descend, rule closes,
lift, carry to `BIN_TARGET`, rule opens. Prints PASS/FAIL per env and the reward each stage paid (should be +5, +10, +5, +20, +50 plus shaping).
`--resets N` mode checks reset states (see Verification). Same CLI pattern as `scripts/play.py`.

### 7. Docs and tests
- README: "Hammer-to-bin task" section (action/observation tables, rewards, train/play commands).
- `tests/test_layout.py`: spawn range (shrunk by the half-diagonal) on the table and clear of the pallet; `BIN_FOOTPRINT` inside the
  pallet; the hammer fits `BIN_FOOTPRINT` at any yaw; pallet pose matches `bin_place` in the base frame.
- `tests/test_registration.py`: new task IDs register.
- `tests/test_hammer_to_bin_mdp.py` (unit, no Kit): yaw error wrap modulo π (0° and 180° both count as 0); corner check rejects a
  hammer across the rim; progress terms telescope, store NaN on reset and gate transport by grasp; one-off flags pay once;
  dt scaling gives the table's values; `dropped_outside_bin` is false while grasped; the arm term's action clip, target clamp
  and joint-rate limit (a large `joint_des` jump is limited to `v_lim · dt`).

## Training workflow
```bash
cd techtory_cobotta_isaaclab
uv run --extra isaacsim python scripts/check_hammer_to_bin.py --resets 1024             # resets valid?
uv run --extra isaacsim python scripts/check_hammer_to_bin.py                           # solvable?
uv run --extra isaacsim isaaclab train --rl_library rsl_rl --task TechtoryCobottaIsaaclab-HammerToBin-COBOTTA --num_envs 1024 --viz none
uv run --extra isaacsim isaaclab play  --rl_library rsl_rl --task TechtoryCobottaIsaaclab-HammerToBin-COBOTTA-Play --checkpoint latest
```
Watch in TensorBoard (`logs/rsl_rl/techtory_cobotta_hammer_to_bin`): `Episode_Reward/reached_hammer` → `picked_hammer` → `near_bin` →
`placed_in_bin` → `Metrics/success_rate`. If a stage stalls, tune that stage's k, c or threshold.

## Later: sim-to-real (not in this change)
- Domain randomization: hammer mass/friction, actuator gains, hammer-pose observation noise (perception), 1–2 step action latency,
  ±0.05 rad arm joint noise at reset.
- Deploy: rsl_rl's exported `policy.onnx` in a ROS 2 node. Inputs: `/joint_states`, TF (TCP) and a hammer pose estimate.
  Output: the integrated TCP target → MoveIt Servo (pose tracking). The gripper rule runs as a node sending open/close to the real gripper action.

## Verification
1. `env -u PYTHONPATH uv run pytest -q` passes (layout, registration, MDP unit tests); `ruff check` is clean.
2. `scripts/sync_assets.py --check`: pallet copied, no drift.
3. Holding test: zero action for 10 s at home, then with the hammer grasped. The TCP must not drift by more than 2 mm (proves the IK drift fix).
   Full-scale actions: no arm joint ever exceeds its velocity limit.
4. `check_hammer_to_bin.py --resets 1024`: every hammer rests flat (tilt < 5°, z within 1 cm), inside the spawn area, not touching the robot or pallet;
   the arm is at home and the gripper fully open after 1 s.
5. `check_hammer_to_bin.py`: the scripted controller passes on most envs. Each reward stage fires once, in order. No early termination while carrying.
6. Short training run (~200 iterations, 256 envs): `approach_progress` and `reached_hammer` rise. Then the full run.
