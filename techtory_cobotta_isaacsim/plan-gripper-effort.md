# Force-Limited Gripper Action — Implementation Plan

**Target:** `open`/`close` on the OnRobot RG6 through the existing
`/onrobot_rg6/gripper_cmd` (`control_msgs/action/ParallelGripperCommand`) action, where the goal's
**effort** becomes a real grip-force limit in Isaac Sim: close hard enough to hold, soft enough not to
crush, regardless of object size; open command goes fully open.

**Environment:** Isaac Sim 6.0.1 (pip, `/home/anm-vi/Main/Environment/isaac6`), ROS 2 Jazzy, PhysX backend.

**Files in play:**
- `techtory_cobotta_workcell_description/urdf/cobotta_hardware.ros2_control.xacro`
- `techtory_cobotta_bringup/config/controller.yaml`
- `techtory_cobotta_isaacsim/scripts/main.py`
- (read-only, for reference) `plugins/controls/topic_based_ros2_control/src/topic_based_system.cpp`

Sibling document: `plan-ft-implement.md` (wrist FT sensor). The two share the articulation facts in §1.

---

## 0. Status

- [x] Command path traced end to end — see §1.
- [x] Root causes of "effort does nothing" identified — see §2.
- [ ] Stage 1 — static force limit (kills the crushing, no ROS changes).
- [ ] Stage 2 — per-goal effort plumbed from the action to the PhysX drive.
- [ ] Stage 3 — calibration + friction.

---

## 1. Established facts (verified against the files/install — do not re-derive)

### The command path

```
ros2 action send_goal /onrobot_rg6/gripper_cmd control_msgs/action/ParallelGripperCommand
      "{command: {name: ['finger_joint'], position: [0.628]}}"
   │   +0.628 rad = fully closed, -0.628 rad = fully open
   ▼
parallel_gripper_action_controller/GripperActionController   controller.yaml:14, :52
   │   claims  finger_joint/position
   │   claims  <max_effort_interface>   ← only if that param is set (it is not, today)
   ▼
topic_based_ros2_control/TopicBasedSystem      cobotta_hardware.ros2_control.xacro:84
   │   publishes sensor_msgs/JointState on /topic_based_joint_commands
   ▼
Isaac  ROS2SubscribeJointState → IsaacArticulationController        scripts/main.py:93-123
   │   set_dof_position_targets(finger_joint)
   ▼
PhysX revolute drive on /onrobot_rg6/base_link/finger_joint
```

### The gripper in USD

Loaded asset is `assets/robots/cvrb0609/cvrb0609_with_graph2.usd`, which references
`assets/grippers/rg6/onrobot_rg6.usd` (physics layer: `grippers/rg6/configuration/onrobot_rg6_physics.usd`).

**Only `finger_joint` has a drive.** The other five gripper joints
(`left/right_inner_knuckle_joint`, `right_outer_knuckle_joint`, `left/right_inner_finger_joint`) are
**PhysX mimic joints** — `PhysxMimicJointAPI:rotX` with
`referenceJoint = </onrobot_rg6/base_link/finger_joint>`, `gearing = ±1`,
`naturalFrequency = 572958`, `dampingRatio = 10` — and their `PhysicsDriveAPI:angular` is explicitly
deleted. Their leftover `drive:*` attribute values are inert.

**Consequence: all grip torque comes from the single `finger_joint` drive.** Limiting that drive's
max force limits the whole grasp. Nothing else needs to be touched.

Authored `finger_joint` drive:

| attribute | value | note |
|---|---:|---|
| `drive:angular:physics:type` | `force` | |
| `drive:angular:physics:stiffness` | 1.745329e10 | per **degree** in USD |
| `drive:angular:physics:damping` | 1.745329e9 | per **degree** in USD |
| `drive:angular:physics:maxForce` | **6000** | torque, N·m — effectively unlimited |
| `physics:lowerLimit` / `upperLimit` | ∓36.0° | = ∓0.628 rad ✓ matches the action range |
| `physxJoint:maxJointVelocity` | 114.59°/s | = 2.0 rad/s |

A 6000 N·m limit against a 1.7e10 stiffness is a rigid position servo. **This is why the gripper
crushes or ejects objects today.**

### Articulation (shared with `plan-ft-implement.md` §1)

One articulation, root `/World/Cobotta/techtory_demo_description/root_joint`, 12 DOF
(6 arm + 6 gripper), 14 links. Mimic joints are still real DOFs in PhysX — mimic is a constraint,
not a reduced coordinate — hence 6 gripper DOFs, not 1.

### Controller facts (Jazzy, from the installed headers)

| Thing | Finding |
|---|---|
| Action type | `control_msgs::action::ParallelGripperCommand` (`parallel_gripper_action_controller.hpp:97`). **Not** `GripperCommand`. |
| Real parameter set | `joint`, `state_interfaces`, `goal_tolerance`, `allow_stalling`, `stall_velocity_threshold`, `stall_timeout`, `max_effort_interface`, `max_effort`, `max_velocity_interface`, `max_velocity`, `action_monitor_rate` (`parallel_gripper_action_controller_parameters.hpp:72-83`). |
| Goal effort | Used **only** when `max_effort_interface` is non-empty; then `command.effort[0]` is written to that interface each cycle, else the `max_effort` param is used (`_impl.hpp:149-156`, `:101-104`). |
| `max_effort_interface` format | A **full** interface name — the controller does not prefix the joint (`_impl.hpp:405-408`). Must be `finger_joint/effort`, not `effort`. |
| Claimed command interfaces | `<joint>/position` always, plus `max_effort_interface` / `max_velocity_interface` if set (`_impl.hpp:402-415`). |
| Stall → success | If `|position error| > goal_tolerance` and `|velocity| < stall_velocity_threshold` for `stall_timeout`, the goal **succeeds** with `stalled: true` when `allow_stalling: true`, aborts otherwise (`_impl.hpp:225-262`). This is the "gripped an object of unknown size" path. |

### Transport facts (`topic_based_system.cpp`)

| Thing | Finding |
|---|---|
| Effort supported | Yes — a joint declaring an `effort` command interface has its value pushed into `JointState.effort` (`:295-298`). |
| **Array alignment** | `name` gets **every** joint; `position`/`velocity`/`effort` get an entry **only** for joints that declare that interface (`:280-305`). Declaring `effort` on `finger_joint` alone ⇒ `name.size()==7`, `effort.size()==1`. |
| Publish gate | `write()` returns early unless the summed position command-vs-state difference exceeds `trigger_joint_command_threshold_` (default **1e-5**, `topic_based_system.hpp:95`; overridable via a hardware param). **An effort-only change with an unchanged position target is never published.** |
| Mimic handling | Only for joints present in the `<ros2_control>` block. The five RG6 mimic joints are not declared there, so nothing extra is published — Isaac's PhysX mimic joints do that work. |

### Isaac-side facts

| Thing | Finding |
|---|---|
| Size mismatch is fatal | `OgnIsaacArticulationController._resolve_command_indices` rejects any command array whose size ≠ the selected joint count and **drops all commands, positions included** ("Ignoring all commands"). A 1-element effort array against 7 joint names would freeze the arm. |
| NaN entries | Dropped per-element ("leave target unchanged") in `_filter_finite_command` — but `topic_based` writes `0.0`, not NaN. |
| `effortCommand` semantics | `set_dof_efforts()` = **additive torque**, not a limit. Wiring goal effort there does not limit grip force. |
| The right API | `isaacsim.core.experimental.prims.Articulation.set_dof_max_efforts(values, dof_indices=[...])` → writes the drive's max force, via `set_dof_max_forces` on the tensor view when physics is live, else the USD `maxForce` attribute (`articulation.py:2863-2915`). |
| Gains API | `Articulation.set_dof_gains(stiffnesses=…, dampings=…)` (`articulation.py:3087`). |
| DOF index lookup | `art.get_dof_indices("finger_joint")` — resolve by name, never hardcode. |

---

## 2. Why effort does nothing today — four independent blockers

1. **The controller never reads the goal's effort.** `controller.yaml:63` sets `use_effort_interface: true`,
   which is not a parameter of this controller. Nor are `command_interfaces`, `constraints`,
   `open_loop_control`, `allow_partial_joints_goal`, `state_publish_rate` (§1 table). They are inert.
   The one that matters, `max_effort_interface`, is unset — so `command.effort` is discarded at the goal.
2. **`finger_joint` has no effort command interface.** `cobotta_hardware.ros2_control.xacro:182-195`
   declares `position` + `acceleration` only. Nothing for the controller to write into.
3. **Adding it naively breaks the arm.** Effort on `finger_joint` alone misaligns the JointState arrays
   and Isaac then drops *all* commands (§1 transport + Isaac facts).
4. **Even delivered, effort ≠ force limit.** `main.py:114` routes `effortCommand → ArtCtrl`, which applies
   additive torque. Against `stiffness = 1.7e10`, `maxForce = 6000` it changes nothing observable.

---

## 3. Architecture (target)

```
ParallelGripperCommand goal {position, effort}
        │
        ├─ position ─► finger_joint/position ─► JointState.position[6] ─► ArtCtrl ─► set_dof_position_targets
        │
        └─ effort   ─► finger_joint/effort   ─► JointState.effort[6]   ─► GripLimit ScriptNode
                                                                            └─► set_dof_max_efforts([τ], dof=finger)
```

`effortCommand` is **disconnected** from `ArtCtrl` and consumed by a new ScriptNode in the same
`/ActionGraph_Robot`, fed from the existing `SubscribeJS` node (no second subscriber, one value per tick).

Behaviour that falls out of this:
- **Close on a large object** — target 0.628 unreachable, drive saturates at τ, fingers hold at τ,
  velocity → 0, controller reports SUCCEEDED with `stalled: true` (needs `allow_stalling: true`).
- **Close on a small object** — same, just further along the stroke.
- **Close on nothing** — reaches 0.628, SUCCEEDED via `goal_tolerance`.
- **Open** — target −0.628, unobstructed, SUCCEEDED via `goal_tolerance`.

---

## 4. Implementation

### Stage 1 — static force limit (no ROS changes, biggest single win)

In `scripts/main.py`, after `world.reset()` and after the articulation is initialised:

```python
from isaacsim.core.experimental.prims import Articulation
_art = Articulation(ARTICULATION_PATH)
_finger_dof = int(_art.get_dof_indices("finger_joint").numpy().item())
_art.set_dof_max_efforts([DEFAULT_GRIP_TORQUE], dof_indices=[_finger_dof])   # e.g. 3.0 N·m
```

Verify: close on the hammer — it should be held, not launched. This alone makes the gripper usable.

### Stage 2 — per-goal effort

**2a. `cobotta_hardware.ros2_control.xacro`** — inside `cobotta_topic_based_ros2_control`, add

```xml
<command_interface name="effort"/>
```

to **all seven** joints (the six arm joints and `finger_joint`). The arm entries are never claimed and
publish as `0.0`; they exist purely to keep `JointState.effort` index-aligned with `JointState.name`
(blocker 3). Do the same in the `mock` / `mujoco` macros only if those backends are used for this.

**2b. `controller.yaml`** — replace the `onrobot_rg6` block (`:52-70`) with valid parameters:

```yaml
onrobot_rg6:
  ros__parameters:
    joint: finger_joint
    state_interfaces: [position, velocity, effort]
    max_effort_interface: finger_joint/effort   # full interface name, not "effort"
    max_effort: 3.0                             # N·m, used when the goal omits effort
    goal_tolerance: 0.01                        # rad
    allow_stalling: true                        # stall on object ⇒ SUCCEEDED
    stall_velocity_threshold: 0.01              # rad/s
    stall_timeout: 0.4                          # s
    action_monitor_rate: 20.0
```

Drop the stray `type:` key from inside `ros__parameters` — the controller type belongs only in the
`controller_manager` section (`:14-15`).

**2c. `scripts/main.py`** — remove this connection from the `/ActionGraph_Robot` edit (`:114`):

```python
("SubscribeJS.outputs:effortCommand", "ArtCtrl.inputs:effortCommand"),   # DELETE
```

and add the grip-limit node. `CREATE_ATTRIBUTES` is required — a ScriptNode has no custom ports by default.

```python
GRIP_SCRIPT = """
import omni.graph.core as og
from isaacsim.core.experimental.prims import Articulation

def setup(db):
    db.per_instance_state.art = None
    db.per_instance_state.dof = None

def compute(db):
    st = db.per_instance_state
    if st.art is None:
        try:
            st.art = Articulation(db.inputs.robotPath)
        except Exception:
            return True
    art = st.art
    if not art.is_physics_tensor_entity_valid():
        return True                              # physics not up yet
    if st.dof is None:
        st.dof = int(art.get_dof_indices(db.inputs.jointName).numpy().item())

    names   = list(db.inputs.jointNames)
    efforts = list(db.inputs.effortCommand)
    tau = db.inputs.defaultEffort
    if db.inputs.jointName in names:
        i = names.index(db.inputs.jointName)
        if i < len(efforts) and efforts[i] > db.inputs.minEffort:
            tau = float(efforts[i])              # 0.0 = 'no goal yet' -> keep default
    tau = max(db.inputs.minEffort, min(tau, db.inputs.maxEffort))
    art.set_dof_max_efforts([tau], dof_indices=[st.dof])
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
"""
```

Nodes / attributes / wiring to add to the existing `og.Controller.edit` block:

```python
CREATE_NODES:      ("GripLimit", "omni.graph.scriptnode.ScriptNode")
CREATE_ATTRIBUTES: ("GripLimit.inputs:robotPath",     "string")
                   ("GripLimit.inputs:jointName",     "token")
                   ("GripLimit.inputs:jointNames",    "token[]")
                   ("GripLimit.inputs:effortCommand", "double[]")
                   ("GripLimit.inputs:defaultEffort", "double")
                   ("GripLimit.inputs:minEffort",     "double")
                   ("GripLimit.inputs:maxEffort",     "double")
SET_VALUES:        ("GripLimit.inputs:script", GRIP_SCRIPT)
                   ("GripLimit.inputs:robotPath", ARTICULATION_PATH)
                   ("GripLimit.inputs:jointName", "finger_joint")
                   ("GripLimit.inputs:defaultEffort", 3.0)
                   ("GripLimit.inputs:minEffort", 0.2)
                   ("GripLimit.inputs:maxEffort", 12.0)
CONNECT:           ("OnTick.outputs:tick",              "GripLimit.inputs:execIn")
                   ("SubscribeJS.outputs:jointNames",   "GripLimit.inputs:jointNames")
                   ("SubscribeJS.outputs:effortCommand","GripLimit.inputs:effortCommand")
```

Also enable the extension next to the others (`main.py:18-22`):

```python
ext_manager.set_extension_enabled_immediate("omni.graph.scriptnode", True)
```

**2d. Drive gains.** Alongside Stage 1's `set_dof_max_efforts`, bring the authored gains down from
1.7e10 / 1.7e9. Those values plus SDF finger collisions invite solver ringing on contact, and they mean
the drive is permanently saturated at `maxForce` during free motion. See §5 for the units warning.

### Stage 3 — calibration and friction

- **Torque → force.** Contact force ≈ τ / lever arm. `onrobot_rg_control` uses `L3 = 0.080 m` for the
  RG6, and the real gripper's range is 25–120 N ⇒ roughly **2–10 N·m** is the useful band. Measure with
  the wrist FT work in `plan-ft-implement.md`, or with
  `art.get_link_incoming_joint_force()` on a finger link, and fill in a real table here.
- **Friction.** Neither the RG6 fingers nor `assets/objects/hammer.usd` has an authored
  `PhysicsMaterial` — both run on engine defaults. With a correctly limited force the object will slip
  before it is crushed. Add a high-friction material to the two `*_inner_finger` collision prims.

### Verify

```bash
cd /home/anm-vi/Techtory && colcon build --packages-select \
    techtory_cobotta_isaacsim techtory_cobotta_bringup techtory_cobotta_workcell_description
ros2 launch techtory_cobotta_isaacsim isaacsim_launcher.launch.py
ros2 launch techtory_cobotta_bringup techtory_cobotta_sw_bringup.launch.py

# soft close on the hammer — should hold, not crush, and return SUCCEEDED/stalled:true
ros2 action send_goal -f /onrobot_rg6/gripper_cmd control_msgs/action/ParallelGripperCommand \
  "{command: {name: ['finger_joint'], position: [0.628], effort: [2.0]}}"

# full open
ros2 action send_goal -f /onrobot_rg6/gripper_cmd control_msgs/action/ParallelGripperCommand \
  "{command: {name: ['finger_joint'], position: [-0.628], effort: [8.0]}}"

# inspect what actually reaches Isaac (name/effort arrays must be the same length)
ros2 topic echo /topic_based_joint_commands
```

**Acceptance:**
1. Both goals terminate — no hanging action, no ABORTED.
2. Closing on the hammer at 2 N·m holds it without ejecting it; at 8 N·m the grip is visibly firmer.
3. `/topic_based_joint_commands` shows `name.size() == effort.size() == 7`.
4. The arm still moves (proof that blocker 3 did not reappear).

---

## 5. Gotchas

- **Angular gain units.** USD angular drive stiffness/damping are per **degree**; the tensor API is
  documented per **radian**. `maxForce` is torque in both, so the effort mapping is unit-safe — but
  verify before trusting any stiffness/damping number set from Python. A 57× error hides here.
- **Effort 0.0 means "no goal yet", not "go limp".** Unclaimed/at-rest command interfaces publish `0.0`.
  Never pass that straight to `set_dof_max_efforts` — with gravity now enabled on the gripper links
  (`plan-ft-implement.md` §0) the fingers would droop. Hence `minEffort`.
- **The publish gate.** Re-sending the same position with a different effort publishes nothing
  (`trigger_joint_command_threshold_`, §1). Effort only reaches Isaac when the position target also
  changes. If per-goal effort tuning at a fixed position is ever needed, raise the threshold's
  companion logic or add `trigger_joint_command_threshold` as a hardware param — do not assume it works.
- **Array alignment is load-bearing.** Any future joint added to the `<ros2_control>` block without an
  `effort` command interface silently re-breaks blocker 3 and freezes the whole arm, not just the gripper.
- **The BT node cannot talk to this controller.** `techtory_cobotta_system/trees/techtory_cobotta_gripper.xml:6`
  uses the `GripperCommand` BT client, which is `control_msgs/action/GripperCommand`
  (`gripper_command_action_bt_client.hpp:28`), while this controller serves `ParallelGripperCommand`.
  Same action name, different type ⇒ the client never discovers the server. A
  `ParallelGripperCommand` BT client is needed before the trees can carry `max_effort`.
- **Real hardware ignores effort too.** `onrobot_rg_control/OnRobotRGControllerServer.py:execute_callback`
  hardcodes `self.gripper.sendCommand(self.max_force/20, …)` and never reads the goal's effort. The RG6
  takes `rgfr` in 0.1 N units up to 1200 (= 120 N). The same plumbing is needed there for hw/sim parity.
- **Don't reintroduce `effortCommand → ArtCtrl`.** It is additive torque; it will fight the drive.
- **`isaacsim.core.api` / `isaacsim.core.prims` are deprecated.** The ScriptNode above deliberately uses
  `isaacsim.core.experimental.prims`, matching `plan-ft-implement.md`.

---

## 6. Open decisions

1. **Effort units on the wire.** The plan treats `command.effort` as **N·m at `finger_joint`**. The
   alternative is newtons of finger force, converted inside the ScriptNode by the lever arm. Newtons are
   friendlier to callers and match the RG6 datasheet (25–120 N); N·m is a pass-through with nothing to
   get wrong. Decide before anything downstream hardcodes numbers.
2. **Force-limited position vs true force mode.** This plan keeps a stiff position drive with a capped
   `maxForce`. A closer analogue of the real RG6 is `stiffness = 0` + velocity target + `maxForce` while
   closing, switching back to position mode to open. More faithful, more state to manage.
3. **Grip detection.** `stalled: true` is currently the only "I am holding something" signal, and it
   cannot distinguish an object from a jam. If the BT needs a reliable grasp check, gate on
   `finger_joint` position being strictly inside the limits plus a non-zero measured effort.
4. **Where the default lives.** `max_effort` in `controller.yaml` and `defaultEffort` in the ScriptNode
   are two defaults for one thing, and they will drift. Pick one as authoritative.
