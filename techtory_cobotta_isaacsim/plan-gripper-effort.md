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
- [x] **Blocker 0 found and fixed: the gripper had no collision shapes at all** — see §1a.
      This was never a force problem in the first place; the fingers were a ghost.
- [x] Stage 1 — static force limit (kills the crushing, no ROS changes). Done in USD authoring
      in `spawners/spawn_robot.py:configure_gripper_drive`, called from `scripts/main.py`.
- [ ] Stage 2 — per-goal effort plumbed from the action to the PhysX drive.
      **Nothing of Stage 2 is implemented**: `cobotta_hardware.ros2_control.xacro` still declares only
      `position` + `acceleration` on every joint, `main.py:148` still wires
      `SubscribeJS.outputs:effortCommand → ArtCtrl.inputs:effortCommand`, and there is no ScriptNode.
      The design in §4 Stage 2 is also **wrong as written** — see §4 Stage 2a and §5.
- [x] Stage 3 — friction (`add_grip_friction`). Torque→force calibration still open.
- [x] **Stage 1b — MoveIt "close on an object" now terminates, and the grip actually holds.**
      Two rounds: the controller's stall threshold was below Isaac's noise floor (§2a), and the
      fingers were ringing on contact while the hammer was un-graspable by construction (§2b).

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

`finger_joint` drive — note the **composed** values differ from what
`onrobot_rg6_physics.usd` authors, because `cvrb0609_with_graph2.usd` overrides them. Always
read the composed stage, not the physics layer:

| attribute | in `onrobot_rg6_physics.usd` | **composed on stage** | note |
|---|---:|---:|---|
| `drive:angular:physics:type` | `force` | `force` | |
| `drive:angular:physics:stiffness` | 1.745329e10 | **1.745329e6** | per **degree** in USD |
| `drive:angular:physics:damping` | 1.745329e9 | **1745.33** | per **degree** in USD |
| `drive:angular:physics:maxForce` | 6000 | **6000** | torque, N·m — effectively unlimited |
| `physics:lowerLimit` / `upperLimit` | ∓36.0° | ∓36.0° | = ∓0.628 rad ✓ matches the action range |
| `physxJoint:maxJointVelocity` | 114.59°/s | 114.59°/s | = 2.0 rad/s |

6000 N·m at the ~0.080 m finger lever is ~75 kN. Nothing an object can do opposes that, so
"close" always runs to the hard stop. **This is why the gripper crushes or ejects objects.**

Measured stroke (after §1a is fixed), commanded angle → clear gap between the pad faces:

| `finger_joint` | −0.628 | −0.314 | 0.000 | +0.314 | +0.628 |
|---|---:|---:|---:|---:|---:|
| jaw opening | 151.4 mm | 128.9 mm | 93.7 mm | 49.2 mm | ~0 mm |

### 1a. Blocker 0 — the gripper had no collision shapes (fixed)

Verified with `omni.physx` scene-query overlaps over the jaw: PhysX reported colliders for the
arm links and **none** for any of the six RG6 links. A 50 mm cube parked between 94 mm jaws was
displaced **0.00 mm** while the drive ran to its hard stop.

Cause: each RG6 link's `collisions` Xform carries `PhysxMeshMergeCollisionAPI`, so PhysX builds
one collision shape out of whatever the prim's `collisionmeshes` collection resolves to. The
asset ships that collection as `expansionRule = "explicitOnly"` with a single include — the
`collisions` Xform itself. "explicitOnly" means exactly the listed paths and nothing under them,
so the collection resolves to one Xform and **zero meshes**, and the merge produces no shape.
The `collisions` prims are also `instanceable = true`, and a collection cannot reach prims inside
an instance prototype.

Fix (`spawn_robot.py:fix_gripper_collisions`): de-instance, then set
`expansionRule = "expandPrims"`. The collision meshes are 15 000 points each and their world
bounds match the visual pads exactly, so `convexHull` is a tight approximation — no need for SDF
or convex decomposition.

**Second-order consequence** — `physxArticulation:enabledSelfCollisions = 0` is authored on
`/onrobot_rg6/gripper_joint`, which is **no longer the articulation root** (the root is the arm's
`root_joint`, where the attribute is unset and defaults to True). Harmless while the gripper had
no shapes; the moment it has them the four-bar's overlapping knuckles and fingers push each other
apart and the jaw jams at 0.097 rad with nothing between the fingers. Hence
`spawn_robot.py:disable_articulation_self_collisions`, which must run alongside the collision fix.

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
| `max_effort_interface` format | A **full** interface name — the controller does not prefix the joint (`_impl.hpp:405-408`). But the claimed interface is only *bound* to `effort_interface_` when its interface name is literally `set_gripper_max_effort` (`_impl.hpp:346-355`). So it must be **`finger_joint/set_gripper_max_effort`** — `finger_joint/effort` would be claimed and then silently never written. |
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

## 2a. Why MoveIt's "close" failed on a held object — **fixed**

Symptom: the force-limited grip worked in Isaac, but closing the `gripper` group from MoveIt on an
object already in the jaw never returned. From `move_group`'s log:

```
[INFO ] trajectory_execution_manager: Validating trajectory with allowed_start_tolerance 0.01
[INFO ] trajectory_execution_manager: Starting trajectory execution ...
[WARN ] parallel_gripper_controller_handle: waitForExecution timed out
[ERROR] trajectory_execution_manager: Controller is taking too long to execute trajectory
        (the expected upper bound for the trajectory execution was 11.997270 seconds). Stopping.
[INFO ] trajectory_execution_manager: Completed trajectory execution with status TIMED_OUT
```

and from `ros2_control_node`, 12 s apart with nothing in between:

```
[INFO] [onrobot_rg6]: Received & accepted new action goal
[INFO] [onrobot_rg6]: Got request to cancel goal
```

Planning was never the problem — `CheckStartStateCollision` passed and OMPL returned a path. The
goal simply never terminated.

**Root cause.** `check_for_success` (`_impl.hpp:209-256`) has exactly two exits: position error
inside `goal_tolerance`, or velocity under `stall_velocity_threshold` for `stall_timeout`. With the
object in the jaw the first is unreachable by construction. The second was never reached either:
`stall_velocity_threshold` was left at its upstream default of **0.001 rad/s**, and Isaac's reported
`finger_joint` velocity chatters above that on every cycle while the PhysX drive presses into the
object. `last_movement_time_ = time` therefore ran on every update and the stall clock never
started. `allow_stalling: true` was already set — it was simply never consulted, which is why the
goal hung instead of aborting at ~1 s.

**Fix** (`techtory_cobotta_bringup/config/controller.yaml`): `stall_velocity_threshold: 0.1`,
`stall_timeout: 0.5`, and the block rewritten to use only parameters this controller actually has
(blocker 1's dead keys removed). 0.1 rad/s sits ~14x below a free close (~1.4 rad/s = the 2.0 rad/s
joint limit x MoveIt's 0.7 velocity scaling), so it cannot be tripped mid-stroke.

**Why an early stall is safe.** The controller keeps writing `position_cmd_` to the position
command interface *after* the goal completes — it does not release or hold-at-current. So the drive
goes on closing to the force limit regardless of when the action returned; declaring the stall
early reports the grip sooner without weakening it.

### 2b. Round two — the threshold was not the whole story

With `stall_velocity_threshold: 0.1` installed and active, a close on the hammer *still* ran 18 s
to `TIMED_OUT` (goal accepted 877.53, cancelled 895.72), while free open/close goals in the same
session finished in 0.27–0.52 s. `finger_joint` can only travel 1.26 rad end to end, so 18 s above
0.1 rad/s is not creep — the finger was **oscillating**. Same session: the gripper closed on the
hammer but did not hold it. One cause, two symptoms.

**Cause A — the fingers are effectively massless.** The RG6 links carry ~1e-4 kg·m² about their
own joints, and `configure_gripper_drive` puts 5 N·m on them: ~5e4 rad/s² of available
acceleration, bounded only by `maxJointVelocity`. The fingers hit the object with far more kinetic
energy than the real gripper has, bounce, and ring. Fixed in
`spawn_robot.py:stabilize_gripper_joints` — `physxJoint:armature = 0.01` on all six gripper joints
(PhysX's model of the gear-train inertia the imported asset lacks, peak accel → ~500 rad/s²), plus
`physxJoint:maxJointVelocity` 2.0 → 0.6 rad/s so touchdown is gentler and any residual ringing
stays well under the controller's threshold.

**Cause B — the hammer cannot be grasped at all, at any force.** `hammer1.usd` authors
`physics:approximation = "convexHull"` on `hamLo`, and the convex hull of a hammer is a solid
wedge from head to handle — it fills in the very notch the pads aim for. The pads therefore never
touch the handle; they close on a sloping hull face and extrude the part out of the jaw. The asset
also has **no `PhysicsMassAPI`** (mass and inertia integrated from that wrong hull at default
density) and **no physics material**, so it ran on the engine default while the pads had 1.2/1.1 —
PhysX averages the two, so binding the pads alone only got halfway. All three fixed in
`spawn_objects.py:configure_graspable_object`, called from `main.py` right after `add_hammer`.

**Armature alone was not enough — measured, not assumed.** Closing on a 50 mm / 0.2 kg cube and
reading `finger_joint`'s |velocity| p95 over the last 1.5 s of the squeeze:

| armature | jointFriction | press p95 (rad/s) | cube held? | stall would fire? |
|---:|---:|---:|:--|:--|
| 0.01 | 0.0 | 0.175 | no | no |
| 0.01 | 0.2 | 0.136 | no | no |
| 0.05 | 0.0 | 0.113 | yes | no |
| 0.05 | 0.2 | 0.088 | yes | yes |
| 0.10 | 0.2 | 0.063 | yes | yes |
| **0.15** | **0.4** | **0.042** | **yes** | **yes** — chosen, 2.4x margin |

Joint friction is doing work armature cannot. At a ~0.35 rad position error the drive is
saturated at `maxForce`, so its damping term is clamped away entirely: the squeeze is a constant
torque with no velocity feedback, which is precisely the condition for a limit cycle. Coulomb
friction at the joint is the dissipation that closes it.

### 2c. Verified

Headless, driving the PhysX articulation directly (no ROS bridge), against the committed config:

```
A. FREE  close 2.42 s, open 2.50 s (MoveIt budget 6.8 s)      -> reached_goal path OK
B. QUIET blocked at +0.1692 rad, press |vel| p95 = 0.0351 rad/s (threshold 0.1)
C. HOLD  cube z 1.2059 -> 1.2059  (+0.0 mm)
D. LIFT  cube moved 140.7 mm with the arm, final z 1.3441
```

Also confirmed along the way, against claims that were previously only inferred:
- **PhysX really does have shapes for all six RG6 links** — scene-query overlap over the jaw
  returns all six `*/collisions` prims. §1a's fix is genuinely in force.
- **The clear jaw opening at −0.62 rad is 151.0 mm**, matching §1's measured 151.4 mm.
- **The inner knuckles sit ~10 mm apart** and occupy the z-band *above* the pads
  (1.2249–1.3003 at open, descending as the jaw closes). An object presented too high is caught
  between the knuckles ~20 mm before the pads ever reach it and is not gripped at all. This is a
  real constraint on grasp poses, not just a test artefact.

**Still unverified:** that MoveIt's close goal now returns SUCCEEDED rather than TIMED_OUT. That
needs the full bringup restarted; B is the mechanism it depends on, but the end-to-end path has
not been exercised. The hammer specifically has not been grasped either — C/D used a cube.

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

### Stage 1 — static force limit — **DONE**, but not the way this section originally proposed

Authored into USD in `build_world()` **before** `world.reset()`, next to
`set_initial_joint_positions`, rather than through the tensor API after reset:

```python
fix_gripper_collisions(stage, "/World/Cobotta/onrobot_rg6")     # §1a — must come first
disable_articulation_self_collisions(stage, "/World/Cobotta")   # §1a
configure_gripper_drive(stage)                                  # maxForce/stiffness/damping
add_grip_friction(stage, "/World/Cobotta/onrobot_rg6")          # Stage 3
```

USD authoring was chosen over `set_dof_max_efforts` because the USD attribute units are
unambiguous (see §5) and because the limit is then in force on the very first physics step.

Values in `spawn_robot.py`: `DEFAULT_GRIP_TORQUE = 5.0` N·m (~62 N),
`GRIP_DRIVE_STIFFNESS = 3.0` /deg, `GRIP_DRIVE_DAMPING = 0.1` /deg.

Measured, 50 mm / 0.2 kg cube standing on a post, close then lift the shoulder 0.25 rad:

| grip torque | ≈ finger force | stall angle | cube displaced while closing | lifted with the arm |
|---:|---:|---:|---:|:--|
| 1 N·m | 12 N | 0.198 rad | 0.04 mm | yes |
| 2 N·m | 25 N | 0.198 rad | 0.01 mm | yes |
| 5 N·m | 62 N | 0.198 rad | 0.01 mm | yes |
| 10 N·m | 125 N | 0.199 rad | 0.02 mm | yes |
| **6000 N·m** (as shipped) | 75 kN | **0.266 rad** | **1.93 mm** | yes |

The shipped drive overshoots the geometric stall by 0.07 rad and shoves the cube ~2 mm — that is
the crushing. Anything in the 1–10 N·m band stops in the same place and holds. Free close with
nothing in the jaw still reaches the 0.628 rad stop, so `goal_tolerance` termination is unaffected.

### Stage 2 — per-goal effort

**2a. Blocker 5 — `topic_based_ros2_control` cannot carry this interface at all.** The name the
controller binds is `set_gripper_max_effort` (§1 table), which is not one of `position` / `velocity`
/ `effort`. `TopicBasedSystem::export_command_interfaces` runs every declared interface through
`getInterface`, which only matches `standard_interfaces_`, and **throws**
`std::runtime_error("Interface is not found in the standard list.")` on anything else
(`topic_based_system.cpp:184-201`) — that kills the whole hardware component, not just the gripper.
Declaring `<command_interface name="effort"/>` instead avoids the throw but is never written to,
because `effort_interface_` stays `nullopt` unless the interface name matches exactly.

So Stage 2 needs one of:

- **(i) Patch `topic_based_system.cpp`** to accept `set_gripper_max_effort` and map it into
  `JointState.effort`. Smallest change that keeps the stock controller; the plugin is vendored under
  `plugins/controls/`, so it is ours to patch. Note the §1 array-alignment rule still applies —
  whatever interface carries it must be declared on all seven joints.
- **(ii) Skip the controller's effort path** and set the drive limit out of band (a small node that
  writes the ScriptNode's `defaultEffort` input, or a service on the Isaac side). Keeps the goal's
  `effort` field unused, which is honest about what the wire actually carries today.

Pick one before writing 2c. **(i)** is the one that makes `command.effort` mean something end to
end, which is what §3's architecture assumes.

**2b. `controller.yaml`** — **DONE** (§2a of this document). The block now carries only real
parameters, with the stall path tuned so a grip terminates. When 2a lands, add:

```yaml
    max_effort_interface: finger_joint/set_gripper_max_effort
```

and drop the "inert today" comment above `max_effort`.

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
- **Friction — done.** `spawn_robot.py:add_grip_friction` defines `/World/PhysicsMaterials/GripperPad`
  (static 1.2 / dynamic 1.1 / restitution 0) and binds it with purpose `physics` to the two
  `*_inner_finger/collisions` prims. The object side is now covered too —
  `spawn_objects.py:configure_graspable_object` defines `/World/PhysicsMaterials/GraspableObject`
  with the same numbers and binds it to the hammer's collider (§2b Cause B). `shelf.usd` is static
  and does not need one.
- **Object collider shape matters more than grip force.** A `convexHull` approximation on any
  object with a concave grasp feature (a hammer, a mug, an L-bracket) fills that feature in, and no
  amount of squeeze will hold it — the pads never reach the real surface. Use
  `convexDecomposition` for anything the gripper is meant to pick up. This is checked per object;
  `soda_can.usd` is convex and is fine as a hull.

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

- **`set_dof_max_efforts` did not behave predictably in testing.** Sweeping the cap at runtime with
  `Articulation.set_dof_max_efforts([tau], dof_indices=[dof])` between trials in one process gave
  non-monotonic stall angles (0.02–0.20 rad) for the same scenario, while authoring the same values
  in USD across separate processes gave a flat 0.198 rad every time. Not root-caused — it may be the
  argument shape (the API is batched over environments, so `[[tau]]` may be what it wants) rather
  than the API. **Stage 2 depends on this call, so verify it against a USD-authored baseline before
  building on it.**
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
- **Non-standard command interfaces take the whole hardware component down.** `TopicBasedSystem`
  throws on any interface outside `position`/`velocity`/`effort`, so a stray
  `<command_interface name="set_gripper_max_effort"/>` does not degrade to "gripper effort ignored" —
  `ros2_control_node` fails to configure. See §4 Stage 2a.
- **Stall detection is velocity-threshold-bound, and sim velocity is noisy.** `stall_velocity_threshold`
  under Isaac's resting chatter means the goal never terminates rather than aborting — a hang, not an
  error. If a gripper goal ever hangs again, echo `/topic_based_joint_states` and look at
  `finger_joint`'s velocity while it presses, then set the threshold above it (§2a).
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
