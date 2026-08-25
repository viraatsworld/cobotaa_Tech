# Wrist Force/Torque Sensor — Implementation Plan

**Target:** 6-axis FT sensor at the cvrb0609 wrist (arm flange ↔ OnRobot RG6 gripper), published to ROS 2 as `geometry_msgs/msg/WrenchStamped` via the Isaac Sim ROS 2 bridge.

**Environment:** Isaac Sim 6.0.1, pip install at `/home/anm-vi/Main/Environment/isaac6`, ROS 2 Jazzy.
**Files in play:** `scripts/main.py`, `assets/robots/cvrb0609/cvrb0609_with_graph2.usd`

---

## 0. Status — what is already done

- [x] **Isaac Sim 6.0 migration.** All `omni.isaac.*` imports in this package moved to the `isaacsim.*` namespace; package rebuilt. (`curobo_motion_generation` is still un-migrated and will fail the same way — out of scope here.)
- [x] **Articulation structure verified** — see §1.
- [x] **Gravity enabled on the gripper links** (`disableGravity` was `True` on all 7 RG6 links, now `False`). The wrist now carries a real static load.
- [ ] FT read + publish — this document.

---

## 1. Established facts (verified empirically — do not re-derive)

### Articulation

There is **ONE** articulation, not two.

```
Articulation root : /World/Cobotta/techtory_demo_description/root_joint   (the only one)
DOF               : 12   (6 arm + 6 gripper)
Links             : 14
```

`onrobot_rg6/gripper_joint` is a `PhysicsFixedJoint` with `ArticulationRootAPI = False`, connecting
`cobotta_pro_J6` → `onrobot_rg6/base_link`. Arm and gripper are one PhysX articulation.

> An earlier scan using a bare `Usd.Stage.Open` *outside* the Isaac runtime reported a second
> articulation root on `gripper_joint`. But now there are onlyThat was wrong — the RG6 physics reference doesn't resolve
> outside the app and `PhysxSchema` isn't even importable there. Only in-app scans are authoritative.

### Link index table

`get_link_incoming_joint_force()` / `get_measured_joint_forces()` are **link-indexed**, shape `(14, 6)`
— *not* `num_dof + 1 = 13`. The fixed joint does get its own row. The docstring's "num_joint + 1"
wording is misleading.

| Idx | Link | Notes |
|----:|------|-------|
| 0 | `cobotta_pro_base_link` | |
| 1–6 | `cobotta_pro_J1` … `J6` | arm |
| **7** | **`base_link`** (RG6 base) | **← WRIST / FT FRAME** |
| 8–13 | knuckles + inner fingers | gripper |

**The fixed joint was not merged away** by PhysX — link 7 exists, so the measurement frame is real.
Resolve it by name (`get_link_indices("base_link")`), never hardcode `7`; the index depends on link ordering.

### Baseline readings (at rest, after enabling gravity)

| Row | Link | Fz (N) | Implied mass |
|----:|------|-------:|-------------:|
| 1 | `cobotta_pro_J1` | 68.669 | 7.0 kg |
| 6 | `cobotta_pro_J6` | 19.619 | 2.0 kg |
| **7** | **`base_link` (wrist)** | **9.809** | **1.0 kg** |
| 8 | `left_outer_knuckle` | 0.980 | 0.1 kg |
| 12 | `left_inner_finger` | 0.490 | 0.05 kg |

Consistent throughout: gripper authored mass = 0.7 + 6 × 0.05 = **1.0 kg**, and row 7 reads exactly
that weight. Ladder decrements 9.81 N per link up to 7.0 kg at J1. Total authored mass 8.0 kg.
Wrist torques ≈ 0.003–0.005 N·m — near zero because in the default pose the gripper CoM sits
essentially on the J6 axis. **These are the numbers to regression-check against.**

### API facts

| Thing | Finding |
|---|---|
| `Articulation.get_link_incoming_joint_force(link_indices=[...])` | In **non-deprecated** `isaacsim.core.experimental.prims`. Returns `(forces, torques)`, shapes `(N, L, 3)`. NVIDIA's own `IsaacArticulationState` node uses it. |
| `isaacsim.core.nodes.IsaacArticulationState` | **Cannot be used.** Its `pick_dofs()` resolves links from **DOF joints only**. The wrist is a *fixed* joint → link 7 unreachable. It can only reach J6 (row 6), which is the J5→J6 reaction *including J6's own 1 kg* — not the flange wrench. |
| `isaacsim.ros2.bridge.ROS2Publisher` | Generic; publishes any message type. **No `ROS2PublishWrench` node exists in 6.0.1** (checked all 34 ROS 2 nodes). |
| `WrenchStamped` dynamic attrs | Nested fields flatten with `:` — `inputs:header:stamp:sec` (int), `inputs:header:stamp:nanosec` (uint), `inputs:header:frame_id` (token), `inputs:wrench:force:{x,y,z}` (double), `inputs:wrench:torque:{x,y,z}` (double) |
| `isaacsim.core.nodes.IsaacTimeSplitter` | `inputs:time` (double) → `outputs:seconds` (int) + `outputs:nanoseconds` (uint). Types match the header stamp exactly. |
| `omni.graph.scriptnode` | Available (`2.10.2+110.0.0`). Attrs: `inputs:script`, `inputs:usePath`, `inputs:scriptPath`. |
| Physics backend | `SimulationApp({...})` with no `experience` resolves to `isaacsim.exp.base.python.kit` = **PhysX**. Do not switch to `isaacsim.exp.full.newton.kit` without re-verifying all of the above. |

---

## 2. Architecture

```
OnPlaybackTick ──> FTScriptNode ──(execOut)──> ROS2Publisher (WrenchStamped)
                        │  fx,fy,fz,tx,ty,tz ──────^
ReadSimTime ──> IsaacTimeSplitter ──> seconds/nanoseconds ──> header:stamp:*
```

A ScriptNode is required because `IsaacArticulationState` cannot reach a fixed-joint link (see §1).
Publishing still goes through the ROS 2 bridge's `ROS2Publisher` node.

New graph lives at `/ActionGraph_FT`, separate from the existing `/ActionGraph_Robot`.

---

## 3. Implementation steps

### Step 1 — enable the extension

In `scripts/main.py`, alongside the existing `set_extension_enabled_immediate` calls:

```python
ext_manager.set_extension_enabled_immediate("omni.graph.scriptnode", True)
```

### Step 2 — script node body

```python
FT_SCRIPT = """
import omni.graph.core as og
from isaacsim.core.experimental.prims import Articulation

def setup(db):
    db.per_instance_state.art = None
    db.per_instance_state.link_idx = None

def compute(db):
    st = db.per_instance_state
    if st.art is None:
        try:
            st.art = Articulation(db.inputs.robotPath)
        except Exception:
            return True
    art = st.art
    if not art.is_physics_tensor_entity_valid():
        return True                      # physics not up yet
    if st.link_idx is None:
        st.link_idx = int(art.get_link_indices(db.inputs.linkName).numpy().item())
    f, t = art.get_link_incoming_joint_force(link_indices=[st.link_idx])
    f = f.numpy().reshape(-1); t = t.numpy().reshape(-1)
    s = -1.0 if db.inputs.negate else 1.0
    db.outputs.fx, db.outputs.fy, db.outputs.fz = s*float(f[0]), s*float(f[1]), s*float(f[2])
    db.outputs.tx, db.outputs.ty, db.outputs.tz = s*float(t[0]), s*float(t[1]), s*float(t[2])
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
"""
```

### Step 3 — build the graph

Place **after `world.reset()`**, next to the existing `/ActionGraph_Robot` block.
`CREATE_ATTRIBUTES` is required — a ScriptNode has no `fx…tz` ports by default.

```python
og.Controller.edit(
    {"graph_path": "/ActionGraph_FT", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick",  "omni.graph.action.OnPlaybackTick"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Split",   "isaacsim.core.nodes.IsaacTimeSplitter"),
            ("FT",      "omni.graph.scriptnode.ScriptNode"),
            ("PubFT",   "isaacsim.ros2.bridge.ROS2Publisher"),
        ],
        og.Controller.Keys.CREATE_ATTRIBUTES: [
            ("FT.inputs:robotPath", "string"), ("FT.inputs:linkName", "token"),
            ("FT.inputs:negate", "bool"),
            ("FT.outputs:fx", "double"), ("FT.outputs:fy", "double"), ("FT.outputs:fz", "double"),
            ("FT.outputs:tx", "double"), ("FT.outputs:ty", "double"), ("FT.outputs:tz", "double"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("FT.inputs:script", FT_SCRIPT),
            ("FT.inputs:robotPath", ARTICULATION_PATH),   # already computed in main.py
            ("FT.inputs:linkName", "base_link"),          # RG6 base = wrist
            ("FT.inputs:negate", False),
            ("PubFT.inputs:messagePackage", "geometry_msgs"),
            ("PubFT.inputs:messageSubfolder", "msg"),
            ("PubFT.inputs:messageName", "WrenchStamped"),
            ("PubFT.inputs:topicName", "/ft_sensor"),
            ("PubFT.inputs:header:frame_id", "ft_sensor_frame"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "FT.inputs:execIn"),
            ("FT.outputs:execOut",  "PubFT.inputs:execIn"),
            ("SimTime.outputs:simulationTime", "Split.inputs:time"),
            ("Split.outputs:seconds",     "PubFT.inputs:header:stamp:sec"),
            ("Split.outputs:nanoseconds", "PubFT.inputs:header:stamp:nanosec"),
            ("FT.outputs:fx", "PubFT.inputs:wrench:force:x"),
            ("FT.outputs:fy", "PubFT.inputs:wrench:force:y"),
            ("FT.outputs:fz", "PubFT.inputs:wrench:force:z"),
            ("FT.outputs:tx", "PubFT.inputs:wrench:torque:x"),
            ("FT.outputs:ty", "PubFT.inputs:wrench:torque:y"),
            ("FT.outputs:tz", "PubFT.inputs:wrench:torque:z"),
        ],
    },
)
```

### Step 4 — build and verify

```bash
cd /home/anm-vi/Techtory && colcon build --packages-select techtory_cobotta_isaacsim
ros2 launch techtory_cobotta_isaacsim isaacsim_launcher.launch.py
# in another terminal:
ros2 topic echo /ft_sensor
ros2 topic hz  /ft_sensor
```

**Acceptance:** at rest, `force.z ≈ 9.81` and torques ≈ 0 (matching §1). Push on the gripper → values move.
If it reads all zeros, the script node is returning early — check `is_physics_tensor_entity_valid()`
and that `robotPath` actually got the articulation root.

> ⚠️ The individual pieces in §1 are all verified against the install. The **assembled graph in
> §3 has not been run yet** — expect to debug the ScriptNode attribute wiring on first launch.

---

## 4. Open decisions

1. **Sign convention.** Row 7 reads **+9.81 N** in Z while gravity pulls −Z, i.e. this is the
   *constraint force the joint exerts on the tool*. Many FT pipelines expect the opposite (force the
   environment applies to the tool). Decide, then set `inputs:negate` accordingly.
2. **`frame_id` + orientation.** The wrench is expressed in the **child link's joint frame**, which is
   not guaranteed to match the URDF link frame orientation in the TF tree. Verify before trusting X/Y —
   the Z magnitude alone will not catch a rotated frame. Pick a real frame name and make sure it exists in TF.
3. **Publish rate.** `OnPlaybackTick` = per *rendered* frame. Reaction forces update per *physics* step,
   so contact spikes get undersampled. `isaacsim.core.nodes.OnPhysicsStep` is the alternative.
4. **Raw vs conditioned.** Raw PhysX reaction forces are noise-free and exact. Real FT sensors have
   noise, bias drift, and finite bandwidth. If this feeds force control or contact detection, add
   Gaussian noise + low-pass + optional bias, or the logic gets tuned against unrealistically clean data.
5. **Gravity/payload compensation.** The 9.81 N baseline is the tool weight — exactly the bias term a
   real FT pipeline tares out. Decide whether to publish raw, tared, or both topics.

---

## 5. Gotchas

- **Watch for finger sag.** `disableGravity=True` on the RG6 links may well have been a workaround for
  drooping fingers or unstable grasps. That risk is now live. It won't show in a static probe — look for
  it with the gripper drives active. **Fix with drive stiffness/damping on the gripper joints, not by
  switching gravity back off.**
- **Arm masses are placeholders** (all 7 arm links = exactly 1.0 kg). Harmless for the wrist sensor —
  it only sees what is *distal* to it — but they make joint-effort values meaningless.
- **Gripper mass is 1.0 kg vs a real RG6's ~1.25 kg.** One-attribute change if the baseline needs to match hardware.
- **Don't hardcode link index 7.** Resolve by name.
- **Solver stiffness.** With low position-iteration counts, reaction forces ring on stiff contacts.
  May need `physxScene` iteration bumps if the signal looks noisy on impact.
- **`isaacsim.core.api` / `isaacsim.core.prims` are in `extsDeprecated`.** The plan above deliberately
  uses `isaacsim.core.experimental.prims` in the script node to avoid adding new deprecated-API usage.
  Note `main.py` still imports `World` from `isaacsim.core.api` — fine for now, but it is on a clock.

---

## 6. Simpler fallback

If the ScriptNode wiring proves painful: reduce the graph to `OnPlaybackTick → ROS2Publisher` and write
the six values into `PubFT`'s input attributes with `og.Controller.set()` from inside the existing
`while world.step()` loop in `main.py`, using
`art.get_link_incoming_joint_force(link_indices=[idx])` directly.

Fewer moving parts and the bridge still publishes — but it can lag one frame, and it breaks if the sim
is ever driven from the GUI instead of that Python loop.
