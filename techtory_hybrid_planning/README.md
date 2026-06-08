# techtory_hybrid_planning

Example package demonstrating how to use the [MoveIt 2 Hybrid Planning
architecture](https://moveit.picknik.ai/main/doc/concepts/hybrid_planning/hybrid_planning.html)
on the Techtory Cobotta cell.

It launches the three hybrid planning components, wires them up to the
existing cobotta MoveIt configuration, and runs a small C++ action client that
sends a single `HybridPlanner` goal so the motion can be observed in RViz.

---

## 1. What is the Hybrid Planner?

Classical MoveIt motion planning is "plan once, execute blindly": OMPL or
Pilz produce a full trajectory and `MoveItSimpleControllerManager` hands it
to a controller. If the world changes mid-motion (a person enters the cell,
a part is moved), there is no built-in way to *react*.

Hybrid Planning splits motion generation into two loops running in parallel:

```
                +----------------------------------+
                |   Hybrid Planning Manager        |
   user goal -->|   (event-driven coordinator,     |<-- feedback
                |    PlannerLogic plugin)          |
                +---------------+------------------+
                                |
                  GlobalPlanner |  LocalPlanner
                       action   |     action
                                v
        +---------------+   +-----------------+   +----------------+
        |   Global      |   |    Local        |   |  ros2_control  |
        |   Planner     |-->|    Planner      |-->|   controller   |
        | (slow, full)  |   | (fast, reactive)|   |  (real robot)  |
        +---------------+   +-----------------+   +----------------+
```

### The three components

| Component | Role | Action | Action type |
|---|---|---|---|
| **Hybrid Planning Manager** | Receives the user request, orchestrates the global and local planners through an event-driven *PlannerLogic* plugin. | `/run_hybrid_planning` | `moveit_msgs/action/HybridPlanner` |
| **Global Planner** | Computes a full collision-free trajectory using a standard MoveIt planning pipeline (OMPL by default). Slow (≈100 ms - seconds), runs on demand. | `/global_planning_action` | `moveit_msgs/action/GlobalPlanner` |
| **Local Planner** | Runs at high frequency (e.g. 100 Hz). Samples the global trajectory, re-checks it against the live planning scene and outputs commands to a controller. | `/local_planning_action` | `moveit_msgs/action/LocalPlanner` |

### The plugins

Behaviour is determined by four plugin types, all configured via YAML:

| Plugin type | Belongs to | Interface | Stock implementation |
|---|---|---|---|
| `planner_logic_plugin_name` | Manager | `PlannerLogicInterface` | `moveit_hybrid_planning/SinglePlanExecution`, `moveit_hybrid_planning/ReplanInvalidatedTrajectory` |
| `global_planner_name` | Global Planner | `GlobalPlannerInterface` | `moveit_hybrid_planning/MoveItPlanningPipeline` |
| `trajectory_operator_plugin_name` | Local Planner | `TrajectoryOperatorInterface` | `moveit_hybrid_planning/SimpleSampler` |
| `local_constraint_solver_plugin_name` | Local Planner | `LocalConstraintSolverInterface` | `moveit_hybrid_planning/ForwardTrajectory` |

The defaults (`ReplanInvalidatedTrajectory` + `MoveItPlanningPipeline` +
`SimpleSampler` + `ForwardTrajectory`) already give you:

* a full OMPL plan,
* live collision re-checking against `/planning_scene`,
* automatic re-planning if the trajectory becomes invalid,
* a stop-on-collision safety behaviour (`stop_before_collision: true`).

To add real reactivity (e.g. dynamic obstacle avoidance, force-control,
admittance), swap the local constraint solver plugin for a custom one that
implements `LocalConstraintSolverInterface`.

### When to use Hybrid Planning

* The planning scene changes during execution (humans, conveyors, sensed
  obstacles).
* You want online retiming, retargeting or compliance on top of a
  collision-free reference path.
* You need re-planning that is **faster than re-issuing a full MoveGroup
  action**, because the local loop keeps the robot moving on the still-valid
  prefix of the trajectory while the global planner replans.

For static cells with a pre-recorded scene, the regular `MoveGroupInterface`
is simpler and sufficient.

---

## 2. Package layout

```
techtory_hybrid_planning/
├── CMakeLists.txt
├── package.xml
├── README.md                       (this file)
├── config/
│   ├── hybrid_planning_manager.yaml
│   ├── global_planner.yaml
│   └── local_planner.yaml
├── launch/
│   ├── hybrid_planning.launch.py        # manager + global + local only
│   └── hybrid_planning_demo.launch.py   # all-in-one demo
└── src/
    └── hybrid_planning_demo_node.cpp    # example HybridPlanner action client
```

The configs are adapted from the upstream `moveit_hybrid_planning` defaults.
The notable customisations for the cobotta:

* `group_name: "arm"` (from the SRDF in `techtory_cobotta_moveit`).
* `local_solution_topic: /denso_joint_group_position_controller/commands`
  with type `std_msgs/Float64MultiArray` — this matches the controller
  spawned by `techtory_cobotta_sw_bringup` by default.
* Pilz industrial motion planner is used as the global pipeline (PTP).

---

## 3. Build

From the workspace root:

```bash
cd /home/ipa326/ros_ws/arena/colcon_techtory_ws
colcon build --packages-select techtory_hybrid_planning
source install/setup.bash
```

---

## 4. Run the demo

```bash
ros2 launch techtory_hybrid_planning hybrid_planning_demo.launch.py
```

This:

1. Brings up the cobotta workcell via
   `techtory_cobotta_bringup/techtory_cobotta_sw_bringup.launch.py` (RViz,
   ros2_control with `denso_joint_group_position_controller`,
   robot_state_publisher).
2. After 4 s, starts the hybrid planning composable container with the
   `global_planner`, `local_planner` and `hybrid_planning_manager` nodes.
3. After 8 s, launches the demo action client which sends one
   `HybridPlanner` goal: move the arm to a non-home joint configuration.

You should see the arm move smoothly in RViz. The client prints feedback
strings published by the planner logic plugin, and finally a
`MoveItErrorCodes` result.

### Run the components only (no demo client, no bringup)

Useful when you already have the workcell running and want to drive the
hybrid planner from your own application:

```bash
ros2 launch techtory_hybrid_planning hybrid_planning.launch.py
```

Then send a goal yourself, e.g. from another node, or with `ros2 action
send_goal /run_hybrid_planning moveit_msgs/action/HybridPlanner "..."`.

---

## 5. The example client in detail

`src/hybrid_planning_demo_node.cpp` builds a `HybridPlanner` goal by:

1. Constructing a `moveit_msgs/MotionPlanRequest` for the `arm` group with a
   joint-space `Constraints` goal.
2. Wrapping it in a single `MotionSequenceItem` (the `HybridPlanner` goal
   takes a `MotionSequenceRequest`, so multi-segment goals are possible).
3. Calling `/run_hybrid_planning` and printing the feedback / result.

Parameters (override via launch or `--ros-args`):

| Parameter | Default | Meaning |
|---|---|---|
| `planning_group` | `arm` | SRDF group to plan for. |
| `hybrid_planning_action` | `/run_hybrid_planning` | Action server exposed by the manager. |
| `joint_names` | the 6 `cobotta_pro_joint_*` joints | Joint goal target. |
| `target_joint_values` | `[0, -0.5, -1.2, 0, 1.5, 0]` | Joint goal positions. |

---

## 5b. demo3 — MTC welding global planner

`demo3` swaps the stock Global Planner for a **MoveIt Task Constructor**
plugin shipped by this package
(`techtory_hybrid_planning/GlobalMTCPlannerComponent`, in
`src/global_mtc_planner.cpp`). Instead of joint-space goals it demonstrates a
Cartesian **approach → weld → retreat** task built from a *start* and a *goal*
pose.

The client (`src/hybrid_planning_demo3_node.cpp`) and the plugin share a
non-standard "welding" request layout: a single `MotionSequenceItem` whose
`goal_constraints[0]` carries **two** `position_constraints` and **two**
`orientation_constraints`:

| Field | Meaning |
|---|---|
| `position_constraints[0].target_point_offset` + `orientation_constraints[0]` | start pose (world frame) |
| `position_constraints[1].target_point_offset` + `orientation_constraints[1]` | goal pose (world frame) |

> Note: `target_point_offset` is normally a *link-frame offset*, not a world
> position. Here it is repurposed as a private channel between this client and
> this plugin only; do not feed these requests to stock MoveIt constraint code.

The plugin offsets the start/goal poses by −0.1 m along the TCP z-axis to form
the approach/retreat waypoints, then plans each segment with Pilz `LIN`.

```bash
ros2 launch techtory_hybrid_planning hybrid_planning_demo3.launch.py
```

The two poses are baked into the demo3 node and overridable via the
`weld_start.{x,y,z,roll,pitch,yaw}` / `weld_goal.{...}` parameters (specified in
`planning_frame`, default `cobotta_pro_base_link`). The launch file loads
`config/global_planner_mtc.yaml` (which only overrides `global_planner_name`),
so demo / demo2 are unaffected.

Requires the `ros-jazzy-moveit-task-constructor-core` and
`ros-jazzy-moveit-task-constructor-msgs` packages.

---

## 6. Tuning and customisation

* **Replanning behaviour** — switch
  `planner_logic_plugin_name` in
  `config/hybrid_planning_manager.yaml`:
  * `SinglePlanExecution`: plan once, never replan (most similar to
    `MoveGroup`).
  * `ReplanInvalidatedTrajectory` (default here): trigger a global replan
    whenever the local planner reports the reference trajectory has become
    invalid.
* **Global pipeline** — edit `config/global_planner.yaml`:
  `planning_pipelines.pipeline_names` and `plan_request_params.planner_id`.
  Pilz or `isaac_ros_cumotion` can be plugged in here.
* **Local controller** — edit `config/local_planner.yaml`:
  * Position streaming (default): `local_solution_topic_type:
    std_msgs/Float64MultiArray` to `denso_joint_group_position_controller`.
  * Trajectory streaming: switch type to `trajectory_msgs/JointTrajectory`
    and topic to `/denso_joint_trajectory_controller/joint_trajectory`, and
    spawn that controller instead.
* **Custom local solver** — implement
  `moveit::hybrid_planning::LocalConstraintSolverInterface` in your own
  package, export it as a pluginlib plugin, and set
  `local_constraint_solver_plugin_name` accordingly. This is the typical
  extension point for reactive behaviours (obstacle avoidance, compliance,
  servoing).

---

## 7. Troubleshooting

* **"HybridPlanner action server not available"** — the manager node failed
  to start. Check the container logs for plugin-load errors; missing
  `robot_description_semantic` is the usual culprit.
* **"Failed to fetch current robot state"** — `/joint_states` is not being
  published, or the local planner started before the controller manager was
  ready. Increase the `TimerAction` delay in the demo launch.
* **Robot does not move but goal succeeds** — the local planner is publishing
  to a controller that is not active. Check
  `ros2 control list_controllers`; the controller named in
  `local_solution_topic` must be in the `active` state.
* **Robot jitters / overshoots** — `ForwardTrajectory` is a pass-through; if
  your controller smoothing is off the high-rate stream can look rough. Drop
  `local_planning_frequency` to 50 Hz or switch to the joint-trajectory
  variant.

---

## 8. References

* Concept: <https://moveit.picknik.ai/main/doc/concepts/hybrid_planning/hybrid_planning.html>
* Tutorial: <https://moveit.picknik.ai/main/doc/examples/hybrid_planning/hybrid_planning_tutorial.html>
* Source: `moveit2/moveit_ros/hybrid_planning` in the
  [moveit2](https://github.com/moveit/moveit2) repository.
