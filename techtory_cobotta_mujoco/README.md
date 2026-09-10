# techtory_cobotta_mujoco

MuJoCo simulation of the Techtory Cobotta workcell: a Denso CVRB0609 arm with an OnRobot RG6
gripper, mounted in the Techtory cell, with a 6-axis force/torque sensor at the wrist.

```bash
ros2 launch techtory_cobotta_mujoco techtory_cobotta_mujoco.launch.py
```

This brings up MuJoCo, `ros2_control`, MoveIt and RViz. Plan and execute from the RViz
MotionPlanning panel, or drive the controllers directly:

```bash
# arm  -- FollowJointTrajectory over cobotta_pro_joint_1..6
/denso_joint_trajectory_controller/follow_joint_trajectory

# gripper -- ParallelGripperCommand; -0.628 rad opens, +0.628 rad closes
ros2 action send_goal /onrobot_rg6/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: [finger_joint], position: [0.62]}}" --feedback

# wrist wrench -- geometry_msgs/WrenchStamped, frame_id `base_link` (the RG6 base)
ros2 topic echo /tcp_wrench
```

## Launch arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `headless_simulation` | `false` | Run MuJoCo without a viewer window |
| `use_moveit` | `true` | Start `move_group` |
| `use_rviz` | `true` | Start RViz |
| `work_dir` | *(temp)* | Where the generated MJCF, meshes and URDF are written |
| `rviz_config` | *(MoveIt's)* | RViz configuration override |
| `initial_positions_file` | `config/initial_positions.yaml` | Startup joint positions |
| `actuator_config` | `config/actuators.yaml` | MuJoCo servo gains and force limits |

## How the scene is built

There is no checked-in MJCF. `model.py` generates one at launch time from the installed
description, the same approach `rox_fr3_demo_mujoco` uses:

1. `techtory_cobotta_mujoco.urdf.xacro` expands
   `techtory_cobotta_workcell_description/urdf/techtory_cobotta_workcell.urdf.xacro` with
   `hardware_type:=mujoco`, and supplies the `cobotta_mujoco_ros2_control` macro that the
   workcell xacro calls but no package previously defined.
2. `generate_scene()` converts the `.dae`/`.STL`/`.stl` meshes to binary STL, imports the URDF
   through MuJoCo's own importer, then post-edits the saved MJCF to add the ground plane,
   lighting, position servos, the RG6 mimic equality constraints and the FT sensor.
3. The launch file rewrites the `mujoco_model` hardware parameter to the generated path and
   hands the result to `robot_state_publisher`, the simulator and MoveIt.

Three corrections are applied along the way, each of which is otherwise a silent failure:

- **Visual meshes are excluded from collision.** MuJoCo's URDF importer makes geoms out of
  both `<visual>` and `<collision>`, and both collide. Left alone, every link's visual mesh
  contacts its own collision mesh and the solver diverges within a second.
- **The URDF's placeholder effort limit is overridden.** `<limit effort="1">` becomes a
  *per-joint* `actuatorfrcrange`, which is applied on top of the actuator's own `forcerange`
  — tighter wins. At 1 N·m the arm cannot hold itself up. The joint limit is widened to match
  the servo configured in `config/actuators.yaml`.
- **SRDF `disable_collisions` pairs become `<contact><exclude>`.** The RG6's four-bar linkage
  is a closed loop held by equality constraints, so its knuckles and fingers overlap by
  design. Reusing the SRDF keeps the simulator's contact set and MoveIt's collision matrix in
  agreement rather than maintaining a second list.

The `cell_link` shell is imported visual-only: MuJoCo collides meshes as convex hulls, and the
cell fills just 18% of its own hull, so as a collider it is a solid 7.8 m³ block with the robot
inside it. The shelf (boxes) and the base plate (95% convex) collide normally.

## The wrist force/torque sensor

A `force`/`torque` sensor pair sits on a site inside the RG6 `base_link` body. MuJoCo reports
the interaction wrench between a site's body and its parent, so this reads the
`cobotta_pro_J6` -> gripper interface — where a real wrist FT sensor bolts in, and the same
quantity `techtory_cobotta_isaacsim/spawners/ft_sensor.py` samples from PhysX.

The MJCF sensor names (`tcp_fts_force`, `tcp_fts_torque`) must stay in step with
`mujoco_sensor_name` in `urdf/cobotta_mujoco.ros2_control.xacro`. **A mismatch fails
silently** — one `RCLCPP_ERROR` and the six state interfaces never appear, which surfaces
only as a `tcp_force_torque_sensor_broadcaster` that will not activate.

## Caveats

- `denso_robot_descriptions` authors placeholder dynamics for the CVRB0609: every link is
  1 kg with an identity inertia tensor and every joint carries `effort="1"`. Servo gains
  therefore come from `config/actuators.yaml`, not the URDF. Motion is kinematically faithful;
  wrench magnitudes are **indicative, not calibrated**.
- The RG6 is instantiated with an empty prefix in the workcell xacro, so its base link — and
  hence `/tcp_wrench`'s `frame_id` — is the generic name `base_link`.
- Startup pose comes from `techtory_cobotta_isaacsim`, which differs from the three other
  "initial pose" definitions in this repo (`techtory_cobotta_moveit/config/initial_positions.yaml`,
  the SRDF `home` state, and `cobotta_sphere_cumotion.xrdf`).
- `force_torque_sensor_broadcaster` has no `topic_name` parameter on Jazzy; it publishes to
  `~/wrench`. `/tcp_wrench` comes from a remap passed through the spawner's per-controller
  mode, so renaming the topic means editing the launch file, not the YAML.
- `move_group` logs two `occupancy_map_monitor` plugin-load errors on startup. Those come from
  `techtory_cobotta_moveit/config/sensors_3d.yaml` referencing octomap updaters that are not
  installed in this workspace (including a leftover Kinect entry); planning is unaffected.
  Pre-existing, not introduced here.

## Requirements

The scene generator needs the Python MuJoCo bindings (the simulator itself uses the C library
from `mujoco_vendor`):

```bash
.venv-mujoco/bin/pip install -r requirements.txt
source setup_mujoco.bash
```
