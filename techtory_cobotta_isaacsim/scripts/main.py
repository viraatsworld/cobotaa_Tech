import os, sys
from isaacsim import SimulationApp

# Add correct paths before starting the app
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

simulation_app = SimulationApp({
    "headless": False,
    "width": 1440,
    "height": 900,
})

import omni.kit.app
import numpy as np

# Enable extensions
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.graph.bundle.action", True)
ext_manager.set_extension_enabled_immediate("omni.graph.nodes", True)
ext_manager.set_extension_enabled_immediate("isaacsim.core.nodes", True)
ext_manager.set_extension_enabled_immediate("isaacsim.core.api", True)
ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

simulation_app.update()

# IMPORT WORLD AFTER SIMULATION APP IS RUNNING
from isaacsim.core.api import World
from spawners.spawn_scene import add_world, add_cell_lights
from spawners.spawn_robot import (add_robot, set_initial_joint_positions, fix_gripper_collisions,
                                  disable_articulation_self_collisions, configure_gripper_drive,
                                  stabilize_gripper_joints, add_grip_friction,
                                  add_pad_contact_colliders)
from spawners.spawn_objects import (add_hammer, add_pallet, add_techtory_cell, add_shelf,
                                    configure_graspable_object)
from spawners.spawn_camera import add_realsense_camera, attach_ros2_camera_graph
from spawners.ft_sensor import WristFTSensor
from spawners.grip_contact_sensor import GripContactSensor

# --- Wrist force/torque sensor -------------------------------------------------
# Reads the flange <-> gripper reaction wrench (the RG6 base_link incoming joint)
# straight from PhysX. See spawners/ft_sensor.py and plan-ft-implement.md.
FT_ENABLE = True
FT_PUBLISH_ROS2 = True             # geometry_msgs/WrenchStamped on FT_TOPIC
FT_TOPIC = "/wrist_ft"
FT_FRAME_ID = "onrobot_rg6_base_link"
FT_NEGATE = True                  # True -> "force the environment applies to the tool"

# --- Grip-load sensor ------------------------------------------------------
# The wrist row above does NOT see a grasped payload -- its weight reaches the
# gripper only through pad<->object contact, which PhysX leaves out of the
# measured-joint-force table. Reading the contact directly does not work on this
# gripper either (articulation-link contact returns zero; the merged-mesh pad
# colliders emit no contact reports). So this sensor is a payload observer: it
# applies Newton's 2nd law to the grasped OBJECT -- wrench on the wrist =
# m*(g - a) with the moment arm to the wrist. /wrist_ft + /grip_contact then add
# up to the full wrench the tool feels. See spawners/grip_contact_sensor.py.
GRIP_CONTACT_ENABLE = True
GRIP_CONTACT_PUBLISH_ROS2 = True
GRIP_CONTACT_TOPIC = "/grip_contact"
GRIP_CONTACT_NEGATE = False        # False -> "force the environment applies to the tool"
GRIP_CONTACT_EMA = 0.25            # accel smoothing; 0 = weight only, no inertial term
GRIP_CONTACT_PAD_COLLIDERS = False # optional: plain box colliders on the pads (grip aid only,
                                   # the observer does not need them)
# The observer auto-finds whatever dynamic rigid body sits in the jaw, so cubes
# you spawn in the GUI need no config. GRIP_CONTACT_OBJECT_PATHS force-tracks
# extra bodies on top of that (leave empty to rely on auto-discovery alone).
GRIP_CONTACT_AUTO_DISCOVER = True
GRIP_CONTACT_OBJECT_PATHS = []

# 1. Initialize the World (This automatically creates the stage and Physics Scene)
world = World(stage_units_in_meters=1.0)
stage = world.scene.stage

# Must match the URDF mount chain exactly, otherwise Isaac and TF/MoveIt disagree:
#   cell_link -> robot_base_plate_link  xyz (-0.275, -0.24, 0.94)   (techtory_cell.xacro)
#   robot_base_plate_link -> cobotta_pro_base_link  xyz (0, 0, 0.02) rpy (0, 0, 1.5708)  (joint_w)
# The base collider starts at its own z=0 and the plate's top face is at 0.956, so anything
# below 0.956 buries the base in the plate; PhysX then fights that penetration every step and
# the arm reads as "stuck" unless collisions are switched off.
robot_spawn_position = np.array([-0.275, -0.24, 0.96])
robot_rotation_deg = np.array([0.0, 0.0, 90.0])

# Start pose, in joint order. The USD default is all-zero, which points the arm straight up
# and puts the wrist through the cell roof, so this has to be authored before physics starts.
HOME_JOINT_POSITIONS = {
    "cobotta_pro_joint_1": 0.0,
    "cobotta_pro_joint_2": 0.349066,
    "cobotta_pro_joint_3": 1.309,
    "cobotta_pro_joint_4": 0.0,
    "cobotta_pro_joint_5": 1.48353,
    "cobotta_pro_joint_6": 0.0,
}

def build_world():
    # Add static environment
    add_world(stage)
    add_techtory_cell(stage, "/World/TechtoryCell")  # prim_path unused (sublayer load)
    # After the cell: the panels hang under its roof, which occludes the global lights.
    add_cell_lights(stage)
    add_shelf(stage, "/World/Shelf")
    add_hammer(stage, "/World/Shelf/Hammer")
    # Must be a sibling of the shelf, not a child: add_pallet authors the pose from the
    # xacro (world-relative, xyz -0.16 0.3 0.94), and under /World/Shelf that would compose
    # with the shelf's own translate (0.61, 0.27, 0.94) + yaw 90 deg -> (0.31, 0.11, 1.88).
    add_pallet(stage, "/World/Pallet")

    configure_graspable_object(stage, "/World/Shelf/Hammer")
    # Add RealSense rsd455 camera + ROS2 publishers (rgb + point cloud)
    # camera_prim_path = add_realsense_camera(
    #     stage,
    #     prim_path="/World/Camera1",
    #     spawn_position=np.array([-0.65, 0.65, 2.0]),
    #     spawn_rotation_deg=np.array([0.0, 45.0, -45.0]),
    # )
    # attach_ros2_camera_graph(
    #     camera_prim_path=camera_prim_path,
    #     graph_path="/World/ROS_Camera1",
    #     rgb_topic="/camera1/rgb",
    #     pcl_topic="/camera1/points",
    #     frame_id="camera1_optical_frame",
    #     resolution=(640, 480),
    # )

    # Add robot
    cobotta = add_robot(stage, "/World/Cobotta", spawn_position=robot_spawn_position, spawn_rotation_deg=robot_rotation_deg)


    # Must happen before world.reset(), while this is still just USD authoring.
    set_initial_joint_positions(stage, "/World/Cobotta", HOME_JOINT_POSITIONS)

    # Same window: the gripper only becomes a physical object here. As imported it has no
    # collision shapes at all (empty mesh-merge collections) and an effectively unlimited
    # drive, which is the pair of reasons the fingers pass through objects and then slam
    # shut. Order matters -- giving the links shapes without disabling self-collision jams
    # the four-bar linkage.
    fix_gripper_collisions(stage, "/World/Cobotta/onrobot_rg6")
    disable_articulation_self_collisions(stage, "/World/Cobotta")
    configure_gripper_drive(stage)
    # Caps how hard the fingers squeeze
    stabilize_gripper_joints(stage, "/World/Cobotta/onrobot_rg6")
    add_grip_friction(stage, "/World/Cobotta/onrobot_rg6")
    # Optional plain box colliders + contact reporting on the pads. The grip-load
    # observer does not need these; they are a grip aid / a hook for a future
    # contact-based path. Off by default.
    if GRIP_CONTACT_PAD_COLLIDERS:
        add_pad_contact_colliders(stage, "/World/Cobotta/onrobot_rg6")


    # 2. Add the robot to the World scene so Isaac Sim tracks its physics
    world.scene.add(cobotta)
    return cobotta

cobotta_robot = build_world()
print("World fully composed")

# 3. Reset the world. This is CRITICAL. It starts the timeline and initializes robot articulations.
world.reset()

# Tick-driven ROS2 bridge for the robot — replaces the action graph baked into
# the workcell USD, which we deactivate below to avoid duplicate publishers.
import omni.graph.core as og
from pxr import Usd, UsdPhysics

def _find_articulation_root(stage, search_root: str) -> str:
    root_prim = stage.GetPrimAtPath(search_root)
    if not root_prim or not root_prim.IsValid():
        raise RuntimeError(f"Prim {search_root} does not exist")
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            path = prim.GetPath().pathString
            print(f"Articulation root discovered at: {path}")
            return path
    raise RuntimeError(f"No PhysicsArticulationRootAPI found under {search_root}")

ARTICULATION_PATH = _find_articulation_root(stage, "/World/Cobotta")

og.Controller.edit(
    {"graph_path": "/ActionGraph_Robot", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishJS", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("SubscribeJS", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArtCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "PublishJS.inputs:execIn"),
            ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnTick.outputs:tick", "SubscribeJS.inputs:execIn"),
            ("OnTick.outputs:tick", "ArtCtrl.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishJS.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("SubscribeJS.outputs:jointNames",      "ArtCtrl.inputs:jointNames"),
            ("SubscribeJS.outputs:positionCommand", "ArtCtrl.inputs:positionCommand"),
            ("SubscribeJS.outputs:velocityCommand", "ArtCtrl.inputs:velocityCommand"),
            ("SubscribeJS.outputs:effortCommand",   "ArtCtrl.inputs:effortCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishJS.inputs:topicName", "/topic_based_joint_states"),
            ("PublishJS.inputs:targetPrim", ARTICULATION_PATH),
            ("SubscribeJS.inputs:topicName", "/topic_based_joint_commands"),
            ("ArtCtrl.inputs:targetPrim", ARTICULATION_PATH),
        ],
    },
)

# 4. The pose was authored into the USD before reset, so the articulation already came up in
# it. Re-assert it here and register it as the default state so any later world.reset() also
# returns to it rather than to the all-zero pose that intersects the cell.
joint_positions = np.array(list(HOME_JOINT_POSITIONS.values()))
# Tell Isaac Sim to only apply these to indices 0 through 5
arm_joint_indices = np.array([0, 1, 2, 3, 4, 5])
cobotta_robot.set_joint_positions(joint_positions, joint_indices=arm_joint_indices)

# Default state covers every DOF; the gripper DOFs stay at 0 (open).
default_positions = np.zeros(len(cobotta_robot.dof_names))
default_positions[arm_joint_indices] = joint_positions
cobotta_robot.set_joints_default_state(positions=default_positions)

# 4b. Wrist FT sensor. Handles exist now that world.reset() has run; it resolves
# the base_link row on the first read once physics is fully live.
ft_sensor = None
if FT_ENABLE:
    ft_sensor = WristFTSensor(cobotta_robot, link_name="base_link", negate=FT_NEGATE)
    if FT_PUBLISH_ROS2:
        ft_sensor.try_enable_ros2(topic=FT_TOPIC, frame_id=FT_FRAME_ID)

# 4c. Grip-load sensor. Built AFTER world.reset(): it is a pure observer (no
# contact reporters to register during the scene parse), and its prim views must
# bind to the simulation view that reset() just created, not to the pre-reset one
# that reset() invalidates.
grip_sensor = None
if GRIP_CONTACT_ENABLE:
    grip_sensor = GripContactSensor(world, GRIP_CONTACT_OBJECT_PATHS,
                                    wrist_prim_path="/World/Cobotta/onrobot_rg6/base_link",
                                    auto_discover=GRIP_CONTACT_AUTO_DISCOVER,
                                    negate=GRIP_CONTACT_NEGATE, ema=GRIP_CONTACT_EMA)
    grip_sensor.prepare()
    if GRIP_CONTACT_PUBLISH_ROS2:
        grip_sensor.try_enable_ros2(topic=GRIP_CONTACT_TOPIC, frame_id=FT_FRAME_ID)

# 5. Step the world instead of just updating the app
while simulation_app.is_running():
    world.step(render=True) # This steps physics, ROS clocks, and renders the frame

    if ft_sensor is not None:
        force, torque = ft_sensor.read()
        if force is not None:
            ft_sensor.publish(force, torque, sim_time=world.current_time)

    if grip_sensor is not None:
        g_force, g_torque = grip_sensor.read()
        if g_force is not None:
            grip_sensor.publish(g_force, g_torque, sim_time=world.current_time)

if ft_sensor is not None:
    ft_sensor.shutdown()
if grip_sensor is not None:
    grip_sensor.shutdown()
simulation_app.close()