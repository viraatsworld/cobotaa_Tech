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
ext_manager.set_extension_enabled_immediate("omni.isaac.core_nodes", True)
ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

simulation_app.update()

# IMPORT WORLD AFTER SIMULATION APP IS RUNNING
from omni.isaac.core import World
from spawners.spawn_scene import add_world
from spawners.spawn_robot import add_robot
from spawners.spawn_objects import add_techtory_cell, add_shelf
from spawners.spawn_camera import add_realsense_camera, attach_ros2_camera_graph

# 1. Initialize the World (This automatically creates the stage and Physics Scene)
world = World(stage_units_in_meters=1.0)
stage = world.scene.stage

robot_spawn_position = np.array([-0.275, -0.24, 0.95])
robot_rotation_deg = np.array([0.0, 0.0, 90.0])

def build_world():
    # Add static environment
    add_world(stage)
    add_techtory_cell(stage, "/World/TechtoryCell")  # prim_path unused (sublayer load)
    add_shelf(stage, "/World/Shelf")
    # Add RealSense rsd455 camera + ROS2 publishers (rgb + point cloud)
    camera_prim_path = add_realsense_camera(
        stage,
        prim_path="/World/Camera1",
        spawn_position=np.array([-0.65, 0.65, 2.0]),
        spawn_rotation_deg=np.array([0.0, 45.0, -45.0]),
    )
    attach_ros2_camera_graph(
        camera_prim_path=camera_prim_path,
        graph_path="/World/ROS_Camera1",
        rgb_topic="/camera1/rgb",
        pcl_topic="/camera1/points",
        frame_id="camera1_optical_frame",
        resolution=(640, 480),
    )

    # Add robot
    cobotta = add_robot(stage, "/World/Cobotta", spawn_position=robot_spawn_position, spawn_rotation_deg=robot_rotation_deg)
    
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

# 4. NOW you can safely set joint positions (Physics is initialized!)
joint_positions = np.array([0.0, 0.349066, 1.309, 0.0, 1.48353, 0.0])
# Tell Isaac Sim to only apply these to indices 0 through 5
arm_joint_indices = np.array([0, 1, 2, 3, 4, 5])
# Apply positions safely
cobotta_robot.set_joint_positions(joint_positions, joint_indices=arm_joint_indices)

# 5. Step the world instead of just updating the app
while simulation_app.is_running():
    world.step(render=True) # This steps physics, ROS clocks, and renders the frame

simulation_app.close()