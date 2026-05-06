import os, sys
from isaacsim import SimulationApp

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

simulation_app = SimulationApp({
    "headless": False,
    "width": 1440,
    "height": 900,
})

import omni.usd
import omni.kit.app
from pxr import Usd
import numpy as np

# Enable extensions BEFORE touching the stage
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.graph.bundle.action", True)
ext_manager.set_extension_enabled_immediate("omni.graph.nodes", True)
ext_manager.set_extension_enabled_immediate("omni.isaac.core_nodes", True)
ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)



# Let the app tick once so extensions finish initializing
simulation_app.update()

# NOW open a new stage — after everything is stable
omni.usd.get_context().new_stage()
simulation_app.update()  # tick again after stage open

stage = omni.usd.get_context().get_stage()


from spawners.spawn_scene import add_world
from spawners.spawn_robot import add_robot
from spawners.spawn_techtory_cell import add_techtory_cell

robot_spawn_position = np.array([-0.275, -0.24, 0.95])
robot=None

def build_world(stage):
    add_world(stage)
    add_techtory_cell(stage, "/World/TechtoryCell")
    add_robot(stage, "/World/Cobotta", spawn_position=robot_spawn_position)
    print("✅ World fully composed")

build_world(stage)

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()  