from omni.isaac.kit import SimulationApp

CONFIG = {
    "headless": False
}

simulation_app = SimulationApp(CONFIG)

from pxr import Usd

# Get stage AFTER app starts
import omni.usd
stage = omni.usd.get_context().get_stage()

# ----------------------------
# Import your spawners
# ----------------------------
from spawners.spawn_scene import add_world
from spawners.spawn_robot import add_robot
from spawners.spawn_techtory_cell import add_table


def build_world(stage: Usd.Stage):
    """Compose full scene from modular USD spawners"""
    
    add_world(stage)
    add_table(stage, "/World/Table")
    add_robot(stage, "/World/Robot")

    print("✅ World fully composed")


# Build scene
build_world(stage)

# Keep simulation running
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()