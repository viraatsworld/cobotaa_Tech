# MUST be first — before ANY omni/pxr import
import os, sys
from isaacsim import SimulationApp

# Ensure the package root is on sys.path so sibling packages like `spawners` can be imported.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

simulation_app = SimulationApp({
    "headless": False,
    "width": 1440,
    "height": 900,
})

# Only import omni/pxr AFTER SimulationApp is created
import omni.usd
from pxr import Usd

# Get or create stage
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()

# Spawner imports also AFTER SimulationApp
from spawners.spawn_scene import add_world
from spawners.spawn_robot import add_robot
from spawners.spawn_techtory_cell import add_techtory_cell

def build_world(stage: Usd.Stage):
    add_world(stage)
    add_techtory_cell(stage, "/World/TechtoryCell")
    add_robot(stage, "/World/Cobotta")
    print("✅ World fully composed")

build_world(stage)

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()