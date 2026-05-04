import os

from ament_index_python import get_package_share_directory
from pxr import Usd, Sdf


pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    
# Path to your main Isaac script
ROBOT_USD_PATH = os.path.join(pkg_path, 'assets', '/workcells/techtory_cell/techtory_cell.usd')

def add_robot(stage: Usd.Stage, prim_path: str):
    """Loads robot USD into stage"""

    robot_prim = stage.DefinePrim(prim_path, "Xform")

    # Reference USD
    robot_prim.GetReferences().AddReference(ROBOT_USD_PATH)

    print(f"🤖 Robot added at {prim_path}")