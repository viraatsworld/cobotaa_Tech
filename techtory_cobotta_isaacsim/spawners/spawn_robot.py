# spawners/spawn_robot.py
import os

def add_robot(stage, prim_path: str):
    # All imports inside the function — never at top level
    from ament_index_python.packages import get_package_share_directory
    from pxr import Sdf

    pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    robot_usd = os.path.join(pkg_path, 'assets', 'robots', 'cvrb0609_onrobot_gripper', 'cobotta_mit_gripper_f.usd')

    robot_prim = stage.DefinePrim(prim_path, "Xform")
    robot_prim.GetReferences().AddReference(robot_usd)
    print(f"🤖 Robot added at {prim_path}")