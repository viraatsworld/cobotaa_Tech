import os
import numpy as np
from pxr import UsdGeom, Gf
from ament_index_python.packages import get_package_share_directory
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot # Corrected import

def add_robot(stage, prim_path: str, spawn_position=np.array([0.0, 0.0, 0.8]), spawn_rotation_deg=np.array([0.0, 0.0, 90.0])):
    pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    robot_usd = os.path.join(pkg_path, 'assets', 'robots', 'cvrb0609_onrobot_gripper', 'cobotta_mit_gripper_f.usd')

    # Add the reference
    add_reference_to_stage(usd_path=robot_usd, prim_path=prim_path)
    
    # Update location
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    
    #translation
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(float(spawn_position[0]), float(spawn_position[1]), float(spawn_position[2])))

    #rotation
    rotate_op = xform.AddRotateXYZOp()
    rotate_op.Set(Gf.Vec3d(float(spawn_rotation_deg[0]), float(spawn_rotation_deg[1]), float(spawn_rotation_deg[2])))

    # Create the Robot wrapper instance
    cobotta_robot = Robot(prim_path=prim_path, name="cobotta")

    # DO NOT set joint positions here. The Articulation is not initialized yet.
    print(f"Robot loaded at {prim_path}. Articulation will initialize on world.reset()")
    
    return cobotta_robot