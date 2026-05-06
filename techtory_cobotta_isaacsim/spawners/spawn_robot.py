import os
import numpy as np
from pxr import UsdGeom, Gf

def add_robot(stage, prim_path: str, spawn_position=np.array([0.0, 0.0, 0.8])):
    # Standard ROS 2 / Ament imports
    from ament_index_python.packages import get_package_share_directory
    # Isaac Sim core imports
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from isaacsim.core.api.robots import Robot

    pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    robot_usd = os.path.join(pkg_path, 'assets', 'robots', 'cvrb0609_onrobot_gripper', 'cobotta_mit_gripper_f.usd')

    # Add the reference using Isaac Sim's utility
    add_reference_to_stage(usd_path=robot_usd, prim_path=prim_path)

    # update location of robot
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    
    # Clear any weird default transforms the original USD might have
    xform.ClearXformOpOrder()
    
    # Add a clean translation operation and set it to our spawn_position
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(float(spawn_position[0]), float(spawn_position[1]), float(spawn_position[2])))
    
    # The name parameter is arbitrary but required for the object registry
    cobotta_robot = Robot(prim_path=prim_path, 
                          name="cobotta",
                          )
    
    joint_positions = np.array([
        -0.942,
        0.0,
        -1.571,
        -3.031,
        1.543,
        0.102,
    ])

    # Apply positions
    cobotta_robot.set_joint_positions(joint_positions)

    print(f"Robot added at {prim_path} with its internal configurations.")
    
    print(cobotta_robot.dof_names)
    return cobotta_robot