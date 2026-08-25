import math
import os
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
from ament_index_python.packages import get_package_share_directory
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot # Corrected import

def add_robot(stage, prim_path: str, spawn_position=np.array([0.0, 0.0, 0.8]), spawn_rotation_deg=np.array([0.0, 0.0, 90.0])):
    pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    robot_usd = os.path.join(pkg_path, 'assets', 'robots', 'cvrb0609', 'cvrb0609_with_graph2.usd')

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

def filter_mount_collisions(stage, robot_prim_path: str, mount_prim_paths,
                            base_link_name: str = "cobotta_pro_base_link"):
    """Stop PhysX from ever evaluating contacts at the robot's mounting interface.

    The base is bolted to the cell's base plate, so those bodies are permanently in
    contact by construction. Left unfiltered, any overlap between the base collider and
    the plate/cell colliders feeds the articulation solver a penetration it can never
    resolve (both articulations are fixed-base), which shows up as the arm being stuck.
    MoveIt already ignores the same pairs via disable_collisions in the SRDF
    (cobotta_pro_base_link vs robot_base_plate_link / cell_link); this keeps the physics
    side consistent without disabling collisions scene-wide.
    """
    base_link = None
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if path.startswith(robot_prim_path + "/") and prim.GetName() == base_link_name:
            base_link = prim
            break

    if base_link is None:
        print(f"WARNING: {base_link_name} not found under {robot_prim_path}; mount collisions not filtered")
        return

    filtered = UsdPhysics.FilteredPairsAPI.Apply(base_link)
    rel = filtered.CreateFilteredPairsRel()
    for mount_path in mount_prim_paths:
        mount_prim = stage.GetPrimAtPath(mount_path)
        if not mount_prim or not mount_prim.IsValid():
            print(f"WARNING: mount prim {mount_path} not found; skipping collision filter")
            continue
        rel.AddTarget(mount_path)
        print(f"Filtered collisions between {base_link.GetPath()} and {mount_path}")


def set_initial_joint_positions(stage, robot_prim_path: str, joint_positions_rad: dict):
    """Author the start pose into the USD *before* physics starts.

    The USD ships every joint at 0, which stands the arm straight up: J6 ends at z=2.23
    while the cell roof is at ~2.05-2.13, so the wrist and gripper spawn inside/through the
    cell's top panel. PhysX then has to resolve a deep penetration on the very first step
    and the articulation blows up. Setting the joint state after world.reset() is too late --
    the articulation has already been created in the broken pose. Writing the joint state and
    the drive target here means the robot is never in that configuration at all.

    Angles are given in radians (ROS convention) and written in degrees (USD convention).
    """
    from pxr import Usd

    root = stage.GetPrimAtPath(robot_prim_path)
    if not root or not root.IsValid():
        raise RuntimeError(f"Prim {robot_prim_path} does not exist")

    remaining = dict(joint_positions_rad)
    for prim in Usd.PrimRange(root):
        name = prim.GetName()
        if name not in remaining:
            continue
        degrees = math.degrees(remaining.pop(name))
        for attr_name in ("state:angular:physics:position", "drive:angular:physics:targetPosition"):
            attr = prim.GetAttribute(attr_name)
            if not attr:
                print(f"WARNING: {name} has no {attr_name}; start pose may not hold")
                continue
            attr.Set(degrees)
        velocity = prim.GetAttribute("state:angular:physics:velocity")
        if velocity:
            velocity.Set(0.0)
        print(f"Start pose {name} = {degrees:.3f} deg")

    if remaining:
        print(f"WARNING: joints not found under {robot_prim_path}: {sorted(remaining)}")
