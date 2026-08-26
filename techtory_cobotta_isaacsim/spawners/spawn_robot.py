import math
import os
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
from ament_index_python.packages import get_package_share_directory
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot # Corrected import

# Grip torque limit on finger_joint, in N*m. The RG6 datasheet range is 25-120 N of finger
# force and onrobot_rg_control uses a 0.080 m lever, so ~2 N*m (soft) to ~10 N*m (full).
# 5 N*m is roughly the middle of the band, i.e. ~60 N.
DEFAULT_GRIP_TORQUE = 5.0
# Drive gains, in USD units: angular drive stiffness/damping are authored *per degree*.
GRIP_DRIVE_STIFFNESS = 3.0
GRIP_DRIVE_DAMPING = 0.1

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




def fix_gripper_collisions(stage, gripper_prim_path: str = "/World/Cobotta/onrobot_rg6"):
    """Give the RG6 links real collision shapes. Without this the gripper is a ghost.

    Every RG6 link's `collisions` Xform carries PhysxMeshMergeCollisionAPI: instead of
    colliding the meshes underneath it directly, PhysX merges whatever the prim's
    `collisionmeshes` collection resolves to into one collision shape. The asset ships that
    collection as

        collection:collisionmeshes:expansionRule = "explicitOnly"
        collection:collisionmeshes:includes      = [<the collisions Xform itself>]

    "explicitOnly" means the collection is *exactly* the listed paths and nothing below them,
    so it resolves to a single Xform and zero meshes. The merge therefore has nothing to merge
    and PhysX ends up with no shape at all on any of the six gripper links -- confirmed with a
    scene-query overlap over the jaw, which reports colliders for the arm links and none for
    the fingers. That is why the fingers sweep straight through a cube: there is nothing there
    to touch it, so the drive is never opposed and always reaches its hard stop.

    Two things have to change:
      * the collision Xforms are `instanceable = true`, and a collection cannot reach prims
        inside an instance prototype, so de-instance them first (same reason as in
        make_cell_collisions_static);
      * switch the expansion rule to "expandPrims" so the collection covers the meshes below
        the Xform and the merge has real geometry to build from.
    """
    from pxr import Usd, PhysxSchema

    root = stage.GetPrimAtPath(gripper_prim_path)
    if not root or not root.IsValid():
        print(f"WARNING: {gripper_prim_path} not found; gripper collisions left as imported")
        return

    # Nested instances only become visible once the outer one is opened, hence the loop.
    while True:
        instances = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()) if p.IsInstance()]
        if not instances:
            break
        for p in instances:
            stage.GetPrimAtPath(p.GetPath()).SetInstanceable(False)

    widened = 0
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(PhysxSchema.PhysxMeshMergeCollisionAPI):
            continue
        rule = prim.GetAttribute("collection:collisionmeshes:expansionRule")
        if rule and rule.Get() != "expandPrims":
            rule.Set("expandPrims")
            widened += 1
        includes = prim.GetRelationship("collection:collisionmeshes:includes")
        if includes and not includes.GetTargets():
            includes.SetTargets([prim.GetPath()])

    print(f"Gripper collisions: {widened} mesh-merge collections widened to expandPrims")


def disable_articulation_self_collisions(stage, robot_prim_path: str = "/World/Cobotta"):
    """Stop the RG6 four-bar linkage from colliding with itself.
    """
    from pxr import Usd, PhysxSchema

    root = stage.GetPrimAtPath(robot_prim_path)
    if not root or not root.IsValid():
        print(f"WARNING: {robot_prim_path} not found; self-collisions left as imported")
        return

    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateEnabledSelfCollisionsAttr(False)
        print(f"Self-collisions disabled on articulation root {prim.GetPath()}")


def configure_gripper_drive(stage,
                            finger_joint_path: str = "/World/Cobotta/onrobot_rg6/base_link/finger_joint",
                            max_torque: float = DEFAULT_GRIP_TORQUE,
                            stiffness: float = GRIP_DRIVE_STIFFNESS,
                            damping: float = GRIP_DRIVE_DAMPING):
    """Turn finger_joint from a rigid position servo into a force-limited one.

    NOTE: USD authors angular drive stiffness/damping per *degree*; the Python tensor API uses
    radians. This writes USD attributes before physics starts, so the values here are per
    degree. maxForce is torque in N*m either way.
    """
    joint = stage.GetPrimAtPath(finger_joint_path)
    if not joint or not joint.IsValid():
        print(f"WARNING: {finger_joint_path} not found; gripper drive left as imported")
        return

    for attr_name, value in (("drive:angular:physics:maxForce", max_torque),
                             ("drive:angular:physics:stiffness", stiffness),
                             ("drive:angular:physics:damping", damping)):
        attr = joint.GetAttribute(attr_name)
        if not attr:
            print(f"WARNING: {finger_joint_path} has no {attr_name}")
            continue
        attr.Set(value)

    print(f"Gripper drive: maxForce={max_torque} N*m, stiffness={stiffness}/deg, "
          f"damping={damping}/deg (~{max_torque / 0.080:.0f} N of finger force)")


def add_grip_friction(stage, gripper_prim_path: str = "/World/Cobotta/onrobot_rg6",
                      static_friction: float = 1.2, dynamic_friction: float = 1.1):
    """Bind a high-friction physics material to the two finger pads.
    """
    from pxr import Sdf, UsdShade

    material_path = Sdf.Path("/World/PhysicsMaterials/GripperPad")
    if not stage.GetPrimAtPath(material_path).IsValid():
        UsdGeom.Scope.Define(stage, material_path.GetParentPath())
        material = UsdShade.Material.Define(stage, material_path)
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr().Set(static_friction)
        physics_material.CreateDynamicFrictionAttr().Set(dynamic_friction)
        physics_material.CreateRestitutionAttr().Set(0.0)

    material = UsdShade.Material.Get(stage, material_path)
    bound = 0
    for side in ("left", "right"):
        pad = stage.GetPrimAtPath(f"{gripper_prim_path}/{side}_inner_finger/collisions")
        if not pad or not pad.IsValid():
            print(f"WARNING: {side}_inner_finger collisions not found; no friction bound")
            continue
        UsdShade.MaterialBindingAPI.Apply(pad).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics")
        bound += 1

    print(f"Gripper friction: static={static_friction} dynamic={dynamic_friction} "
          f"bound to {bound} finger pads")


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
