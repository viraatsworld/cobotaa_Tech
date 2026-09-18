import os
def add_techtory_cell(stage, prim_path: str):
    from pxr import Sdf
    from ament_index_python import get_package_share_directory
    pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    WORKCELL_USD_PATH = os.path.join(pkg_path, 'assets', 'workcells', 'techtory_cell.usd')

    # Load as a sublayer so the camera/ActionGraph prims keep their ORIGINAL
    # absolute prim paths. AddReference re-roots prims under prim_path, which
    # breaks ROS2CameraHelper nodes whose `inputs:renderProductPath` strings
    # were authored against the original absolute paths.
    root_layer = stage.GetRootLayer()
    if WORKCELL_USD_PATH not in root_layer.subLayerPaths:
        root_layer.subLayerPaths.append(WORKCELL_USD_PATH)

    print(f"Workcell sublayered from {WORKCELL_USD_PATH}")

    # The shipped techtory_cell.usd still has a stale RealSense reference and
    # a leftover /Graph/ROS_Camera authored against a now-broken prim path.
    # Deactivate them so they stop firing "No valid sensor paths" every tick.
    for stale_path in (
        "/World/techtory_demo_description/cell_link/camera1_link",
        "/Graph/ROS_Camera",
        "/World/Cobotta/techtory_demo_description/ActionGraph",
    ):
        p = stage.GetPrimAtPath(stale_path)
        if p and p.IsValid():
            p.SetActive(False)
            print(f"Deactivated stale prim {stale_path}")

    make_cell_collisions_static(stage, "/World/techtory_demo_description")


def make_cell_collisions_static(stage, root_path: str):
    """Turn the cell into static, exact-mesh colliders.
    """
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        print(f"WARNING: {root_path} not found; cell collisions left as imported")
        return

    # The importer references the collision meshes as instances, and instance proxies cannot be
    # edited, so de-instance first. Nested instances appear only once the outer one is opened.
    while True:
        instances = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()) if p.IsInstance()]
        if not instances:
            break
        for p in instances:
            stage.GetPrimAtPath(p.GetPath()).SetInstanceable(False)

    colliders = bodies = 0
    joint_prims = []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI(prim).CreateApproximationAttr("none")
            colliders += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            bodies += 1
        if prim.IsA(UsdPhysics.Joint):
            joint_prims.append(prim.GetPath())

    # Deactivate after the traversal -- deactivating mid-range prunes the subtree we are
    # still walking. This also drops PhysicsArticulationRootAPI, which sits on root_joint.
    for joint_path in joint_prims:
        stage.GetPrimAtPath(joint_path).SetActive(False)

    print(f"Cell collisions: {colliders} mesh colliders set to exact, {bodies} links made "
          f"static, {len(joint_prims)} fixed joints removed")

def configure_graspable_object(stage, prim_path: str, mass: float = 0.3,
                               static_friction: float = 1.2, dynamic_friction: float = 1.1,
                               max_convex_hulls: int = 32):
    """Make a spawned object something the gripper can actually hold.

    As authored, hammer1.usd gives PhysX three reasons to refuse a grasp:

      * `physics:approximation = "convexHull"` on the mesh. The convex hull of a hammer is a
        solid wedge spanning head to handle -- it fills the entire notch you are trying to grab.
        The pads therefore never touch the handle; they close on a sloping hull face, and a
        sloping face plus finite friction means the part is squeezed straight out of the jaw.
        convexDecomposition keeps the head and the handle as separate hulls, so the pads land on
        the flat sides of the handle and the grasp is force-closed.
      * No PhysicsMassAPI, so mass and inertia are integrated from that same wrong hull at the
        default density -- the object PhysX is simulating is not the object you can see.
      * No physics material, so the object runs on the engine default while spawn_robot's
        add_grip_friction() gives the finger pads 1.2/1.1. PhysX combines the two materials
        (default: average), so the pad material alone only gets you halfway.

    Safe to call on any spawned object; it walks the subtree, so it does not care how deep the
    reference put the rigid body.
    """
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, PhysxSchema

    root = stage.GetPrimAtPath(prim_path)
    if not root or not root.IsValid():
        print(f"WARNING: {prim_path} not found; object physics left as imported")
        return

    # Instance proxies cannot be edited (same reason as make_cell_collisions_static).
    while True:
        instances = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()) if p.IsInstance()]
        if not instances:
            break
        for p in instances:
            stage.GetPrimAtPath(p.GetPath()).SetInstanceable(False)

    material_path = Sdf.Path("/World/PhysicsMaterials/GraspableObject")
    if not stage.GetPrimAtPath(material_path).IsValid():
        UsdGeom.Scope.Define(stage, material_path.GetParentPath())
        material = UsdShade.Material.Define(stage, material_path)
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr().Set(static_friction)
        physics_material.CreateDynamicFrictionAttr().Set(dynamic_friction)
        physics_material.CreateRestitutionAttr().Set(0.0)
    material = UsdShade.Material.Get(stage, material_path)

    bodies = colliders = decomposed = 0
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(float(mass))
            bodies += 1
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        # Only mesh colliders have an approximation to change. Primitive shapes (a Cube, a
        # Sphere) are already exact and must not be reported as decomposed.
        if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI(prim).CreateApproximationAttr("convexDecomposition")
            decomposition = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            decomposition.CreateMaxConvexHullsAttr().Set(int(max_convex_hulls))
            decomposition.CreateErrorPercentageAttr().Set(2.0)
            decomposed += 1
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics")
        colliders += 1

    print(f"Graspable {prim_path}: mass={mass} kg on {bodies} body(ies), friction "
          f"{static_friction}/{dynamic_friction} on {colliders} collider(s), "
          f"{decomposed} mesh collider(s) set to convexDecomposition")


def add_shelf(stage, prim_path: str):
    from pxr import Usd, Sdf, UsdGeom, Gf
    from ament_index_python import get_package_share_directory
    import numpy as np
    pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    SHELF_USD_PATH = os.path.join(pkg_path, 'assets', 'objects', 'shelf.usd')

    shelf_prim = stage.DefinePrim(prim_path, "Xform")
    shelf_prim.GetReferences().AddReference(SHELF_USD_PATH)

    # Update location
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()


    #postions
    spawn_position=np.array([0.61, 0.27, 0.94])
    spawn_rotation_deg=np.array([0.0, 0.0, 90.0])
    
    #translation
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(float(spawn_position[0]), float(spawn_position[1]), float(spawn_position[2])))

    #rotation
    rotate_op = xform.AddRotateXYZOp()
    rotate_op.Set(Gf.Vec3d(float(spawn_rotation_deg[0]), float(spawn_rotation_deg[1]), float(spawn_rotation_deg[2])))
    print(f"Shelf added at {prim_path}")


def add_pallet(stage, prim_path: str):
    from pxr import Usd, Sdf, UsdGeom, Gf
    from ament_index_python import get_package_share_directory
    import numpy as np
    pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    PALLETS_USD_PATH = os.path.join(pkg_path, 'assets', 'objects', 'pallet.usd')

    pallet_prim = stage.DefinePrim(prim_path, "Xform")
    pallet_prim.GetReferences().AddReference(PALLETS_USD_PATH)

    # Update location
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()


    #postions
    spawn_position=np.array([-0.16, 0.3, 0.94])
    spawn_rotation_deg=np.array([0.0, 0.0, 0.0])
    
    #translation
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(float(spawn_position[0]), float(spawn_position[1]), float(spawn_position[2])))

    #rotation
    rotate_op = xform.AddRotateXYZOp()
    rotate_op.Set(Gf.Vec3d(float(spawn_rotation_deg[0]), float(spawn_rotation_deg[1]), float(spawn_rotation_deg[2])))
    print(f"Pallet added at {prim_path}")



def add_hammer(stage, prim_path: str):
    from pxr import Usd, Sdf, UsdGeom, Gf
    from ament_index_python import get_package_share_directory
    import numpy as np
    pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    HAMMER_USD_PATH = os.path.join(pkg_path, 'assets', 'objects', 'hammer1.usd')

    hammer_prim = stage.DefinePrim(prim_path, "Xform")
    hammer_prim.GetReferences().AddReference(HAMMER_USD_PATH)

    # Update location
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()


    #postions
    spawn_position=np.array([0.04318, 0, 0.43525])
    spawn_rotation_deg=np.array([90.0, 0.0, 180.0])
    
    #translation
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(float(spawn_position[0]), float(spawn_position[1]), float(spawn_position[2])))

    #rotation
    rotate_op = xform.AddRotateXYZOp()
    rotate_op.Set(Gf.Vec3d(float(spawn_rotation_deg[0]), float(spawn_rotation_deg[1]), float(spawn_rotation_deg[2])))

    # #scale
    # scale_op = xform.AddScaleOp()
    # scale_op.Set(Gf.Vec3d(0.001, 0.001, 0.001))
    print(f"Hammer added at {prim_path}")