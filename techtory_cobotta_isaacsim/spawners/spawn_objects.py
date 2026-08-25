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

    The cell was imported from URDF, so its links are rigid bodies in an articulation and the
    collision meshes default to physics:approximation = "convexHull". The hull of cell_link is
    the hull of the *whole* cell frame -- a solid ~2.2 m block filling the entire cell interior.
    Everything mounted inside the cell, the robot included, therefore starts fully inside a
    solid collider, and PhysX spends every step trying to push it out: the arm gets thrown
    around and dropped objects get expelled through the floor. That is the reason collisions
    had to be switched off to get anything to run.

    The cell never moves, so the right model is static geometry with the exact triangle mesh:
    disable the rigid bodies (their colliders stay, as static colliders) and switch the
    approximation to "none", which is only legal for static/kinematic bodies. Objects then rest
    on the table and the arm is left alone unless it really touches the frame.

    The joints have to go with them. The URDF importer wrote a fixed joint per link plus a
    root_joint carrying PhysicsArticulationRootAPI, and PhysX refuses to build a joint whose
    two ends are both static -- that is the "cannot create a joint between static bodies"
    error. The cell is 3 links and 2 fixed joints, i.e. zero DOF, so the joints carry no
    information: each static collider keeps the world transform USD already composed for it.
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