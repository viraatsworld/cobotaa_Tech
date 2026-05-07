import os
def add_techtory_cell(stage, prim_path: str):
    from pxr import Usd, Sdf
    from ament_index_python import get_package_share_directory
    pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    WORKCELL_USD_PATH = os.path.join(pkg_path, 'assets', 'workcells', 'techtory_cell.usd')

    cell_prim = stage.DefinePrim(prim_path, "Xform")
    cell_prim.GetReferences().AddReference(WORKCELL_USD_PATH)

    print(f"Workcell added at {prim_path}")

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
    spawn_position=np.array([0.61, 0.27, 0.9])
    spawn_rotation_deg=np.array([0.0, 0.0, 90.0])
    
    #translation
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(float(spawn_position[0]), float(spawn_position[1]), float(spawn_position[2])))

    #rotation
    rotate_op = xform.AddRotateXYZOp()
    rotate_op.Set(Gf.Vec3d(float(spawn_rotation_deg[0]), float(spawn_rotation_deg[1]), float(spawn_rotation_deg[2])))
    print(f"Shelf added at {prim_path}")

    