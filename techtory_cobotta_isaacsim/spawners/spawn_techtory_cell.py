from pxr import Usd, Sdf
from ament_index_python import get_package_share_directory

pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    
# Path to your main Isaac script
WORKCELL_USD_PATH = os.path.join(pkg_path, 'assets', '/workcells/techtory_cell/techtory_cell.usd')

def add_techtory_cell(stage: Usd.Stage, prim_path: str):

    cell_prim = stage.DefinePrim(prim_path, "Xform")
    cell_prim.GetReferences().AddReference(WORKCELL_USD_PATH)

    print(f"🪑 Workcell added at {prim_path}")