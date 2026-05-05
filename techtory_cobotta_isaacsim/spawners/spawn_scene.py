import os

def add_world(stage):
    from pxr import Usd
    """Creates base world settings"""

    default_prim_path = "/World"

    if not stage.GetPrimAtPath(default_prim_path):
        stage.DefinePrim(default_prim_path, "Xform")

    print("🌍 Base world created")