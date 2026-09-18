import os

# --- Cell interior lighting ----------------------------------------------------
# cell_link.STL bounding box, in the cell_link frame (== world, the cell mounts at
# the origin in techtory_cell.xacro):
#   x -1.283 .. 0.896, y -0.862 .. 1.316, z 0.000 .. 2.135
# The work surface is at z = 0.94. The roof at 2.135 blocks the DomeLight and the
# two CylinderLights (which sit at z = 15), so without fixtures *inside* the cell
# the interior only gets what leaks in through the open front.
CELL_X_CENTER = -0.194
CELL_CEILING_Z = 2.135

# Brightness knobs. normalize is left OFF, so these are radiance: enlarging a panel
# also makes it brighter. Scale intensity by 1/area if you resize and want the same
# exposure. Raise CEILING_INTENSITY for a brighter cell; the two panels are large
# and close, so the light stays soft (broad source -> soft shadow edges) and most of
# the extra reads as fill rather than glare.
CEILING_INTENSITY = 10000.0
FRONT_FILL_INTENSITY = 3000.0
LIGHT_TEMPERATURE_K = 6500.0   # neutral white; 3000 = warm, 6500 = cold daylight


def add_cell_lights(stage):
    """Two ceiling panels plus a front fill, all inside the cell envelope.

    Call this AFTER add_techtory_cell -- the panels hang just under the roof, and
    placing them relies on the cell already being where the bounding box says.
    """
    from pxr import UsdLux, UsdGeom, Gf

    def _panel(path, translate, width, height, intensity, rotate=None):
        light = UsdLux.RectLight.Define(stage, path)
        light.CreateWidthAttr(width)
        light.CreateHeightAttr(height)
        light.CreateIntensityAttr(intensity)
        light.CreateNormalizeAttr(False)
        light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        light.CreateEnableColorTemperatureAttr(True)
        light.CreateColorTemperatureAttr(LIGHT_TEMPERATURE_K)
        # Diffuse only -- a specular highlight of the panel mirrored in the cell's
        # sheet metal and in the robot's painted links is the main thing that makes
        # this kind of setup read as "CG lighting" rather than as a room.
        light.CreateSpecularAttr(0.35)

        xform_api = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(path))
        xform_api.SetTranslate(Gf.Vec3d(*translate))
        if rotate is not None:
            xform_api.SetRotate(Gf.Vec3f(*rotate), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        return light

    # A RectLight lies in its local XY plane and emits along -Z, so an unrotated
    # panel at the ceiling already points straight down at the bench.
    panel_z = CELL_CEILING_Z - 0.07

    # Front panel: over the robot (base at x -0.275, y -0.24) and the working volume.
    _panel("/World/CellLightFront", (CELL_X_CENTER, -0.10, panel_z),
           width=1.70, height=0.45, intensity=CEILING_INTENSITY)

    # Rear panel: over the shelf (0.61, 0.27) and the pallet (-0.16, 0.3).
    _panel("/World/CellLightRear", (CELL_X_CENTER, 0.55, panel_z),
           width=1.70, height=0.45, intensity=CEILING_INTENSITY)

    # Front fill. Two overhead sources alone leave the vertical faces that matter
    # for depth -- the shelf uprights, the pallet walls, the gripper jaws -- lit only
    # from straight above, which flattens them. This panel stands just inside the
    # open front face (y = -0.862) and is tilted back and down into the cell.
    _panel("/World/CellLightFill", (CELL_X_CENTER, -0.80, 1.60),
           width=1.60, height=0.70, intensity=FRONT_FILL_INTENSITY,
           rotate=(60.0, 0.0, 0.0))

    print("Cell lighting: 2 ceiling panels + 1 front fill added inside the cell")


def add_world(stage):
    from pxr import Usd, UsdLux, UsdGeom, Gf, UsdShade, Sdf, UsdPhysics, PhysxSchema
    """Creates base world settings with environment, lights, and flat grid"""

    default_prim_path = "/World"

    if not stage.GetPrimAtPath(default_prim_path):
        stage.DefinePrim(default_prim_path, "Xform")

    
    #Add physics scene (THIS IS MISSING)(Remove physics , cause isaacsim already have in world)
    # physics_scene_path = "/World/PhysicsScene"
    # scene = UsdPhysics.Scene.Define(stage, physics_scene_path)
    # scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
    # scene.CreateGravityMagnitudeAttr().Set(9.81)

    # # Enable PhysX
    # physx_scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(physics_scene_path))

    # Add dome light for environment lighting
    dome_light_path = "/World/DomeLight"
    dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)
    dome_light.CreateIntensityAttr(800)



    # =========================
    # Ground plane (robust grid)
    # =========================
    ground_plane_path = "/World/GroundPlane"

    ground_plane = UsdGeom.Mesh.Define(stage, ground_plane_path)

    # Large quad
    ground_plane.CreatePointsAttr([
        (-50.0, -50.0, 0.0),
        (50.0, -50.0, 0.0),
        (50.0, 50.0, 0.0),
        (-50.0, 50.0, 0.0),
    ])

    ground_plane.CreateFaceVertexCountsAttr([4])
    ground_plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])

    # Normals (important for lighting!)
    ground_plane.CreateNormalsAttr([(0.0, 0.0, 1.0)] * 4)
    ground_plane.SetNormalsInterpolation("vertex")

    # =========================
    # Blue material
    # =========================
    material_path = "/World/Materials/BlueMaterial"
    material = UsdShade.Material.Define(stage, material_path)

    shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.05, 0.1, 0.35))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    # Correct connection (important!)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )

    UsdShade.MaterialBindingAPI(ground_plane).Bind(material)


    UsdPhysics.CollisionAPI.Apply(ground_plane.GetPrim())
    print("Base world created with environment, lights, and blue flat grid")