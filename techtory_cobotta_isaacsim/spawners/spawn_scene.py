import os

def add_world(stage):
    from pxr import Usd, UsdLux, UsdGeom, Gf, UsdShade, Sdf, UsdPhysics, PhysxSchema
    """Creates base world settings with environment, lights, and flat grid"""

    default_prim_path = "/World"

    if not stage.GetPrimAtPath(default_prim_path):
        stage.DefinePrim(default_prim_path, "Xform")

    
    #Add physics scene (THIS IS MISSING)
    physics_scene_path = "/World/PhysicsScene"
    scene = UsdPhysics.Scene.Define(stage, physics_scene_path)
    scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    # Enable PhysX
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(physics_scene_path))

    # Add dome light for environment lighting
    dome_light_path = "/World/DomeLight"
    dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)
    dome_light.CreateIntensityAttr(800)

    # Add cylinder light 1
    light_path = "/World/CylinderLight"
    cylinder_light = UsdLux.CylinderLight.Define(stage, light_path)
    cylinder_light.CreateIntensityAttr(200)
    cylinder_light.CreateRadiusAttr(5)
    cylinder_light.CreateLengthAttr(100)
    xform_api = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(light_path))
    xform_api.SetTranslate(Gf.Vec3d(3.85931, -11.67055, 12.94707))
    xform_api.SetRotate(Gf.Vec3f(90.0, 90.0, 0.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    # Add cylinder light 2
    light_path2 = "/World/CylinderLight2"
    cylinder_light2 = UsdLux.CylinderLight.Define(stage, light_path2)
    cylinder_light2.CreateIntensityAttr(200)
    cylinder_light2.CreateRadiusAttr(5)
    cylinder_light2.CreateLengthAttr(100)
    xform_api2 = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(light_path2))
    xform_api2.SetTranslate(Gf.Vec3d(-3.85931, 11.67055, 12.94707))
    xform_api2.SetRotate(Gf.Vec3f(90.0, 270.0, 0.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

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
    #lift Lights
    xform_api.SetTranslate(Gf.Vec3d(3.8, -11.6, 15.0))
    xform_api2.SetTranslate(Gf.Vec3d(-3.8, 11.6, 15.0))

    UsdPhysics.CollisionAPI.Apply(ground_plane.GetPrim())
    print("Base world created with environment, lights, and blue flat grid")