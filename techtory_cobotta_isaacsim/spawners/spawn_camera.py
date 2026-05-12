import numpy as np
from pxr import UsdGeom, Gf, Usd, UsdPhysics


REALSENSE_NUCLEUS_SUBPATH = "/Isaac/Sensors/Intel/RealSense/rsd455.usd"


def _disable_rigid_bodies(stage, root_path: str) -> None:
    """The shipped rsd455 asset has a PhysicsRigidBodyAPI on its root so that
    gravity drops it on Play. We want a static sensor, so author a local
    opinion that disables every RigidBodyAPI under the referenced subtree."""
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim or not root_prim.IsValid():
        return
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)


def _find_first_camera(stage, root_path: str) -> str:
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim or not root_prim.IsValid():
        return ""
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Camera):
            return prim.GetPath().pathString
    return ""


def add_realsense_camera(stage,
                         prim_path: str = "/World/Camera",
                         spawn_position=np.array([-0.65, 0.65, 2.0]),
                         spawn_rotation_deg=np.array([-180, -45, 45])):
    """Reference the Isaac RealSense rsd455 asset into the stage and pose it."""
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.nucleus import get_assets_root_path

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("Could not resolve Isaac Sim assets root (Nucleus).")
    camera_usd = assets_root + REALSENSE_NUCLEUS_SUBPATH

    add_reference_to_stage(usd_path=camera_usd, prim_path=prim_path)

    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()

    xform.AddTranslateOp().Set(
        Gf.Vec3d(float(spawn_position[0]),
                 float(spawn_position[1]),
                 float(spawn_position[2]))
    )
    xform.AddRotateXYZOp().Set(
        Gf.Vec3d(float(spawn_rotation_deg[0]),
                 float(spawn_rotation_deg[1]),
                 float(spawn_rotation_deg[2]))
    )

    _disable_rigid_bodies(stage, prim_path)

    camera_prim_path = _find_first_camera(stage, prim_path)
    if not camera_prim_path:
        raise RuntimeError(f"No UsdGeom.Camera found beneath {prim_path}")

    print(f"RealSense rsd455 referenced at {prim_path}; camera prim: {camera_prim_path}")
    return camera_prim_path


def attach_ros2_camera_graph(camera_prim_path: str,
                             graph_path: str = "/World/ROS_Camera",
                             rgb_topic: str = "/camera/rgb",
                             pcl_topic: str = "/camera/pcl",
                             frame_id: str = "camera1_link",
                             resolution=(640, 480)):
    """Build an OmniGraph that publishes RGB image and point cloud for the
    given camera prim through the Isaac Sim ROS2 bridge."""
    import omni.graph.core as og
    import omni.usd
    from pxr import Sdf

    width, height = int(resolution[0]), int(resolution[1])
    keys = og.Controller.Keys

    (graph_handle, _, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct",
                 "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraHelperPcl", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:width", width),
                ("CreateRenderProduct.inputs:height", height),
                ("CameraHelperRgb.inputs:type", "rgb"),
                ("CameraHelperRgb.inputs:topicName", rgb_topic),
                ("CameraHelperRgb.inputs:frameId", frame_id),
                # depth_pcl emits sensor_msgs/PointCloud2 from the depth buffer
                ("CameraHelperPcl.inputs:type", "depth_pcl"),
                ("CameraHelperPcl.inputs:topicName", pcl_topic),
                ("CameraHelperPcl.inputs:frameId", frame_id),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut",
                 "CameraHelperRgb.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut",
                 "CameraHelperPcl.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath",
                 "CameraHelperRgb.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath",
                 "CameraHelperPcl.inputs:renderProductPath"),
            ],
        },
    )

    # `inputs:cameraPrim` is a USD relationship, not a token. og.Controller's
    # SET_VALUES can't populate relationship targets, so author them directly,
    # otherwise IsaacCreateRenderProduct fails with "No valid sensor paths".
    stage = omni.usd.get_context().get_stage()
    rp_prim = stage.GetPrimAtPath(f"{graph_path}/CreateRenderProduct")
    cam_rel = rp_prim.GetRelationship("inputs:cameraPrim")
    if not cam_rel:
        cam_rel = rp_prim.CreateRelationship("inputs:cameraPrim")
    cam_rel.SetTargets([Sdf.Path(camera_prim_path)])

    print(f"ROS2 camera graph created at {graph_path}: "
          f"rgb={rgb_topic}, pcl={pcl_topic}, frame_id={frame_id}")
    return graph_handle
