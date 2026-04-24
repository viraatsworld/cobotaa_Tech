#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point
from moveit_msgs.srv import ApplyPlanningScene
import time
import os
import trimesh
from ament_index_python.packages import get_package_share_directory


def load_stl_as_mesh(file_path: str) -> Mesh:
    """Standalone helper — no 'self' needed."""
    tm = trimesh.load(file_path, force="mesh")   # force single mesh
    mesh_msg = Mesh()

    for v in tm.vertices:
        p = Point()
        p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
        mesh_msg.vertices.append(p)

    for f in tm.faces:
        tri = MeshTriangle()
        tri.vertex_indices = [int(f[0]), int(f[1]), int(f[2])]
        mesh_msg.triangles.append(tri)

    return mesh_msg


class CellPlanningSceneNode(Node):
    def __init__(self):
        super().__init__("techtory_spawn_planning_scene")

        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_logger().info("Waiting for /apply_planning_scene service...")
        self.client.wait_for_service()
        time.sleep(2.0)

        self.add_cell()

    def add_cell(self):
        package_path = get_package_share_directory("techtory_cell_description")
        mesh_path = os.path.join(package_path, "urdf/mesh")

        meshes = [
            ("techtory_cell_links", "cell_link.STL",        (0.0, 0.0, 0.0)),
            ("base_plate",          "robot_base_link.STL",  (1.0, 0.0, 0.5)),
        ]

        for name, file, pos in meshes:
            full_path = os.path.join(mesh_path, file)
            if not os.path.exists(full_path):
                self.get_logger().error(f"Mesh file not found: {full_path}")
                continue
            self.publish_mesh(name, full_path, pos)

    def publish_mesh(self, name: str, file_path: str, position: tuple):
        self.get_logger().info(f"Loading mesh: {file_path}")

        try:
            mesh = load_stl_as_mesh(file_path)   # ← plain function call
        except Exception as e:
            self.get_logger().error(f"Failed to load {file_path}: {e}")
            return

        self.get_logger().info(
            f"Mesh '{name}': {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles"
        )

        co = CollisionObject()
        co.id = name
        co.header.frame_id = "world"
        co.header.stamp = self.get_clock().now().to_msg()   # ← add timestamp

        co.meshes.append(mesh)

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.w = 1.0
        co.mesh_poses.append(pose)

        co.operation = CollisionObject.ADD

        req = ApplyPlanningScene.Request()
        req.scene.world.collision_objects.append(co)
        req.scene.is_diff = True   # ← CRITICAL: merge, don't replace

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"✓ Added mesh: {name}")
        else:
            self.get_logger().error(f"✗ Failed to add mesh: {name}")


def main():
    rclpy.init()
    node = CellPlanningSceneNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()