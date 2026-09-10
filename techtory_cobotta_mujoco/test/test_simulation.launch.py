"""Launch the headless simulation and drive the arm, the gripper and the wrench topic."""

import os
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import WrenchStamped
import launch
import launch_testing
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = [f"cobotta_pro_joint_{index}" for index in range(1, 7)]
HOME = [0.0, 0.349066, 1.309, 0.0, 1.48353, 0.0]
CONTROLLERS = {
    "joint_state_broadcaster",
    "denso_joint_trajectory_controller",
    "onrobot_rg6",
    "tcp_force_torque_sensor_broadcaster",
}
STARTUP_TIMEOUT = 120.0


@pytest.mark.launch_test
def generate_test_description():
    bringup = Path(get_package_share_directory("techtory_cobotta_mujoco"))
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup / "launch/techtory_cobotta_mujoco.launch.py")
            ),
            launch_arguments={
                "headless_simulation": "true",
                "use_moveit": "false",
                "use_rviz": "false",
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestCobottaMujoco(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_techtory_cobotta_mujoco")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout, message):
        """Spin the node until predicate() is true. Never sleeps blindly."""
        deadline = self.node.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        self.fail(message)

    def _active_controllers(self, client):
        if not client.service_is_ready():
            return set()
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if future.result() is None:
            return set()
        return {c.name for c in future.result().controller if c.state == "active"}

    def test_01_controllers_activate(self):
        client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._spin_until(
            lambda: CONTROLLERS.issubset(self._active_controllers(client)),
            STARTUP_TIMEOUT,
            "controllers did not all reach the active state",
        )

    def test_02_wrench_is_published(self):
        received = []
        self.node.create_subscription(
            WrenchStamped, "/tcp_wrench", received.append, 10
        )
        self._spin_until(
            lambda: len(received) >= 5,
            30.0,
            "no WrenchStamped messages arrived on /tcp_wrench -- the FT sensor name in the "
            "MJCF and the ros2_control <sensor> block may disagree (this fails silently)",
        )
        wrench = received[-1].wrench
        components = [
            wrench.force.x, wrench.force.y, wrench.force.z,
            wrench.torque.x, wrench.torque.y, wrench.torque.z,
        ]
        for value in components:
            self.assertFalse(value != value, "wrench contains NaN")
        self.assertTrue(
            any(abs(value) > 1e-6 for value in components),
            "wrench is identically zero; the sensor is not reading the flange interface",
        )

    def test_03_arm_accepts_a_trajectory(self):
        client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/denso_joint_trajectory_controller/follow_joint_trajectory",
        )
        self.assertTrue(
            client.wait_for_server(timeout_sec=30.0), "arm action server never appeared"
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        # A small, safely-inside-limits nudge off the home pose.
        point.positions = [HOME[0] + 0.1, *HOME[1:]]
        point.time_from_start.sec = 3
        goal.trajectory.points = [point]

        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=15.0)
        handle = send.result()
        self.assertIsNotNone(handle, "arm goal was never acknowledged")
        self.assertTrue(handle.accepted, "arm goal was rejected")

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=30.0)
        self.assertIsNotNone(result.result(), "arm trajectory did not finish")
        self.assertEqual(
            result.result().result.error_code,
            FollowJointTrajectory.Result.SUCCESSFUL,
            f"arm trajectory failed: {result.result().result.error_string}",
        )

    def test_04_gripper_closes(self):
        client = ActionClient(
            self.node, ParallelGripperCommand, "/onrobot_rg6/gripper_cmd"
        )
        self.assertTrue(
            client.wait_for_server(timeout_sec=30.0),
            "gripper action server never appeared",
        )

        goal = ParallelGripperCommand.Goal()
        goal.command.name = ["finger_joint"]
        goal.command.position = [0.4]  # towards closed; -0.628 open, +0.628 closed
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=15.0)
        handle = send.result()
        self.assertIsNotNone(handle, "gripper goal was never acknowledged")
        self.assertTrue(handle.accepted, "gripper goal was rejected")

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=30.0)
        self.assertIsNotNone(result.result(), "gripper command did not finish")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15]
        )
