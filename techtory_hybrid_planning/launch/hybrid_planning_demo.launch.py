"""End-to-end demo launch.

Brings up:
  * the cobotta workcell (robot_state_publisher, ros2_control, RViz) via
    `techtory_cobotta_sw_bringup`,
  * the hybrid planning composable container (manager + global + local),
  * the example HybridPlanner action client which sends a single goal.

Run with:
    ros2 launch techtory_hybrid_planning hybrid_planning_demo.launch.py
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _hybrid_container(context, *args, **kwargs):
    del context, args, kwargs
    spec = importlib.util.spec_from_file_location(
        "hybrid_planning_launch",
        os.path.join(
            get_package_share_directory("techtory_hybrid_planning"),
            "launch",
            "hybrid_planning.launch.py",
        ),
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return [mod.generate_hybrid_planning_container()]


def _demo_client_moveit_params():
    description_pkg_share = get_package_share_directory(
        "techtory_cobotta_workcell_description"
    )
    moveit_pkg_share = get_package_share_directory("techtory_cobotta_moveit")

    moveit_config = (
        MoveItConfigsBuilder(
            "techtory_demo_description", package_name="techtory_cobotta_moveit"
        )
        .robot_description(
            file_path=os.path.join(
                description_pkg_share, "urdf", "techtory_cobotta_workcell.urdf.xacro"
            )
        )
        .robot_description_semantic(
            file_path=os.path.join(
                moveit_pkg_share, "config", "techtory_demo_description.srdf"
            )
        )
        .robot_description_kinematics(
            file_path=os.path.join(moveit_pkg_share, "config", "kinematics.yaml")
        )
        .joint_limits(
            file_path=os.path.join(moveit_pkg_share, "config", "joint_limits.yaml")
        )
        .to_moveit_configs()
    )
    return moveit_config.to_dict()


def generate_launch_description():
    run_demo_client = LaunchConfiguration("run_demo_client")
    robot_controller = LaunchConfiguration("robot_controller")
    move_group_enable = LaunchConfiguration("move_group_enable")
    demo_client_moveit_params = _demo_client_moveit_params()

    declared_arguments = [
        DeclareLaunchArgument(
            "run_demo_client",
            default_value="true",
            description="Whether to start the example HybridPlanner client.",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="denso_joint_group_position_controller",
            description="Robot controller to spawn.",
        ),
        DeclareLaunchArgument(
            "move_group_enable",
            default_value="false",
            description="Launch the MoveIt move_group node.",
        ),
    ]

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("techtory_cobotta_bringup"),
                        "launch",
                        "techtory_cobotta_mock_bringup.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_controller": robot_controller,
            "move_group_enable": move_group_enable,
        }.items(),
    )

    hybrid_container = OpaqueFunction(function=_hybrid_container)

    demo_client_node = Node(
        package="techtory_hybrid_planning",
        executable="hybrid_planning_demo_node",
        name="hybrid_planning_demo_node",
        output="screen",
        parameters=[
            demo_client_moveit_params,
            {
                "planning_group": "arm",
                "hybrid_planning_action": "/run_hybrid_planning",
                # Each composable component (global, local, manager) loads a
                # robot model + planning scene monitor + OMPL pipeline before
                # the manager exposes /run_hybrid_planning. Empirically this
                # takes 50-90s on this workstation, so the client patiently
                # polls.
                "wait_for_server_timeout": 180.0,
            },
        ],
        condition=IfCondition(run_demo_client),
    )

    # Start the client shortly after the container — it polls patiently.
    delayed_demo_client = TimerAction(period=10.0, actions=[demo_client_node])

    return LaunchDescription(
        declared_arguments
        + [
            bringup_launch,
            TimerAction(period=4.0, actions=[hybrid_container]),
            delayed_demo_client,
        ]
    )
