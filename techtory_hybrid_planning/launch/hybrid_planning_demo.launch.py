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


def generate_launch_description():
    run_demo_client = LaunchConfiguration("run_demo_client")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("techtory_cobotta_bringup"),
                        "launch",
                        "techtory_cobotta_sw_bringup.launch.py",
                    ]
                )
            ]
        )
    )

    hybrid_container = OpaqueFunction(function=_hybrid_container)

    demo_client_node = Node(
        package="techtory_hybrid_planning",
        executable="hybrid_planning_demo_node",
        name="hybrid_planning_demo_node",
        output="screen",
        parameters=[
            {
                "planning_group": "arm",
                "hybrid_planning_action": "/run_hybrid_planning",
                # Each composable component (global, local, manager) loads a
                # robot model + planning scene monitor + OMPL pipeline before
                # the manager exposes /run_hybrid_planning. Empirically this
                # takes 50-90s on this workstation, so the client patiently
                # polls.
                "wait_for_server_timeout": 180.0,
            }
        ],
        condition=IfCondition(run_demo_client),
    )

    # Start the client shortly after the container — it polls patiently.
    delayed_demo_client = TimerAction(period=10.0, actions=[demo_client_node])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_demo_client",
                default_value="true",
                description="Whether to start the example HybridPlanner client.",
            ),
            bringup_launch,
            TimerAction(period=4.0, actions=[hybrid_container]),
            delayed_demo_client,
        ]
    )
