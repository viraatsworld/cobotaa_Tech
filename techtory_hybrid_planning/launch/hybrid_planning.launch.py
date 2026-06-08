"""Launch the three hybrid planning components for the Techtory Cobotta cell.

This launch file does NOT bring up ros2_control, robot_state_publisher or
RViz. It assumes the robot is already running (e.g. via
`techtory_cobotta_sw_bringup.launch.py`) so that:
  * /joint_states is published,
  * /planning_scene is published by a move_group node OR by the global
    planner's planning scene monitor,
  * the `denso_joint_group_position_controller` (or your chosen controller)
    is active and accepting commands.

For an all-in-one experience use `hybrid_planning_demo.launch.py` instead.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_hybrid_planning_container(
    global_planner_config: str = "config/global_planner.yaml",
):
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
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])
        .trajectory_execution(
            file_path=os.path.join(moveit_pkg_share, "config", "moveit_controllers.yaml")
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    global_planner_param = load_yaml(
        "techtory_hybrid_planning", global_planner_config
    )
    local_planner_param = load_yaml(
        "techtory_hybrid_planning", "config/local_planner.yaml"
    )
    hybrid_planning_manager_param = load_yaml(
        "techtory_hybrid_planning", "config/hybrid_planning_manager.yaml"
    )

    container = ComposableNodeContainer(
        name="hybrid_planning_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner",
                parameters=[
                    global_planner_param,
                    moveit_config.to_dict(),
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner",
                parameters=[
                    local_planner_param,
                    moveit_config.to_dict(),
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                parameters=[hybrid_planning_manager_param],
            ),
        ],
        output="screen",
    )

    return container


def generate_launch_description():
    return LaunchDescription([generate_hybrid_planning_container()])
