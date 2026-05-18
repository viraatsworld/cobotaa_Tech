import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("techtory_demo_description", package_name="techtory_cobotta_moveit")
        .robot_description(file_path="config/techtory_demo_description.urdf.xacro")
        .robot_description_semantic(file_path="config/techtory_demo_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    global_planner_param = load_yaml("techtory_cobotta_moveit", "config/hybrid_planning/global_planner.yaml")
    local_planner_param = load_yaml("techtory_cobotta_moveit", "config/hybrid_planning/local_planner.yaml")
    hybrid_planning_manager_param = load_yaml("techtory_cobotta_moveit", "config/hybrid_planning/hybrid_planning_manager.yaml")

    container = ComposableNodeContainer(
        name="hybrid_planning_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner",
                parameters=[
                    global_planner_param,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner",
                parameters=[
                    local_planner_param,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
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

    # Demo node
    demo_node = Node(
        package="techtory_cobotta_hybrid_planning",
        executable="hybrid_planning_demo_node",
        name="hybrid_planning_demo_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([container, demo_node])
