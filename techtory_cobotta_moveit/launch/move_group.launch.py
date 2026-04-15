import os
from pathlib import Path
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import LaunchDescription
from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
    add_debuggable_node,
)


def generate_launch_description():

    moveit_pkg_share = Path(get_package_share_directory("techtory_cobotta_moveit"))
    description_pkg_share = Path(get_package_share_directory("techtory_cobotta_workcell_description"))

    moveit_config = MoveItConfigsBuilder("techtory_demo_description", package_name="techtory_cobotta_moveit"
    ).robot_description(
            file_path=str(
                description_pkg_share / "urdf" / "techtory_cobotta_workcell.urdf"
            )
    ).robot_description_semantic(
            file_path=str(moveit_pkg_share / "config" / "techtory_demo_description.srdf")
    ).planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"] # Add isaac_ros_cumotion pipeline if available
    ).trajectory_execution(
            file_path=str(moveit_pkg_share / "config" / "moveit_controllers.yaml")
    ).robot_description_kinematics(
            file_path=str(moveit_pkg_share / "config" / "kinematics.yaml")
    ).joint_limits(
            file_path=str(moveit_pkg_share / "config" / "joint_limits.yaml")
    ).to_moveit_configs()

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("use_sim_time", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    return ld