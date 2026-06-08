import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    bt_operator_params_file = LaunchConfiguration("bt_operator_params_file")
    lifecycle_manager_params_file = LaunchConfiguration("lifecycle_manager_params_file")
    current_bt_xml_path = LaunchConfiguration("current_bt_xml_path")

    robot_name = LaunchConfiguration("robot_name")
    robot_moveit_config_pkg = LaunchConfiguration("robot_moveit_config_pkg")
    robot_description_file_path = LaunchConfiguration("robot_description_file_path")

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name.perform(context),
            package_name=robot_moveit_config_pkg.perform(context),
        )
        .robot_description(file_path=robot_description_file_path.perform(context))
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # *** ROS 2 nodes ***

    launch_bt_core = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("man2_bt_bringup"),
                    "launch",
                    "bringup_bt_core.launch.py",
                ]
            )
        ),
        launch_arguments={
            "bt_operator_parameter_file": bt_operator_params_file,
            "autostart": autostart,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    moveit_config_server = Node(
        package="moveit_skills",
        executable="moveit_config_server",
        output="screen",
        name="moveit_config_server",
        parameters=[
            {
                "robot_name": robot_name,  # robot_description: config/{robot_name}.urdf.xacro
                "moveit_config_pkg": robot_moveit_config_pkg,
            }
        ],
    )

    moveit_skill_server_node = Node(
        package="moveit_skills",
        executable="moveit_skill_server_node",
        name="moveit_skill_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            ParameterFile(bt_operator_params_file.perform(context), allow_substs=True),
        ],
        prefix=[
            "gdb -batch -ex 'set pagination off' "
            "-ex 'handle SIGPIPE nostop noprint pass' "
            "-ex run -ex 'thread apply all bt' --args"
        ],
    )

    launch_hybrid_planning = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("techtory_hybrid_planning"),
                    "launch",
                    "hybrid_planning.launch.py",
                ]
            )
        ),
    )

    return [
        launch_bt_core,
        moveit_config_server,
        moveit_skill_server_node,
        # launch_hybrid_planning,
    ]


def generate_launch_description():
    techtory_cobotta_system_share = get_package_share_directory("techtory_cobotta_system")

    declared_arguments = [
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the lifecycle nodes",
        ),
        DeclareLaunchArgument(
            "bt_operator_params_file",
            default_value=os.path.join(
                techtory_cobotta_system_share, "config", "bt_operator.yaml"
            ),
            description="Full path to the bt_operator and skill server parameters file",
        ),
        DeclareLaunchArgument(
            "lifecycle_manager_params_file",
            default_value=os.path.join(
                techtory_cobotta_system_share, "config", "lifecycle_manager.yaml"
            ),
            description="Full path to the lifecycle manager parameters file",
        ),
        DeclareLaunchArgument(
            "current_bt_xml_path",
            default_value=os.path.join(
                techtory_cobotta_system_share,
                "trees",
                "techtory_cobotta_system.xml",
            ),
            description="Full path to the behavior tree xml file to execute",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="techtory_demo_description",
            description="Robot name as defined in the moveit config package",
        ),
        DeclareLaunchArgument(
            "robot_moveit_config_pkg",
            default_value="techtory_cobotta_moveit",
            description="Name of the robot's moveit config package",
        ),
        DeclareLaunchArgument(
            "robot_description_file_path",
            default_value="config/techtory_demo_description.urdf.xacro",
            description="Relative path to the robot description xacro inside the moveit config package",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
