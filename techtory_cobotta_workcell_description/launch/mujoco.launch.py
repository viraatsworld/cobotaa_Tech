import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("techtory_cobotta_workcell_description")
    namespace = LaunchConfiguration("namespace").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "techtory_cobotta_workcell.urdf.xacro"]),
            " hardware_type:=mujoco",
            " namespace:=",
            LaunchConfiguration("namespace"),
        ]
    ).perform(context)

    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    controllers_file = PathJoinSubstitution([pkg_share, "config", "mujoco_controllers.yaml"])

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(controllers_file, allow_substs=True),
                robot_description,
            ],
            remappings=(
                [("~/robot_description", "/robot_description")]
                if os.environ.get("ROS_DISTRO") == "humble"
                else []
            ),
            on_exit=Shutdown(),
        ),
    ]

    for controller in ["joint_state_broadcaster", "joint_trajectory_controller"]:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", controllers_file],
                output="both",
            )
        )

    if launch_rviz == "true":
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                parameters=[robot_description, {"use_sim_time": True}],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="cobotta_pro_",
                description="Prefix for robot joints.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo simulation without GUI window.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz2.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
