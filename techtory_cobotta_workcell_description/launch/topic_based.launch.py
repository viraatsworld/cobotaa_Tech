from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
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
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "techtory_cobotta_workcell.urdf.xacro"]),
            " hardware_type:=topic_based",
            " namespace:=",
            LaunchConfiguration("namespace"),
        ]
    ).perform(context)

    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    controllers_file = PathJoinSubstitution(
        [pkg_share, "config", "topic_based_controllers.yaml"]
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                robot_description,
                ParameterFile(controllers_file, allow_substs=True),
            ],
        ),
    ]

    for controller in ["joint_state_broadcaster", "joint_trajectory_controller"]:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", controllers_file],
                output="screen",
            )
        )

    if launch_rviz == "true":
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                parameters=[robot_description],
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
                "launch_rviz",
                default_value="true",
                description="Launch RViz2.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
