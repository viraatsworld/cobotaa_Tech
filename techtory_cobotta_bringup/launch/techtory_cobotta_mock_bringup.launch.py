import yaml
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="techtory_cobotta_workcell_description",
            description="Description package with robot URDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="urdf/techtory_cobotta_workcell.urdf.xacro",
            description="URDF/XACRO description file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="cobotta_pro_",
            description="Prefix for robot joints.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cvrb_prefix",
            default_value="cobotta_pro_",
            description="Prefix used for robot links in workcell xacro.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="config/controller.yaml",
            description="YAML file with controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller_pkg",
            default_value="techtory_cobotta_bringup",
            description="Package that contains controllers_file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="denso_joint_trajectory_controller",
            description="Robot controller to spawn.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_state_broadcaster",
            default_value="denso_joint_state_broadcaster",
            description="Joint state broadcaster controller name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_controller",
            default_value="onrobot_rg6",
            description="Gripper controller to spawn.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "move_group_enable",
            default_value="true",
            description="Launch the MoveIt move_group node.",
        )
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    namespace = LaunchConfiguration("namespace")
    cvrb_prefix = LaunchConfiguration("cvrb_prefix")
    controllers_file = LaunchConfiguration("controllers_file")
    robot_controller_pkg = LaunchConfiguration("robot_controller_pkg")
    robot_controller = LaunchConfiguration("robot_controller")
    joint_state_broadcaster = LaunchConfiguration("joint_state_broadcaster")
    gripper_controller = LaunchConfiguration("gripper_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    move_group_enable = LaunchConfiguration("move_group_enable")

    # MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "techtory_demo_description", package_name="techtory_cobotta_moveit"
    ).planning_pipelines(
        pipelines=["ompl", "pilz_industrial_motion_planner", "isaac_ros_cumotion"]
    ).to_moveit_configs()

    # Robot description with hardware_type:=mock
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([FindPackageShare(description_package), description_file]),
                " ",
                "hardware_type:=mock",
                " ",
                "namespace:=",
                namespace,
                " ",
                "cvrb_prefix:=",
                cvrb_prefix,
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # Kinematics
    kinematics_yaml_path = os.path.join(
        get_package_share_directory("techtory_cobotta_moveit"),
        "config",
        "kinematics.yaml",
    )
    with open(kinematics_yaml_path, "r") as f:
        kinematics_yaml = yaml.safe_load(f)
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Joint limits
    joint_limits_yaml_path = os.path.join(
        get_package_share_directory("techtory_cobotta_moveit"),
        "config",
        "joint_limits.yaml",
    )
    with open(joint_limits_yaml_path, "r") as f:
        joint_limits_yaml = yaml.safe_load(f)
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    # SRDF
    srdf_path = os.path.join(
        get_package_share_directory("techtory_cobotta_moveit"),
        "config",
        "techtory_demo_description.srdf",
    )
    with open(srdf_path, "r") as f:
        srdf_content = f.read()
    robot_description_semantic = {"robot_description_semantic": srdf_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(robot_controller_pkg),
            controllers_file,
        ]
    )

    # --- Nodes ---

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(robot_controllers, allow_substs=True),
        ],
        output={"stdout": "screen", "stderr": "screen"},
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[joint_state_broadcaster, "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[gripper_controller, "--controller-manager", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(launch_rviz),
        output="screen",
        parameters=[
            robot_description,
            robot_description_kinematics,
            robot_description_planning,
            robot_description_semantic,
            moveit_config.planning_pipelines,
        ],
    )

    move_group_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("techtory_cobotta_moveit"),
                        "launch",
                        "move_group.launch.py",
                    ]
                )
            ]
        ),
        condition=IfCondition(move_group_enable),
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
            gripper_controller_spawner,
            rviz_node,
            move_group_node,
        ]
    )
