import yaml
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
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
            "launch_rviz",
            default_value="true",
            description="Launch RViz2.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run MuJoCo simulation without GUI window.",
        )
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    namespace = LaunchConfiguration("namespace")
    cvrb_prefix = LaunchConfiguration("cvrb_prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "techtory_demo_description", package_name="techtory_cobotta_moveit"
    ).planning_pipelines(
        pipelines=["ompl", "pilz_industrial_motion_planner"]
    ).to_moveit_configs()

    # Robot description with hardware_type:=mujoco
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([FindPackageShare(description_package), description_file]),
                " ",
                "hardware_type:=mujoco",
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

    # MuJoCo controllers config
    controllers_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "mujoco_controllers.yaml"]
    )

    # --- Nodes ---

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        emulate_tty=True,
        output="both",
        parameters=[
            {"use_sim_time": True},
            ParameterFile(controllers_file, allow_substs=True),
            robot_description,
        ],
        on_exit=Shutdown(),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", controllers_file],
        output="both",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["denso_joint_trajectory_controller", "--param-file", controllers_file],
        output="both",
    )

    # Static TF
    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "cobotta_pro_tool0"],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("techtory_cobotta_bringup"), "config", "rviz.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(launch_rviz),
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_kinematics,
            robot_description_planning,
            robot_description_semantic,
            moveit_config.planning_pipelines,
            {"use_sim_time": True},
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
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
            # world2robot_tf_node,
            rviz_node,
            # move_group_node,
        ]
    )
