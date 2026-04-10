from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    ld = LaunchDescription()
    techtory_cobotta_system_share = get_package_share_directory("techtory_cobotta_system")
    rviz_config_path = PathJoinSubstitution(
        [
            techtory_cobotta_system_share,
            "config",
            "techtory_cobotta_system.rviz",
        ]
    )

    # *** PARAMETERS ***
    autostart_arg = DeclareLaunchArgument(
        "autostart", default_value=TextSubstitution(text="true")
    )
    ld.add_action(autostart_arg)
    node_names_arg = DeclareLaunchArgument(
        "node_names",
        default_value=TextSubstitution(text="[bt_operator]"),
    )
    ld.add_action(node_names_arg)
    bond_timeout_arg = DeclareLaunchArgument(
        "bond_timeout", default_value=TextSubstitution(text="0.0")
    )
    ld.add_action(bond_timeout_arg)
    current_bt_xml_name_arg = DeclareLaunchArgument(
        "current_bt_xml_name",
        default_value=TextSubstitution(text="techtory_cobotta_system.xml"),
    )
    ld.add_action(current_bt_xml_name_arg)
    default_plugin_lib_names_arg = DeclareLaunchArgument(
        "default_plugin_lib_names",
        default_value=TextSubstitution(
            text="[keep_running_until_success, sequence_start_from]"
        ),
    )
    ld.add_action(default_plugin_lib_names_arg)
    main_bt_groot_port_arg = DeclareLaunchArgument(
        "main_bt_groot_port",
        default_value=TextSubstitution(text="1667"),
        description="Groot2 TCP port for main bt_operator.",
    )
    ld.add_action(main_bt_groot_port_arg)
    # *** ROS 2 nodes ***
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        output="screen",
        name="lifecycle_manager",
        parameters=[
            {
                "autostart": LaunchConfiguration("autostart"),
                "node_names": LaunchConfiguration("node_names"),
                "bond_timeout": LaunchConfiguration("bond_timeout"),
            }
        ],
    )
    bt_operator = Node(
        package="man2_bt_operator",
        executable="bt_operator",
        output="screen",
        name="bt_operator",
        remappings=[
            (
                "start_application/_action/feedback",
                "start_application/_action/feedback",
            ),
            ("start_application/_action/status", "start_application/_action/status"),
            (
                "start_application/_action/cancel_goal",
                "start_application/_action/cancel_goal",
            ),
            (
                "start_application/_action/get_result",
                "start_application/_action/get_result",
            ),
            (
                "start_application/_action/send_goal",
                "start_application/_action/send_goal",
            ),
        ],
        parameters=[
            {
                "current_bt_xml_filename": LaunchConfiguration("current_bt_xml_name"),
                "default_plugin_lib_names": LaunchConfiguration(
                    "default_plugin_lib_names"
                ),
                "connect_to_groot2": True,
                "groot_server_port": LaunchConfiguration("main_bt_groot_port"),
            }
        ],
    )

    # *** ROS 2 subsystems (include launch files)***

    # *** Add actions ***
    ld.add_action(lifecycle_manager)
    ld.add_action(bt_operator)

    return ld
