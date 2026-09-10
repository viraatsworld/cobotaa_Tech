"""Generate the Cobotta workcell MJCF and run MuJoCo, ros2_control and optional MoveIt."""

from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

from techtory_cobotta_mujoco.model import generate_scene


PACKAGE = "techtory_cobotta_mujoco"
MOVEIT_PACKAGE = "techtory_cobotta_moveit"
# Must match the SRDF robot name and the IKFast plugin's group.
MOVEIT_ROBOT = "techtory_demo_description"
NAMESPACE = "cobotta_pro_"
CONTROLLERS = [
    "joint_state_broadcaster",
    "denso_joint_trajectory_controller",
    "onrobot_rg6",
]
FT_BROADCASTER = "tcp_force_torque_sensor_broadcaster"
# force_torque_sensor_broadcaster publishes to a fixed `~/wrench`; it has no topic_name
# parameter on Jazzy, so the friendly topic has to come from a remap.
WRENCH_TOPIC = "/tcp_wrench"


def _spawn_failed(event, context):
    if event.returncode != 0:
        return [Shutdown(reason="MuJoCo controller activation failed")]
    return []


def _setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        return value(name).lower() == "true"

    share = Path(get_package_share_directory(PACKAGE))
    work = Path(value("work_dir") or tempfile.mkdtemp(prefix="techtory_cobotta_mujoco-"))
    work.mkdir(parents=True, exist_ok=True)

    # hardware_type is passed as a mapping rather than relying on arg defaults, so it wins
    # over techtory_cobotta_workcell.urdf.xacro's own `topic_based` default deterministically.
    robot_xml = xacro.process_file(
        str(share / "urdf/techtory_cobotta_mujoco.urdf.xacro"),
        mappings={
            "hardware_type": "mujoco",
            "headless": value("headless_simulation"),
            "namespace": NAMESPACE,
            "cvrb_prefix": NAMESPACE,
            "initial_positions_file": value("initial_positions_file")
            or str(share / "config/initial_positions.yaml"),
        },
    ).toxml()

    scene = generate_scene(
        robot_xml,
        str(work),
        actuator_config=value("actuator_config") or str(share / "config/actuators.yaml"),
    )

    # The xacro carries a placeholder path; only now do we know where the scene landed.
    robot = ET.fromstring(robot_xml)
    robot.find("ros2_control/hardware/param[@name='mujoco_model']").text = scene
    robot_xml = ET.tostring(robot, encoding="unicode")
    (work / "robot.urdf").write_text(robot_xml, encoding="utf-8")

    description = {"robot_description": ParameterValue(robot_xml, value_type=str)}
    sim_time = {"use_sim_time": True}
    controllers_file = str(share / "config/controllers.yaml")

    simulator = Node(
        package="mujoco_ros2_control", executable="ros2_control_node",
        parameters=[controllers_file, sim_time], output="screen",
        on_exit=Shutdown(reason="MuJoCo exited"),
    )
    # The spawner's advanced per-controller mode keeps a single atomic --activate-as-group
    # switch while still letting one controller carry its own remap.
    spawner_arguments = [
        "--activate-as-group",
        "--controller-manager-timeout", "60",
        "--switch-timeout", "30",
    ]
    for controller in CONTROLLERS:
        spawner_arguments += ["--controller", controller, "-p", controllers_file]
    spawner_arguments += [
        "--controller", FT_BROADCASTER, "-p", controllers_file,
        "--controller-ros-args", f"-r ~/wrench:={WRENCH_TOPIC}",
    ]
    spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=spawner_arguments, output="screen",
    )
    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[description, sim_time], output="screen"),
        simulator,
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=_spawn_failed)),
        spawner,
    ]

    moveit = None
    if boolean("use_moveit") or boolean("use_rviz"):
        from moveit_configs_utils import MoveItConfigsBuilder

        # Point MoveIt at the generated URDF rather than including the package's own
        # move_group.launch.py, which rebuilds the description with topic_based hardware
        # and would contend with ours over the same joints.
        moveit = (
            MoveItConfigsBuilder(MOVEIT_ROBOT, package_name=MOVEIT_PACKAGE)
            .robot_description(file_path=str(work / "robot.urdf"))
            .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
            .to_moveit_configs()
        )
    if boolean("use_moveit"):
        actions.append(Node(
            package="moveit_ros_move_group", executable="move_group",
            parameters=[moveit.to_dict(), sim_time], output="screen",
        ))
    if boolean("use_rviz"):
        rviz_config = value("rviz_config") or str(
            Path(get_package_share_directory(MOVEIT_PACKAGE)) / "config/moveit.rviz"
        )
        actions.append(Node(
            package="rviz2", executable="rviz2", arguments=["-d", rviz_config],
            parameters=[moveit.to_dict(), sim_time], output="log",
        ))
    return actions


def generate_launch_description():
    boolean_arguments = {
        "headless_simulation": ("false", "Run MuJoCo without a viewer"),
        "use_moveit": ("true", "Launch MoveIt arm planning and execution"),
        "use_rviz": ("true", "Launch RViz"),
    }
    declarations = [DeclareLaunchArgument(
        name, default_value=default, description=description, choices=["true", "false"]
    ) for name, (default, description) in boolean_arguments.items()]
    for name, default, description in [
        ("work_dir", "", "Generated URDF/MJCF and meshes; empty creates a unique temp directory"),
        ("rviz_config", "", "RViz configuration override"),
        ("initial_positions_file", "", "Startup joint positions; empty uses this package's"),
        ("actuator_config", "", "MuJoCo actuator gains; empty uses this package's"),
    ]:
        declarations.append(DeclareLaunchArgument(name, default_value=default, description=description))
    return LaunchDescription([*declarations, OpaqueFunction(function=_setup)])
