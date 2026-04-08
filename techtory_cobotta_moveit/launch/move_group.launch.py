from pathlib import Path
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import LaunchDescription, generate_move_group_launch

def generate_launch_description():

    moveit_pkg_share = Path(get_package_share_directory("techtory_cobotta_moveit"))
    description_pkg_share= Path(get_package_share_directory("techtory_cobotta_workcell_description"))
        # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time"
    )
    moveit_config = MoveItConfigsBuilder("techtory_demo_description", package_name="techtory_cobotta_moveit"
    ).robot_description(
            file_path=str(
                description_pkg_share / "urdf" / "techtory_cobotta_workcell.urdf"
            )
    ).robot_description_semantic(
            file_path=str(moveit_pkg_share / "config" / "techtory_demo_description.srdf"
                          )
    ).planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "isaac_ros_cumotion"]
                         
      ).trajectory_execution(
            file_path=str(moveit_pkg_share / "config" / "moveit_controllers.yaml")
        ).robot_description_kinematics(
            file_path=str(moveit_pkg_share / "config" / "kinematics.yaml")
        ).joint_limits(
            file_path=str(moveit_pkg_share / "config" / "joint_limits.yaml")
        ).to_moveit_configs()
    move_group_launch = generate_move_group_launch(moveit_config)
    
    return LaunchDescription(
        [use_sim_time_arg] + move_group_launch.entities
        )