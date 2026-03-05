from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    description_pkg_share = Path(
        get_package_share_directory("techtory_cvrb0609_workcell_description")
    )
    moveit_pkg_share = Path(get_package_share_directory("techtory_cobotta_moveit_config"))

    moveit_config = (
        MoveItConfigsBuilder(
            "techtory_cvrb0609_workcell", package_name="techtory_cobotta_moveit_config"
        )
        .robot_description(
            file_path=str(
                description_pkg_share / "urdf" / "techtory_cvrb0609_workcell.urdf"
            )
        )
        .robot_description_semantic(
            file_path=str(moveit_pkg_share / "config" / "techtory_cvrb0609_workcell.srdf")
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .trajectory_execution(
            file_path=str(moveit_pkg_share / "config" / "moveit_controllers.yaml")
        )
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
