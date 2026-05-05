from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    isaac_script = os.path.join(pkg_path, 'scripts', 'main.py')

    isaac_sim_process = ExecuteProcess(
        cmd=[
            '/home/anm-vi/Main/Environment/isaaclab/bin/python3',
            isaac_script
        ],
        additional_env={
            'ROS_DISTRO': os.environ.get('ROS_DISTRO', 'humble'),
            'AMENT_PREFIX_PATH': os.environ.get('AMENT_PREFIX_PATH', ''),
            'COLCON_PREFIX_PATH': os.environ.get('COLCON_PREFIX_PATH', ''),
            'PYTHONPATH': os.environ.get('PYTHONPATH', ''),
            'LD_LIBRARY_PATH': os.environ.get('LD_LIBRARY_PATH', ''),
            'DISPLAY': os.environ.get('DISPLAY', ':1'),  # pass display for GUI
        },
        output='screen'
    )

    return LaunchDescription([isaac_sim_process])