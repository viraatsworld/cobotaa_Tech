from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_path = get_package_share_directory('techtory_cobotta_isaacsim')
    
    # Path to Isaac Sim python (EDIT THIS)
    isaac_python = "/home/$USER/isaacsim/python.sh"

    # Path to your main Isaac script
    isaac_script = os.path.join(pkg_path, 'scripts', 'main.py')

    # Path to your main Isaac script

    isaac_sim_process = ExecuteProcess(
        cmd=[
            isaac_python,
            isaac_script
        ],
        output="screen"
    )

    return LaunchDescription([
        isaac_sim_process
    ])