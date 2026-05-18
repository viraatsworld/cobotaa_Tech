# Techtory Cell Description

This package provides a URDF description of a techtory cell with a Cobotta robot.

## Getting Started

1. Clone the repository:

    ```bash
    cd colcon_techtory_ws
    git clone git@gitlab.cc-asp.fraunhofer.de:ipa326/demonstrator/arena2036/dynamic_planning_demo.git src
    ```

2. Import upstream dependencies:

    ```bash
    cd src
    vcs import . < upstream.repo
    cd ..
    ```

3. Install ROS dependencies:

    ```bash
    source /opt/ros/jazzy/setup.bash
    rosdep update
    rosdep install --from-paths src -iry
    colcon build --symlink-install --packages-up-to techtory_cobotta_bringup techtory_cobotta_system techtory_hybrid_planning onrobot_rg_description techtory_cobotta_isaacsim
    ```

## Visualize in RViz

```bash
ros2 launch techtory_cobotta_workcell_description display.launch.py
```

## MuJoCo Simulation

Launch the Cobotta workcell in MuJoCo with RViz visualization:

```bash
source install/setup.bash
ros2 launch techtory_cobotta_bringup techtory_cobotta_mj_bringup.launch.py 
```

BT Application:

```bash
ros2 launch techtory_cobotta_system techtory_cobotta_system.launch.py 
```

Send Goal: 

```bash
ros2 action send_goal /start_application man2_msgs/action/RunApplication "{behavior_tree_filename: '/home/adm-vsp/ros_ws/arena/colcon_techtory_ws/src/techtory_cobotta_system/trees/techtory_cobotta_system.xml'}"
```

## Installation Guides

- [Isaac Sim Environment Setup](./INSTALL_ISAAC_SIM.md)
- [Cumotion - Isaac ROS Cumotion + MoveIt Integration](./INSTALL_CUMOTION.md)
