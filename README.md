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
    ```

## Visualize in RViz

```bash
ros2 launch techtory_cobotta_workcell_description display.launch.py
```

## MuJoCo Simulation

Launch the Cobotta workcell in MuJoCo with RViz visualization:

```bash
source install/setup.bash
ros2 launch techtory_cobotta_workcell_description mujoco.launch.py
```

To run without RViz:

```bash
ros2 launch techtory_cobotta_workcell_description mujoco.launch.py launch_rviz:=false
```

To run headless (no GUI):

```bash
ros2 launch techtory_cobotta_workcell_description mujoco.launch.py headless:=true
```

## Installation Guides

- [Isaac Sim Environment Setup](./INSTALL_ISAAC_SIM.md)
- [Cumotion - Isaac ROS Cumotion + MoveIt Integration](./INSTALL_CUMOTION.md)
