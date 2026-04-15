# Cumotion - Isaac ROS Cumotion + MoveIt Integration

Cumotion is the high-performance motion planning and control integration for Isaac ROS, combining Curobo with ROS 2 for advanced robot motion generation.

## Prerequisites

### 1. Isaac ROS Environment

Follow the [Isaac ROS Installation Guide](https://nvidia-isaac-ros.github.io/getting_started/index.html).

### 2. Isaac ROS CLI

Install from the `isaac-ros-dev` folder:

```bash
cd ~/isaac-ros-dev
pip install -e ./
```

Alternatively, install via apt:

```bash
sudo apt-get install isaac-ros-cli
```

If installing via apt, set up the virtual environment by following [these instructions](https://nvidia-isaac-ros.github.io/getting_started/index.html#initialize-isaac-ros-cli) (about 8-9 steps).

Then activate the environment:

```bash
isaac-ros activate
```

> **Tip:** To make `isaac-ros` available globally, add the following to your `~/.bashrc`:
>
> ```bash
> export ISAAC_ROS_WS="${ISAAC_ROS_WS:-/your-path/src/isaac-ros-dev}"
> ```

### 3. Isaac ROS Cumotion Package

Clone and build the [isaac_ros_cumotion](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion) repository:

```bash
cd ~/colcon_techtory_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git
cd ~/colcon_techtory_ws
colcon build --symlink-install --packages-up-to techtory_cobotta_bringup
```

> **Important:** Ensure a `cobotta.xrdf` file exists inside `isaac_ros_cumotion/isaac_ros_cumotion_robot_description/xrdf/`.

### 4. Topic-Based ROS 2 Control

The [topic_based_ros2_control](https://github.com/PickNikRobotics/topic_based_ros2_control/tree/main) package is included in `upstream.repo`. Follow the [Getting Started](./README.md#getting-started) section in the main README to import it.

---

## Quick Start - 3 Terminal Setup

The Cumotion workflow requires **three separate terminals**:

### Terminal 1: Isaac Sim Server

Runs the Isaac Sim physics engine with the techtory workcell.

1. Ensure your Isaac Sim virtual environment is active (see [Isaac Sim setup](./INSTALL_ISAAC_SIM.md)).
2. Launch Isaac Sim:

    ```bash
    isaacsim
    ```

3. Open the USD file `techtory_cvrb0609_moveit-final.usd` from `techtory_cobotta_workcell_description/urdf/`.

### Terminal 2: Robot Bringup

Launches robot control drivers. Choose **simulation** or **real hardware**:

**Simulation (recommended for testing):**

```bash
source install/setup.bash
ros2 launch techtory_cobotta_bringup techtory_cobotta_sw_bringup.launch.py
```

**Real hardware:**

```bash
source install/setup.bash
ros2 launch techtory_cobotta_bringup techtory_cobotta_hw_bringup.launch.py
```

This initializes robot controllers and connects to either the simulator or real hardware via ROS 2 interfaces.

### Terminal 3: Cumotion Motion Planner

Runs the motion planning engine:

```bash
source install/setup.bash

ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
  cumotion_planner.robot:=<XRDF_FILE_PATH> \
  cumotion_planner.urdf_path:=<URDF_FILE_PATH>
```

**Parameters:**

| Parameter | Description | Example |
|-----------|-------------|---------|
| `cumotion_planner.robot` | Path to the robot XRDF file | `~/colcon_techtory_ws/src/isaac_ros_cumotion/isaac_ros_cumotion_robot_description/xrdf/cobotta.xrdf` |
| `cumotion_planner.urdf_path` | Path to the workcell URDF file | `~/colcon_techtory_ws/src/techtory_cobotta_workcell_description/urdf/techtory_cobotta_workcell.urdf` |

**Full example:**

```bash
source install/setup.bash

ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
  cumotion_planner.robot:=/home/my-pc/colcon_techtory_ws/src/isaac_ros_cumotion/isaac_ros_cumotion_robot_description/xrdf/cobotta.xrdf \
  cumotion_planner.urdf_path:=/home/my-pc/colcon_techtory_ws/src/techtory_cobotta_workcell_description/urdf/techtory_cobotta_workcell.urdf
```

---

## Workflow Summary

| Terminal | Purpose | Command |
|----------|---------|---------|
| 1 - Isaac Sim | Physics simulation & visualization | `isaacsim` + open USD file |
| 2 - Bringup | Robot drivers & control | `ros2 launch techtory_cobotta_bringup techtory_cobotta_sw_bringup.launch.py` |
| 3 - Cumotion | Motion planning & execution | `ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py ...` |

## Configuration Tips

Always source your workspace before launching any terminal:

```bash
source ~/colcon_techtory_ws/install/setup.bash
```

## Additional Resources

- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Cumotion Repository](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/index.html)
- [Curobo Motion Planning](https://curobo.org/)
