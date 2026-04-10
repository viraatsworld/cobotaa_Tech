# techtory cell description

## Creating Isaac Sim Environment

To create the Isaac Sim environment, run the following command:

```bash
mkdir -p colcon_tectory_ws/src 
cd colcon_tectory_ws
python3.11 -m venv .myvirtualenv
source .myvirtualenv/bin/activate
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
```

## Getting started

This package provides a URDF description of a techtory cell + cobotta robot. To visualize the cell in RViz, run:

```bash
cd colcon_techtory_ws
git clone git@gitlab.cc-asp.fraunhofer.de:ipa326/demonstrator/arena2036/dynamic_planning_demo.git src
cd src 
vcs import . < upstream.repo
cd ..
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src -iry
``` 

```bash
ros2 launch techtory_cobotta_workcell_description display.launch.py
```

## Cumotion - Isaac ROS Cumotion + Moveit Integration

Cumotion is the high-performance motion planning and control integration for Isaac ROS, combining the power of Curobo with ROS 2 for advanced robot motion generation.

### 📋 Prerequisites

Before starting, ensure you have the following installed and configured:

- **Isaac ROS Environment**: Follow the [Isaac ROS Installation Guide](https://nvidia-isaac-ros.github.io/getting_started/index.html)
  
- **Isaac ROS CLI**: Install from the isaac-ros-dev folder
    ```bash
  cd ~/isaac-ros-dev
    pip install -e ./  # Install isaac-ros CLI utilities
  ```
    - Alternatively, you can install **Isaac ROS CLI** using [command line](https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli)
        ```bash
        sudo apt-get install isaac-ros-cli
        ```
        - Create Virtual environment setup (total 8-9 instructions) by follwing [these instructions](https://nvidia-isaac-ros.github.io/getting_started/index.html#initialize-isaac-ros-cli)

    Finally activate the environment :
     ```bash
    isaac-ros activate
    ```
    **Tip** : To make the isaac-ros global, add the path in the `bashrc` file 
    ```bash
    export ISAAC_ROS_WS="${ISAAC_ROS_WS:-/your-path/src/isaac-ros-dev}"
    ```
- **Isaac ROS Cumotion Package**: Clone and build the [isaac-ros-cumotion repository](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion?tab=readme-ov-file)
  ```bash
  cd ~/colcon_techtory_ws/src
  git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git
  cd ~/colcon_techtory_ws
  colcon build --symlink-install --packages-up-to techtory_cobotta_bringup
  ```
    - Make sure there is `cobotta.xrdf` file already defined inside the folder `isaac-ros-cumotion/isaac_ros_cumotion_robot_description/xrdf/`
- **Topic based Ros2 Control** - Clone and build the [topic_based_ros2_control](https://github.com/PickNikRobotics/topic_based_ros2_control/tree/main) repository.

    - The package is present in upstream.repo.
    - Please follow the [getting started](#getting-started) section

### 🚀 Quick Start - 3 Terminal Setup

The Cumotion workflow requires **three separate terminals**, each with a specific role:

#### **Terminal 1: 🏢 Isaac Sim Server**

This terminal runs the Isaac Sim physics engine with the techtory workcell visualization.

Step 1: Make sure isaaclab/virtual environment is running for Isaac Sim.

Step 2: Launch the isaacsim by typing ```isaacsim``` in terminal

```bash
# Launch Isaac Sim
isaac-sim 
```

Step 3: Open the already existing usd file `techtory_cvrb0609_moveit-final.usd ` present inside `techtory_cobotta_workcell_description/urdf`


**Note**:  
- Replace the path with your actual Isaac Sim installation and USD file location.

```

---

#### **Terminal 2: 🎮 Robot Bringup (Choose One)**

This terminal launches the robot control drivers. Choose either **simulation** or **real hardware**:

**For Simulation (Recommended for Testing):**
```bash
source install/setup.bash
ros2 launch techtory_cobotta_bringup techtory_cobotta_sw_bringup.launch.py
```

**For Real Hardware:**
```bash
source install/setup.bash
ros2 launch techtory_cobotta_bringup techtory_cobotta_hw_bringup.launch.py
```

**What this does:**
- Initializes robot controllers
- Connects to either the physics simulator or real robot hardware
- Sets up ROS 2 interfaces for motion commands

---

#### **Terminal 3: 🤖 Cumotion Motion Planner**

This terminal runs the high-performance motion planning engine:

```bash
source install/setup.bash

ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
  cumotion_planner.robot:=<XRDF_FILE_PATH> \
  cumotion_planner.urdf_path:=<URDF_FILE_PATH>
```

**Parameter Configuration:**

Replace the placeholders with your actual file paths:

- `<XRDF_FILE_PATH>`: Path to the robot XRDF description file
  - Example: `~/colcon_techtory_ws/src/isaac_ros_cumotion/isaac_ros_cumotion_robot_description/xrdf/cobotta.xrdf`

- `<URDF_FILE_PATH>`: Path to the workcell URDF file
  - Example: `~/colcon_techtory_ws/src/techtory_cobotta_workcell_description/urdf/techtory_cobotta_workcell.urdf`

**Example Full Command:**
```bash
source install/setup.bash

ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py cumotion_planner.robot:=/home/my-pc/colcon_techtory_ws/src/isaac_ros_cumotion/isaac_ros_cumotion_robot_description/xrdf/cobotta.xrdf cumotion_planner.urdf_path:=/home/my-pc/colcon_techtory_ws/src/techtory_cobotta_workcell_description/urdf/techtory_cobotta_workcell.urdf
```

---

### 📝 Workflow Summary

| Terminal | Purpose | Command |
|----------|---------|---------|
| 1️⃣ **Isaac Sim** | Physics simulation & visualization | `isaac-sim with <USD_FILE>` |
| 2️⃣ **Bringup** | Robot drivers & control | `ros2 launch techtory_cobotta_bringup techtory_cobotta_sw_bringup.launch.py` |
| 3️⃣ **Cumotion** | Motion planning & execution | `ros2 launch isaac_ros_cumotion ...` |

---

### ⚙️ Configuration Tips

- **Workspace Setup**: Always source your workspace before launching:
  ```bash
  source ~/colcon_techtory_ws/install/setup.bash
  ```

---

### 🔗 Additional Resources

- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Cumotion GitHub Repository](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/index.html)
- [Curobo Motion Planning](https://curobo.org/)