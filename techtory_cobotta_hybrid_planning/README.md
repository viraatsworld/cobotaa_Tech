# Techtory Cobotta Hybrid Planning

This package provides a demonstration of the MoveIt Hybrid Planning architecture using the Techtory Cobotta setup.

## Features

- Uses the MoveIt 2 Hybrid Planning Architecture (Global Planner, Local Planner, Hybrid Planning Manager).
- Configured for the `arm` joint group of the Cobotta robot.
- Includes an example C++ node that sends a simple joint space goal to the Hybrid Planner action server.

## Prerequisites

- ROS 2 Jazzy (or your active ROS 2 distro)
- Built workspace with `techtory_cobotta_moveit` and `moveit_hybrid_planning` available.

## How to use

1. **Build the workspace**

   Navigate to the root of your colcon workspace and run:

   ```bash
   colcon build --packages-up-to techtory_cobotta_hybrid_planning
   ```

2. **Source the workspace**

   ```bash
   source install/setup.bash
   ```

3. **Run the demo**

   Launch the hybrid planning node and its components:

   ```bash
   ros2 launch techtory_cobotta_hybrid_planning hybrid_planning_demo.launch.py
   ```

   This will:
   - Load the global and local planner configurations from `techtory_cobotta_moveit`.
   - Start the component container holding the Global Planner, Local Planner, and Hybrid Planning Manager.
   - Run the C++ demo node which requests a short motion plan and monitors the execution.
