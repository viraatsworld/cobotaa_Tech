cmake_minimum_required(VERSION 3.16)
project(techtory_hybrid_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_hybrid_planning REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(hybrid_planning_demo_node src/hybrid_planning_demo_node.cpp)
ament_target_dependencies(hybrid_planning_demo_node
  rclcpp
  rclcpp_action
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  shape_msgs
  trajectory_msgs
  tf2
  tf2_ros
)

add_executable(hybrid_planning_demo2_node src/hybrid_planning_demo2_node.cpp)
ament_target_dependencies(hybrid_planning_demo2_node
  rclcpp
  rclcpp_action
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  shape_msgs
  trajectory_msgs
  tf2
  tf2_ros
)

# --- demo3: MTC welding global planner plugin + client ---------------------

# Global Planner plugin (loaded by moveit_hybrid_planning's GlobalPlannerComponent).
add_library(global_mtc_planner_component SHARED src/global_mtc_planner.cpp)
target_include_directories(global_mtc_planner_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(global_mtc_planner_component
  rclcpp
  rclcpp_action
  pluginlib
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_hybrid_planning
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  geometry_msgs
  tf2
  tf2_eigen
  Eigen3
)
pluginlib_export_plugin_description_file(moveit_hybrid_planning plugins.xml)

# demo3 action client.
add_executable(hybrid_planning_demo3_node src/hybrid_planning_demo3_node.cpp)
ament_target_dependencies(hybrid_planning_demo3_node
  rclcpp
  rclcpp_action
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  shape_msgs
  trajectory_msgs
  tf2
  tf2_ros
  Eigen3
)

install(TARGETS hybrid_planning_demo_node hybrid_planning_demo2_node hybrid_planning_demo3_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS global_mtc_planner_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
