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
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

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

install(TARGETS hybrid_planning_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
