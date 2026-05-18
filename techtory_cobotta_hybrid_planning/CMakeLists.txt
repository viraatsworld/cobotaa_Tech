cmake_minimum_required(VERSION 3.8)
project(techtory_cobotta_hybrid_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(techtory_cobotta_moveit REQUIRED)

add_executable(hybrid_planning_demo_node src/hybrid_planning_demo_node.cpp)
ament_target_dependencies(hybrid_planning_demo_node
  rclcpp
  rclcpp_action
  moveit_msgs
  moveit_ros_planning_interface
)

install(TARGETS
  hybrid_planning_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
