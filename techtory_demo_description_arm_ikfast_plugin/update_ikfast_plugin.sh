search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=techtory_demo_description.srdf
robot_name_in_srdf=techtory_demo_description
moveit_config_pkg=techtory_demo_description_moveit_config
robot_name=techtory_demo_description
planning_group_name=arm
ikfast_plugin_pkg=techtory_demo_description_arm_ikfast_plugin
base_link_name=cobotta_pro_base_link
eef_link_name=cobotta_pro_tool0
ikfast_output_path=/home/adm-vsp/ros_ws/arena/colcon_techtory_ws/src/techtory_cobotta_workcell_description/urdf/techtory_demo_description_arm_ikfast_plugin/src/techtory_demo_description_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
