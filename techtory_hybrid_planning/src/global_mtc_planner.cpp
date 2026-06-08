#include <techtory_hybrid_planning/global_mtc_planner.hpp>

#include <algorithm>

// MTC stages / solvers
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <rclcpp/duration.hpp>

namespace techtory_hybrid_planning
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("global_mtc_planner_component");
const std::string PLANNING_PIPELINES_NS = "ompl.";
const std::string PLAN_REQUEST_PARAM_NS = "plan_request_params.";
const std::string UNDEFINED = "<undefined>";

// Cobotta-specific planning context (see techtory_demo_description.srdf).
const std::string GROUP_NAME = "arm";
const std::string IK_FRAME = "cobotta_pro_tool0";
const std::string PLANNING_FRAME = "world";
constexpr double APPROACH_RETREAT_OFFSET_Z = -0.1;  // metres, along TCP z

using namespace std::chrono_literals;
using namespace moveit::task_constructor;

bool GlobalMTCPlannerComponent::initialize(const rclcpp::Node::SharedPtr& node)
{
  // Declare planning pipeline parameters (Pilz industrial motion planner).
  node->declare_parameter<std::vector<std::string>>(PLANNING_PIPELINES_NS + "pipeline_names",
                                                    std::vector<std::string>({ "pilz_industrial_motion_planner" }));
  node->declare_parameter<std::string>(PLANNING_PIPELINES_NS + "namespace", UNDEFINED);
  node->declare_parameter<std::string>(PLANNING_PIPELINES_NS + "planning_plugin",
                                       "pilz_industrial_motion_planner/CommandPlanner");

  // Declare PlanRequestParameters.
  node->declare_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planner_id", "LIN");
  node->declare_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planning_pipeline", "pilz_industrial_motion_planner");
  node->declare_parameter<int>(PLAN_REQUEST_PARAM_NS + "planning_attempts", 5);
  node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "planning_time", 1.0);
  node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_velocity_scaling_factor", 0.1);
  node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_acceleration_scaling_factor", 0.1);

  task_ = std::make_shared<moveit::task_constructor::Task>();
  node_ptr_ = node;
  return true;
}

bool GlobalMTCPlannerComponent::reset() noexcept
{
  // Drop any task/solution state from a previous plan() call.
  task_.reset();
  return true;
}

moveit_msgs::msg::MotionPlanResponse GlobalMTCPlannerComponent::plan(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> global_goal_handle)
{
  // Result
  moveit_msgs::msg::MotionPlanResponse planning_solution;
  planning_solution.error_code.val = planning_solution.error_code.FAILURE;
  planning_solution.group_name = GROUP_NAME;

  // Process goal
  const auto& items = (global_goal_handle->get_goal())->motion_sequence.items;
  if (items.empty())
  {
    RCLCPP_ERROR(LOGGER, "Global planner received an empty motion sequence; nothing to plan.");
    return planning_solution;
  }
  if (items.size() > 1)
  {
    RCLCPP_WARN(LOGGER, "Global planner received motion sequence request with more than one item but this MTC "
                        "plugin only uses the first item as the global planning goal!");
  }
  auto motion_plan_req = items[0].req;

  // Validate the request carries the welding layout (start + goal poses).
  if (motion_plan_req.goal_constraints.empty() ||
      motion_plan_req.goal_constraints[0].position_constraints.size() < 2 ||
      motion_plan_req.goal_constraints[0].orientation_constraints.size() < 2)
  {
    RCLCPP_ERROR(LOGGER, "MTC global planner expects goal_constraints[0] with two position_constraints and two "
                         "orientation_constraints (start + goal). Got %zu position / %zu orientation constraints.",
                 motion_plan_req.goal_constraints.empty()
                     ? 0UL
                     : motion_plan_req.goal_constraints[0].position_constraints.size(),
                 motion_plan_req.goal_constraints.empty()
                     ? 0UL
                     : motion_plan_req.goal_constraints[0].orientation_constraints.size());
    return planning_solution;
  }

  task_ = std::make_shared<moveit::task_constructor::Task>();
  moveit::task_constructor::Task& t = *task_;
  t.stages()->setName("global_mtc_task");
  t.loadRobotModel(node_ptr_);

  // Sampling planner (Pilz). LIN gives linear Cartesian motion between poses.
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node_ptr_, "pilz_industrial_motion_planner");
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
  sampling_planner->setProperty("max_velocity_scaling_factor", motion_plan_req.max_velocity_scaling_factor);
  sampling_planner->setProperty("max_acceleration_scaling_factor", motion_plan_req.max_acceleration_scaling_factor);
  sampling_planner->setProperty("planning_attempts", motion_plan_req.num_planning_attempts);
  sampling_planner->setProperty("planning_time", motion_plan_req.allowed_planning_time);
  sampling_planner->setPlannerId("pilz_industrial_motion_planner", "LIN");

  // Task-level properties. No end-effector group is set: the Cobotta SRDF only
  // defines an EEF on the "gripper" group, not on "arm", so MTC IK is driven by
  // the IK frame alone.
  t.setProperty("group", GROUP_NAME);
  t.setProperty("ik_frame", IK_FRAME);

  /****************************************************
   *               Current State                      *
   ***************************************************/
  {
    auto current_state = std::make_unique<stages::CurrentState>("Initial State");
    t.add(std::move(current_state));
  }

  /****************************************************
   *               WELDING                            *
   ***************************************************/
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header.frame_id = PLANNING_FRAME;
  start_pose.pose.position.x = motion_plan_req.goal_constraints[0].position_constraints[0].target_point_offset.x;
  start_pose.pose.position.y = motion_plan_req.goal_constraints[0].position_constraints[0].target_point_offset.y;
  start_pose.pose.position.z = motion_plan_req.goal_constraints[0].position_constraints[0].target_point_offset.z;
  start_pose.pose.orientation = motion_plan_req.goal_constraints[0].orientation_constraints[0].orientation;

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = PLANNING_FRAME;
  goal_pose.pose.position.x = motion_plan_req.goal_constraints[0].position_constraints[1].target_point_offset.x;
  goal_pose.pose.position.y = motion_plan_req.goal_constraints[0].position_constraints[1].target_point_offset.y;
  goal_pose.pose.position.z = motion_plan_req.goal_constraints[0].position_constraints[1].target_point_offset.z;
  goal_pose.pose.orientation = motion_plan_req.goal_constraints[0].orientation_constraints[1].orientation;

  // Move to approach (start pose offset back along TCP z).
  {
    Eigen::Isometry3d goal;
    tf2::fromMsg(start_pose.pose, goal);
    Eigen::Isometry3d approach_offset = Eigen::Isometry3d::Identity();
    approach_offset.translation().z() = APPROACH_RETREAT_OFFSET_Z;
    goal = goal * approach_offset;
    geometry_msgs::msg::PoseStamped approach_pose = start_pose;
    tf2::convert(goal, approach_pose.pose);

    auto stage = std::make_unique<stages::MoveTo>("Move to Approach Pose", sampling_planner);
    stage->setGroup(GROUP_NAME);
    stage->setIKFrame(IK_FRAME);
    stage->properties().set("marker_ns", "approach");
    stage->setGoal(approach_pose);
    t.add(std::move(stage));
  }

  // Approach to start point.
  {
    auto stage = std::make_unique<stages::MoveTo>("Approach to start point", sampling_planner);
    stage->setGroup(GROUP_NAME);
    stage->setIKFrame(IK_FRAME);
    stage->properties().set("marker_ns", "approach");
    stage->setGoal(start_pose);
    t.add(std::move(stage));
  }

  // Weld (start -> goal).
  {
    auto stage = std::make_unique<stages::MoveTo>("Welding Motion", sampling_planner);
    stage->setGroup(GROUP_NAME);
    stage->setIKFrame(IK_FRAME);
    stage->properties().set("marker_ns", "weld");
    stage->setGoal(goal_pose);
    t.add(std::move(stage));
  }

  // Retreat (goal pose offset back along TCP z).
  {
    Eigen::Isometry3d goal;
    tf2::fromMsg(goal_pose.pose, goal);
    Eigen::Isometry3d retreat_offset = Eigen::Isometry3d::Identity();
    retreat_offset.translation().z() = APPROACH_RETREAT_OFFSET_Z;
    goal = goal * retreat_offset;
    geometry_msgs::msg::PoseStamped retreat_pose = goal_pose;
    tf2::convert(goal, retreat_pose.pose);

    auto stage = std::make_unique<stages::MoveTo>("Retreat Motion", sampling_planner);
    stage->setGroup(GROUP_NAME);
    stage->setIKFrame(IK_FRAME);
    stage->properties().set("marker_ns", "retreat");
    stage->setGoal(retreat_pose);
    t.add(std::move(stage));
  }

  /****************************************************
   *               Execution                          *
   ***************************************************/
  try
  {
    t.init();
  }
  catch (const moveit::task_constructor::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "MTC task init failed: " << e);
    return planning_solution;
  }

  constexpr size_t max_solutions = 1;
  const moveit::core::MoveItErrorCode plan_result = t.plan(max_solutions);
  if (!plan_result || t.numSolutions() == 0)
  {
    RCLCPP_ERROR(LOGGER, "MTC failed to find a solution for the welding task (error code %d, %zu solutions).",
                 plan_result.val, t.numSolutions());
    return planning_solution;
  }

  moveit_task_constructor_msgs::msg::Solution solution;
  t.solutions().front()->appendTo(solution, &t.introspection());

  // Concatenate sub-trajectories. Each MTC sub-trajectory restarts its
  // time_from_start at zero, so re-stamp points with a running time offset to
  // produce a single monotonically-increasing trajectory for the local planner.
  auto& solution_traj = planning_solution.trajectory;
  rclcpp::Duration time_offset(0, 0);
  for (const auto& sub_traj : solution.sub_trajectory)
  {
    const auto& traj = sub_traj.trajectory;
    if (solution_traj.joint_trajectory.joint_names.empty())
      solution_traj.joint_trajectory.joint_names = traj.joint_trajectory.joint_names;
    if (solution_traj.multi_dof_joint_trajectory.joint_names.empty())
      solution_traj.multi_dof_joint_trajectory.joint_names = traj.multi_dof_joint_trajectory.joint_names;

    // Sub-trajectory duration is the latest time stamp across BOTH the joint
    // and multi-dof point lists (either may be empty or the longer of the two).
    rclcpp::Duration sub_duration(0, 0);
    for (auto point : traj.joint_trajectory.points)
    {
      const rclcpp::Duration original(point.time_from_start);
      sub_duration = std::max(sub_duration, original);
      point.time_from_start = (time_offset + original);
      solution_traj.joint_trajectory.points.push_back(point);
    }
    for (auto point : traj.multi_dof_joint_trajectory.points)
    {
      const rclcpp::Duration original(point.time_from_start);
      sub_duration = std::max(sub_duration, original);
      point.time_from_start = (time_offset + original);
      solution_traj.multi_dof_joint_trajectory.points.push_back(point);
    }
    time_offset = time_offset + sub_duration;
  }

  planning_solution.error_code.val = planning_solution.error_code.SUCCESS;
  return planning_solution;
}
}  // namespace techtory_hybrid_planning

// Register the component as plugin
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(techtory_hybrid_planning::GlobalMTCPlannerComponent,
                       moveit::hybrid_planning::GlobalPlannerInterface);
