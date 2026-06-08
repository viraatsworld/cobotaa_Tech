// Hybrid Planning demo client (variant 2) for the Techtory Cobotta cell.
//
// Sends two hard-coded Cartesian waypoints (in the planning frame) to the
// Hybrid Planning Manager. Each pose is solved with collision-aware IK and
// the resulting joint values are logged before the goal is dispatched.

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>

#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>

using HybridPlanner = moveit_msgs::action::HybridPlanner;

namespace
{
struct CartesianWaypoint
{
  std::string name;
  double x, y, z;
  double roll, pitch, yaw;
  double blend_radius = 0.0;
  double velocity_scaling = 0.01;
  double acceleration_scaling = 0.01;
  std::string pipeline_id = "ompl";
  std::string planner_id = "RRTConnect";
  double allowed_planning_time = 5.0;
};

template <typename T>
T getOr(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& fallback)
{
  T value;
  if (node->get_parameter(name, value))
    return value;
  return fallback;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "hybrid_planning_demo2_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  const std::string planning_group = getOr<std::string>(node, "planning_group", "arm");
  const std::string action_name =
      getOr<std::string>(node, "hybrid_planning_action", "/run_hybrid_planning");
  const std::string planning_frame =
      getOr<std::string>(node, "planning_frame", "cobotta_pro_base_link");
  const std::string tip_link = getOr<std::string>(node, "tip_link", "cobotta_pro_tool0");
  const double wait_for_server_timeout = getOr<double>(node, "wait_for_server_timeout", 120.0);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // Two hard-coded Cartesian goals in `planning_frame`.
  const std::vector<CartesianWaypoint> waypoints = {
      { "pose_a", 0.450, 0.100, 0.410, 3.140, 0.0, 3.140 },
      { "pose_b", 0.450, 0.100, 0.410, 3.140, 0.0, 3.140 },
      { "pose_c", 0.450, 0.200, 0.410, 3.140, 0.0, 3.140 },
      { "pose_d", 0.450, 0.300, 0.410, 3.140, 0.0, 3.140 },
      { "pose_e", 0.450, 0.400, 0.410, 3.140, 0.0, 3.140 },
  };

  auto hybrid_planning_client = rclcpp_action::create_client<HybridPlanner>(node, action_name);

  RCLCPP_INFO(logger, "Waiting up to %.0fs for action server '%s'...", wait_for_server_timeout,
              action_name.c_str());
  if (!hybrid_planning_client->wait_for_action_server(
          std::chrono::duration<double>(wait_for_server_timeout)))
  {
    RCLCPP_ERROR(logger, "Hybrid Planner action server '%s' not available.", action_name.c_str());
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }
  RCLCPP_INFO(logger, "Action server available.");

  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  if (!robot_model)
  {
    RCLCPP_ERROR(logger, "Failed to load robot model from /robot_description.");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  const moveit::core::JointModelGroup* joint_model_group =
      robot_model->getJointModelGroup(planning_group);
  if (!joint_model_group)
  {
    RCLCPP_ERROR(logger, "Planning group '%s' not found in SRDF.", planning_group.c_str());
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  psm->startStateMonitor("/joint_states");
  psm->startSceneMonitor("/planning_scene");
  if (!psm->waitForCurrentRobotState(node->now(), 5.0))
  {
    RCLCPP_WARN(logger, "Timed out waiting for current robot state; IK seeds may be cold.");
  }

  moveit::core::RobotState seed_state(robot_model);
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(psm);
    seed_state = scene->getCurrentState();
  }
  seed_state.update();

  auto is_state_valid = [&psm](moveit::core::RobotState* state,
                               const moveit::core::JointModelGroup* group,
                               const double* joint_group_positions) {
    state->setJointGroupPositions(group, joint_group_positions);
    state->update();
    planning_scene_monitor::LockedPlanningSceneRO scene(psm);
    collision_detection::CollisionRequest req;
    req.group_name = group->getName();
    collision_detection::CollisionResult res;
    scene->checkCollision(req, res, *state, scene->getAllowedCollisionMatrix());
    return !res.collision;
  };

  HybridPlanner::Goal goal_action_request;
  goal_action_request.planning_group = planning_group;

  moveit::core::RobotState current_state = seed_state;
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    const CartesianWaypoint& wp = waypoints[i];
    moveit::core::RobotState goal_state = current_state;
    goal_state.update();

    // setFromIK interprets the pose in the model root frame; transform from
    // `planning_frame` to world explicitly.
    const Eigen::Isometry3d& world_T_planning =
        goal_state.getGlobalLinkTransform(planning_frame);

    Eigen::Isometry3d target_in_planning;
    target_in_planning.translation() = Eigen::Vector3d(wp.x, wp.y, wp.z);
    target_in_planning.linear() = (Eigen::AngleAxisd(wp.roll, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(wp.pitch, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(wp.yaw, Eigen::Vector3d::UnitZ()))
                                      .toRotationMatrix();
    const Eigen::Isometry3d world_T_target = world_T_planning * target_in_planning;

    geometry_msgs::msg::Pose pose;
    pose.position.x = world_T_target.translation().x();
    pose.position.y = world_T_target.translation().y();
    pose.position.z = world_T_target.translation().z();
    Eigen::Quaterniond q(world_T_target.rotation());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    RCLCPP_INFO(logger,
                "Waypoint '%s': target in '%s' xyz=[%.3f,%.3f,%.3f] rpy=[%.3f,%.3f,%.3f].",
                wp.name.c_str(), planning_frame.c_str(), wp.x, wp.y, wp.z, wp.roll, wp.pitch,
                wp.yaw);

    constexpr double kIkTimeout = 0.5;
    bool ok = goal_state.setFromIK(joint_model_group, pose, tip_link, kIkTimeout, is_state_valid);
    if (!ok)
    {
      moveit::core::RobotState probe = goal_state;
      if (probe.setFromIK(joint_model_group, pose, tip_link, kIkTimeout))
      {
        RCLCPP_ERROR(logger,
                     "Waypoint '%s': IK solution exists but every candidate collides.",
                     wp.name.c_str());
      }
      else
      {
        RCLCPP_ERROR(logger, "Waypoint '%s': pose is unreachable by IK.", wp.name.c_str());
      }
      rclcpp::shutdown();
      spin_thread.join();
      return 1;
    }

    std::vector<double> joint_values;
    goal_state.copyJointGroupPositions(joint_model_group, joint_values);
    std::ostringstream oss;
    oss << "Waypoint '" << wp.name << "' joint solution:";
    for (std::size_t j = 0; j < joint_values.size(); ++j)
    {
      oss << " " << joint_names[j] << "=" << joint_values[j];
    }
    RCLCPP_INFO(logger, "%s", oss.str().c_str());

    moveit_msgs::msg::MotionSequenceItem item;
    item.req.group_name = planning_group;
    item.req.pipeline_id = wp.pipeline_id;
    item.req.planner_id = wp.planner_id;
    item.req.allowed_planning_time = wp.allowed_planning_time;
    item.req.max_velocity_scaling_factor = wp.velocity_scaling;
    item.req.max_acceleration_scaling_factor = wp.acceleration_scaling;
    item.req.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group));
    item.blend_radius = (i + 1 == waypoints.size()) ? 0.0 : wp.blend_radius;
    goal_action_request.motion_sequence.items.push_back(item);

    current_state = goal_state;
  }

  std::promise<rclcpp_action::ResultCode> result_promise;
  auto result_future = result_promise.get_future();

  auto send_goal_options = rclcpp_action::Client<HybridPlanner>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [&logger](const rclcpp_action::ClientGoalHandle<HybridPlanner>::SharedPtr& handle) {
        if (!handle)
          RCLCPP_ERROR(logger, "Hybrid Planning goal was rejected by the server.");
        else
          RCLCPP_INFO(logger, "Hybrid Planning goal accepted by the server.");
      };
  send_goal_options.feedback_callback =
      [&logger](rclcpp_action::ClientGoalHandle<HybridPlanner>::SharedPtr,
                const std::shared_ptr<const HybridPlanner::Feedback> feedback) {
        RCLCPP_INFO(logger, "Hybrid Planner feedback: %s", feedback->feedback.c_str());
      };
  send_goal_options.result_callback =
      [&logger,
       &result_promise](const rclcpp_action::ClientGoalHandle<HybridPlanner>::WrappedResult& result) {
        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(logger, "Hybrid Planning goal succeeded.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(logger, "Hybrid Planning goal aborted: %s",
                         result.result->error_message.c_str());
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(logger, "Hybrid Planning goal was canceled.");
            break;
          default:
            RCLCPP_ERROR(logger, "Hybrid Planning goal returned unknown code.");
            break;
        }
        result_promise.set_value(result.code);
      };

  RCLCPP_INFO(logger, "Sending Hybrid Planning goal with %zu waypoint(s)...",
              goal_action_request.motion_sequence.items.size());
  hybrid_planning_client->async_send_goal(goal_action_request, send_goal_options);

  result_future.wait();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
