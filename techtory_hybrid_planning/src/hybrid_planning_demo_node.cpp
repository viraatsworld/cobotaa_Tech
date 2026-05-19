// Hybrid Planning demo client for the Techtory Cobotta cell.
//
// Follows the structure of the upstream moveit2_tutorials hybrid_planner
// example: build a RobotModel, define a Cartesian goal, solve IK to obtain a
// joint-space target, wrap it as a MotionSequenceRequest and dispatch it to
// the HybridPlanner action server exposed by the manager.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>

#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using HybridPlanner = moveit_msgs::action::HybridPlanner;

namespace
{
geometry_msgs::msg::PoseStamped makePose(const std::string& frame_id, double x, double y, double z,
                                         double roll, double pitch, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "hybrid_planning_demo_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  const std::string planning_group =
      node->get_parameter_or<std::string>("planning_group", "arm");
  const std::string action_name = node->get_parameter_or<std::string>(
      "hybrid_planning_action", "/run_hybrid_planning");
  const std::string planning_frame =
      node->get_parameter_or<std::string>("planning_frame", "world");
  const std::string tip_link =
      node->get_parameter_or<std::string>("tip_link", "cobotta_pro_tool0");
  const double wait_for_server_timeout =
      node->get_parameter_or<double>("wait_for_server_timeout", 120.0);

  // Spin the node in the background so that the planning scene monitor /
  // robot model loader can service its callbacks while we set things up.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // ---------------------------------------------------------------------------
  // Action client targeting the Hybrid Planning Manager.
  // ---------------------------------------------------------------------------
  auto hybrid_planning_client =
      rclcpp_action::create_client<HybridPlanner>(node, action_name);

  RCLCPP_INFO(logger, "Waiting up to %.0fs for action server '%s'...",
              wait_for_server_timeout, action_name.c_str());
  if (!hybrid_planning_client->wait_for_action_server(
          std::chrono::duration<double>(wait_for_server_timeout)))
  {
    RCLCPP_ERROR(logger, "Hybrid Planner action server '%s' not available.",
                 action_name.c_str());
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }
  RCLCPP_INFO(logger, "Action server available.");

  // ---------------------------------------------------------------------------
  // Robot model & IK to resolve the Cartesian goal into joint constraints.
  // ---------------------------------------------------------------------------
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
    RCLCPP_ERROR(logger, "Planning group '%s' not found in SRDF.",
                 planning_group.c_str());
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  // Define the Cartesian goal pose.
  geometry_msgs::msg::PoseStamped desired_pose =
      makePose(planning_frame, 0.563, 0.120, 0.3, -3.114, 0.156, -3.138);

  RCLCPP_INFO(logger,
              "Target pose (frame=%s): xyz=[%.3f, %.3f, %.3f] rpy=[%.3f, %.3f, %.3f]",
              planning_frame.c_str(), desired_pose.pose.position.x,
              desired_pose.pose.position.y, desired_pose.pose.position.z, -3.114,
              0.156, -3.138);

  // Build a PlanningSceneMonitor so IK can reject configurations that would
  // collide with the workcell (cell_link, table, etc.) or self-collide. The
  // monitor subscribes to /joint_states so we also seed IK with the robot's
  // current pose instead of the default zero configuration — that biases
  // KDL towards a feasible nearby solution.
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node, "robot_description");
  psm->startStateMonitor("/joint_states");
  psm->startSceneMonitor("/planning_scene");
  if (!psm->waitForCurrentRobotState(node->now(), 5.0))
  {
    RCLCPP_WARN(logger,
                "Timed out waiting for current robot state; IK will use the "
                "default seed.");
  }

  moveit::core::RobotState goal_state(robot_model);
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(psm);
    goal_state = scene->getCurrentState();
  }

  // Validity callback used by setFromIK to discard colliding IK solutions.
  auto is_state_valid =
      [&psm, &logger](moveit::core::RobotState* state,
                      const moveit::core::JointModelGroup* group,
                      const double* joint_group_positions) {
        state->setJointGroupPositions(group, joint_group_positions);
        state->update();
        planning_scene_monitor::LockedPlanningSceneRO scene(psm);
        collision_detection::CollisionRequest req;
        req.group_name = group->getName();
        collision_detection::CollisionResult res;
        scene->checkCollision(req, res, *state, scene->getAllowedCollisionMatrix());
        if (res.collision)
          RCLCPP_DEBUG(logger, "IK candidate rejected: collision detected.");
        return !res.collision;
      };

  // Retry IK with random seeds; setFromIK only randomizes from the second
  // attempt onward, so we explicitly randomize between calls.
  constexpr int kIkAttempts = 25;
  constexpr double kIkTimeout = 0.2;
  bool ik_success = false;
  for (int attempt = 0; attempt < kIkAttempts; ++attempt)
  {
    if (goal_state.setFromIK(joint_model_group, desired_pose.pose, tip_link,
                             kIkTimeout, is_state_valid))
    {
      ik_success = true;
      RCLCPP_INFO(logger, "IK solved (collision-free) on attempt %d.", attempt + 1);
      break;
    }
    goal_state.setToRandomPositions(joint_model_group);
  }

  if (!ik_success)
  {
    RCLCPP_ERROR(logger,
                 "Inverse kinematics failed to find a collision-free solution "
                 "for the requested Cartesian goal after %d attempts.",
                 kIkAttempts);
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  moveit_msgs::msg::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

  // ---------------------------------------------------------------------------
  // Formulate the HybridPlanner action goal.
  // ---------------------------------------------------------------------------
  HybridPlanner::Goal goal_action_request;
  goal_action_request.planning_group = planning_group;

  moveit_msgs::msg::MotionSequenceItem sequence_item;
  sequence_item.req.group_name = planning_group;
  sequence_item.req.pipeline_id = "ompl";
  sequence_item.req.planner_id = "RRTConnect";
  sequence_item.req.allowed_planning_time = 5.0;
  sequence_item.req.max_velocity_scaling_factor = 0.1;
  sequence_item.req.max_acceleration_scaling_factor = 0.1;
  sequence_item.req.goal_constraints.push_back(joint_goal);
  // blend_radius is only meaningful for multi-segment sequences; 0.0 = stop
  // at the waypoint.
  sequence_item.blend_radius = 0.0;

  goal_action_request.motion_sequence.items.push_back(sequence_item);

  // ---------------------------------------------------------------------------
  // Send the goal.
  // ---------------------------------------------------------------------------
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
        RCLCPP_INFO(logger, "Hybrid Planner feedback: %s",
                    feedback->feedback.c_str());
      };
  send_goal_options.result_callback =
      [&logger, &result_promise](
          const rclcpp_action::ClientGoalHandle<HybridPlanner>::WrappedResult& result) {
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

  RCLCPP_INFO(logger, "Sending Hybrid Planning goal...");
  hybrid_planning_client->async_send_goal(goal_action_request, send_goal_options);

  result_future.wait();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
