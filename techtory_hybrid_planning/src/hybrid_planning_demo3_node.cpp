// Hybrid Planning demo client (variant 3) for the Techtory Cobotta cell.
//
// Unlike demo / demo2 (which build joint-space goal constraints), demo3 speaks
// the "welding" request layout consumed by the MTC global planner
// (techtory_hybrid_planning::GlobalMTCPlannerComponent): a single
// MotionSequenceItem whose goal_constraints[0] carries TWO position_constraints
// and TWO orientation_constraints, encoding a start pose and a goal pose:
//
//   position_constraints[0].target_point_offset / orientation_constraints[0] -> start
//   position_constraints[1].target_point_offset / orientation_constraints[1] -> goal
//
// The MTC plugin interprets both poses in the "world" frame, so the two
// user-facing Cartesian poses (specified in `planning_frame`) are transformed
// to world via the robot model before being written into the constraints.

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>

using HybridPlanner = moveit_msgs::action::HybridPlanner;

namespace
{
struct CartesianPose
{
  std::string name;
  double x, y, z;          // position in `planning_frame`
  double roll, pitch, yaw;  // orientation in `planning_frame`
};

template <typename T>
T getOr(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& fallback)
{
  T value;
  if (node->get_parameter(name, value))
    return value;
  return fallback;
}

// Transform a pose given in `planning_frame` into the model root ("world")
// frame, using the current robot state for the planning_frame transform.
geometry_msgs::msg::Pose toWorld(const moveit::core::RobotState& state, const std::string& planning_frame,
                                 const CartesianPose& p)
{
  const Eigen::Isometry3d& world_T_planning = state.getGlobalLinkTransform(planning_frame);

  Eigen::Isometry3d target_in_planning = Eigen::Isometry3d::Identity();
  target_in_planning.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  target_in_planning.linear() = (Eigen::AngleAxisd(p.roll, Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(p.pitch, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(p.yaw, Eigen::Vector3d::UnitZ()))
                                    .toRotationMatrix();
  const Eigen::Isometry3d world_T_target = world_T_planning * target_in_planning;

  geometry_msgs::msg::Pose pose;
  pose.position.x = world_T_target.translation().x();
  pose.position.y = world_T_target.translation().y();
  pose.position.z = world_T_target.translation().z();
  const Eigen::Quaterniond q(world_T_target.rotation());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "hybrid_planning_demo3_node",
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

  // Start and goal welding poses, in `planning_frame`. Override via parameters
  // (weld_start.{x,y,z,roll,pitch,yaw} / weld_goal.{...}) or edit these defaults.
  CartesianPose start{ "weld_start",
                       getOr<double>(node, "weld_start.x", 0.450),
                       getOr<double>(node, "weld_start.y", 0.100),
                       getOr<double>(node, "weld_start.z", 0.410),
                       getOr<double>(node, "weld_start.roll", 3.140),
                       getOr<double>(node, "weld_start.pitch", 0.0),
                       getOr<double>(node, "weld_start.yaw", 3.140) };
  CartesianPose goal{ "weld_goal",
                      getOr<double>(node, "weld_goal.x", 0.450),
                      getOr<double>(node, "weld_goal.y", 0.300),
                      getOr<double>(node, "weld_goal.z", 0.410),
                      getOr<double>(node, "weld_goal.roll", 3.140),
                      getOr<double>(node, "weld_goal.pitch", 0.0),
                      getOr<double>(node, "weld_goal.yaw", 3.140) };

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

  // Robot model + planning scene monitor to obtain the current state (used to
  // resolve the planning_frame -> world transform).
  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  if (!robot_model)
  {
    RCLCPP_ERROR(logger, "Failed to load robot model from /robot_description.");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }
  if (!robot_model->getJointModelGroup(planning_group))
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
    RCLCPP_WARN(logger, "Timed out waiting for current robot state; using model defaults.");
  }

  moveit::core::RobotState state(robot_model);
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(psm);
    state = scene->getCurrentState();
  }
  state.update();

  const geometry_msgs::msg::Pose start_world = toWorld(state, planning_frame, start);
  const geometry_msgs::msg::Pose goal_world = toWorld(state, planning_frame, goal);

  RCLCPP_INFO(logger,
              "Welding request: start (in '%s') xyz=[%.3f,%.3f,%.3f] -> goal xyz=[%.3f,%.3f,%.3f].",
              planning_frame.c_str(), start.x, start.y, start.z, goal.x, goal.y, goal.z);

  // Build a single Constraints carrying both poses in the welding layout.
  auto make_position = [&tip_link](const geometry_msgs::msg::Pose& p) {
    moveit_msgs::msg::PositionConstraint pc;
    pc.header.frame_id = "world";
    pc.link_name = tip_link;
    pc.target_point_offset.x = p.position.x;
    pc.target_point_offset.y = p.position.y;
    pc.target_point_offset.z = p.position.z;
    pc.weight = 1.0;
    return pc;
  };
  auto make_orientation = [&tip_link](const geometry_msgs::msg::Pose& p) {
    moveit_msgs::msg::OrientationConstraint oc;
    oc.header.frame_id = "world";
    oc.link_name = tip_link;
    oc.orientation = p.orientation;
    oc.weight = 1.0;
    return oc;
  };

  moveit_msgs::msg::Constraints welding_constraints;
  welding_constraints.position_constraints.push_back(make_position(start_world));
  welding_constraints.position_constraints.push_back(make_position(goal_world));
  welding_constraints.orientation_constraints.push_back(make_orientation(start_world));
  welding_constraints.orientation_constraints.push_back(make_orientation(goal_world));

  moveit_msgs::msg::MotionSequenceItem item;
  item.req.group_name = planning_group;
  item.req.allowed_planning_time = 5.0;
  item.req.num_planning_attempts = 5;
  item.req.max_velocity_scaling_factor = 0.5;
  item.req.max_acceleration_scaling_factor = 0.5;
  item.req.goal_constraints.push_back(welding_constraints);
  item.blend_radius = 0.0;

  HybridPlanner::Goal goal_action_request;
  goal_action_request.planning_group = planning_group;
  goal_action_request.motion_sequence.items.push_back(item);

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

  RCLCPP_INFO(logger, "Sending welding Hybrid Planning goal (start + goal in one item)...");
  hybrid_planning_client->async_send_goal(goal_action_request, send_goal_options);

  // Bound the wait so a server that dies after accepting the goal (no result
  // callback) cannot hang this process forever.
  const double result_timeout = getOr<double>(node, "result_timeout", 120.0);
  int exit_code = 0;
  if (result_future.wait_for(std::chrono::duration<double>(result_timeout)) != std::future_status::ready)
  {
    RCLCPP_ERROR(logger, "Timed out after %.0fs waiting for the Hybrid Planning result.", result_timeout);
    exit_code = 1;
  }

  rclcpp::shutdown();
  spin_thread.join();
  return exit_code;
}
