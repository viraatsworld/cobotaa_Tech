// Hybrid Planning demo client for the Techtory Cobotta cell.
//
// Waypoints are declared via ROS parameters so adding/removing waypoints does
// not require a recompile. See hybrid_planning_demo.launch.py for the default
// list.

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
struct Waypoint
{
  std::string name;
  std::string type;  // "joint" (named SRDF group_state) or "cartesian"
  // Joint waypoint:
  std::string joint_state_name;
  // Cartesian waypoint (in `planning_frame`):
  double x = 0.0, y = 0.0, z = 0.0;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  // Per-segment params:
  double blend_radius = 0.0;
  double velocity_scaling = 0.1;
  double acceleration_scaling = 0.1;
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

std::vector<Waypoint> loadWaypoints(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger)
{
  std::vector<Waypoint> waypoints;
  std::vector<std::string> names;
  if (!node->get_parameter("waypoint_names", names) || names.empty())
  {
    RCLCPP_WARN(logger, "Parameter 'waypoint_names' is empty; nothing to do.");
    return waypoints;
  }

  for (const std::string& name : names)
  {
    Waypoint wp;
    wp.name = name;
    const std::string base = "waypoints." + name + ".";
    wp.type = getOr<std::string>(node, base + "type", "joint");
    wp.blend_radius = getOr<double>(node, base + "blend_radius", 0.0);
    wp.velocity_scaling = getOr<double>(node, base + "velocity_scaling", 0.1);
    wp.acceleration_scaling = getOr<double>(node, base + "acceleration_scaling", 0.1);
    wp.pipeline_id = getOr<std::string>(node, base + "pipeline_id", "ompl");
    wp.planner_id = getOr<std::string>(node, base + "planner_id", "RRTConnect");
    wp.allowed_planning_time = getOr<double>(node, base + "allowed_planning_time", 5.0);

    if (wp.type == "joint")
    {
      wp.joint_state_name = getOr<std::string>(node, base + "joint_state", name);
    }
    else if (wp.type == "cartesian" || wp.type == "cartesian_relative")
    {
      wp.x = getOr<double>(node, base + "x", 0.0);
      wp.y = getOr<double>(node, base + "y", 0.0);
      wp.z = getOr<double>(node, base + "z", 0.0);
      wp.roll = getOr<double>(node, base + "roll", 0.0);
      wp.pitch = getOr<double>(node, base + "pitch", 0.0);
      wp.yaw = getOr<double>(node, base + "yaw", 0.0);
    }
    else
    {
      RCLCPP_ERROR(logger, "Waypoint '%s' has unknown type '%s' (expected 'joint' or 'cartesian').",
                   name.c_str(), wp.type.c_str());
      return {};
    }
    waypoints.push_back(std::move(wp));
  }
  return waypoints;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "hybrid_planning_demo_node",
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

  // ---------------------------------------------------------------------------
  // Action client targeting the Hybrid Planning Manager.
  // ---------------------------------------------------------------------------
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

  // ---------------------------------------------------------------------------
  // Robot model and planning scene monitor (collision-aware IK for Cartesian
  // waypoints).
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

  // Seed IK from the current robot state — the robot is already in its
  // operating pose at launch (set via initial_positions.yaml).
  moveit::core::RobotState seed_state(robot_model);
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(psm);
    seed_state = scene->getCurrentState();
  }
  seed_state.update();
  {
    const Eigen::Isometry3d planning_T_tip =
        seed_state.getGlobalLinkTransform(planning_frame).inverse() *
        seed_state.getGlobalLinkTransform(tip_link);
    const auto& t = planning_T_tip.translation();
    Eigen::Vector3d rpy = planning_T_tip.rotation().eulerAngles(0, 1, 2);
    RCLCPP_INFO(logger,
                "Seed state FK '%s' in '%s': xyz=[%.3f,%.3f,%.3f] rpy=[%.3f,%.3f,%.3f] (use to "
                "choose reachable Cartesian waypoints).",
                tip_link.c_str(), planning_frame.c_str(), t.x(), t.y(), t.z(), rpy.x(), rpy.y(),
                rpy.z());
  }

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

  // ---------------------------------------------------------------------------
  // Load waypoints from parameters and turn each into a MotionSequenceItem.
  // ---------------------------------------------------------------------------
  std::vector<Waypoint> waypoints = loadWaypoints(node, logger);
  if (waypoints.empty())
  {
    RCLCPP_ERROR(logger, "No waypoints provided; aborting.");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  HybridPlanner::Goal goal_action_request;
  goal_action_request.planning_group = planning_group;

  moveit::core::RobotState current_state = seed_state;

  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    const Waypoint& wp = waypoints[i];
    moveit::core::RobotState goal_state = current_state;

    if (wp.type == "joint")
    {
      if (!goal_state.setToDefaultValues(joint_model_group, wp.joint_state_name))
      {
        RCLCPP_ERROR(logger, "Waypoint '%s': SRDF named state '%s' not found.", wp.name.c_str(),
                     wp.joint_state_name.c_str());
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
      }
    }
    else  // cartesian (absolute in planning_frame) or cartesian_relative (delta from prev TCP)
    {
      current_state.update();
      // setFromIK(Pose, tip) interprets `Pose` in the robot model's root frame
      // (typically world/cell_link), NOT in `planning_frame`. Transform
      // explicitly so user-facing poses can be specified in cobotta_pro_base_link.
      const Eigen::Isometry3d& world_T_planning =
          current_state.getGlobalLinkTransform(planning_frame);

      Eigen::Isometry3d target_in_planning;
      if (wp.type == "cartesian_relative")
      {
        // Delta is applied in the previous TCP frame (planning_frame coords).
        const Eigen::Isometry3d& world_T_tip = current_state.getGlobalLinkTransform(tip_link);
        const Eigen::Isometry3d planning_T_tip = world_T_planning.inverse() * world_T_tip;
        Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
        delta.translation() = Eigen::Vector3d(wp.x, wp.y, wp.z);
        delta.linear() = (Eigen::AngleAxisd(wp.roll, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(wp.pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(wp.yaw, Eigen::Vector3d::UnitZ()))
                             .toRotationMatrix();
        target_in_planning = planning_T_tip * delta;
      }
      else
      {
        target_in_planning.translation() = Eigen::Vector3d(wp.x, wp.y, wp.z);
        target_in_planning.linear() = (Eigen::AngleAxisd(wp.roll, Eigen::Vector3d::UnitX()) *
                                       Eigen::AngleAxisd(wp.pitch, Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(wp.yaw, Eigen::Vector3d::UnitZ()))
                                          .toRotationMatrix();
      }

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
                  "Waypoint '%s': solving IK; target in '%s' xyz=[%.3f,%.3f,%.3f] "
                  "rpy=[%.3f,%.3f,%.3f].",
                  wp.name.c_str(), planning_frame.c_str(), target_in_planning.translation().x(),
                  target_in_planning.translation().y(), target_in_planning.translation().z(),
                  wp.roll, wp.pitch, wp.yaw);
      constexpr double kIkTimeout = 0.5;
      // IKFast is analytic — randomization does not help. Seed from the
      // previous state once; if rejected by the validity callback, also try
      // without the callback so the failure mode (unreachable vs colliding)
      // is unambiguous.
      bool ok = goal_state.setFromIK(joint_model_group, pose, tip_link, kIkTimeout, is_state_valid);
      if (!ok)
      {
        moveit::core::RobotState probe = goal_state;
        if (probe.setFromIK(joint_model_group, pose, tip_link, kIkTimeout))
        {
          RCLCPP_ERROR(logger,
                       "Waypoint '%s': IK solution exists but every candidate collides. Adjust "
                       "the pose or update the SRDF.",
                       wp.name.c_str());
        }
        else
        {
          RCLCPP_ERROR(logger,
                       "Waypoint '%s': pose is unreachable by IKFast (no analytic solution).",
                       wp.name.c_str());
        }
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
      }
    }

    moveit_msgs::msg::MotionSequenceItem item;
    item.req.group_name = planning_group;
    item.req.pipeline_id = wp.pipeline_id;
    item.req.planner_id = wp.planner_id;
    item.req.allowed_planning_time = wp.allowed_planning_time;
    item.req.max_velocity_scaling_factor = wp.velocity_scaling;
    item.req.max_acceleration_scaling_factor = wp.acceleration_scaling;
    item.req.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group));
    // blend_radius must be 0.0 for the final waypoint.
    item.blend_radius = (i + 1 == waypoints.size()) ? 0.0 : wp.blend_radius;
    goal_action_request.motion_sequence.items.push_back(item);

    RCLCPP_INFO(logger, "Waypoint %zu/%zu '%s' (%s) prepared.", i + 1, waypoints.size(),
                wp.name.c_str(), wp.type.c_str());
    current_state = goal_state;
  }

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
