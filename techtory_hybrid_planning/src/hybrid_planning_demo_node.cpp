// Demo client for the MoveIt 2 Hybrid Planning architecture.
//
// Sends a two-waypoint MotionSequenceRequest goal (pose-space) to the
// HybridPlanner action server exposed by the Hybrid Planning Manager. Each
// waypoint is a target pose for the end-effector link expressed in the
// cobotta arm base frame.

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>

using HybridPlanner = moveit_msgs::action::HybridPlanner;
using HybridPlannerGoalHandle = rclcpp_action::ClientGoalHandle<HybridPlanner>;

namespace
{
moveit_msgs::msg::Constraints makePoseGoal(
  const std::string & ee_link, const std::string & frame_id,
  double x, double y, double z,
  double roll, double pitch, double yaw,
  double position_tol, double orientation_tol)
{
  moveit_msgs::msg::Constraints goal;
  goal.name = "pose_goal";

  // Position: a small sphere around the target point.
  moveit_msgs::msg::PositionConstraint pc;
  pc.header.frame_id = frame_id;
  pc.link_name = ee_link;
  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere.dimensions = {position_tol};
  pc.constraint_region.primitives.push_back(sphere);

  geometry_msgs::msg::Pose region_pose;
  region_pose.position.x = x;
  region_pose.position.y = y;
  region_pose.position.z = z;
  region_pose.orientation.w = 1.0;
  pc.constraint_region.primitive_poses.push_back(region_pose);
  pc.weight = 1.0;
  goal.position_constraints.push_back(pc);

  // Orientation: convert RPY -> quaternion.
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header.frame_id = frame_id;
  oc.link_name = ee_link;
  oc.orientation.x = q.x();
  oc.orientation.y = q.y();
  oc.orientation.z = q.z();
  oc.orientation.w = q.w();
  oc.absolute_x_axis_tolerance = orientation_tol;
  oc.absolute_y_axis_tolerance = orientation_tol;
  oc.absolute_z_axis_tolerance = orientation_tol;
  oc.weight = 1.0;
  goal.orientation_constraints.push_back(oc);

  return goal;
}
}  // namespace

class HybridPlanningDemo : public rclcpp::Node
{
public:
  explicit HybridPlanningDemo(const rclcpp::NodeOptions & options)
  : Node("hybrid_planning_demo_node", options)
  {
    planning_group_ = this->declare_parameter<std::string>("planning_group", "arm");
    action_name_ = this->declare_parameter<std::string>("hybrid_planning_action", "/run_hybrid_planning");
    wait_timeout_sec_ = this->declare_parameter<double>("wait_for_server_timeout", 180.0);

    base_frame_ = this->declare_parameter<std::string>("base_frame", "cobotta_pro_base_link");
    ee_link_ = this->declare_parameter<std::string>("ee_link", "cobotta_pro_tool0");
    position_tolerance_ = this->declare_parameter<double>("position_tolerance", 0.01);
    orientation_tolerance_ = this->declare_parameter<double>("orientation_tolerance", 0.05);

    // Two Cartesian waypoints expressed in `base_frame`. Each is
    // [x, y, z, roll, pitch, yaw]. Defaults match the demo poses the user
    // requested.
    waypoint_a_ = this->declare_parameter<std::vector<double>>(
      "waypoint_a", {0.240, 0.12, 0.5, -3.142, 0.0, 3.142});
    waypoint_b_ = this->declare_parameter<std::vector<double>>(
      "waypoint_b", {0.540, 0.12, 0.5, -3.142, 0.0, 3.142});

    action_client_ = rclcpp_action::create_client<HybridPlanner>(this, action_name_);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        timer_->cancel();
        sendGoal();
      });
  }

private:
  moveit_msgs::msg::MotionSequenceItem makeItem(const std::vector<double> & waypoint) const
  {
    moveit_msgs::msg::MotionPlanRequest plan_request;
    plan_request.group_name = planning_group_;
    plan_request.pipeline_id = "ompl";
    plan_request.planner_id = "RRTConnect";
    plan_request.allowed_planning_time = 2.0;
    plan_request.num_planning_attempts = 5;
    plan_request.max_velocity_scaling_factor = 0.5;
    plan_request.max_acceleration_scaling_factor = 0.5;

    plan_request.goal_constraints.push_back(
      makePoseGoal(
        ee_link_, base_frame_,
        waypoint[0], waypoint[1], waypoint[2],
        waypoint[3], waypoint[4], waypoint[5],
        position_tolerance_, orientation_tolerance_));

    moveit_msgs::msg::MotionSequenceItem item;
    item.req = plan_request;
    // blend_radius must be 0 for the final item; we keep it 0 for both since
    // we want each waypoint reached precisely.
    item.blend_radius = 0.0;
    return item;
  }

  void sendGoal()
  {
    RCLCPP_INFO(get_logger(),
                "Waiting up to %.0fs for HybridPlanner action server '%s' (composable components can take ~60s to come up)...",
                wait_timeout_sec_, action_name_.c_str());

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(wait_timeout_sec_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        break;
      }
      RCLCPP_INFO(get_logger(), "... still waiting for '%s'", action_name_.c_str());
    }
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_ERROR(get_logger(),
                   "HybridPlanner action server '%s' not available after %.0fs, shutting down.",
                   action_name_.c_str(), wait_timeout_sec_);
      rclcpp::shutdown();
      return;
    }

    if (waypoint_a_.size() != 6 || waypoint_b_.size() != 6) {
      RCLCPP_ERROR(get_logger(),
                   "waypoint_a/waypoint_b must each have 6 entries [x,y,z,roll,pitch,yaw], got %zu and %zu",
                   waypoint_a_.size(), waypoint_b_.size());
      rclcpp::shutdown();
      return;
    }

    moveit_msgs::msg::MotionSequenceRequest sequence_request;
    sequence_request.items.push_back(makeItem(waypoint_a_));
    sequence_request.items.push_back(makeItem(waypoint_b_));

    HybridPlanner::Goal goal_msg;
    goal_msg.planning_group = planning_group_;
    goal_msg.motion_sequence = sequence_request;

    auto send_goal_options = rclcpp_action::Client<HybridPlanner>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const HybridPlannerGoalHandle::SharedPtr & gh) {
        if (!gh) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by the Hybrid Planning Manager.");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted; hybrid planning in progress...");
        }
      };
    send_goal_options.feedback_callback =
      [this](HybridPlannerGoalHandle::SharedPtr,
             const std::shared_ptr<const HybridPlanner::Feedback> feedback) {
        RCLCPP_INFO(get_logger(), "Feedback: %s", feedback->feedback.c_str());
      };
    send_goal_options.result_callback =
      [this](const HybridPlannerGoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Hybrid planning finished, error_code=%d",
                        result.result->error_code.val);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Hybrid planning aborted: %s",
                         result.result->error_message.c_str());
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Hybrid planning canceled.");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result code.");
            break;
        }
        rclcpp::shutdown();
      };

    RCLCPP_INFO(get_logger(),
                "Sending HybridPlanner goal: 2 pose waypoints for link '%s' in frame '%s' (group '%s')",
                ee_link_.c_str(), base_frame_.c_str(), planning_group_.c_str());
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  std::string planning_group_;
  std::string action_name_;
  std::string base_frame_;
  std::string ee_link_;
  double wait_timeout_sec_{180.0};
  double position_tolerance_{0.01};
  double orientation_tolerance_{0.05};
  std::vector<double> waypoint_a_;
  std::vector<double> waypoint_b_;
  rclcpp_action::Client<HybridPlanner>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Don't enable automatically_declare_parameters_from_overrides — this node
  // declares each parameter explicitly with a default, and the auto-declare
  // would clash with those declarations when launch passes overrides.
  rclcpp::NodeOptions options;
  auto node = std::make_shared<HybridPlanningDemo>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
